package sequenceplayback

import (
	"context"
	"errors"
	"math"
	"testing"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

type svcRig struct {
	svc     *armPlayback
	arm     *inject.Arm
	release chan struct{} // closed by the test to let blocked moves return
	fetches int
}

func newSvc(t *testing.T, cfg *Config) *svcRig {
	t.Helper()
	r := &svcRig{arm: &inject.Arm{}, release: make(chan struct{})}
	r.arm.JointPositionsFunc = func(context.Context, map[string]any) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0, 0}, nil
	}
	r.arm.KinematicsFunc = func(context.Context) (referenceframe.Model, error) { return nil, errors.New("no model") }
	r.arm.MoveToJointPositionsFunc = func(ctx context.Context, _ []referenceframe.Input, _ map[string]any) error {
		select {
		case <-r.release:
			return nil
		case <-ctx.Done():
			return ctx.Err()
		}
	}
	r.arm.StopFunc = func(context.Context, map[string]any) error { return nil }
	grip := &inject.Gripper{}
	grip.DoFunc = func(context.Context, map[string]any) (map[string]any, error) { return nil, nil }

	deps := resource.Dependencies{arm.Named(cfg.Arm): r.arm}
	if cfg.Gripper != "" {
		deps[gripper.Named(cfg.Gripper)] = grip
	}
	conf := resource.Config{Name: "pb", API: generic.API, Model: ArmPlayback, ConvertedAttributes: cfg}
	res, err := NewArmPlayback(context.Background(), deps, conf, logging.NewTestLogger(t))
	test.That(t, err, test.ShouldBeNil)
	r.svc = res.(*armPlayback)
	r.svc.player.clock = &fakeClock{now: t0}
	r.svc.fetch = func(_ context.Context, id string) (*Sequence, error) {
		r.fetches++
		if id == "missing" {
			return nil, errors.New("sequence not found")
		}
		s := threeFrames()
		s.ID = id
		return s, nil
	}
	t.Cleanup(func() { test.That(t, r.svc.Close(context.Background()), test.ShouldBeNil) })
	return r
}

func do(t *testing.T, s *armPlayback, cmd map[string]any) (map[string]any, error) {
	t.Helper()
	return s.DoCommand(context.Background(), cmd)
}

func waitIdle(t *testing.T, s *armPlayback) map[string]any {
	t.Helper()
	deadline := time.Now().Add(5 * time.Second)
	for time.Now().Before(deadline) {
		st, _ := do(t, s, map[string]any{"command": "status"})
		if st["state"] == "idle" {
			return st
		}
		time.Sleep(5 * time.Millisecond)
	}
	t.Fatal("playback never returned to idle")
	return nil
}

func TestDoCommandUnknown(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	_, err := do(t, r.svc, map[string]any{"command": "dance"})
	test.That(t, err, test.ShouldNotBeNil)
	_, err = do(t, r.svc, map[string]any{})
	test.That(t, err, test.ShouldNotBeNil)
}

func TestPlayStatusStop(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a", Gripper: "g"})

	resp, err := do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["status"], test.ShouldEqual, "playing")
	test.That(t, resp["sequence_id"], test.ShouldEqual, "s1")
	test.That(t, resp["frame_count"], test.ShouldEqual, 3)
	test.That(t, resp["duration_s"], test.ShouldEqual, 0.3)
	test.That(t, resp["has_gripper"], test.ShouldBeFalse)

	st, _ := do(t, r.svc, map[string]any{"command": "status"})
	test.That(t, st["state"], test.ShouldEqual, "playing")
	test.That(t, st["sequence_id"], test.ShouldEqual, "s1")
	test.That(t, st["frame_count"], test.ShouldEqual, 3)

	_, err = do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "playing")

	resp, err = do(t, r.svc, map[string]any{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["status"], test.ShouldEqual, "stopped")
	st = waitIdle(t, r.svc)
	test.That(t, st["last_error"], test.ShouldBeEmpty) // a user stop is not an error

	// stop when idle is a no-op
	_, err = do(t, r.svc, map[string]any{"command": "stop"})
	test.That(t, err, test.ShouldBeNil)
}

func TestPlayCompletesAndCachesFetch(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	close(r.release)
	_, err := do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldBeNil)
	st := waitIdle(t, r.svc)
	test.That(t, st["frame_index"], test.ShouldEqual, 3)
	test.That(t, st["last_error"], test.ShouldBeEmpty)

	_, err = do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldBeNil)
	waitIdle(t, r.svc)
	test.That(t, r.fetches, test.ShouldEqual, 1)
}

func TestPlayValidation(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	_, err := do(t, r.svc, map[string]any{"command": "play"})
	test.That(t, err.Error(), test.ShouldContainSubstring, "sequence_id")

	_, err = do(t, r.svc, map[string]any{"command": "play", "sequence_id": "missing"})
	test.That(t, err.Error(), test.ShouldContainSubstring, "not found")

	r.arm.JointPositionsFunc = func(context.Context, map[string]any) ([]referenceframe.Input, error) {
		return []referenceframe.Input{0, 0, 0}, nil
	}
	_, err = do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err.Error(), test.ShouldContainSubstring, "joint count")
}

func TestPlayErrorSurfacesThenClears(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	r.arm.MoveToJointPositionsFunc = func(context.Context, []referenceframe.Input, map[string]any) error {
		return errors.New("servo fault")
	}
	_, err := do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldBeNil)
	st := waitIdle(t, r.svc)
	test.That(t, st["last_error"], test.ShouldContainSubstring, "servo fault")

	r.arm.MoveToJointPositionsFunc = func(context.Context, []referenceframe.Input, map[string]any) error { return nil }
	_, err = do(t, r.svc, map[string]any{"command": "play", "sequence_id": "s1"})
	test.That(t, err, test.ShouldBeNil)
	st = waitIdle(t, r.svc)
	test.That(t, st["last_error"], test.ShouldBeEmpty)
}

func TestPlayEndPositionUsesMoveToPosition(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a", Method: "EndPosition"})
	// No joints on the arm at all: the pose path must not consult them.
	r.arm.JointPositionsFunc = func(context.Context, map[string]any) ([]referenceframe.Input, error) {
		return nil, errors.New("not called")
	}
	var got []spatialmath.Pose
	r.arm.MoveToPositionFunc = func(_ context.Context, p spatialmath.Pose, _ map[string]any) error {
		got = append(got, p)
		return nil
	}
	r.svc.fetch = func(_ context.Context, id string) (*Sequence, error) {
		s := threeFrames()
		s.JointsDeg = nil
		s.Poses = []spatialmath.Pose{spatialmath.NewZeroPose(), spatialmath.NewZeroPose(), spatialmath.NewZeroPose()}
		return s, nil
	}
	resp, err := do(t, r.svc, map[string]any{"command": "play", "sequence_id": "p1"})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, resp["frame_count"], test.ShouldEqual, 3)
	st := waitIdle(t, r.svc)
	test.That(t, st["last_error"], test.ShouldBeEmpty)
	test.That(t, st["frame_count"], test.ShouldEqual, 3)
	test.That(t, len(got), test.ShouldEqual, 3)
}

func TestListRequiresDatasetID(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	_, err := do(t, r.svc, map[string]any{"command": "list"})
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "dataset_id")
}

func TestToInputsFallsBackToDegreesWhenNoKinematics(t *testing.T) {
	r := newSvc(t, &Config{Arm: "a"})
	in, err := r.svc.toInputs(context.Background(), [][]float64{{90, 180}})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, in[0][0], test.ShouldAlmostEqual, math.Pi/2)
	test.That(t, in[0][1], test.ShouldAlmostEqual, math.Pi)
}
