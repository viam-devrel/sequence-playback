package sequenceplayback

import (
	"context"
	"errors"
	"sync"
	"testing"
	"time"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/test"
)

// fakeClock advances only when slept on, so pacing is checked without real time.
type fakeClock struct {
	mu  sync.Mutex
	now time.Time
}

func (c *fakeClock) Now() time.Time {
	c.mu.Lock()
	defer c.mu.Unlock()
	return c.now
}

func (c *fakeClock) Sleep(ctx context.Context, d time.Duration) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	c.mu.Lock()
	c.now = c.now.Add(d)
	c.mu.Unlock()
	return nil
}

type moveCall struct {
	at     time.Time
	joints []referenceframe.Input
	extra  map[string]any
}

type rig struct {
	clock  *fakeClock
	arm    *inject.Arm
	grip   *inject.Gripper
	mu     sync.Mutex
	moves  []moveCall
	grips  []map[string]any
	stops  int
	player *player
}

func newRig(t *testing.T, withGripper bool) *rig {
	t.Helper()
	r := &rig{clock: &fakeClock{now: t0}, arm: &inject.Arm{}}
	r.arm.MoveToJointPositionsFunc = func(ctx context.Context, p []referenceframe.Input, extra map[string]any) error {
		r.mu.Lock()
		defer r.mu.Unlock()
		r.moves = append(r.moves, moveCall{r.clock.Now(), p, extra})
		return nil
	}
	r.arm.StopFunc = func(context.Context, map[string]any) error {
		r.mu.Lock()
		defer r.mu.Unlock()
		r.stops++
		return nil
	}
	r.player = &player{arm: r.arm, armExtra: map[string]any{"streamed": true}, clock: r.clock}
	if withGripper {
		r.grip = &inject.Gripper{}
		r.grip.DoFunc = func(_ context.Context, cmd map[string]any) (map[string]any, error) {
			r.mu.Lock()
			defer r.mu.Unlock()
			r.grips = append(r.grips, cmd)
			return nil, nil
		}
		r.player.gripper = r.grip
		r.player.gripperKey = "position_percentage"
	}
	return r
}

func threeFrames() *Sequence {
	return &Sequence{
		ID:        "s",
		Offsets:   []time.Duration{0, 100 * time.Millisecond, 300 * time.Millisecond},
		JointsDeg: [][]float64{{0, 0}, {90, 0}, {180, 0}},
	}
}

func rads(deg ...float64) []referenceframe.Input {
	out := make([]referenceframe.Input, len(deg))
	for i, d := range deg {
		out[i] = d * 3.141592653589793 / 180
	}
	return out
}

func TestRunPacesArmFramesByRecordedOffsets(t *testing.T) {
	r := newRig(t, false)
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), threeFrames(), inputs)
	test.That(t, err, test.ShouldBeNil)

	test.That(t, len(r.moves), test.ShouldEqual, 3)
	// Safe entry: first frame, no streaming extra, before the clock starts.
	test.That(t, r.moves[0].joints, test.ShouldResemble, inputs[0])
	test.That(t, r.moves[0].extra, test.ShouldBeNil)
	start := r.moves[0].at
	test.That(t, r.moves[1].joints, test.ShouldResemble, inputs[1])
	test.That(t, r.moves[1].extra, test.ShouldResemble, map[string]any{"streamed": true})
	test.That(t, r.moves[1].at.Sub(start), test.ShouldEqual, 100*time.Millisecond)
	test.That(t, r.moves[2].at.Sub(start), test.ShouldEqual, 300*time.Millisecond)
	test.That(t, r.stops, test.ShouldEqual, 1)
	test.That(t, r.player.frameIndex.Load(), test.ShouldEqual, int64(3))
}

func TestRunDrivesGripperOnSharedClockAndDedupes(t *testing.T) {
	r := newRig(t, true)
	seq := threeFrames()
	seq.GripperOffsets = []time.Duration{-20 * time.Millisecond, 100 * time.Millisecond, 200 * time.Millisecond, 250 * time.Millisecond}
	seq.GripperPositions = []float64{90, 90, 10, 10}
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), seq, inputs)
	test.That(t, err, test.ShouldBeNil)

	// Safe entry sets the first value; the repeated 90 and repeated 10 are skipped.
	test.That(t, r.grips, test.ShouldResemble, []map[string]any{
		{"command": "set_position", "position_percentage": 90.0},
		{"command": "set_position", "position_percentage": 10.0},
	})
	test.That(t, r.stops, test.ShouldEqual, 1)
}

func TestRunWritesGetShorthandRecordingsWithSet(t *testing.T) {
	r := newRig(t, true)
	r.player.gripperKey = "position"
	seq := threeFrames()
	seq.GripperOffsets = []time.Duration{0, 200 * time.Millisecond}
	seq.GripperPositions = []float64{90, 10}
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), seq, inputs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r.grips, test.ShouldResemble, []map[string]any{{"set": 90.0}, {"set": 10.0}})
}

func TestRunWithoutGripperDataLeavesGripperAlone(t *testing.T) {
	r := newRig(t, true)
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), threeFrames(), inputs)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, r.grips, test.ShouldBeEmpty)
}

func TestRunStopsArmOnMoveError(t *testing.T) {
	r := newRig(t, false)
	boom := errors.New("servo fault")
	n := 0
	r.arm.MoveToJointPositionsFunc = func(context.Context, []referenceframe.Input, map[string]any) error {
		n++
		if n == 2 {
			return boom
		}
		return nil
	}
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), threeFrames(), inputs)
	test.That(t, errors.Is(err, boom), test.ShouldBeTrue)
	test.That(t, n, test.ShouldEqual, 2)
	test.That(t, r.stops, test.ShouldEqual, 1)
}

func TestRunStopsArmOnCancel(t *testing.T) {
	r := newRig(t, false)
	ctx, cancel := context.WithCancel(context.Background())
	r.arm.MoveToJointPositionsFunc = func(context.Context, []referenceframe.Input, map[string]any) error {
		cancel() // cancelled after safe entry
		return nil
	}
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(ctx, threeFrames(), inputs)
	test.That(t, errors.Is(err, context.Canceled), test.ShouldBeTrue)
	test.That(t, r.stops, test.ShouldEqual, 1)
}

func TestRunSafeEntryGripperErrorAborts(t *testing.T) {
	r := newRig(t, true)
	r.grip.DoFunc = func(context.Context, map[string]any) (map[string]any, error) {
		return nil, errors.New("gripper offline")
	}
	seq := threeFrames()
	seq.GripperOffsets = []time.Duration{0}
	seq.GripperPositions = []float64{50}
	inputs := [][]referenceframe.Input{rads(0, 0), rads(90, 0), rads(180, 0)}
	err := r.player.run(context.Background(), seq, inputs)
	test.That(t, err, test.ShouldNotBeNil)
	test.That(t, err.Error(), test.ShouldContainSubstring, "gripper offline")
	test.That(t, len(r.moves), test.ShouldBeLessThanOrEqualTo, 1)
	test.That(t, r.stops, test.ShouldEqual, 1)
}
