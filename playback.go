package sequenceplayback

import (
	"context"
	"fmt"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"golang.org/x/sync/errgroup"
)

// clock is the time source for pacing; tests substitute one that advances only on Sleep.
type clock interface {
	Now() time.Time
	Sleep(ctx context.Context, d time.Duration) error
}

type realClock struct{}

func (realClock) Now() time.Time { return time.Now() }

func (realClock) Sleep(ctx context.Context, d time.Duration) error {
	if d <= 0 {
		return ctx.Err()
	}
	t := time.NewTimer(d)
	defer t.Stop()
	select {
	case <-ctx.Done():
		return ctx.Err()
	case <-t.C:
		return nil
	}
}

// player replays one Sequence on an arm and optional gripper at the recorded timing.
type player struct {
	arm        arm.Arm
	gripper    gripper.Gripper // nil when not configured
	gripperKey string
	armExtra   map[string]any
	clock      clock

	frameIndex atomic.Int64 // frames issued so far, for status
}

func (p *player) setGripper(ctx context.Context, pos float64) error {
	_, err := p.gripper.DoCommand(ctx, map[string]any{"command": "set_position", p.gripperKey: pos})
	return err
}

// moveFunc issues arm frame i with the given extra; the caller decides whether that is a
// joint or a pose move.
type moveFunc func(ctx context.Context, i int, extra map[string]any) error

// run blocks until the sequence has played, failed, or ctx is cancelled. The arm is always
// sent Stop before returning.
func (p *player) run(ctx context.Context, seq *Sequence, move moveFunc) (err error) {
	p.frameIndex.Store(0)
	defer func() {
		if stopErr := p.arm.Stop(context.Background(), nil); stopErr != nil && err == nil {
			err = fmt.Errorf("stopping arm after playback: %w", stopErr)
		}
	}()

	useGripper := p.gripper != nil && seq.HasGripper()

	// Safe entry: a blocking move to the first frame, and the gripper to its first value.
	g, gctx := errgroup.WithContext(ctx)
	g.Go(func() error { return move(gctx, 0, nil) })
	if useGripper {
		g.Go(func() error { return p.setGripper(gctx, seq.GripperPositions[0]) })
	}
	if err := g.Wait(); err != nil {
		return fmt.Errorf("safe entry: %w", err)
	}
	p.frameIndex.Store(1)

	// Main motion: both tracks pace off one start time.
	start := p.clock.Now()
	waitUntil := func(ctx context.Context, off time.Duration) error {
		return p.clock.Sleep(ctx, start.Add(off).Sub(p.clock.Now()))
	}
	g, gctx = errgroup.WithContext(ctx)
	g.Go(func() error {
		for i := 1; i < seq.Len(); i++ {
			if err := waitUntil(gctx, seq.Offsets[i]); err != nil {
				return err
			}
			if err := move(gctx, i, p.armExtra); err != nil {
				return fmt.Errorf("frame %d: %w", i, err)
			}
			p.frameIndex.Store(int64(i + 1))
		}
		return nil
	})
	if useGripper {
		g.Go(func() error {
			last := seq.GripperPositions[0]
			for i := 1; i < len(seq.GripperPositions); i++ {
				pos := seq.GripperPositions[i]
				if pos == last {
					continue
				}
				if err := waitUntil(gctx, seq.GripperOffsets[i]); err != nil {
					return err
				}
				if err := p.setGripper(gctx, pos); err != nil {
					return fmt.Errorf("gripper step %d: %w", i, err)
				}
				last = pos
			}
			return nil
		})
	}
	return g.Wait()
}
