package sequenceplayback

import (
	"cmp"
	"context"
	"errors"
	"fmt"
	"sync"

	pb "go.viam.com/api/component/arm/v1"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

var ArmPlayback = resource.NewModel("devrel", "sequence-playback", "arm-playback")

func init() {
	resource.RegisterService(generic.API, ArmPlayback,
		resource.Registration[resource.Resource, *Config]{
			Constructor: NewArmPlayback,
		},
	)
}

type Config struct {
	Arm                string         `json:"arm"`
	Method             string         `json:"method,omitempty"` // JointPositions (default) or EndPosition
	Gripper            string         `json:"gripper,omitempty"`
	GripperPositionKey string         `json:"gripper_position_key,omitempty"`
	SourceArm          string         `json:"source_arm,omitempty"`
	SourceGripper      string         `json:"source_gripper,omitempty"`
	DatasetID          string         `json:"dataset_id,omitempty"`
	ArmExtra           map[string]any `json:"arm_extra,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.SourceGripper != "" && cfg.Gripper == "" {
		return nil, nil, fmt.Errorf("%s: source_gripper requires gripper", path)
	}
	switch cfg.Method {
	case "", "JointPositions", "EndPosition":
	default:
		return nil, nil, fmt.Errorf("%s: method must be JointPositions or EndPosition, got %q", path, cfg.Method)
	}
	deps := []string{cfg.Arm}
	if cfg.Gripper != "" {
		deps = append(deps, cfg.Gripper)
	}
	return deps, nil, nil
}

// armExtra is passed to every paced MoveToJointPositions. The default is the SO-101
// driver's streaming-setpoint mode; drivers that don't know these keys ignore them.
func (cfg *Config) armExtra() map[string]any {
	if cfg.ArmExtra != nil {
		return cfg.ArmExtra
	}
	return map[string]any{"wait": false, "streamed": true}
}

type armPlayback struct {
	resource.AlwaysRebuild
	resource.Named
	logger logging.Logger
	cfg    *Config
	arm    arm.Arm
	player *player

	// cloud access; tests replace fetch
	fetch    func(ctx context.Context, id string) (*Sequence, error)
	viam     *app.ViamClient
	dc       *app.DataClient
	cloudErr error

	mu        sync.Mutex
	cache     map[string]*Sequence
	sequence  string // active or most recent sequence id
	lastError string
	cancel    context.CancelFunc // non-nil while playing
	done      chan struct{}
}

func NewArmPlayback(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	cfg, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	a, err := arm.FromDependencies(deps, cfg.Arm)
	if err != nil {
		return nil, err
	}
	s := &armPlayback{
		Named:  rawConf.ResourceName().AsNamed(),
		logger: logger,
		cfg:    cfg,
		arm:    a,
		cache:  map[string]*Sequence{},
		player: &player{arm: a, armExtra: cfg.armExtra(), clock: realClock{},
			gripperKey: cmp.Or(cfg.GripperPositionKey, "position_percentage")},
	}
	if cfg.Gripper != "" {
		if s.player.gripper, err = gripper.FromDependencies(deps, cfg.Gripper); err != nil {
			return nil, err
		}
	}

	// The module manager supplies VIAM_API_KEY / VIAM_API_KEY_ID. Without them the
	// resource still builds; play and list report the problem when called.
	if s.viam, err = app.CreateViamClientFromEnvVars(ctx, nil, logger); err != nil {
		logger.Warnw("cloud client unavailable; play and list will fail", "error", err)
		s.cloudErr = fmt.Errorf("cloud client unavailable: %w", err)
	} else {
		s.dc = s.viam.DataClient()
	}
	s.fetch = func(ctx context.Context, id string) (*Sequence, error) {
		if s.cloudErr != nil {
			return nil, s.cloudErr
		}
		return fetchSequence(ctx, s.dc, id, cmp.Or(cfg.Method, "JointPositions"),
			cmp.Or(cfg.SourceArm, cfg.Arm), cmp.Or(cfg.SourceGripper, cfg.Gripper), s.player.gripperKey)
	}
	return s, nil
}

func (s *armPlayback) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	switch cmd["command"] {
	case "play":
		return s.play(ctx, cmd)
	case "stop":
		return s.stop()
	case "status":
		return s.Status(ctx)
	case "list":
		if s.cfg.DatasetID == "" {
			return nil, errors.New("list requires dataset_id in the service config")
		}
		if s.cloudErr != nil {
			return nil, s.cloudErr
		}
		seqs, err := listSequences(ctx, s.dc, s.cfg.DatasetID)
		if err != nil {
			return nil, err
		}
		return map[string]any{"sequences": seqs}, nil
	}
	return nil, fmt.Errorf("unknown command %q; expected play, stop, status, or list", cmd["command"])
}

// toInputs converts captured degrees to the arm's inputs, the inverse of the capture
// collector. Without kinematics every joint is treated as revolute (degrees to radians).
func (s *armPlayback) toInputs(ctx context.Context, deg [][]float64) ([][]referenceframe.Input, error) {
	kin, err := s.arm.Kinematics(ctx)
	if err != nil {
		s.logger.Debugw("no kinematics; converting all joints degrees->radians", "error", err)
		kin = nil
	}
	out := make([][]referenceframe.Input, len(deg))
	for i, d := range deg {
		if out[i], err = referenceframe.InputsFromJointPositions(kin, &pb.JointPositions{Values: d}); err != nil {
			return nil, fmt.Errorf("frame %d: %w", i, err)
		}
	}
	return out, nil
}

func (s *armPlayback) play(ctx context.Context, cmd map[string]any) (map[string]any, error) {
	id, _ := cmd["sequence_id"].(string)
	if id == "" {
		return nil, errors.New("play requires a non-empty sequence_id")
	}
	s.mu.Lock()
	if s.cancel != nil {
		s.mu.Unlock()
		return nil, fmt.Errorf("already playing %q; stop it first", s.sequence)
	}
	seq := s.cache[id]
	s.mu.Unlock()

	if seq == nil {
		var err error
		if seq, err = s.fetch(ctx, id); err != nil {
			return nil, err
		}
	}
	var move moveFunc
	if seq.HasPoses() {
		move = func(ctx context.Context, i int, extra map[string]any) error {
			return s.arm.MoveToPosition(ctx, seq.Poses[i], extra)
		}
	} else {
		current, err := s.arm.JointPositions(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("reading arm joints: %w", err)
		}
		if len(current) != len(seq.JointsDeg[0]) {
			return nil, fmt.Errorf("joint count mismatch: arm has %d, sequence %q has %d", len(current), id, len(seq.JointsDeg[0]))
		}
		inputs, err := s.toInputs(ctx, seq.JointsDeg)
		if err != nil {
			return nil, err
		}
		move = func(ctx context.Context, i int, extra map[string]any) error {
			return s.arm.MoveToJointPositions(ctx, inputs[i], extra)
		}
	}
	useGripper := s.player.gripper != nil && seq.HasGripper()
	if s.player.gripper != nil && !seq.HasGripper() {
		s.logger.Infow("sequence has no gripper data; playing arm only", "sequence_id", id)
	}

	s.mu.Lock()
	defer s.mu.Unlock()
	if s.cancel != nil {
		return nil, fmt.Errorf("already playing %q; stop it first", s.sequence)
	}
	s.cache[id] = seq
	s.sequence = id
	s.lastError = ""
	wctx, cancel := context.WithCancel(context.Background())
	s.cancel, s.done = cancel, make(chan struct{})
	go s.worker(wctx, seq, move)
	s.logger.Infow("playback started", "sequence_id", id, "frames", seq.Len(), "duration", seq.Duration())
	return map[string]any{
		"status": "playing", "sequence_id": id, "frame_count": seq.Len(),
		"duration_s": seq.Duration().Seconds(), "has_gripper": useGripper,
	}, nil
}

func (s *armPlayback) worker(ctx context.Context, seq *Sequence, move moveFunc) {
	err := s.player.run(ctx, seq, move)
	s.mu.Lock()
	defer s.mu.Unlock()
	switch {
	case err == nil:
		s.logger.Infow("playback complete", "sequence_id", seq.ID)
	case ctx.Err() != nil:
		s.logger.Infow("playback stopped", "sequence_id", seq.ID)
	default:
		s.lastError = err.Error()
		s.logger.Errorw("playback failed", "sequence_id", seq.ID, "error", err)
	}
	s.cancel = nil
	close(s.done)
}

func (s *armPlayback) stop() (map[string]any, error) {
	s.mu.Lock()
	cancel, done := s.cancel, s.done
	s.mu.Unlock()
	if cancel != nil {
		cancel()
		<-done
	}
	return map[string]any{"status": "stopped"}, nil
}

// Status reports playback state; also served by the "status" DoCommand.
func (s *armPlayback) Status(context.Context) (map[string]any, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	st := map[string]any{"state": "idle", "sequence_id": s.sequence, "last_error": s.lastError,
		"frame_index": int(s.player.frameIndex.Load())}
	if seq := s.cache[s.sequence]; seq != nil {
		st["frame_count"] = seq.Len()
	}
	if s.cancel != nil {
		st["state"] = "playing"
	}
	return st, nil
}

func (s *armPlayback) Close(context.Context) error {
	s.stop()
	if s.viam != nil {
		return s.viam.Close()
	}
	return nil
}
