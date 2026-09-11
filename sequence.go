package sequenceplayback

import (
	"context"
	"encoding/json"
	"fmt"
	"sort"
	"time"

	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/app"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/spatialmath"
	"google.golang.org/protobuf/encoding/protojson"
)

// Sequence is one captured demonstration, ready to replay. Exactly one of JointsDeg or
// Poses is populated, depending on the captured arm method. Joint values are kept in the
// captured unit (degrees); conversion to the arm's inputs happens at play time.
type Sequence struct {
	ID               string
	Offsets          []time.Duration    // per frame, from the first arm row
	JointsDeg        [][]float64        // JointPositions captures
	Poses            []spatialmath.Pose // EndPosition captures
	GripperOffsets   []time.Duration    // same origin as Offsets; may be negative
	GripperPositions []float64
}

func (s *Sequence) HasGripper() bool        { return len(s.GripperPositions) > 0 }
func (s *Sequence) HasPoses() bool          { return len(s.Poses) > 0 }
func (s *Sequence) Len() int                { return len(s.Offsets) }
func (s *Sequence) Duration() time.Duration { return s.Offsets[len(s.Offsets)-1] }

func sortRows(rows []*app.ExportTabularDataResponse) {
	sort.SliceStable(rows, func(i, j int) bool { return rows[i].TimeCaptured.Before(rows[j].TimeCaptured) })
}

func jointValues(payload map[string]any) ([]float64, error) {
	pos, _ := payload["positions"].(map[string]any)
	vals, ok := pos["values"].([]any)
	if !ok {
		return nil, fmt.Errorf("payload has no positions.values list")
	}
	out := make([]float64, len(vals))
	for i, v := range vals {
		f, ok := v.(float64)
		if !ok {
			return nil, fmt.Errorf("positions.values[%d] is %T, not a number", i, v)
		}
		out[i] = f
	}
	return out, nil
}

// poseValue decodes an EndPosition payload. protojson accepts both the exported snake_case
// keys (o_x) and lowerCamel (oX).
func poseValue(payload map[string]any) (spatialmath.Pose, error) {
	raw, ok := payload["pose"]
	if !ok {
		return nil, fmt.Errorf("payload has no pose")
	}
	b, err := json.Marshal(raw)
	if err != nil {
		return nil, err
	}
	var p commonpb.Pose
	if err := (protojson.UnmarshalOptions{DiscardUnknown: true}).Unmarshal(b, &p); err != nil {
		return nil, fmt.Errorf("decoding pose: %w", err)
	}
	return spatialmath.NewPoseFromProtobuf(&p), nil
}

func gripperValue(payload map[string]any, key string) (float64, error) {
	out, _ := payload["docommand_output"].(map[string]any)
	v, ok := out[key]
	if !ok {
		return 0, fmt.Errorf("docommand_output missing key %q", key)
	}
	f, ok := v.(float64)
	if !ok {
		return 0, fmt.Errorf("docommand_output[%q] is %T, not a number", key, v)
	}
	return f, nil
}

// buildSequence turns exported arm rows (JointPositions or EndPosition, per method) and
// optional gripper DoCommand rows into a Sequence. Rows may arrive in any order.
func buildSequence(id, method string, armRows, gripperRows []*app.ExportTabularDataResponse, gripperKey string) (*Sequence, error) {
	if len(armRows) == 0 {
		return nil, fmt.Errorf("sequence %q has no %s rows", id, method)
	}
	sortRows(armRows)
	sortRows(gripperRows)
	origin := armRows[0].TimeCaptured

	seq := &Sequence{ID: id}
	for i, r := range armRows {
		if method == "EndPosition" {
			p, err := poseValue(r.Payload)
			if err != nil {
				return nil, fmt.Errorf("sequence %q arm row %d: %w", id, i, err)
			}
			seq.Poses = append(seq.Poses, p)
		} else {
			j, err := jointValues(r.Payload)
			if err != nil {
				return nil, fmt.Errorf("sequence %q arm row %d: %w", id, i, err)
			}
			if i > 0 && len(j) != len(seq.JointsDeg[0]) {
				return nil, fmt.Errorf("sequence %q arm row %d has %d joints, expected %d (joint count changed mid-sequence)", id, i, len(j), len(seq.JointsDeg[0]))
			}
			seq.JointsDeg = append(seq.JointsDeg, j)
		}
		seq.Offsets = append(seq.Offsets, r.TimeCaptured.Sub(origin))
	}
	for i, r := range gripperRows {
		v, err := gripperValue(r.Payload, gripperKey)
		if err != nil {
			return nil, fmt.Errorf("sequence %q gripper row %d: %w", id, i, err)
		}
		seq.GripperPositions = append(seq.GripperPositions, v)
		seq.GripperOffsets = append(seq.GripperOffsets, r.TimeCaptured.Sub(origin))
	}
	return seq, nil
}

// fetchSequence pulls one captured sequence from Viam Cloud.
func fetchSequence(ctx context.Context, dc *app.DataClient, id, method, sourceArm, sourceGripper, gripperKey string) (*Sequence, error) {
	meta, err := dc.GetSequence(ctx, id)
	if err != nil {
		return nil, fmt.Errorf("getting sequence %q: %w", id, err)
	}
	interval := app.CaptureInterval{Start: meta.StartTime, End: meta.EndTime}
	armRows, err := dc.ExportTabularData(ctx, meta.PartID, sourceArm, arm.API.String(), method, interval, nil)
	if err != nil {
		return nil, fmt.Errorf("exporting %s %s for sequence %q: %w", sourceArm, method, id, err)
	}
	var gripperRows []*app.ExportTabularDataResponse
	if sourceGripper != "" {
		gripperRows, err = dc.ExportTabularData(ctx, meta.PartID, sourceGripper, gripper.API.String(), "DoCommand", interval, nil)
		if err != nil {
			return nil, fmt.Errorf("exporting %s DoCommand for sequence %q: %w", sourceGripper, id, err)
		}
	}
	return buildSequence(id, method, armRows, gripperRows, gripperKey)
}

// listSequences summarizes every sequence in a dataset.
func listSequences(ctx context.Context, dc *app.DataClient, datasetID string) ([]map[string]any, error) {
	out := []map[string]any{}
	page, err := dc.SequencesByDatasetID(ctx, datasetID, 100)
	// NextPage hands back an empty page (not nil) once the token runs out.
	for ; err == nil && len(page.Sequences) > 0; page, err = page.NextPage(ctx) {
		for _, s := range page.Sequences {
			out = append(out, map[string]any{
				"id": s.ID, "tags": s.SequenceTags,
				"start": s.StartTime.UTC().Format(time.RFC3339), "duration_s": s.EndTime.Sub(s.StartTime).Seconds(),
			})
		}
	}
	if err != nil {
		return nil, fmt.Errorf("listing sequences in dataset %q: %w", datasetID, err)
	}
	return out, nil
}
