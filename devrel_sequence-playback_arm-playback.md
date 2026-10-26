# Model devrel:sequence-playback:arm-playback

Replays a recorded sequence on an arm, and optionally a gripper, at the recorded speed.

## Configuration

```json
{
  "arm": "so101",
  "gripper": "so101-gripper",
  "dataset_id": "<dataset id>"
}
```

### Attributes

| Name | Type | Required | Default | Description |
|---|---|---|---|---|
| `arm` | string | Yes | | The arm to move. |
| `gripper` | string | No | | The gripper to move, using its `set_position` DoCommand. |
| `gripper_position_key` | string | No | `position_percentage` | The field in the recorded gripper `DoCommand` response that holds the position. Also used when calling `set_position`, except `position`: see below. |
| `source_arm` | string | No | same as `arm` | The name of the arm in the recording. Set this to the leader arm to replay its motion on the follower. |
| `source_gripper` | string | No | same as `gripper` | The name of the gripper in the recording. |
| `dataset_id` | string | No | | The dataset to list sequences from. Needed for the `list` command. |
| `arm_extra` | object | No | `{"wait": false, "streamed": true}` | Extra options sent with each arm move. The default works well for the SO-101 arm. Set to `{}` to send nothing. |

## What the recording needs to contain

- For the arm: `JointPositions` captured by the data manager.
- For the gripper: a `DoCommand` capture with `{"command": "get_position"}` as the input.
  A capture with the `{"get": true}` shorthand also works: set `gripper_position_key` to
  `position`, and playback writes with the matching `{"set": <position>}`.

## How playback works

1. The arm and gripper move to the first recorded position. This move runs at the arm's
   default speed, from wherever the arm is.
2. Each later frame is sent at the time it was recorded. The gripper follows the same clock
   and only moves when its recorded value changes.
3. When playback ends, stops, or fails, the arm is told to stop. Failures show up in
   `status` under `last_error`.

If the arm can't keep up, playback slows down rather than skipping frames.

## DoCommand

### list

Lists the sequences in `dataset_id`.

```json
{ "command": "list" }
```

Returns `sequences`, each with `id`, `tags`, `start`, and `duration_s`.

### play

Starts playing a sequence in the background and returns right away. Fails if something is
already playing or if the recording's joint count doesn't match the arm.

```json
{ "command": "play", "sequence_id": "<sequence id>" }
```

Returns `status`, `sequence_id`, `frame_count`, `duration_s`, and `has_gripper`.

### stop

Stops the current playback and halts the arm. Does nothing if idle.

```json
{ "command": "stop" }
```

### status

```json
{ "command": "status" }
```

Returns `state` (`idle` or `playing`), `sequence_id`, `frame_index`, `frame_count`, and
`last_error`. The same map is returned by the resource's `Status` method.
