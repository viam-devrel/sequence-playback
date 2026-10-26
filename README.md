# sequence-playback

A Viam module that replays recorded arm demonstrations. Record a demonstration with
sequence data capture, then play it back on the real arm (and gripper) at the speed it was
recorded.

Useful for VLA demos and learning content: you can show the exact motion a policy is going to
learn from.

## Models

- [`devrel:sequence-playback:arm-playback`](devrel_sequence-playback_arm-playback.md): a
  generic service. Loads a sequence from Viam Cloud by ID and replays it. Controlled with
  `DoCommand`.

## Notes

- This module does not record. Use the data manager's sequence capture to record
  `JointPositions` on the arm and `get_position` or `{"get": true}` (via `DoCommand`) on the
  gripper.
- The module reads from Viam Cloud using the API key that viam-server gives modules. If you
  run the binary by hand without those environment variables, `play` and `list` will fail
  with a clear error.
- The first move of every playback goes to the recording's start pose at the arm driver's
  default speed. Put the arm near that pose before testing on hardware.

## Development

```bash
make test
make
```
