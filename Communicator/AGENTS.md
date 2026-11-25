# RAshko Communicator agent handoff

This repository is the Raspberry-Pi-side runtime for the RAshko arm.

## Architecture boundary

- Do not add ROS, MoveIt, inverse kinematics, trajectory planning, model
  inference, or GUI code here. Those remain in the laptop-side `RAshko`
  repository.
- This project owns authenticated UDP reception, replay protection, watchdog
  behavior, GPIO17 OE, PCA9685 output, and Camera Module streaming.
- The wire-level joint order is exactly:
  `[base, shoulder, elbow, wrist, wrist_rotate, gripper]`.
- Acknowledged angles are commanded estimates. There is no physical joint
  position feedback.

## Safety invariants

- Authenticated packets are required unless insecure mode is explicitly
  selected for isolated commissioning.
- Startup and every exception path must leave OE HIGH (outputs disabled).
- E-stop is latched at startup and after watchdog expiry.
- Heartbeats never enable outputs.
- Resetting E-stop does not enable outputs; a subsequent fresh, valid motion
  command is required.
- Reject invalid commands. Never silently clamp them.
- Hardware mode requires an explicit servo calibration file.

## Validation

Run:

```bash
python -m unittest discover -s tests -v
```

Before the first powered movement, physically verify OE polarity, channel
mapping, joint direction, pulse widths, limits, startup pose, and the real power
cutoff.
