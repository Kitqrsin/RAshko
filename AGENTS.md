# RAshko agent handoff

This file is for coding agents, not end-user documentation.
See also [`GEMINI.md`](GEMINI.md) for the Ethernet auto-discovery, live `.env.windows`
endpoint reload, GUI Pi-target bar, and CIDR allowlist reference log.

## Supported architecture

- `RAshko` is Windows/Docker-side only: ROS 2 Humble, GUI, IK, MoveIt, vision,
  trajectory execution, and UDP transport.
- The `Communicator/` is Raspberry-Pi-side only: authenticated UDP,
  safety/watchdog logic, GPIO17 OE, PCA9685 output, and Camera Module 2 MJPEG
  streaming. It must not depend on ROS.
- Never add direct GPIO, I2C, Adafruit, or PCA9685 ownership back into `RAshko`.
- Hardware backend: `ros2 launch arm_controller arm.launch.py`.
- Offline backend: `ros2 launch arm_controller simulation.launch.py`.
- Simulation GUI companion: `ros2 launch arm_controller view_with_gui.launch.py`.

## Wire and ROS contracts

- `/joint_commands`: `std_msgs/Float32MultiArray`, exactly six logical degrees in
  this order: `[base, shoulder, elbow, wrist, wrist_rotate, gripper]`.
- `/servo_speed`: `std_msgs/Float32`, 1..100 percent.
- `/emergency_stop`: `std_msgs/Bool`; true is latched and repeatedly transmitted.
- `/raspberry/status`: compact JSON in `std_msgs/String`; angles are commanded
  estimates because the hardware has no position feedback.
- In hardware mode `udp_servo_bridge` is the sole `/joint_states` authority and
  publishes those authenticated Pi estimates. The GUI must never publish a
  desired target as measured state.
- Do not silently clamp invalid commands in the network bridge. Reject and log.
- `arm_controller/joint_contract.py` is the canonical radians/degrees mapping.
  Do not reimplement joint inversion or limits in another node.

## Safety invariants

- Authenticated packets are the default; insecure mode must remain explicit.
- The Pi starts with OE HIGH, keeps E-stop latched until reset, rejects stale
  sequences, and disables outputs when the heartbeat watchdog expires.
- A first hardware motion test requires physical verification of startup pose,
  pulse widths, joint directions, limits, OE polarity, and a real power cutoff.
- UDP is allowed for targets because heartbeat + sequence + ACK + watchdog cover
  the failure modes. Do not weaken those mechanisms to reduce code size.

## Validation

- Communicator: `python -m unittest discover -s tests -v`.
- Compose: `docker compose --env-file .env.windows -f docker-compose.windows.yml config`.
- ROS workspace: build in the Windows Compose container with `colcon build --symlink-install`.
- Offline end-to-end: `ros2 launch arm_controller simulation.launch.py rviz:=false smoke_test:=true`.
- Cartesian orientation: run `./windows.ps1 CartesianTest` while the simulation
  backend is running. It must pass up, horizontal, and down tool directions.
- The smoke test must cover servo motion, E-stop/reset, TF, MoveIt IK, and a real
  `FollowJointTrajectory` action. A pass that checks topics alone is insufficient.

## Validated simulation state (2026-08-04)

- Docker image build and the complete colcon workspace build pass on Windows.
- Focused `arm_controller` Python tests pass 21/21, MoveIt configuration checks pass
  4/4, and the sibling Communicator tests pass 30/30.
- Headless smoke test reaches `PASS`, including OMPL, IK, and trajectory execution.
- RViz and the Tk GUI run through VcXsrv; `windows.ps1` owns starting display `:0`.
- Live command `[105, 80, 110, 70, 100, 95]` reached the exact simulated target
  at 25 percent speed and both UI processes remained healthy.
- The launch extracts a compact joint chain from the RViz URDF for GUI
  Cartesian IK and the Tk schematic. Keep the GUI free of visual meshes and
  draw only the five active motor joints and a separate tool-tip segment.
- GUI point following leaves tool pitch free unless the user selects a pitch.
  Never restore the nearby-point fallback: it made the gripper stop separately from the target.
  GUI and RViz markers use `base_link`; never restore the removed hard-coded
  2.23697-radian marker rotation.
- The calibrated logical home is `[90, 88, 90, 96, 98, 0]` degrees. The final
  value is the gripper on PCA9685 channel 6. The URDF limits are derived from
  the logical servo ranges around this per-joint home.
- The standalone exact-URDF Cartesian residuals are up 0.4 mm/1.7 degrees,
  horizontal 3.1 mm/3.2 degrees, and down 1.4 mm/0.5 degrees. The CI Tk suite
  additionally tests canvas drag through simulated servos and published TF.
- `windows.ps1` deliberately refuses to start duplicate backend or GUI launch
  processes. Multiple `/compute_ik` servers caused nondeterministic CLI hangs.
- ROS 2 Humble's packaged MoveIt 2.5.9 currently segfaults only while the
  smoke-test process tears down, after `PASS`. Do not hide this exit status; test
  a newer MoveIt package or source build before claiming clean production shutdown.
- The OctoMap "no 3D sensor plugin" message is expected until a depth sensor is
  configured. It does not invalidate the current kinematic simulation.
- The seven-cap gripper-camera prototype is perception and visualization only.
  Its leased high/vertical/high output is an unverified Cartesian sketch, never
  motion authorization. Keep `hardware_motion_authorized` false until a full
  MoveIt PlanningScene, actual tool/payload geometry, and dummy-battery trials
  are integrated.
- Publisher counting is not a hardware command arbiter. Before autonomous
  execution, put all manual/action sources behind one leased mux upstream of
  the UDP bridge and replace or safely retime the sequential checkpoint
  executor. Never run action motion beside the direct-command GUI.

## Current cleanup direction

The obsolete direct-hardware servo node, calibration script, stop-servos script,
and stray hardware test were removed. Do not restore GPIO/Adafruit ownership to
this project. The Flask UI remains excluded from package entry points; do not
remove recent vision/face-tracking/pick-place work merely because it is not in the
default launch.
