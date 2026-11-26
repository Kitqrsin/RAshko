# RAshko 6-DOF arm

RAshko is the Windows/Docker half of the robot-arm system. It owns the GUI,
inverse kinematics, MoveIt planning, vision, trajectory execution, and offline
simulation. The `Communicator/` project is the Raspberry Pi runtime;
it owns the PCA9685, receives authenticated six-angle commands, and streams the
Raspberry Pi Camera Module 2 to the laptop.

## Architecture

```text
GUI / MoveIt / IK
        |
        v
/joint_commands (six logical degrees)
        |
        +-- offline --> simulated servo_node --> /joint_states --> RViz
        |
        +-- hardware -> UDP bridge -> Raspberry Pi Communicator -> PCA9685

Pi Camera Module 2 -> Pi MJPEG :8080 -> camera_node -> /camera/image_raw
```

The canonical degree order is:

```text
[base, shoulder, elbow, wrist, wrist_rotate, gripper]
```

The base accepts `0..245` degrees. The other joints accept `0..180` degrees.
ROS and URDF code call the sixth joint `gripper_base`; the wire-level position
in the six-value array does not change.

## Feature projects

Standalone features live under `src/arm_controller/arm_controller/projects/`.
The ROS launch files in `src/arm_controller/launch/` remain the entry points.

- `caps/inspection/` holds live cap inspection and pick preview.
- `caps/cap_sequence_simulation/` holds the offline cap sequence demo.
- `caps/workcell/` holds cap geometry, perception, and fixture configuration
  shared by the inspection and simulation features.
- `camera_calibration/`, `face_tracking/`, `pick_and_place/`, and
  `bartender/` hold their respective implementations.

Shared kinematics, camera capture, servo mapping, and motion transport live at
the top level of the Python package. Configure examples in the repository's
`config/` folder.

## Windows setup

Requirements:

- Docker Desktop in Linux-container mode
- PowerShell
- VcXsrv for RViz and the Tk GUI

Install the display server once:

```powershell
winget install --id marha.VcXsrv --exact
```

Keep Windows Firewall enabled. The helper starts VcXsrv for Docker display
access; the firewall remains the network boundary.

From this directory:

```powershell
.\windows.ps1 Setup
.\windows.ps1 Start
```

`Setup` creates the ignored `.env.windows` file and a random shared secret.
`Start` reuses the existing image, starts an idle development container, and
runs `colcon build --symlink-install`. Compose builds the image only when it is
missing. It does not contact the Raspberry Pi or start any ROS nodes. Use
`Rebuild` explicitly after changing Dockerfile dependencies.

`RASHKO_GRIPPER_CLOSED_DEGREES` defaults to `0`, which locks the gripper at its
logical wide-open command in both the GUI and hardware UDP bridge. Leave it at
zero until the gripper's open pulse, horn position, and collision-free closed
endpoint have been physically measured. Then set it to that measured positive
logical close angle and recreate the Compose container.

## Offline simulation

Open two PowerShell windows after `Start` completes.

Backend and RViz:

```powershell
.\windows.ps1 Simulation
```

GUI:

```powershell
.\windows.ps1 SimulationGui
```

The backend starts the URDF/STL model, `servo_node`, `robot_state_publisher`,
MoveIt, and the trajectory executor. The GUI publishes commands but does not
fake joint feedback; feedback comes from the simulated servos.

To run the dedicated seven-cap sequence in one terminal instead, use:

```powershell
.\windows.ps1 CapSimulation
```

This starts its own offline backend, the `cap_sequence_simulation` sequencer
with `/workspace/config/cap_sequence.simulation.yaml`, and a dedicated RViz
PlanningScene view. Do not run it alongside `Simulation` or `Hardware`.
`CapSimulation` starts no camera or UDP bridge and cannot contact the Pi or
physical servos. The scene now uses the measured 205 mm width, 140 mm battery
body, 20 mm cap projection, 8.3 mm cap diameter, 30 mm pitch, and the 120 mm
gap from the battery-facing aluminum plate edge. Unmeasured depth/setback and
post geometry are visibly documented as simulation assumptions. Exact-URDF
preflight currently rejects a vertical-down pick at the measured cap height;
this is an intentional fail-closed result, not permission to try an angled
hardware grasp. The 120 mm plate gap is roughly 191 mm from the actual base
rotation axis before cap-row setback, so the current arrangement needs a
mechanical height/distance redesign before autonomous pickup.

The GUI draws a lightweight five-joint schematic. The launcher extracts the
joint transforms, limits, and tool-tip offset from the same URDF used by RViz.
The canvas and Cartesian IK use that compact chain in `base_link`. Live joint
feedback drives the schematic and the green tool-tip marker. Enable
**Set tool pitch** to specify orientation while dragging or moving to an XYZ
target. A pitch of `-90°` points down, `0°` is horizontal, and `+90°` points up.

There is also a headless end-to-end test:

```powershell
.\windows.ps1 Smoke
```

It checks rate-limited movement, emergency-stop latching, reset behavior, TF,
MoveIt's `/compute_ik` service, and a complete `FollowJointTrajectory` action.
The smoke test is disabled by default and is only started by the command above.

With the simulation backend running, validate the standalone URDF/MoveIt
Cartesian orientation path for up, horizontal, and down poses:

```powershell
.\windows.ps1 CartesianTest
```

The test is bounded to 30 seconds and fails if simulated joint feedback, exact
URDF forward kinematics, and the published `tool_tip` transform disagree.
CI also drives an actual Tk canvas drag and checks the simulated tool transform
and canvas tip in a downward-facing pose.

This is a kinematic/control simulation, not a force or torque simulator. It can
find software, topic, launch, URDF, and IK faults; it cannot prove the real arm's
directions, pulse widths, power integrity, backlash, or collision behavior.

### Known simulation limitations

- MoveIt 2.5.9 in the ROS 2 Humble image logs a class-loader error and exits
  with `-11` while the smoke-test launch is shutting down. This happens after
  the test reports `PASS`; normal IK, OMPL planning, and trajectory execution
  remain operational. Treat it as an unclean binary teardown, not a passed
  production shutdown test.
- MoveIt logs that no OctoMap 3D sensor plugin is configured. That is expected
  until a depth sensor is added; self-collision and URDF collision geometry are
  still available, but the simulation cannot perceive external obstacles.
- The camera is disabled in simulation mode. Hardware camera behavior is not
  covered by the offline test.

## Hardware mode

Prepare the Raspberry Pi using [`Communicator/README.md`](Communicator/README.md),
then edit `.env.windows` with the Pi's LAN address and copy the shared secret to
the Pi configuration. Motor traffic uses authenticated UDP port `5005`. Camera
traffic uses firewall-restricted HTTP MJPEG port `8080`; vision processing
remains on the laptop.

Before commissioning hardware, launch the Raspberry-only connection console by
double-clicking `raspberry-console.cmd`, or run:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\raspberry.ps1
```

Choose **Test every connection**. It checks the Docker engine and container,
authenticates an SSH session to both Pi services, checks the camera health
endpoint, and sends an authenticated UDP status request. It never resets
E-stop, enables outputs, or sends a joint target. Continue commissioning only
after the console reports `[READY]`.

The same console can open an interactive Pi Linux shell in the current or a new
PowerShell window. Inside SSH, `Ctrl+C` stops the current command and `exit` or
`Ctrl+D` returns to Windows. Its log actions follow only the controller and
camera services used by this project.

Backend:

```powershell
.\windows.ps1 Hardware
```

GUI in a second terminal:

```powershell
.\windows.ps1 Gui
```

`windows.ps1 EStop` and `ResetEStop` publish `true`/`false` on
`/emergency_stop` when `udp_servo_bridge` is running, keeping the bridge's
authenticated stop state synchronized. E-stop can also go directly over UDP
when the bridge is unavailable. Reset requires the running bridge. The Pi
accepts motion only from the session that reset it, starting with the next
valid joint command.

Do the first physical test unloaded, at low speed, with a current-limited supply
and a real power cutoff within reach.

In hardware mode, `udp_servo_bridge` is the sole `/joint_states` publisher. It
converts each authenticated Pi ACK into canonical ROS radians, so TF, MoveIt,
and the GUI follow the Pi controller's rate-limited applied-command estimate
instead of jumping immediately to the requested target. These are open-loop
estimates—the hobby servos still provide no measured shaft position.

## Main ROS interfaces

- `/joint_commands`: `std_msgs/Float32MultiArray`, exactly six logical degrees
- `/servo_speed`: `std_msgs/Float32`, `1..100` percent
- `/emergency_stop`: `std_msgs/Bool`
- `/joint_states`: simulated state offline; Pi applied-command estimate on hardware
- `/raspberry/status`: authenticated Pi acknowledgements in hardware mode
- `/camera/image_raw`: Pi Camera Module 2 frames republished on the laptop
- `/camera/preview/compressed`: newest Pi JPEG for the companion GUI, at the
  source rate. The camera node's frame-rate setting still applies to raw RGB
  and camera info. Set the Pi camera service's `RASHKO_CAMERA_FPS` to change the
  upstream frame rate.
- `/camera/camera_info`: camera metadata; calibrate before metric vision work
- `/cap_workcell/status`: leased seven-slot fixture observation; never a motor command
- `/cap_workcell/pick_preview`: unverified high/vertical/high Cartesian sketch JSON
- `/cap_workcell/pick_preview_markers`: battery envelope and safety-sketch RViz markers
- `/cap_sequence_simulation/status`: dedicated offline sequence state
- `/cap_sequence_simulation/markers`: dedicated offline cap/fixture visualization
- `/simulation/servo_status`: JSON status from the simulated servo controller
- `/simulation/test_result`: automated smoke-test result

## Useful commands

```text
.\windows.ps1 Discover       auto-discover Pi on Ethernet and update .env.windows
.\windows.ps1 SetPi          set RASHKO_PI_HOST or Ethernet adapter from CLI
.\windows.ps1 Build          build the Docker image
.\windows.ps1 Rebuild        rebuild image, recreate container, and build ROS
.\windows.ps1 Start          start container and build the ROS workspace
.\windows.ps1 Restart        recreate container without rebuilding the image
.\windows.ps1 Display        start VcXsrv display :0 if needed
.\windows.ps1 Simulation     offline backend plus RViz
.\windows.ps1 SimulationGui  GUI companion for offline simulation
.\windows.ps1 Gui -NoCamera  manual GUI with the camera pipeline disabled
.\windows.ps1 Stats          snapshot of Docker, ROS processes, and Windows display memory
.\windows.ps1 CapSimulation  dedicated offline seven-cap sequence and RViz
.\windows.ps1 Smoke          headless servo + safety + IK test
.\windows.ps1 CartesianTest  bounded URDF/MoveIt versus RViz/TF orientation test
.\windows.ps1 CameraCalibrate camera-only 8x6/20mm intrinsic calibration GUI
.\windows.ps1 CameraSave      validate/export the saved intrinsic YAML
.\windows.ps1 CapInspect     perception-only seven-cap inspection and RViz preview
.\windows.ps1 CapStatus      print one current camera/fixture inspection report
.\windows.ps1 CapFixture     right-handed exact-size SVG from measured fixture data
.\windows.ps1 CapTest        focused camera, geometry, and trajectory safety tests
.\windows.ps1 Hardware       real Pi transport backend
.\windows.ps1 Gui            GUI companion for hardware mode
.\windows.ps1 EStop          send an authenticated Pi emergency stop
.\windows.ps1 ResetEStop     reset the Pi latch without enabling outputs
.\windows.ps1 Logs           show container logs
.\windows.ps1 Shell          open a ROS-ready shell
.\windows.ps1 Stop           stop the container
.\docker-cleanup.ps1         report aggregate Docker usage without deleting data
.\docker-cleanup.ps1 -Apply  prune old dangling images and dangling build cache
.\raspberry-console.cmd Check         verify every Windows-to-Pi connection safely
.\raspberry-console.cmd Discover      auto-find Pi on Ethernet and save IP
.\raspberry-console.cmd Ethernet      switch Windows Ethernet DHCP <-> Direct Static IP
.\raspberry-console.cmd SyncAllowlist sync laptop Ethernet subnet to Pi allowlist
.\raspberry-console.cmd Shell         open an interactive Raspberry Pi Linux shell
```

The camera cap prototype is intentionally preview-only: it has no action client
and always reports hardware motion unauthorized. Raw color pixel centering does
not require camera calibration; metric image geometry still does. Real
autonomous execution remains blocked until measured fixture/battery/gripper geometry,
dummy-battery trials, and a full MoveIt PlanningScene are commissioned. Its
Cartesian sketch is not robot collision checking and must never be connected
directly to a motion topic. A production autonomous stage also needs a single
leased command arbiter before the UDP bridge and correctly timed/retimed
trajectory execution; publisher counting and the current sequential checkpoint
executor do not provide those guarantees.

See [instructions.md](instructions.md) for the operator startup, Pi-shell, and
pre-motor checklist; [WINDOWS_RASPBERRY_SETUP.md](WINDOWS_RASPBERRY_SETUP.md)
contains commissioning details, and [AGENTS.md](AGENTS.md) and
[GEMINI.md](GEMINI.md) are the maintainer/LLM handoff contracts.
