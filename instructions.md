# RAshko Windows and Raspberry Pi instructions

These commands cover only the current RAshko Windows/Docker-to-Raspberry-Pi
setup. Keep the servo power supply disconnected until the pre-motor gate at the
end of this document is complete.

## Verified connection state — 2026-08-04

- Docker container `rashko_ros2_windows` is running.
- The authenticated UDP heartbeat returns `code: OK`.
- The Pi controller reports `HARDWARE_READY`, `ESTOP_LATCHED`, and
  `WATCHDOG_TRIPPED`; it does not report `OUTPUTS_ENABLED`.
- The Pi Camera Module 2 reports `status: ok`, `picamera2`, and a ready
  `640x480 @ 15 FPS` frame stream.
- The latest commanded estimate was `[90, 88, 90, 96, 98, 0]`. This is not
  measured position feedback.
- The checked-in Pi configuration, ROS joint contract, GUI, IK paths, and URDF
  limits use that same calibrated home.
- Validation passes: 30 Pi-controller tests, 37 arm-controller tests, four
  MoveIt configuration checks, the full offline smoke test, and the Cartesian
  up/horizontal/down test.
- Cartesian residuals are `0.4 mm / 1.7°` up, `3.1 mm / 3.2°` horizontal, and
  `1.4 mm / 0.5°` down; TF and exact URDF FK agree at `0.00 mm / 0.00°`.

This confirms communication and safe disabled state only. It does not approve
motor power or prove the physical pose, wiring, directions, pulse widths, or
mechanical limits.

ROS 2 Humble's packaged MoveIt 2.5.9 still exits with a segmentation fault
during shutdown, after the tests print `PASS`. Runtime planning and execution
pass, but this is not a clean production shutdown and must not be hidden.

## 1. Start Docker Desktop

Open PowerShell and run:

```powershell
cd C:\Users\User\Desktop\RAshko
Start-Process "$env:ProgramFiles\Docker\Docker\Docker Desktop.exe"
```

Wait for Docker Desktop to report that its engine is running. Then verify the
Linux engine:

```powershell
docker context use desktop-linux
docker version
```

Do not continue unless `docker version` shows both a **Client** section and a
**Server** section whose OS/architecture is Linux.

## 2. Start the RAshko Docker container

Direct `.ps1` execution is blocked by the current Windows policy, so use a
one-process bypass. This does not change the machine-wide execution policy:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Start
```

Confirm the container:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Status
```

Expected: `rashko_ros2_windows` is `Up`.

### Open the Docker Linux/ROS shell

From the RAshko directory in any PowerShell window:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Shell
```

You are then inside the container at a Linux `bash` prompt. Load the workspace
before running ROS commands:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

Type `exit` or press `Ctrl+D` to return to Windows. To keep both Linux shells
open, run the Docker command in one PowerShell window and the Pi-shell command
from section 5 in a second PowerShell window.

## 3. Dedicated Windows command runbook

Run every command in this section from the project directory:

```powershell
cd C:\Users\User\Desktop\RAshko
```

Every `windows.ps1` example deliberately uses the one-process execution-policy
bypass. It does not change the machine-wide PowerShell policy.

### 3.1 One-time setup and normal container startup

Run `Setup` only if `.env.windows` does not exist. It creates the environment
file and shared secret and will not overwrite an existing file:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Setup
```

After checking `.env.windows`, start the container from the existing image,
build the ROS workspace, and check the Compose state. If the image does not
exist yet, Compose builds it once:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Start
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Status
```

`Start` returns after container startup and the workspace build finish.
`Status` reports only the Docker Compose state; it does not prove that a ROS
backend, Raspberry Pi, or servo output is active.

### 3.2 Offline GUI manipulation — no Raspberry Pi or servo movement

Keep these two long-running commands open in separate PowerShell terminals.

Terminal A — simulated servos, `/joint_states`, MoveIt, trajectory execution,
robot-state publisher, and RViz:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Simulation
```

Terminal B — companion GUI with the camera disabled:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 SimulationGui
```

Leave both terminals open while manipulating the simulated arm. In the GUI:

1. enable **Real-time Mode** to have Cartesian or joint sliders update the
   simulated arm immediately; or
2. press **Start** and drag the orange target for direct auto-follow;
3. press **Stop** before changing workflows.

Manual joint input, **Home**, other presets, and **Send to Robot** now cancel
pending mouse IK, delayed Cartesian updates, and Cartesian interpolation.
Uncommitted XYZ entry text is discarded when switching to joint control, so
leaving an entry cannot restart an old IK move. With **Real-time Mode** enabled,
Home/presets send once; with it disabled, they only stage the pose until you
press **Send to Robot**. Turning Real-time off cancels its queued auto-updates;
the separate Cartesian **Start/Stop** button controls mouse Auto-Follow.

These controls do not arbitrate external MoveIt/action commands or the legacy
face/handshake/grab modes. Leave those modes off for manual/Cartesian testing.
The existing geometric solver and calibrated home `[90, 88, 90, 96, 98, 0]`
have not been reversed or replaced. GUI direction tests cannot establish the
physical motor directions; a mismatch between hardware and the model still
requires checking the deployed Pi calibration with motor power disconnected.

`SimulationGui` by itself has no simulator and no `/joint_states` source. Its
embedded arm now shows a local command preview, but that preview is not measured
or simulated motion. Run both commands above when testing dependencies,
constraints, trajectories, or feedback.

Optional terminal C — run the bounded Cartesian regression while terminal A is
still running:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CartesianTest
```

When finished, press `Ctrl+C` in terminal B and then terminal A. This stops the
GUI and offline backend without removing the Docker container.

#### Dedicated seven-cap sequence simulation

Run this as a separate offline workflow when no `Simulation`, `Hardware`, or
MoveIt backend is active:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CapSimulation
```

`CapSimulation` starts the offline backend, waits three seconds, then starts
`cap_sequence_simulation` with
`/workspace/config/cap_sequence.simulation.yaml` and its dedicated RViz view.
For each source it first plans to a deliberately offset high pre-grasp pose,
renders all still-present red caps into a synthetic eye-in-hand camera frame,
detects the red contours, and applies bounded XY corrections of at most 3 mm.
It requires three centered frames before descending. Descent and retreat are
rejected if they contain more than 0.5 mm of lateral travel.
The PowerShell action refuses a second ROS backend. This launch explicitly has
no camera and no UDP bridge, sends no Pi traffic, and cannot move hardware.
Inspect `/cap_sequence_simulation/status` and the
`/cap_sequence_simulation/markers` RViz display while it runs. Press `Ctrl+C`
to stop the complete offline workflow. A successful sequence prints
`PASS: all seven synthetic caps were collision-planned and placed`.

### 3.3 Hardware GUI manipulation

Complete the mandatory safety and commissioning gates in sections 8 through 11
before allowing powered movement. Start from a latched E-stop and verify the Pi
connection:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Ping
```

`EStop` and `ResetEStop` first check whether `udp_servo_bridge` is running. If
it is, they publish `true` or `false` on `/emergency_stop` so the bridge owns the
authenticated Pi exchange and keeps its latch state synchronized. With no
bridge process, they fall back to a direct authenticated UDP E-stop/reset
request. This routing does not weaken the Pi latch or watchdog.

Keep these two long-running commands open in separate terminals.

Terminal A — hardware backend, authenticated UDP bridge, MoveIt, trajectory
execution, and robot-state publisher:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Hardware
```

Terminal B — companion GUI and Raspberry Pi camera client:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Gui
```

`Hardware` and `Gui` must remain running while the physical arm is controlled.
The GUI is not a hardware backend by itself. Only after the physical checklist
passes, clear the latch from a third terminal:

The current GUI is a direct manual command source. Do not send MoveIt
`FollowJointTrajectory` goals while it or any legacy `/joint_commands`
publisher is running. The executor rejects a graph it can already see as
non-exclusive, but ROS publisher counting is not a command lease and cannot
close the graph-discovery race. Autonomous hardware operation requires a
single command arbiter/mux before the UDP bridge; that is not implemented.

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 ResetEStop
```

`ResetEStop` only requests that the latch be cleared. With the bridge active it
does so through `/emergency_stop`; otherwise it uses direct authenticated UDP.
It does not enable outputs by itself, but the next valid GUI joint command can
enable outputs and cause movement. Keep a free terminal ready for the
authenticated stop command:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
```

### 3.4 Turning the servo PSU off and on again

Before switching the servo PSU off, send E-stop and verify the acknowledgement:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Ping
```

You do **not** need to run `Stop`, `Start`, or restart Docker for every PSU power
cycle if the `Hardware` and `Gui` terminals are still healthy. The Pi keeps the
E-stop latched. After restoring PSU power, keep the workspace clear, verify the
safe physical pose and status, and run `ResetEStop` only when another movement
session is intentionally ready. If either long-running terminal was closed, run
the `Hardware` and `Gui` commands from section 3.3 again.

### 3.5 Normal hardware shutdown

Send E-stop, verify `ESTOP_LATCHED` and the absence of `OUTPUTS_ENABLED`, then
disconnect servo power:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Ping
```

Press `Ctrl+C` in the GUI terminal and then the hardware terminal. Stop Docker
only if it is no longer needed:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Stop
```

`Stop` does not send E-stop and does not stop the Raspberry Pi systemd services.

### 3.6 Monitoring, maintenance, and standalone diagnostics

Follow Compose logs; press `Ctrl+C` to stop following:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Logs
```

The live ROS output from `Hardware`, `Gui`, `Simulation`, or `SimulationGui`
remains in the terminal that launched it and may not appear in Compose logs.

Open an interactive container shell; type `exit` to close it:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Shell
```

Start VcXsrv manually if needed. GUI actions already do this automatically:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Display
```

Build only the Docker image. This may require internet access when Docker must
refresh or download the `ros:humble` base image:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Build
```

Restart the container and rebuild the ROS workspace. E-stop first if hardware
has been used. Normal restart reuses the existing image and works without
contacting Docker Hub:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Restart
```

Explicitly rebuild the Docker image only after changing the Dockerfile or its
system/Python dependencies:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Rebuild
```

Inspect Docker disk usage without deleting anything:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\docker-cleanup.ps1
```

Delete only dangling images and dangling BuildKit cache older than seven days:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\docker-cleanup.ps1 -Apply -OlderThanDays 7
```

The cleanup never deletes named images, containers, or volumes. Both dangling
images and dangling BuildKit cache are global to Docker Desktop, so qualifying
data from other projects may also be removed. Report mode shows aggregate disk
usage, not an exact list of prune candidates. A later image rebuild may need to
download dependencies again. Do not replace this with
`docker system prune --all --volumes`.

`Start`, `Restart`, `Build`, `Rebuild`, and applied cleanup refuse to run while
RAshko ROS processes are active. E-stop the arm, turn the servo PSU off, and
stop Hardware/GUI/Simulation terminals before any container lifecycle action.
`Restart` requires the named RAshko image to exist locally because it uses
`--no-build`; use `Rebuild` with working internet only when an image rebuild is
actually required.

Run the standalone offline smoke test only when no ROS backend is active:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Smoke
```

Start standalone MoveIt only for diagnostics when no `Hardware`, `Simulation`,
or other backend is active. It is long-running and is not part of either normal
GUI startup sequence:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 MoveIt
```

### 3.7 Seven-cap battery prototype — inspection and Cartesian safety sketch

This first version deliberately **cannot move the arm**. It calibrates and
checks the gripper camera, assigns stable IDs to the seven cap slots, and draws
the measured battery envelope plus an **unverified Cartesian safety sketch** in
RViz. It is not a robot collision preview and proves neither IK feasibility nor
arm-link clearance.
`/cap_workcell/status` always reports `hardware_motion_authorized:false`, even
when vision is stable. Do not connect this preview topic to `/joint_commands`.

The fixture uses an explicit right-handed ROS frame. Looking at the printed
face, its origin is the lower-left corner, +X points right, +Y points toward the
page top, and +Z points out of the page. The SVG generator reverses page-pixel Y
when printing; never enter marker or cap coordinates using a page-Y-down image
convention. Measure `fixture_in_base.yaw_deg` from base +X to printed fixture +X.

The current reduced workflow does not require ArUco tags. It requires a rigid,
keyed battery jig so the battery can only return in one position and direction.
At the stationary observation pose, the cap row must appear left-to-right. The
seven collinear caps validate row order, spacing, and repeatability only; they
cannot determine a full 2D plane pose. The close-up camera then corrects only
XY at the safe pre-grasp height; Z is a separately measured vertical move.

Create the private measured configuration; it is ignored by Git:

```powershell
Copy-Item .\config\cap_workcell.example.yaml .\config\cap_workcell.yaml
notepad .\config\cap_workcell.yaml
```

`CapInspect` automatically prefers `config/cap_workcell.yaml` when that local
file exists. Before running the optional legacy `CapFixture`, set the container
to the measured copy. Edit
`.env.windows` and change this line, then restart Docker/ROS so the environment
inside the existing container actually changes:

```text
RASHKO_CAP_WORKCELL_CONFIG=/workspace/config/cap_workcell.yaml
```

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Restart
```

The current local profile records the measurements supplied so far: 205 mm
battery width, 140 mm battery body, 20 mm cap projection, 8.3 mm cap diameter,
30 mm cap-centre pitch, and a 120 mm gap from the battery-facing edge of the
second aluminum base plate (not the motor axis). The CAD-derived plate edge is
`base_link X=-43.047 mm`; with the motor on the opposite side, the simulated
battery near face is therefore `X=-163.047 mm`. If the battery and arm share
one supporting surface, the CAD plate bottom puts the battery body top at
`base_link Z=124.065 mm` and cap tops at `Z=144.065 mm`.

Still measure the battery front-to-back depth, distance from its near face to
the red-cap row, dark-post sizes/positions, common-support-surface assumption,
fixture pose
in `base_link`, gripper outer width, cap radius, safe transit height, grasp
height, terminals/lips, seven destination poses, and the exact six-degree
stationary camera pose under `observation.joint_positions_deg`. Leave all four geometry
attestations `false` until those checks have physically passed. In particular:

- every `protected_regions` entry needs a measured fixture-plane circle and
  `top_z_fixture_m`; `safe_transit_z_base_m` must clear the highest battery or
  protected-region top plus `minimum_vertical_clearance_m` and the measured
  `tool_payload_below_tcp_m` (gripper/held-cap extent below the transit TCP);
- the full gripper radius, pose uncertainty, collision padding, and permitted
  vision error must fit between each cap and every battery side;
- destination slots must be clear of the battery and separated far enough that
  the gripper cannot hit previously placed caps; the current schema does not
  model a table/tray volume, so every destination support, wall, and lower
  travel bound must be represented later in the mandatory MoveIt PlanningScene;
- `close_degrees` and `RASHKO_GRIPPER_CLOSED_DEGREES` stay disabled until the
  gripper endpoint has been physically calibrated.

`CapFixture` remains available only for the optional legacy ArUco workflow; do
not print or mount it for `camera.localization_mode: fixed_cap_row`.

Intrinsic camera calibration is optional for the current raw-pixel,
eye-in-hand centering preview. `CapInspect` can draw and centre red blobs without
it, because no pixel is converted directly into a metric cap coordinate. It is
still required before using any metric image measurement. For that optional
metric workflow, print
`config/camera_calibration_board_8x6_20mm.svg` at **100% / Actual Size** and
physically verify its checkerboard is exactly 180 x 140 mm (20.000 mm per
square). Do not use a print whose measured scale is wrong. With no ROS backend
or other camera node running, start the camera-only calibration GUI:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CameraCalibrate
```

Move and tilt the checkerboard through the image so the GUI fills its X, Y,
size, and skew coverage bars. Keep the camera mount and focus exactly as they
will be used on the gripper. After the GUI reports good coverage, press
**Calibrate**, then **Save**, and stop it with `Ctrl+C`. Export only the saved
standard ROS YAML through the strict validator:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CameraSave
```

`CameraSave` refuses to overwrite an existing calibration and writes
`config/camera_intrinsics.yaml` atomically only after its resolution, pinhole
matrix, distortion model, and coefficient count validate. `CapInspect`
automatically uses this file when present, but otherwise continues with a color
overlay labelled `PIXEL PREVIEW ONLY`. To recalibrate, archive the old file
first; never silently replace a known calibration.

Keep the measured workcell selection in `.env.windows`:

```text
# Keep RASHKO_CAP_WORKCELL_CONFIG=/workspace/config/cap_workcell.yaml
RASHKO_CAP_CAMERA_WIDTH=640
RASHKO_CAP_CAMERA_HEIGHT=480
```

Restart the container after changing `.env.windows`. Run the focused prototype
tests:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Restart
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CapTest
```

For a no-motor camera trial, run:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CapInspect
```

`CapInspect` starts the real camera, fail-closed detector, safety-sketch node,
and RViz. In `fixed_cap_row` mode it does not look for ArUco tags. With all
seven caps visible, it publishes a `fixed_cap_row_candidate` containing ordered
slot pixels, row direction, straightness error, and minimum neighbor spacing.
If the close view contains fewer caps, the color RViz image still circles and
numbers every detected red blob and reports `N/7`; that partial view is useful
for local centering but cannot validate the whole row. It remains
diagnostic-only. Read one report from another terminal:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 CapStatus
```

Record `fixed_cap_row_candidate.slot_pixels`, remove and replace the battery in
its keyed stops at least 20 times, and require all seven centroids to return
within the configured 1 mm stability limit. Do not convert this row observation
into a homography: collinear points do not define one.
The camera-to-tool 2x2 XY correction matrix must be measured separately using
small, supervised high-Z moves; keep its `verified:false` until both axes,
signs, scale, 2 mm step bound, and target pixel have been checked. No value in
this section authorizes hardware motion.

The measurement-informed offline scene deliberately runs exact-URDF
top-down reachability at the real cap height. With the arm and battery on the
same surface, the present joint limits cannot point the gripper vertically down
at `Z≈151 mm` over this battery. Do not try the seven-cap sequence on hardware.
A mechanically raised arm base, a lower battery fixture, or a separately proven
non-top-down grasp is required; rerun `CapSimulation` after the physical
arrangement changes. An angled tool can reach the location, but it is not
accepted as a vertical cap pick because it can strike the battery or side-load
the cap.

The reason is geometric, not camera noise. The base rotation axis is at
`base_link X=28.402 mm`, which is `71.449 mm` behind the battery-facing plate
edge. The measured 120 mm gap therefore places the battery near face about
191 mm from the rotation axis. With the provisional 20 mm cap-row setback and
the 180 mm first-to-last cap span, every exact top-down grasp fails. A coarse
IK-only mechanical scan found all seven solutions only after reducing the
plate-to-battery gap to at most about 95 mm and raising the arm base roughly
220 mm; a less boundary-sensitive trial used an 80 mm gap and about 200 mm
base lift. These are design-study values, not hardware clearance approval.
Measure the real cap-row setback first, then rerun the full PlanningScene with
the chosen riser/gap and dummy battery.

The red transparent box is a drawing of the measured battery envelope,
not a MoveIt collision object. The yellow rectangle is the configured minimum
lateral-transit height, and green lines show the intended point pattern:
vertical descent, vertical lift, high lateral travel, vertical placement, and
vertical retreat. They are not proof that the arm or tool is collision-free. If
measurements remain null, RViz intentionally shows no green path and the status
explains the blocker.

Inspect the two atomic JSON status streams from another Docker shell:

```bash
ros2 topic echo --once /cap_workcell/status
ros2 topic echo --once /cap_workcell/pick_preview
```

Required dry-run checks before any executor is designed:

1. all seven IDs remain correct while the stationary camera collects at least
   one second of distinct frames;
2. moving any arm joint slowly, repeating a camera timestamp, losing a tag,
   blur, bad exposure, or stopping the camera immediately clears validity;
3. covering/removing a cap produces an expected-occupancy failure rather than a
   guessed pose;
4. every 2D source-corridor sketch clears the battery side and every protected
   terminal/lip by the configured uncertainty and padding;
5. all seven Cartesian sketches are reviewed on a foam/dummy battery for
   repeated trials.

The next stage, not implemented here, must send each preview through a full
MoveIt PlanningScene containing the arm links, camera mount, gripper, battery,
terminals, tray, and attached cap. It must require 100% Cartesian descent/lift
fractions and operator confirmation for each pick. It also needs one leased
command arbiter upstream of the UDP bridge and a trajectory controller that
honors or safely retimes `time_from_start`; the current sequential checkpoint
executor is only a commissioning path. The Pi angle feedback is only a
commanded estimate, so current-limited power and a real cutoff remain mandatory
even after software planning passes.

## 4. Test every Windows-to-Pi connection

Use the execution-policy-safe launcher:

```powershell
.\raspberry-console.cmd Check
```

SSH may ask for the password for `tnt@192.168.188.96`. The check verifies:

1. Docker Desktop Linux engine;
2. RAshko Docker container;
3. authenticated SSH and both Pi systemd services;
4. Pi camera `/healthz`;
5. authenticated UDP heartbeat on port `5005`.

It sends no joint target, does not reset E-stop, and cannot enable outputs.
Continue only when it prints:

```text
[READY] Every Windows-to-Raspberry connection passed.
```

## 5. Open the Raspberry Pi Linux shell from Windows

Use the current PowerShell window:

```powershell
.\raspberry-console.cmd Shell
```

Or open a separate PowerShell window:

```powershell
.\raspberry-console.cmd NewShell
```

Inside the Pi shell:

- type Linux commands normally;
- press `Ctrl+C` to stop the current command;
- type `exit` or press `Ctrl+D` to return to Windows.

Useful Pi-only status commands:

```bash
sudo systemctl status rashko-controller.service --no-pager -l
sudo systemctl status rashko-camera.service --no-pager -l
sudo ss -lunp | grep ':5005'
sudo ss -ltnp | grep ':8080'
```

Follow logs from Windows without opening a general shell:

```powershell
.\raspberry-console.cmd ControllerLogs
.\raspberry-console.cmd CameraLogs
```

Press `Ctrl+C` to stop following logs.

## 6. Direct connection checks

Authenticated UDP heartbeat:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Ping
```

Camera health:

```powershell
Invoke-RestMethod http://192.168.188.96:8080/healthz
```

Open the live MJPEG stream:

```powershell
Start-Process "http://192.168.188.96:8080/stream.mjpg"
```

## 7. Normal safe shutdown

If motor power was ever connected, send authenticated E-stop first:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
```

Verify `ESTOP_LATCHED` and the absence of `OUTPUTS_ENABLED`, then physically
disconnect the servo power supply. Stop Windows Docker afterward:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Stop
```

Stopping Docker does not stop the Pi systemd services. They are designed to stay
running safely with OE HIGH and E-stop latched.

## 8. Mandatory pre-motor gate

Do not connect servo power until every item below passes.

### 8.1 Read the active Pi calibration

Open the Pi shell:

```powershell
.\raspberry-console.cmd Shell
```

Then run:

```bash
sudo python3 -c 'import json; c=json.load(open("/etc/rashko-servos.json")); print("home:", c["home_degrees"]); print("channels:", [(s["name"], s["channel"]) for s in c["servos"]])'
sudo grep -E '^(RASHKO_HARDWARE_BACKEND|RASHKO_SERVO_CONFIG|RASHKO_WATCHDOG_MS)=' /etc/rashko-controller.env
```

Required channel order:

```text
base=0, shoulder=1, elbow=2, wrist=4, wrist_rotate=5, gripper=6
```

Required gripper home: the sixth `home_degrees` value is `0.0`.

Expected controller environment:

```text
RASHKO_HARDWARE_BACKEND=pca9685
RASHKO_SERVO_CONFIG=/etc/rashko-servos.json
RASHKO_WATCHDOG_MS=500
```

Never display or paste `RASHKO_SHARED_SECRET`.

### 8.2 Confirm the synchronized home definition

The active Pi calibration and the checked-in Pi and laptop sources use this
exact logical home:

```text
[base, shoulder, elbow, wrist, wrist_rotate, gripper]
[90,   88,       90,    96,    98,           0]
```

In particular, the gripper is channel 6 and its home is `0°`. The laptop maps
this complete vector to the URDF/MoveIt zero-radian pose; it does not assume a
six-joint `90°` home.

Verify the installed laptop mapping from the Docker shell:

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
python3 -c 'from arm_controller.joint_contract import HOME_DEGREES; print(list(HOME_DEGREES))'
```

Expected:

```text
[90.0, 88.0, 90.0, 96.0, 98.0, 0.0]
```

If either the Pi command in section 8.1 or this Docker command prints anything
different, stop: do not use the GUI **Home** or **Send** controls, execute a
MoveIt trajectory on hardware, or connect servo power.

### 8.3 Physical and electrical checks

With the servo supply still disconnected, verify physically:

- GPIO17 connects to the PCA9685 active-low OE pin;
- an external pull-up keeps OE HIGH during boot, restart, or process failure;
- OE is electrically HIGH at boot and after E-stop;
- PCA9685 uses I2C address `0x40`;
- servo V+ comes from a suitable current-limited external supply, never the
  Raspberry Pi 5 V rail;
- the Pi, PCA9685, and servo supply have the required common signal ground;
- the arm is unloaded and its full work envelope is clear;
- a real power cutoff is within immediate reach;
- each joint is physically placed at the exact active `home_degrees` pose,
  including gripper channel 6 at logical `0°`.

With servo V+ still disconnected, check the I2C address while the controller is
stopped, then restart it immediately:

```bash
sudo systemctl stop rashko-controller.service
sudo i2cdetect -y 1
sudo systemctl start rashko-controller.service
sudo systemctl status rashko-controller.service --no-pager -l
```

The scan must show `40` at address `0x40`, and the service must return to
`active (running)`. Verify OE stayed electrically HIGH during the stop/restart;
software status alone is not a substitute for measuring the pin.

The controller has no position sensors. A reported angle is only the last PWM
command estimate and cannot prove that the mechanism is physically there.

### 8.4 Software safety checks with servo power disconnected

Run the connection gate again:

```powershell
.\raspberry-console.cmd Check
```

Test reset without enabling outputs:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 ResetEStop
```

The ACK must keep `WATCHDOG_TRIPPED` and must not contain `OUTPUTS_ENABLED`.
Re-latch E-stop immediately:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
```

Confirm OE remains electrically HIGH. This is the stopping point before the
first unpowered command/enable test.

## 9. Required checkpoint before motor power

Before continuing, record and review:

1. the printed active `home_degrees` array;
2. the printed six-channel mapping;
3. measured OE HIGH at startup, reset-only state, and E-stop;
4. PCA9685 address `0x40`;
5. external supply voltage/current limit and common-ground arrangement;
6. confirmation that the laptop and Pi home definitions have been synchronized.

Only after this checkpoint should an exact low-speed home command be prepared
for the unpowered OE/watchdog test and then the first powered single-joint test.

## 10. Unpowered output and watchdog test

Do this only after every item in section 9 is recorded. Keep the external servo
V+ supply physically disconnected for this entire section.

In PowerShell terminal A, start the hardware ROS stack:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Hardware
```

In terminal B, open the Docker shell and monitor signed Pi status:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Shell
```

```bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 topic echo /raspberry/status
```

In terminal C, clear the latch while power is still disconnected:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 ResetEStop
```

Before a command, the status must not contain `OUTPUTS_ENABLED`. In a Docker
shell, set 1% speed and send the exact home vector:

```bash
ros2 topic pub --once /servo_speed std_msgs/msg/Float32 "{data: 1.0}"
ros2 topic pub --once /joint_commands std_msgs/msg/Float32MultiArray "{data: [90.0, 88.0, 90.0, 96.0, 98.0, 0.0]}"
```

With a meter or logic analyzer, verify that OE becomes LOW (active) only after this
valid command and that all six configured PCA9685 channels produce the expected
home pulses. Then press `Ctrl+C` in terminal A. Within the configured 500 ms
watchdog window, OE must return HIGH and the Pi status must re-latch E-stop.
Also send E-stop explicitly:

```powershell
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 EStop
```

Do not proceed if OE polarity, the cutoff time, a channel, or a pulse is wrong.

## 11. First powered test: one connected servo only

This is permitted only after section 10 passes. Keep E-stop latched, support the
mechanism, remove the load, and connect only the one servo being tested. Use an
external supply set to the servo manufacturer's voltage and a conservative
current limit; never power the servo from the Pi 5 V rail.

For each servo, start from its physical home and repeat the 1% home command from
section 10. Then change only that servo by one logical degree in a direction that
has verified mechanical clearance. Suggested positive test targets are:

For the gripper specifically, logical `0°` must already be the unloaded,
wide-open pose. If it is nearly closed at `0°`, do not send the `+1°` vector:
cut power, remove the linkage, and re-index the horn at the active open pulse or
calibrate the Pi gripper `min_pulse_us` first. Keep
`RASHKO_GRIPPER_CLOSED_DEGREES=0` on Windows until a safe positive close endpoint
has been measured.

| Joint | PCA9685 channel | Home | +1° test vector |
|---|---:|---:|---|
| base | 0 | 90° | `[91, 88, 90, 96, 98, 0]` |
| shoulder | 1 | 88° | `[90, 89, 90, 96, 98, 0]` |
| elbow | 2 | 90° | `[90, 88, 91, 96, 98, 0]` |
| wrist | 4 | 96° | `[90, 88, 90, 97, 98, 0]` |
| wrist rotate | 5 | 98° | `[90, 88, 90, 96, 99, 0]` |
| gripper | 6 | 0° | `[90, 88, 90, 96, 98, 1]` |

Publish only the chosen vector with the same `/joint_commands` command used in
section 10. Be ready to cut power, confirm the correct channel and direction,
then E-stop before connecting the next servo. Do not test full travel or raise
speed until pulse endpoints and collision-free mechanical limits have been
measured for every joint.
