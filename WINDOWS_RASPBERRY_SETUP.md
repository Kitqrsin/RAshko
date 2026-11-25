# Windows and Raspberry Pi commissioning

## Supported split

- Windows Docker: ROS 2 Humble, GUI, MoveIt/IK, trajectory generation, vision,
  simulation, and authenticated UDP transport.
- Raspberry Pi 5: `Communicator/`, GPIO17 OE cutoff, watchdog, and
  PCA9685 output, plus Camera Module 2 MJPEG streaming. The Pi does not run ROS,
  calculate trajectories, or run vision inference.

## 1. Prepare Windows without hardware

Start Docker Desktop in Linux-container mode. In PowerShell:

```powershell
cd RAshko
.\windows.ps1 Setup
.\windows.ps1 Start
```

Run the headless software check before opening either GUI window:

```powershell
.\windows.ps1 Smoke
```

When the smoke test passes, run the interactive simulation in two terminals:

```powershell
.\windows.ps1 Simulation
```

```powershell
.\windows.ps1 SimulationGui
```

These commands send no Raspberry Pi traffic.

## 2. Network and Pi

Give the Windows machine and Pi stable LAN IPv4 addresses. Allow inbound UDP
port `5005` and inbound TCP port `8080` on the Pi only from the Windows
machine. Never expose either publicly.

Follow [`Communicator/README.md`](Communicator/README.md). Keep servo power
off through its dry-run checks.

Edit `.env.windows` and set `RASHKO_PI_HOST`. Copy its generated
`RASHKO_SHARED_SECRET` into `/etc/rashko-controller.env` on the Pi. Do not commit
either secret-bearing file.

Keep `RASHKO_GRIPPER_CLOSED_DEGREES=0` during initial commissioning. If logical
`0°` is not mechanically wide open, disconnect the linkage and either re-index
the horn at the active open pulse or calibrate the Pi's gripper `min_pulse_us`
without load. After measuring a safe positive close endpoint, put that logical
angle in `.env.windows` and recreate the container. Values above it are rejected
by the Windows UDP bridge.

## 3. Connection check

With the Pi receiver running and servo power still off:

```powershell
.\raspberry-console.cmd Check
```

Alternatively, double-click `raspberry-console.cmd` and choose **Test every
connection**. The connection gate verifies Docker, an authenticated SSH login,
both Pi services, the camera health endpoint, and the signed UDP heartbeat. It
does not reset E-stop, enable outputs, or send motor targets. The console also
provides interactive Pi shell and service-log options for repairs; rerun the
gate after making changes and continue only when it reports `[READY]`.

Inside `Shell`, inspect acknowledgements with:

```bash
ros2 topic echo /raspberry/status
```

A heartbeat never enables motor outputs. A healthy response reports `OK` and
`HARDWARE_READY`; `WATCHDOG_TRIPPED` is expected before the first command.

Check the Camera Module 2 service separately:

```powershell
Invoke-RestMethod http://<pi-ip>:8080/healthz
```

The laptop camera node consumes `http://<pi-ip>:8080/stream.mjpg` and republishes
it as `/camera/image_raw`.

## 4. First real motion

Remove the arm's load, clear its work envelope, use a current-limited servo
supply, and keep a physical cutoff reachable. Start the backend:

```powershell
.\windows.ps1 Hardware
```

Then start the GUI in another terminal:

```powershell
.\windows.ps1 Gui
```

The backend, not the GUI, owns `/joint_states`. In hardware mode the UDP bridge
publishes the Pi's authenticated, rate-limited angle estimate. A healthy graph
must show exactly one publisher:

```powershell
.\windows.ps1 Shell
ros2 topic info /joint_states --verbose
```

The publisher must be `udp_servo_bridge`. The values are commanded estimates,
not encoder feedback, so a stalled or slipping servo still requires physical
inspection.

Explicitly clear the startup latch before the first valid motion command:

```powershell
.\windows.ps1 ResetEStop
```

Verify, one joint at a time:

1. OE polarity and watchdog cutoff.
2. Startup pose matches the configured logical home
   `[90, 88, 90, 96, 98, 0]` degrees; the final value is gripper channel 6.
3. Channel mapping and direction.
4. Pulse widths and mechanical endpoints.
5. Emergency stop under real load.

Use `.\windows.ps1 EStop` if the GUI is unavailable. The Pi keeps E-stop
latched until an explicit reset, and outputs remain disabled until a fresh
valid command arrives.

## Known boundary

The offline simulator verifies the software pipeline, not the mechanics. Real
hardware sign-off remains mandatory for direction, load, power, backlash,
limits, collision clearance, and thermal behavior.
