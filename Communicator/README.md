# RAshko Raspberry Pi Communicator

This is the Raspberry-Pi-side runtime for the RAshko arm. The laptop performs
ROS 2, MoveIt, inverse kinematics, trajectory generation, vision inference, and
the GUI. The Pi behaves as a small hardware controller:

```text
Laptop / Docker / ROS 2
    |
    +-- authenticated UDP :5005 --> watchdog + GPIO17 OE + PCA9685
    |
    +-- HTTP MJPEG :8080 <-------- Raspberry Pi Camera Module 2
```

Camera frames use HTTP MJPEG instead of UDP. Motor commands are small and can
be protected by sequence numbers, acknowledgements, and a watchdog; JPEG frames
are much larger and should not be fragmented into many independent UDP
datagrams.

The Pi does not need ROS.

## Safety model

- GPIO17 controls the PCA9685 active-low OE pin.
- OE HIGH means every PCA9685 output is disabled.
- The controller starts with OE HIGH, E-stop latched, and the watchdog tripped.
- A heartbeat never enables outputs.
- Resetting E-stop never enables outputs.
- Only a subsequent authenticated, fresh, valid six-angle command can enable
  outputs.
- Missing laptop heartbeats latch E-stop and drive OE HIGH.
- Invalid commands are rejected, not clamped.
- Hardware mode refuses to start without an explicit servo calibration file.
- Pi acknowledgements contain commanded angle estimates. There is no physical
  joint-position feedback.

Use an external pull-up, typically 10 kOhm, from PCA9685 OE to its 3.3 V logic
supply so OE stays HIGH while the Pi is booting, restarting, or has not yet
configured GPIO17. Verify this electrically for the actual board before
connecting servo power.

## Wire contract

Every motion command contains exactly six logical degrees:

```text
[base, shoulder, elbow, wrist, wrist_rotate, gripper]
```

The accepted ranges are:

```text
base: 0..245 degrees
all other positions: 0..180 degrees
speed: 1..100 percent
```

`config/rashko-servos.json` contains migration values recovered from the old Pi
controller:

| Joint | PCA9685 channel | Inverted | Pulse range |
|---|---:|---:|---:|
| base | 0 | no | 1000..2000 us |
| shoulder | 1 | yes | 833..2167 us |
| elbow | 2 | yes | 500..2500 us |
| wrist | 4 | no | 500..2500 us |
| wrist rotate | 5 | no | 1000..2000 us |
| gripper | 6 | no | 1000..2000 us |

These values are not a substitute for physical calibration. Keep servo power
off until every channel, direction, pulse width, and endpoint has been checked.

## 1. Prepare Raspberry Pi OS

Use a current 64-bit Raspberry Pi OS or Raspberry Pi OS Lite image. Configure a
stable LAN IPv4 address and enable SSH.

Enable the ARM I2C interface:

```bash
sudo raspi-config
```

Choose `Interface Options` -> `I2C` -> `Enable`, then reboot:

```bash
sudo reboot
```

After reconnecting, verify the PCA9685:

```bash
i2cdetect -y 1
```

The default address should appear as `40`.

Verify the Camera Module 2 before installing this service:

```bash
rpicam-hello --list-cameras
rpicam-hello --timeout 5000
```

Raspberry Pi OS Bookworm and later use the `rpicam-*` application names.
Picamera2 should be installed through `apt`, not from PyPI.

Official references:

- <https://www.raspberrypi.com/documentation/configuration/computers/raspberry-pi.html#enable-or-disable-i2c>
- <https://www.raspberrypi.com/documentation/computers/camera_software.html>

## 2. Put this code on the Pi

From Windows PowerShell, copy the dedicated repository:

```powershell
scp -r C:\Users\User\Desktop\Communicator <pi-user>@<pi-ip>:/home/<pi-user>/
ssh <pi-user>@<pi-ip>
```

On the Pi:

```bash
cd ~/Communicator
python3 -m unittest discover -s tests -v
sudo bash scripts/install.sh
```

The installer:

- installs I2C, GPIO, Picamera2, and camera packages from Raspberry Pi OS;
- creates the unprivileged `rashko` service account;
- installs an isolated Python environment under
  `/opt/rashko-communicator/.venv`;
- installs `/etc/rashko-controller.env` without a secret;
- installs `/etc/rashko-servos.json`;
- installs, but does not start, both systemd services.

## 3. Configure the shared secret and network

On the laptop, run this once from the RAshko directory:

```powershell
.\windows.ps1 Setup
```

That creates `.env.windows` with a random 64-character secret. On the Pi:

```bash
sudo nano /etc/rashko-controller.env
```

Set:

```text
RASHKO_SHARED_SECRET=<the exact RASHKO_SHARED_SECRET from .env.windows>
RASHKO_ALLOWED_CLIENTS=<the laptop's LAN IPv4 address>
RASHKO_HARDWARE_BACKEND=dry-run
```

Do not commit, email, or paste the secret into logs. The laptop and Pi clocks
must be synchronized because signed packets have a five-second freshness
window:

```bash
timedatectl status
```

Keep both devices on the same private LAN. Permit only the laptop to reach:

- UDP `5005` for motor control;
- TCP `8080` for the camera stream.

If using UFW, allow SSH before enabling it:

```bash
sudo apt install -y ufw
sudo ufw allow OpenSSH
sudo ufw allow from <laptop-ip> to any port 5005 proto udp
sudo ufw allow from <laptop-ip> to any port 8080 proto tcp
sudo ufw enable
```

Do not forward either port through the router.

The motor protocol is authenticated. The MJPEG stream is intentionally simple
and unauthenticated for OpenCV compatibility, so its firewall restriction is
mandatory.

## 4. Start in dry-run mode

Keep servo power disconnected and confirm this remains in the environment file:

```text
RASHKO_HARDWARE_BACKEND=dry-run
```

Start both services:

```bash
sudo systemctl enable --now rashko-controller.service
sudo systemctl enable --now rashko-camera.service
sudo systemctl status rashko-controller.service --no-pager
sudo systemctl status rashko-camera.service --no-pager
```

Follow their logs:

```bash
journalctl -u rashko-controller.service -f
journalctl -u rashko-camera.service -f
```

From Windows, verify the camera health endpoint:

```powershell
Invoke-RestMethod http://<pi-ip>:8080/healthz
```

The live stream is:

```text
http://<pi-ip>:8080/stream.mjpg
```

## 5. Test authenticated control from the laptop

The same Communicator directory must be beside RAshko on Windows:

```text
C:\Users\User\Desktop\
|-- RAshko\
`-- Communicator\
```

Start Docker Desktop, then:

```powershell
cd C:\Users\User\Desktop\RAshko
.\windows.ps1 Start
.\windows.ps1 Ping
```

`Ping` sends a signed application heartbeat. It is not ICMP ping and it does
not enable outputs. A healthy dry-run response contains an `OK` code with
`HARDWARE_READY`, `ESTOP_LATCHED`, and `WATCHDOG_TRIPPED`.

Test the E-stop:

```powershell
.\windows.ps1 EStop
```

To test reset without moving hardware:

```powershell
.\windows.ps1 ResetEStop
```

The reset response should still include `WATCHDOG_TRIPPED` and should not
include `OUTPUTS_ENABLED`.

## 6. Commission the PCA9685 with servo power off

Stop the controller:

```bash
sudo systemctl stop rashko-controller.service
```

Review every field in:

```bash
sudo nano /etc/rashko-servos.json
```

Then change:

```text
RASHKO_HARDWARE_BACKEND=pca9685
```

in `/etc/rashko-controller.env` and restart:

```bash
sudo systemctl restart rashko-controller.service
sudo systemctl status rashko-controller.service --no-pager
journalctl -u rashko-controller.service -n 100 --no-pager
```

With servo power still off, verify:

1. OE is HIGH immediately through boot and service startup.
2. The PCA9685 is detected at the configured I2C address.
3. `Ping` receives authenticated ACKs.
4. `ResetEStop` leaves OE HIGH.
5. A fresh motion command may lower OE only after reset.
6. Stopping laptop heartbeats drives OE HIGH within the configured watchdog
   period.
7. `EStop` immediately drives OE HIGH and remains latched.

Do not continue if any check fails.

## 7. First powered movement

Before applying servo power:

1. Remove the arm's load and clear its complete work envelope.
2. Mechanically place the arm at the configured `home_degrees` pose. The
   controller has no sensors and treats this as its startup estimate.
3. Use a current-limited servo supply.
4. Keep a real power cutoff within reach.
5. Use the lowest practical speed.

Start the laptop backend in one PowerShell window:

```powershell
.\windows.ps1 Hardware
```

Start the GUI in another:

```powershell
.\windows.ps1 Gui
```

Monitor Pi acknowledgements from a third shell:

```powershell
.\windows.ps1 Shell
```

```bash
ros2 topic echo /raspberry/status
```

Explicitly reset E-stop, then move only one joint a few degrees around its
verified home value. For every joint, verify:

1. PCA9685 channel;
2. direction/inversion;
3. neutral position;
4. minimum and maximum pulse width;
5. mechanical endpoint and collision clearance;
6. watchdog and E-stop behavior under load.

The status angles are PWM command estimates, not measured feedback.

## Camera configuration

Pi defaults in `/etc/rashko-controller.env`:

```text
RASHKO_CAMERA_PORT=8080
RASHKO_CAMERA_WIDTH=640
RASHKO_CAMERA_HEIGHT=480
RASHKO_CAMERA_FPS=15
RASHKO_CAMERA_JPEG_QUALITY=70
RASHKO_CAMERA_BACKEND=picamera2
```

The laptop derives its stream URL from `RASHKO_PI_HOST` and
`RASHKO_CAMERA_PORT`. An explicit `RASHKO_CAMERA_URL` in `.env.windows`
overrides it.

The laptop camera node republishes frames as:

```text
/camera/image_raw
/camera/camera_info
```

All computer vision and object detection remain on the laptop.

## Updating

After copying a newer Communicator checkout to the Pi:

```bash
cd ~/Communicator
python3 -m unittest discover -s tests -v
sudo bash scripts/install.sh
sudo systemctl restart rashko-controller.service rashko-camera.service
```

The installer preserves existing `/etc/rashko-controller.env` and
`/etc/rashko-servos.json`.
