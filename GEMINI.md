# GEMINI.md — RAshko Cross-Model Reference & Change Log

The Ethernet change log below records the original implementation. The current
status protocol and operating sequence are in [WINDOWS_RASPBERRY_SETUP.md](WINDOWS_RASPBERRY_SETUP.md).

This document provides architectural context and an explicit record of changes made to `RAshko` and `Communicator/` for other coding models (such as Codex, Claude, and future Gemini sessions). Read this alongside [`AGENTS.md`](AGENTS.md) and [`Communicator/AGENTS.md`](Communicator/AGENTS.md).

---

## 1. System Overview & Kinematics-to-Hardware Dataflow

`RAshko` controls a **5-DOF + 1-DOF gripper** robotic arm (6 hobby servos on a PCA9685 at I2C `0x40`, active-low OE safety cutoff on Raspberry Pi `GPIO17`):

- **URDF & Joint Contract**:
  - Canonical URDF: `robot_urdf/robot_fixed.urdf` (keep massless root `base_footprint` -> `base_link` so KDL/MoveIt work cleanly).
  - Canonical 6-value wire & `/joint_commands` degree order (`arm_controller/joint_contract.py` & `Communicator/rashko_communicator/protocol.py`):
    `[base, shoulder, elbow, wrist, wrist_rotate, gripper]`
    - `base`: `0..245°` (`PCA9685` ch `0`, non-inverted, `270°` range)
    - `shoulder`: `0..180°` (`PCA9685` ch `1`, inverted)
    - `elbow`: `0..180°` (`PCA9685` ch `2`, inverted)
    - `wrist`: `0..180°` (`PCA9685` ch `4`, non-inverted)
    - `wrist_rotate`: `0..180°` (`PCA9685` ch `5`, non-inverted)
    - `gripper` (`gripper_base` in URDF): `0..180°` (`PCA9685` ch `6`, non-inverted; locked to `RASHKO_GRIPPER_CLOSED_DEGREES=0` until physically calibrated)
  - Calibrated logical home: `[90, 88, 90, 96, 98, 0]` degrees (`(0.0,) * 6` radians in URDF/MoveIt).
- **Kinematics & GUI (`src/arm_controller/arm_controller/arm_gui.py`)**:
  - Interactive mouse-drag and XYZ controls use the compact joint chain extracted from the same URDF that MoveIt and RViz use. The Tk schematic draws five motor joints and a tool tip, without loading CAD visuals.
  - `arm_gui` publishes `/joint_commands`, `/servo_speed`, and `/emergency_stop`, and subscribes to `/joint_states` and `/raspberry/status`. It **never** publishes desired targets to `/joint_states`.
- **Transport Split**:
  - **Windows / Docker (`RAshko`)**: `udp_servo_bridge` translates `/joint_commands`, `/servo_speed`, and `/emergency_stop` into HMAC-SHA256 authenticated UDP datagrams (`:5005`) via `rashko_communicator.client.UdpArmClient`, and is the sole publisher of `/joint_states` and `/raspberry/status` in hardware mode. `camera_node` reads `http://<pi_host>:8080/stream.mjpg`.
  - **Raspberry Pi (`Communicator/`)**: Zero-ROS Python runtime (`rashko-controller.service` on UDP `:5005` + `rashko-camera.service` on HTTP `:8080`). Enforces allowlist, HMAC-SHA256 auth, 5 s timestamp freshness, session/sequence replay protection, 500 ms heartbeat watchdog, latched E-stop, and `GPIO17` OE cutoff.

---

## 2. Dynamic Ethernet Connection & Auto-Discovery Changes (2026-11-26)

### Problem Solved
Previously, physically moving the Raspberry Pi across desks, switches, or direct PC-to-Pi Ethernet cables changed its IPv4 address, requiring manual edits to `.env.windows` (`RASHKO_PI_HOST`), recreating the Docker container (`windows.ps1 Restart`) because `docker compose exec` kept stale container startup env vars, and manually updating `RASHKO_ALLOWED_CLIENTS` on the Pi whenever the laptop's Ethernet IP changed.

### Summary of Architectural Changes

#### A. Windows Ethernet Adapter Selection, Discovery & Mode Switching (`raspberry-network.ps1`)
Added [`raspberry-network.ps1`](raspberry-network.ps1) (covered by [`tests/test_raspberry_network.ps1`](tests/test_raspberry_network.ps1)), shared by `raspberry.ps1` and `windows.ps1`:
- **Physical Ethernet Filtering (`Get-RashkoEthernetAdapters`, `Select-RashkoEthernetAdapter`)**:
  - Automatically selects active wired Ethernet interfaces while excluding virtual/VPN adapters (`vEthernet`, `WSL`, `Hyper-V`, `VMware`, `VirtualBox`, `ZeroTier`, `Radmin`, `Hamachi`, `Tailscale`, `WireGuard`, `Loopback`).
  - Configurable via `RASHKO_ETHERNET_ADAPTER` in `.env.windows` (`auto` by default) or `-Adapter <Name>` CLI parameter.
- **Multi-Stage Safe Pi Discovery (`Find-RashkoPi`, `Resolve-RashkoPiEndpoint`)**:
  1. Checks configured `RASHKO_PI_HOST` (if on the active Ethernet subnet) and mDNS/DNS names (`RASHKO_PI_HOSTNAME`, `rashko.local`, `raspberrypi.local`, `puppypi.local`).
  2. Warms up Ethernet broadcast/link-local ARP (`169.254.x.x/16` direct-cable friendly) and prioritizes neighbors matching Raspberry Pi Ltd MAC OUIs (`28-CD-C1`, `2C-CF-67`, `3A-35-41`, `88-A2-9E`, `B8-27-EB`, `D8-3A-DD`, `DC-A6-32`, `E4-5F-01`).
  3. Runs a batched asynchronous `.NET` TCP sweep on `:8080` and `:22` across the active `/24` Ethernet neighborhood.
  4. Verifies candidates without Docker or host Python by sending a native `.NET` HMAC-SHA256 signed RAshko `heartbeat` (`Test-RashkoUdpHeartbeatDirect`) and probing `/healthz` (`Test-RashkoCameraHealth`).
  5. Updates `RASHKO_PI_HOST` and `RASHKO_ETHERNET_SUBNET` in `.env.windows` (`Set-RashkoDotEnvValues`) while preserving comments, CRLF/LF line endings, and `RASHKO_SHARED_SECRET`.
- **Adjustable Windows Ethernet Mode (`Set-RashkoEthernetMode`)**:
  - Switches a physical Windows Ethernet adapter between `Dhcp` (router/switch LAN or automatic `169.254.x.x` link-local direct cable) and `Static` (e.g., `192.168.137.1/24` with **no default gateway**, keeping Wi-Fi internet intact), elevating cleanly via UAC when needed.

#### B. Upgraded Operator Consoles (`raspberry.ps1`, `windows.ps1`, `.env.windows.example`)
- **[`raspberry.ps1`](raspberry.ps1) / [`raspberry-console.cmd`](raspberry-console.cmd)**:
  - Automatically discovers and updates `RASHKO_PI_HOST` when `auto`, outside the active Ethernet subnet, or unreachable before `Check`, `Shell`, `NewShell`, `ControllerLogs`, `CameraLogs`, and `SyncAllowlist` (bypass with `-NoAutoDiscover`).
  - Added CLI actions and interactive menu items:
    - `Discover`: scans the active Ethernet adapter and saves `RASHKO_PI_HOST`.
    - `SetHost`: sets `RASHKO_PI_HOST`, `RASHKO_PI_USER`, or `RASHKO_ETHERNET_ADAPTER` without manual file edits.
    - `Ethernet`: inspects physical NICs or toggles Windows Ethernet IPv4 between `Dhcp` and `Static` direct-cable mode.
    - `SyncAllowlist`: SSHes to the Pi, checks whether the deployed `/opt/rashko-communicator` supports CIDR subnets, updates `RASHKO_ALLOWED_CLIENTS` in `/etc/rashko-controller.env` (to `<ethernet_cidr>,169.254.0.0/16` if CIDR-capable, or `<exact_laptop_ip>` if legacy), and restarts `rashko-controller.service` without reading or printing `RASHKO_SHARED_SECRET`.
  - Uses `-o StrictHostKeyChecking=accept-new` on SSH calls so DHCP IP rotation does not block non-interactive checks, and adds a direct Windows-to-Pi signed UDP heartbeat check in `Check` (works even before Docker starts).
- **[`windows.ps1`](windows.ps1)**:
  - Added `Discover` and `SetPi` actions plus `-PiHost`, `-Adapter`, and `-NoAutoDiscover` flags.
  - Added `Invoke-ContainerExec` passing live `-e RASHKO_PI_HOST=...`, `RASHKO_PI_PORT`, `RASHKO_CAMERA_PORT`, `RASHKO_CAMERA_URL`, `RASHKO_CAMERA_BACKEND`, `RASHKO_ETHERNET_SUBNET`, and `RASHKO_PI_HOSTNAME` into every `docker compose exec`, eliminating stale container env vars without `windows.ps1 Restart`.
  - Auto-resolves `RASHKO_PI_HOST` before hardware/camera actions (`Ping`, `ResetEStop`, `Hardware`, `Gui`, `CameraCalibrate`, `CapInspect`). `EStop` uses `-FastEmergencyPath` so an emergency stop is never delayed by discovery when a concrete host is already configured.

#### C. Live Endpoint Reloading & GUI Controls (`src/arm_controller`)
- **[`src/arm_controller/arm_controller/pi_endpoint.py`](src/arm_controller/arm_controller/pi_endpoint.py)** (+ [`test/test_pi_endpoint.py`](src/arm_controller/test/test_pi_endpoint.py)):
  - Centralizes `.env.windows` parsing (`resolve_pi_endpoint`, `default_camera_url`), strict host validation (`validate_pi_host`), secret-preserving updates (`update_dotenv_pi_host`, including Docker bind-mount `EBUSY` fallback), and read-only `probe_pi_candidate` / `discover_pi_host` helpers.
  - Live `.env.windows` values take precedence over stale container startup `os.environ` (`RASHKO_PI_HOST_OVERRIDE` overrides both).
- **Launch files & `camera_node.py` & `udp_servo_bridge.py`**:
  - `arm.launch.py`, `view_with_gui.launch.py`, `camera_intrinsic_calibration.launch.py`, `cap_inspection.launch.py`, and `camera_node.py` now share `default_camera_url()`.
  - `CameraNode._network_mjpeg_worker` re-reads `_default_camera_url()` on reconnect iterations when using derived Pi camera URLs, automatically switching streams if `RASHKO_PI_HOST` changes.
  - `UdpServoBridge` checks `/workspace/.env.windows` `mtime_ns` (rate-limited to 1 Hz in `_heartbeat_callback`) when following the env file. On `(pi_host, pi_port)` change, it constructs the new `UdpArmClient` first, closes the previous socket, resets session sequence tracking (`highest_ack_sequence = None`, `reset_pending = False`), and **preserves `estop_active`** so local E-stop latches are never cleared across reconnects.
- **[`src/arm_controller/arm_controller/arm_gui.py`](src/arm_controller/arm_controller/arm_gui.py)**:
  - Displays the active `Pi [host:port]:` endpoint in the bottom status strip (while keeping `self._pi_status` as the exact `(updated, code, states)` 3-tuple expected by GUI tests).
  - Added a non-blocking **Pi Target (Ethernet)** bar (`Apply IP` + `Auto-Find Pi`) in `info_frame` that writes `RASHKO_PI_HOST` to `/workspace/.env.windows` so `udp_servo_bridge`, `camera_node`, and host PowerShell scripts immediately synchronize.

#### D. Portable Subnet Allowlists on the Raspberry Pi (`Communicator/`)
- **[`Communicator/rashko_communicator/config.py`](Communicator/rashko_communicator/config.py)** & **[`controller.py`](Communicator/rashko_communicator/controller.py)**:
  - Extended `RASHKO_ALLOWED_CLIENTS` (`frozenset[str]`) to accept both exact IPv4 addresses (`192.168.100.15`) and IPv4 CIDR subnets (`192.168.100.0/24,169.254.0.0/16`), pre-compiled at `UdpControllerServer` startup (`compile_allowed_clients` / `matches_allowed_clients`).
  - Rejects IPv6 entries and unsafe `/0` wildcards (`0.0.0.0/0`); empty `RASHKO_ALLOWED_CLIENTS` remains the explicit way to allow any LAN client.
  - Per-session exact-client-IP binding inside `ArmController.process` remains untouched.

---

## 3. Current Safety Invariants

1. **Discovery** selects a Pi using a signed `status` reply. A status request neither claims the motion session nor renews its watchdog.
2. **Endpoint changes** stop the former Pi and require a fresh ACK and an explicit reset on the selected endpoint. The old session loses its watchdog renewal. A derived MJPEG stream reconnects when the live URL changes, even while the old stream is healthy.
3. **Signing keys** stay in local configuration. Network tests use a dummy key.
4. **Architecture split**: `Communicator/` remains 100% ROS-free; `RAshko` owns zero GPIO/I2C/PCA9685 hardware drivers.

---

## 4. Validation Commands

- **Communicator unit tests**:
  ```powershell
  cd Communicator
  py -m unittest discover -s tests -v
  ```
- **Windows Ethernet & signed UDP wire-interop tests**:
  ```powershell
  powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\tests\test_raspberry_network.ps1
  ```
- **Python endpoint & UDP bridge unit tests**:
  ```powershell
  $env:PYTHONPATH = "src/arm_controller;Communicator"
  py -m unittest discover -s src/arm_controller/test -p "test_pi_endpoint.py" -v
  py -m unittest discover -s src/arm_controller/test -p "test_udp_servo_bridge.py" -v
  ```
- **Full ROS / MoveIt offline smoke test (requires Docker Desktop Linux engine)**:
  ```powershell
  powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Smoke
  ```
