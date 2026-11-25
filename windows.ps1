[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet("Setup", "Build", "Rebuild", "Start", "Stop", "Restart", "Logs", "Shell", "Status", "Stats", "Ping", "EStop", "ResetEStop", "Hardware", "MoveIt", "Gui", "Display", "Simulation", "SimulationGui", "CapSimulation", "Smoke", "CartesianTest", "CameraCalibrate", "CameraSave", "CapInspect", "CapStatus", "CapFixture", "CapTest")]
    [string]$Action = "Status",
    [switch]$NoCamera
)

$ErrorActionPreference = "Stop"
$ProjectDir = $PSScriptRoot
$EnvFile = Join-Path $ProjectDir ".env.windows"
$ComposeFile = Join-Path $ProjectDir "docker-compose.windows.yml"

function Invoke-DockerCompose {
    & docker compose --env-file $EnvFile -f $ComposeFile @args
    if ($LASTEXITCODE -ne 0) {
        throw "docker compose failed with exit code $LASTEXITCODE"
    }
}

function Require-EnvironmentFile {
    if (-not (Test-Path -LiteralPath $EnvFile)) {
        throw "Missing .env.windows. Run: .\windows.ps1 Setup"
    }
}

function Build-RosWorkspace {
    Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && cd /workspace && colcon build --symlink-install'
}

function Get-ActiveContainerProcesses {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Pattern
    )

    $processRows = & docker compose --env-file $EnvFile -f $ComposeFile `
        exec -T ros2_arm ps -eo 'stat=,args=' 2>$null
    $probeExitCode = $LASTEXITCODE
    if ($probeExitCode -ne 0) {
        throw "Could not inspect container processes (exit $probeExitCode)."
    }

    $activeRows = @()
    foreach ($row in $processRows) {
        $columns = $row.Trim() -split '\s+', 2
        if (
            $columns.Count -eq 2 -and
            $columns[0] -notmatch '^Z' -and
            $columns[1] -match $Pattern
        ) {
            $activeRows += $row.Trim()
        }
    }
    return $activeRows
}

function Assert-NoContainerProcess {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Pattern,
        [Parameter(Mandatory = $true)]
        [string]$Description
    )

    $activeRows = @(Get-ActiveContainerProcesses -Pattern $Pattern)
    if ($activeRows.Count -gt 0) {
        throw "$Description is already running in the container:`n$($activeRows -join [Environment]::NewLine)"
    }
}

function Test-ContainerProcess {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Pattern
    )

    return @(Get-ActiveContainerProcesses -Pattern $Pattern).Count -gt 0
}

function Assert-NoRosBackend {
    Assert-NoContainerProcess `
        -Pattern '[s]imulation.launch.py|[a]rm.launch.py|[m]oveit.launch.py' `
        -Description 'A ROS backend'
}

function Assert-SafeContainerLifecycle {
    $containerId = & docker compose --env-file $EnvFile -f $ComposeFile `
        ps -q ros2_arm 2>$null
    if ($LASTEXITCODE -ne 0) {
        throw "Could not inspect the RAshko container before a lifecycle action."
    }
    if (-not $containerId) {
        return
    }

    $rosPattern = '[r]os2 launch|[u]dp_servo_bridge|[t]rajectory_executor|[m]ove_group|[a]rm_gui|[c]amera_node|[s]im_servo_node|[r]obot_state_publisher'
    $activeRows = @(Get-ActiveContainerProcesses -Pattern $rosPattern)
    if ($activeRows.Count -gt 0) {
        throw (
            "Container lifecycle/build refused while RAshko ROS processes are active. " +
            "E-stop the arm, turn the servo PSU off, and stop Hardware/GUI/Simulation terminals first.`n" +
            ($activeRows -join [Environment]::NewLine)
        )
    }
}

function Start-XServer {
    $xServerPath = Join-Path $Env:ProgramFiles "VcXsrv\vcxsrv.exe"
    if (-not (Test-Path -LiteralPath $xServerPath)) {
        throw "VcXsrv is required. Install it with: winget install --id marha.VcXsrv --exact"
    }
    if (-not (Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue)) {
        # Windows Firewall must remain enabled. -ac is required for the Docker
        # NAT address; the host firewall remains the network access boundary.
        Start-Process -FilePath $xServerPath `
            -ArgumentList ':0','-multiwindow','-clipboard','-wgl','-ac','-listen','tcp' `
            -WindowStyle Hidden
        Start-Sleep -Seconds 1
    }
}

function Start-ArmGui {
    param([ValidateSet("true", "false")][string]$Camera)

    Assert-NoContainerProcess -Pattern '[v]iew_with_gui.launch.py|[a]rm_controller/arm(\s|$)' -Description 'The arm GUI'
    Start-XServer
    $launchCommand = 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller view_with_gui.launch.py camera:={0}' -f $Camera
    Invoke-DockerCompose exec ros2_arm bash -lc $launchCommand
}

if ($Action -eq "Setup") {
    if (Test-Path -LiteralPath $EnvFile) {
        Write-Host ".env.windows already exists; leaving it untouched."
        exit 0
    }
    $randomBytes = New-Object byte[] 32
    $generator = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    try { $generator.GetBytes($randomBytes) } finally { $generator.Dispose() }
    $secret = (($randomBytes | ForEach-Object { $_.ToString("x2") }) -join "")
    @"
RASHKO_PI_HOST=192.168.1.50
RASHKO_PI_USER=tnt
RASHKO_PI_PORT=5005
RASHKO_SHARED_SECRET=$secret
RASHKO_CAMERA_PORT=8080
RASHKO_CAMERA_URL=
RASHKO_CAMERA_BACKEND=auto
RASHKO_CAMERA_INFO_FILE=
RASHKO_CAP_WORKCELL_CONFIG=/workspace/config/cap_workcell.example.yaml
RASHKO_CAP_CAMERA_WIDTH=640
RASHKO_CAP_CAMERA_HEIGHT=480
RASHKO_GRIPPER_CLOSED_DEGREES=0
ROS_DOMAIN_ID=42
DISPLAY=host.docker.internal:0.0
"@ | Set-Content -LiteralPath $EnvFile -Encoding ascii
    Write-Host "Created $EnvFile"
    Write-Host "Edit RASHKO_PI_HOST, then copy the generated secret to /etc/rashko-controller.env on the Pi."
    exit 0
}

Require-EnvironmentFile

switch ($Action) {
    "Build"   { Assert-SafeContainerLifecycle; Invoke-DockerCompose build }
    "Rebuild" { Assert-SafeContainerLifecycle; Invoke-DockerCompose build; Invoke-DockerCompose up -d --force-recreate --no-build; Build-RosWorkspace }
    "Start"   { Assert-SafeContainerLifecycle; Invoke-DockerCompose up -d; Build-RosWorkspace }
    "Stop"    { Assert-SafeContainerLifecycle; Invoke-DockerCompose down }
    "Restart" { Assert-SafeContainerLifecycle; Invoke-DockerCompose up -d --force-recreate --no-build; Build-RosWorkspace }
    "Logs"    { Invoke-DockerCompose logs -f --tail 200 }
    "Shell"   { Invoke-DockerCompose exec ros2_arm bash }
    "Status"  { Invoke-DockerCompose ps }
    "Stats" {
        Get-Process -Name 'vcxsrv','vmmem','vmmemWSL','com.docker.backend' -ErrorAction SilentlyContinue |
            Select-Object ProcessName, Id, @{Name='WorkingSetMiB'; Expression={[Math]::Round($_.WorkingSet64 / 1MB, 1)}} |
            Format-Table -AutoSize
        $containerId = Invoke-DockerCompose ps -q ros2_arm
        if ($containerId) {
            & docker stats --no-stream $containerId
            if ($LASTEXITCODE -ne 0) { throw "Could not read RAshko container statistics." }
            Invoke-DockerCompose exec -T ros2_arm ps -eo 'pid,rss,pcpu,comm' --sort=-rss
        }
    }
    "Ping" {
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /workspace/install/setup.bash && python3 -m rashko_communicator.send_angles "$RASHKO_PI_HOST" --port "$RASHKO_PI_PORT" --heartbeat'
    }
    "EStop" {
        if (Test-ContainerProcess -Pattern '[u]dp_servo_bridge') {
            Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"'
        } else {
            Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /workspace/install/setup.bash && python3 -m rashko_communicator.send_angles "$RASHKO_PI_HOST" --port "$RASHKO_PI_PORT" --estop'
        }
    }
    "ResetEStop" {
        if (Test-ContainerProcess -Pattern '[u]dp_servo_bridge') {
            Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: false}"'
        } else {
            Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /workspace/install/setup.bash && python3 -m rashko_communicator.send_angles "$RASHKO_PI_HOST" --port "$RASHKO_PI_PORT" --reset-estop'
        }
    }
    "Hardware" {
        Assert-NoRosBackend
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller arm.launch.py gui:=false camera:=false moveit:=true bridge:=true trajectory:=true robot_state:=true'
    }
    "MoveIt" {
        Assert-NoRosBackend
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_moveit_config moveit.launch.py'
    }
    "Display" { Start-XServer }
    "Gui" {
        if ($NoCamera) { Start-ArmGui -Camera "false" }
        else { Start-ArmGui -Camera "true" }
    }
    "Simulation" {
        Start-XServer
        Assert-NoRosBackend
        Write-Host "Starting the offline backend and RViz. No UDP packets are sent to the Raspberry Pi."
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller simulation.launch.py'
    }
    "SimulationGui" {
        Start-ArmGui -Camera "false"
    }
    "CapSimulation" {
        Start-XServer
        Assert-NoRosBackend
        Assert-NoContainerProcess `
            -Pattern '[c]ap_sequence_simulation.launch.py|[c]ap_sequence_simulation' `
            -Description 'The cap sequence simulation'
        Write-Host "Starting the dedicated OFFLINE cap sequence simulation. Camera and UDP bridge are disabled; no Raspberry Pi or hardware commands are sent."
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller cap_sequence_simulation.launch.py'
    }
    "Smoke" {
        Assert-NoRosBackend
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller simulation.launch.py rviz:=false smoke_test:=true'
    }
    "CartesianTest" {
        Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && timeout 30 ros2 run arm_controller cartesian_simulation_test'
    }
    "CameraCalibrate" {
        Start-XServer
        Assert-NoRosBackend
        Assert-NoContainerProcess `
            -Pattern '[c]amera_node|[c]amera_calibration_source|[c]ameracalibrator' `
            -Description 'A camera or camera calibrator'
        Write-Host "Starting CAMERA-ONLY intrinsic calibration. This cannot move the arm."
        Write-Host "Use the 8x6-inner-corner board with measured 20.000 mm squares."
        Invoke-DockerCompose exec ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller camera_intrinsic_calibration.launch.py'
    }
    "CameraSave" {
        Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run arm_controller camera_calibration_export --archive /tmp/calibrationdata.tar.gz --output /workspace/config/camera_intrinsics.yaml'
    }
    "CapInspect" {
        Start-XServer
        Assert-NoContainerProcess -Pattern '[c]ap_inspection.launch.py|[c]ap_inspection' -Description 'Cap inspection'
        Assert-NoContainerProcess -Pattern '[c]amera_node' -Description 'A ROS camera node'
        Write-Host "Starting camera-only cap inspection. This command cannot move the arm."
        $localCapConfigPath = Join-Path $ProjectDir "config\cap_workcell.yaml"
        if (Test-Path -LiteralPath $localCapConfigPath) {
            $containerCapConfigPath = "/workspace/config/cap_workcell.yaml"
            Write-Host "Using local partially measured config/cap_workcell.yaml"
        } else {
            $containerCapConfigPath = "/workspace/config/cap_workcell.example.yaml"
            Write-Warning "config/cap_workcell.yaml is missing; using the locked example profile."
        }
        $cameraCalibrationPath = Join-Path $ProjectDir "config\camera_intrinsics.yaml"
        if (Test-Path -LiteralPath $cameraCalibrationPath) {
            Write-Host "Using validated camera calibration config/camera_intrinsics.yaml"
            $capInspectCommand = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller cap_inspection.launch.py config_file:=$containerCapConfigPath camera_info_file:=/workspace/config/camera_intrinsics.yaml"
        } else {
            Write-Host "No camera calibration: raw-color pixel-centering preview remains available; metric inspection remains locked."
            $capInspectCommand = "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 launch arm_controller cap_inspection.launch.py config_file:=$containerCapConfigPath"
        }
        Invoke-DockerCompose exec ros2_arm bash -lc $capInspectCommand
    }
    "CapStatus" {
        Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 topic echo --once --field data /cap_workcell/status std_msgs/msg/String'
    }
    "CapFixture" {
        Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && ros2 run arm_controller cap_fixture_svg --config "$RASHKO_CAP_WORKCELL_CONFIG" --output /workspace/config/cap_fixture.svg'
    }
    "CapTest" {
        Invoke-DockerCompose exec -T ros2_arm bash -lc 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest -q src/arm_controller/test/test_cap_perception.py src/arm_controller/test/test_cap_workcell_config.py src/arm_controller/test/test_cap_motion_safety.py src/arm_controller/test/test_cap_fixture_svg.py src/arm_controller/test/test_camera_info_loader.py src/arm_controller/test/test_camera_calibration_export.py src/arm_controller/test/test_fixture_pose.py src/arm_controller/test/test_red_cap_visual_servo.py src/arm_controller/test/test_cap_sequence_simulation_config.py src/arm_controller/test/test_cap_sequence_simulation_safety.py src/arm_controller/test/test_trajectory_safety.py src/arm_controller/test/test_trajectory_executor_safety.py src/arm_controller/test/test_udp_servo_bridge.py'
    }
}
