[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet("Menu", "Check", "Shell", "NewShell", "ControllerLogs", "CameraLogs", "Help")]
    [string]$Action = "Menu",

    [string]$PiUser = ""
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$ProjectDir = $PSScriptRoot
$EnvFile = Join-Path $ProjectDir ".env.windows"
$ComposeFile = Join-Path $ProjectDir "docker-compose.windows.yml"
$WindowsScript = Join-Path $ProjectDir "windows.ps1"

function Read-DotEnvFile {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "Missing .env.windows. Run: powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Setup"
    }

    $values = @{}
    foreach ($originalLine in Get-Content -LiteralPath $Path) {
        $line = $originalLine.Trim()
        if (-not $line -or $line.StartsWith("#")) {
            continue
        }
        $separator = $line.IndexOf("=")
        if ($separator -le 0) {
            continue
        }
        $key = $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        $values[$key] = $value
    }
    return $values
}

function Get-ConfiguredValue {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$Values,
        [Parameter(Mandatory = $true)]
        [string]$Name,
        [string]$Default = ""
    )

    if ($Values.ContainsKey($Name) -and -not [string]::IsNullOrWhiteSpace($Values[$Name])) {
        return [string]$Values[$Name]
    }
    return $Default
}

function Invoke-ExternalCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$FilePath,
        [string[]]$ArgumentList = @()
    )

    if (-not (Get-Command $FilePath -ErrorAction SilentlyContinue)) {
        return [pscustomobject]@{
            ExitCode = 127
            Output = "Command not found: $FilePath"
        }
    }

    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = "Continue"
        $output = & $FilePath @ArgumentList 2>&1
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }

    return [pscustomobject]@{
        ExitCode = $exitCode
        Output = (($output | ForEach-Object { [string]$_ }) -join [Environment]::NewLine).Trim()
    }
}

function Write-Check {
    param([string]$Message)
    Write-Host "[....] $Message" -ForegroundColor Cyan
}

function Write-Pass {
    param([string]$Message)
    Write-Host "[PASS] $Message" -ForegroundColor Green
}

function Write-Failure {
    param(
        [string]$Message,
        [string]$Detail = ""
    )
    Write-Host "[FAIL] $Message" -ForegroundColor Red
    if ($Detail) {
        Write-Host $Detail -ForegroundColor DarkYellow
    }
}

$EnvironmentValues = Read-DotEnvFile -Path $EnvFile
$PiHost = Get-ConfiguredValue -Values $EnvironmentValues -Name "RASHKO_PI_HOST"
$PiPort = Get-ConfiguredValue -Values $EnvironmentValues -Name "RASHKO_PI_PORT" -Default "5005"
$CameraPort = Get-ConfiguredValue -Values $EnvironmentValues -Name "RASHKO_CAMERA_PORT" -Default "8080"

if ([string]::IsNullOrWhiteSpace($PiUser)) {
    $PiUser = Get-ConfiguredValue -Values $EnvironmentValues -Name "RASHKO_PI_USER" -Default "tnt"
}

if ([string]::IsNullOrWhiteSpace($PiHost)) {
    throw "RASHKO_PI_HOST is empty in .env.windows"
}
if ($PiHost -notmatch '^[A-Za-z0-9][A-Za-z0-9.:-]*$') {
    throw "RASHKO_PI_HOST contains unsupported characters: $PiHost"
}
if ($PiUser -notmatch '^[A-Za-z_][A-Za-z0-9_-]*$') {
    throw "Pi user contains unsupported characters: $PiUser"
}
if ($PiPort -notmatch '^\d+$' -or [int]$PiPort -lt 1 -or [int]$PiPort -gt 65535) {
    throw "RASHKO_PI_PORT must be in 1..65535"
}
if ($CameraPort -notmatch '^\d+$' -or [int]$CameraPort -lt 1 -or [int]$CameraPort -gt 65535) {
    throw "RASHKO_CAMERA_PORT must be in 1..65535"
}

$SshTarget = "${PiUser}@${PiHost}"
$CameraHealthUrl = "http://${PiHost}:${CameraPort}/healthz"

function Test-DockerEngine {
    Write-Check "Docker Desktop Linux engine"
    $result = Invoke-ExternalCommand -FilePath "docker" -ArgumentList @("info", "--format", "{{.OSType}}")
    if ($result.ExitCode -eq 0 -and $result.Output -match '(^|\s)linux($|\s)') {
        Write-Pass "Docker Linux engine is running"
        return $true
    }
    Write-Failure "Docker Linux engine is unavailable" $result.Output
    return $false
}

function Test-RosContainer {
    Write-Check "RAshko Docker container"
    $arguments = @(
        "compose", "--env-file", $EnvFile,
        "-f", $ComposeFile,
        "ps", "--status", "running", "-q", "ros2_arm"
    )
    $result = Invoke-ExternalCommand -FilePath "docker" -ArgumentList $arguments
    if ($result.ExitCode -eq 0 -and -not [string]::IsNullOrWhiteSpace($result.Output)) {
        Write-Pass "RAshko Docker container is running"
        return $true
    }
    Write-Failure "RAshko Docker container is not running" "Run: powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Start"
    return $false
}

function Test-RaspberrySshAndServices {
    Write-Check "SSH authentication and Raspberry Pi services"
    Write-Host "       SSH may ask for the password for $SshTarget." -ForegroundColor DarkGray
    $remoteCommand = "systemctl is-active rashko-controller.service rashko-camera.service"
    $result = Invoke-ExternalCommand -FilePath "ssh" -ArgumentList @(
        "-o", "ConnectTimeout=7",
        "-o", "ServerAliveInterval=5",
        "-o", "ServerAliveCountMax=1",
        $SshTarget,
        $remoteCommand
    )
    if ($result.ExitCode -eq 0) {
        $states = @($result.Output -split "`r?`n" | Where-Object { $_ -eq "active" })
        if ($states.Count -eq 2) {
            Write-Pass "SSH works; controller and camera services are active"
            return $true
        }
    }
    Write-Failure "SSH or a Raspberry Pi service check failed" $result.Output
    Write-Host "       Use menu option 2 to open the Pi shell and repair the service." -ForegroundColor DarkYellow
    return $false
}

function Test-CameraHealth {
    Write-Check "Camera Module HTTP health endpoint"
    try {
        $health = Invoke-RestMethod -Uri $CameraHealthUrl -TimeoutSec 7
        if ($health.status -eq "ok" -and $health.frame_ready -eq $true) {
            Write-Pass "Camera is ready ($($health.backend), $($health.width)x$($health.height) at $($health.fps) FPS)"
            return $true
        }
        Write-Failure "Camera answered but is not ready" ($health | Out-String)
    }
    catch {
        Write-Failure "Camera health request failed: $CameraHealthUrl" $_.Exception.Message
    }
    return $false
}

function Test-AuthenticatedUdp {
    Write-Check "Authenticated UDP heartbeat to ${PiHost}:${PiPort}"
    $result = Invoke-ExternalCommand -FilePath "powershell.exe" -ArgumentList @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", $WindowsScript,
        "Ping"
    )
    if ($result.ExitCode -eq 0 -and $result.Output -match '"code"\s*:\s*"OK"') {
        Write-Pass "Authenticated UDP heartbeat received an OK acknowledgement"
        $ackLine = @($result.Output -split "`r?`n" | Where-Object { $_.Trim().StartsWith("{") }) | Select-Object -Last 1
        if ($ackLine) {
            Write-Host "       $ackLine" -ForegroundColor DarkGray
        }
        return $true
    }
    Write-Failure "Authenticated UDP heartbeat failed" $result.Output
    return $false
}

function Test-AllConnections {
    Write-Host ""
    Write-Host "RAshko Raspberry Pi connection gate" -ForegroundColor White
    Write-Host "Windows: $env:COMPUTERNAME   Pi: $SshTarget   UDP: $PiPort   Camera: $CameraPort" -ForegroundColor DarkGray
    Write-Host "This check sends only a signed heartbeat. It never resets E-stop or sends motor targets." -ForegroundColor Yellow
    Write-Host ""

    $dockerReady = Test-DockerEngine
    $containerReady = $false
    if ($dockerReady) {
        $containerReady = Test-RosContainer
    }
    else {
        Write-Failure "RAshko Docker container check skipped" "Start Docker Desktop first."
    }

    $sshReady = Test-RaspberrySshAndServices
    $cameraReady = Test-CameraHealth

    $udpReady = $false
    if ($containerReady) {
        $udpReady = Test-AuthenticatedUdp
    }
    else {
        Write-Failure "Authenticated UDP heartbeat skipped" "The RAshko container must be running."
    }

    $allReady = $dockerReady -and $containerReady -and $sshReady -and $cameraReady -and $udpReady
    Write-Host ""
    if ($allReady) {
        Write-Host "[READY] Every Windows-to-Raspberry connection passed." -ForegroundColor Green
        Write-Host "You may continue with the Pi shell or the commissioning procedure." -ForegroundColor Green
        return $true
    }

    Write-Host "[BLOCKED] One or more connections failed. Motor commissioning must not continue." -ForegroundColor Red
    Write-Host "Open the Pi shell or logs from this menu, repair the failure, then run Check again." -ForegroundColor Yellow
    return $false
}

function Open-RaspberryShell {
    Write-Host ""
    Write-Host "Opening an interactive Linux shell on $SshTarget" -ForegroundColor Cyan
    Write-Host "Type commands normally. Use Ctrl+C to stop a running command; type exit or press Ctrl+D to return." -ForegroundColor Yellow
    Write-Host "Keep servo power disconnected until the connection gate passes." -ForegroundColor Yellow
    & ssh $SshTarget
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
        Write-Failure "SSH session ended with exit code $LASTEXITCODE"
        return $false
    }
    Write-Pass "Returned from Raspberry Pi shell"
    return $true
}

function Open-NewRaspberryShell {
    $command = "Write-Host 'RAshko Raspberry Pi shell: type exit to close.' -ForegroundColor Cyan; ssh $SshTarget"
    $encodedCommand = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    Start-Process -FilePath "powershell.exe" -ArgumentList @(
        "-NoLogo",
        "-NoProfile",
        "-NoExit",
        "-EncodedCommand",
        $encodedCommand
    )
    Write-Pass "Opened a separate PowerShell window for $SshTarget"
}

function Follow-RaspberryLog {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet("rashko-controller.service", "rashko-camera.service")]
        [string]$Service
    )

    Write-Host "Press Ctrl+C to stop following $Service and return." -ForegroundColor Yellow
    & ssh -t $SshTarget "sudo journalctl -u $Service -f"
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
        Write-Failure "Log session ended with exit code $LASTEXITCODE"
    }
}

function Show-Help {
    Write-Host @"
RAshko Raspberry Pi connection console

Double-click: raspberry-console.cmd

Execution-policy-safe actions:
  .\raspberry-console.cmd Check            Test Docker, SSH/services, camera, and signed UDP
  .\raspberry-console.cmd Shell            Use the Pi Linux shell in the current window
  .\raspberry-console.cmd NewShell         Open the Pi Linux shell in a new PowerShell window
  .\raspberry-console.cmd ControllerLogs   Follow the Pi controller journal; Ctrl+C returns
  .\raspberry-console.cmd CameraLogs       Follow the Pi camera journal; Ctrl+C returns

Optional SSH user override:
  .\raspberry-console.cmd Check -PiUser tnt

The checks do not reset E-stop, enable outputs, or send joint commands.
"@
}

function Show-Menu {
    $lastCheckPassed = $false
    while ($true) {
        Write-Host ""
        Write-Host "RAshko Raspberry connection console" -ForegroundColor White
        Write-Host "  1. Test every connection"
        Write-Host "  2. Open Pi shell in this window"
        Write-Host "  3. Open Pi shell in a new PowerShell window"
        Write-Host "  4. Follow controller logs"
        Write-Host "  5. Follow camera logs"
        Write-Host "  6. Show help"
        Write-Host "  0. Exit"
        if ($lastCheckPassed) {
            Write-Host "  Connection gate: READY" -ForegroundColor Green
        }
        else {
            Write-Host "  Connection gate: not passed in this session" -ForegroundColor DarkYellow
        }

        $choice = Read-Host "Choose"
        if ($null -eq $choice) {
            return
        }
        $choice = $choice.Trim()
        if (-not $choice) {
            return
        }
        switch ($choice) {
            "1" { $lastCheckPassed = Test-AllConnections }
            "2" { [void](Open-RaspberryShell) }
            "3" { Open-NewRaspberryShell }
            "4" { Follow-RaspberryLog -Service "rashko-controller.service" }
            "5" { Follow-RaspberryLog -Service "rashko-camera.service" }
            "6" { Show-Help }
            "0" { return }
            default { Write-Host "Unknown choice: $choice" -ForegroundColor Yellow }
        }
    }
}

try {
    switch ($Action) {
        "Menu" { Show-Menu }
        "Check" {
            if (Test-AllConnections) {
                exit 0
            }
            exit 1
        }
        "Shell" {
            if (Open-RaspberryShell) {
                exit 0
            }
            exit 1
        }
        "NewShell" { Open-NewRaspberryShell }
        "ControllerLogs" { Follow-RaspberryLog -Service "rashko-controller.service" }
        "CameraLogs" { Follow-RaspberryLog -Service "rashko-camera.service" }
        "Help" { Show-Help }
    }
}
catch {
    Write-Host "[ERROR] $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
