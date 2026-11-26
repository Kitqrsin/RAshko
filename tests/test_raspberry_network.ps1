param([string]$PythonPath = "py")

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $ProjectRoot "raspberry-network.ps1")

function Assert-Equal {
    param($Actual, $Expected, [string]$Label)
    if ($Actual -ne $Expected) {
        throw "ASSERT FAILED [$Label]: expected '$Expected', got '$Actual'"
    }
}

function Assert-True {
    param([bool]$Condition, [string]$Label)
    if (-not $Condition) {
        throw "ASSERT FAILED [$Label]: expected True"
    }
}

Write-Host "1. Testing IPv4 subnet math and Raspberry Pi MAC matching..." -ForegroundColor Cyan
$net24 = Get-IPv4NetworkSummary -IPAddress "192.168.100.15" -PrefixLength 24
Assert-Equal $net24.Cidr "192.168.100.0/24" "CIDR /24"
Assert-Equal $net24.BroadcastAddress "192.168.100.255" "Broadcast /24"
Assert-Equal @($net24.ScanHosts).Count 253 "ScanHosts excludes self, network, broadcast"
Assert-True (Test-IPv4InSubnet -CandidateIp "192.168.100.42" -Subnet $net24) "In /24 subnet"
Assert-True (-not (Test-IPv4InSubnet -CandidateIp "10.1.155.15" -Subnet $net24)) "Out of /24 subnet"

$net16 = Get-IPv4NetworkSummary -IPAddress "169.254.44.10" -PrefixLength 16
Assert-Equal $net16.Cidr "169.254.0.0/16" "CIDR /16 link-local"
Assert-True $net16.IsLinkLocal "Link-local flag"
Assert-Equal $net16.ScanCidr "169.254.44.0/24" "ScanCidr bounded to /24"
Assert-True (Test-IPv4InSubnet -CandidateIp "169.254.200.99" -Subnet $net16) "Any 169.254.x.x is in /16 subnet"

Assert-True (Test-RaspberryPiMac -MacAddress "DC-A6-32-11-22-33") "Pi 4 MAC dash"
Assert-True (Test-RaspberryPiMac -MacAddress "2c:cf:67:aa:bb:cc") "Pi 5 MAC colon"
Assert-True (-not (Test-RaspberryPiMac -MacAddress "A8-A1-59-BD-0C-40")) "Intel NIC MAC excluded"

Write-Host "2. Testing physical Ethernet adapter filtering on host..." -ForegroundColor Cyan
$physicalAdapters = @(Get-RashkoEthernetAdapters)
foreach ($nic in $physicalAdapters) {
    Assert-True (-not $nic.IsVirtual) "Virtual adapter excluded: $($nic.Name)"
}
$selected = Select-RashkoEthernetAdapter -AdapterName "auto"
if ($null -ne $selected) {
    Write-Host "   Selected physical adapter: $($selected.Name) ($($selected.InterfaceDescription)) -> $($selected.IPAddress)/$($selected.PrefixLength)" -ForegroundColor DarkGray
    Assert-True (-not $selected.IsVirtual) "Auto-selected adapter is physical"
}

Write-Host "3. Testing .env.windows round-trip preservation (secrets & comments intact)..." -ForegroundColor Cyan
$tempDir = Join-Path ([System.IO.Path]::GetTempPath()) ("rashko_net_test_" + [Guid]::NewGuid().ToString("N"))
New-Item -ItemType Directory -Path $tempDir | Out-Null
try {
    $tempEnv = Join-Path $tempDir ".env.windows"
    $secret = "a" * 64
    $initial = @(
        "# Header comment",
        "RASHKO_PI_HOST=10.1.155.15",
        "RASHKO_PI_PORT=5005",
        "RASHKO_SHARED_SECRET=$secret",
        "# Keep camera port",
        "RASHKO_CAMERA_PORT=8080"
    ) -join "`r`n"
    [System.IO.File]::WriteAllText($tempEnv, $initial + "`r`n", (New-Object System.Text.UTF8Encoding($false)))

    Set-RashkoDotEnvValues -Path $tempEnv -Updates @{
        RASHKO_PI_HOST = "192.168.100.42"
        RASHKO_ETHERNET_ADAPTER = "Ethernet"
    }
    $parsed = Read-RashkoDotEnv -Path $tempEnv
    Assert-Equal $parsed["RASHKO_PI_HOST"] "192.168.100.42" "Updated RASHKO_PI_HOST"
    Assert-Equal $parsed["RASHKO_SHARED_SECRET"] $secret "Preserved RASHKO_SHARED_SECRET"
    Assert-Equal $parsed["RASHKO_ETHERNET_ADAPTER"] "Ethernet" "Appended RASHKO_ETHERNET_ADAPTER"
    $rawAfter = [System.IO.File]::ReadAllText($tempEnv)
    Assert-True ($rawAfter.Contains("# Header comment") -and $rawAfter.Contains("# Keep camera port")) "Preserved comments"
    Assert-True ($rawAfter.Contains("`r`n")) "Preserved CRLF line endings"
}
finally {
    Remove-Item -LiteralPath $tempDir -Recurse -Force -ErrorAction SilentlyContinue
}

Write-Host "4. Testing signed PowerShell UDP status probe <-> Python UdpControllerServer interoperability..." -ForegroundColor Cyan
$pyScript = @"
import json, socket, sys, threading
sys.path.insert(0, r'$($ProjectRoot)\Communicator')
from pathlib import Path
from rashko_communicator.config import ControllerConfig, default_hardware_config
from rashko_communicator.controller import ArmController, UdpControllerServer
from rashko_communicator.hardware import DryRunHardware

secret = 'a' * 64
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('127.0.0.1', 0))
port = sock.getsockname()[1]
cfg = ControllerConfig(
    bind_host='127.0.0.1',
    port=port,
    shared_secret=secret,
    allow_insecure=False,
    allowed_clients=frozenset({'127.0.0.0/8'}),
    watchdog_ms=500,
    freshness_ms=5000,
    update_hz=50.0,
    max_sessions=8,
    hardware_backend='dry-run',
    servo_config_path=Path('unused.json'),
    hardware=default_hardware_config(),
)
ctrl = ArmController(cfg, DryRunHardware(cfg.hardware))
srv = UdpControllerServer(cfg, ctrl, sock)
stop = threading.Event()
t = threading.Thread(target=srv.serve_forever, args=(stop,), daemon=True)
t.start()
print(port, flush=True)
sys.stdin.readline()
stop.set()
t.join(timeout=1.0)
srv.close()
"@

$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = $PythonPath
$psi.Arguments = "-c `"$($pyScript -replace '"', '\"')`""
$psi.RedirectStandardInput = $true
$psi.RedirectStandardOutput = $true
$psi.RedirectStandardError = $true
$psi.UseShellExecute = $false
$proc = [System.Diagnostics.Process]::Start($psi)
try {
    $startup = $proc.StandardOutput.ReadLineAsync()
    if (-not $startup.Wait(5000)) {
        throw "Timed out waiting for the local dry-run UDP server."
    }
    $ephemeralPortLine = $startup.Result
    if ([string]::IsNullOrWhiteSpace($ephemeralPortLine)) {
        if ($proc.WaitForExit(1000)) {
            throw "Failed to start ephemeral Python UDP server: $($proc.StandardError.ReadToEnd())"
        }
        throw "The local dry-run UDP server did not announce a port."
    }
    $udpPort = [int]$ephemeralPortLine.Trim()
    $ack = Invoke-RashkoUdpRequest -TargetHost "127.0.0.1" -Port $udpPort -Secret ("a" * 64) -TimeoutMs 1000
    Assert-True ($null -ne $ack) "Received signed ACK from Python UdpControllerServer"
    Assert-Equal $ack.Code "OK" "Signed ACK code is OK"
    Assert-True ($ack.State -contains "ESTOP_LATCHED") "E-stop stays latched after the status probe"
    Assert-True (-not ($ack.State -contains "OUTPUTS_ENABLED")) "A status probe never enables outputs"

    # Wrong secret must return null.
    $badAck = Invoke-RashkoUdpRequest -TargetHost "127.0.0.1" -Port $udpPort -Secret ("b" * 64) -TimeoutMs 300
    Assert-True ($null -eq $badAck) "Wrong secret receives no valid ACK"

    $stopAck = Invoke-RashkoUdpRequest -TargetHost "127.0.0.1" -Port $udpPort -MessageType estop -Secret ("a" * 64)
    Assert-True ($null -ne $stopAck) "Native Windows E-stop received a signed ACK"
    Assert-Equal $stopAck.Detail "emergency stop latched" "Controller processed E-stop"
    Assert-True ($stopAck.State -contains "ESTOP_LATCHED") "Native Windows E-stop confirmed the latch"
}
finally {
    try { $proc.StandardInput.WriteLine("quit") } catch {}
    if (-not $proc.WaitForExit(2000)) {
        $proc.Kill()
    }
}

Write-Host "[PASS] All raspberry-network.ps1 tests succeeded!" -ForegroundColor Green
