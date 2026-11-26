[CmdletBinding()]
param(
    [Parameter(Position = 0)]
    [ValidateSet("Menu", "Check", "Discover", "SetHost", "Ethernet", "SyncAllowlist", "Shell", "NewShell", "ControllerLogs", "CameraLogs", "Help")]
    [string]$Action = "Menu",

    [string]$PiUser = "",
    [string]$PiHost = "",
    [string]$Adapter = "",

    [ValidateSet("", "Dhcp", "Static")]
    [string]$EthernetMode = "",
    [string]$StaticIp = "192.168.137.1",
    [int]$PrefixLength = 24,

    [switch]$NoAutoDiscover
)

Set-StrictMode -Version 2.0
$ErrorActionPreference = "Stop"

$ProjectDir = $PSScriptRoot
$EnvFile = Join-Path $ProjectDir ".env.windows"
$ComposeFile = Join-Path $ProjectDir "docker-compose.windows.yml"
$WindowsScript = Join-Path $ProjectDir "windows.ps1"
$NetworkLib = Join-Path $ProjectDir "raspberry-network.ps1"

if (-not (Test-Path -LiteralPath $NetworkLib)) {
    throw "Missing raspberry-network.ps1 in $ProjectDir"
}
. $NetworkLib

if (-not (Test-Path -LiteralPath $EnvFile)) {
    throw "Missing .env.windows. Run: powershell.exe -NoProfile -ExecutionPolicy Bypass -File .\windows.ps1 Setup"
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
            Output   = "Command not found: $FilePath"
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
        Output   = (($output | ForEach-Object { [string]$_ }) -join [Environment]::NewLine).Trim()
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

$script:EnvironmentValues = @{}
$script:ResolvedHost = ""
$script:PiPort = "5005"
$script:CameraPort = "8080"
$script:EffectivePiUser = "tnt"
$script:SharedSecret = ""
$script:AllowInsecure = $false
$script:ActiveAdapter = $null
$script:SshTarget = ""
$script:CameraHealthUrl = ""

function Refresh-SessionConfig {
    $script:EnvironmentValues = Read-RashkoDotEnv -Path $EnvFile
    $configuredHost = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_PI_HOST" -Default "auto"
    if (-not [string]::IsNullOrWhiteSpace($PiHost)) {
        $configuredHost = $PiHost.Trim()
    }
    $script:ResolvedHost = $configuredHost
    $script:PiPort = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_PI_PORT" -Default "5005"
    $script:CameraPort = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_CAMERA_PORT" -Default "8080"
    $script:SharedSecret = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_SHARED_SECRET" -Default ""
    $script:AllowInsecure = (Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_ALLOW_INSECURE" -Default "false").ToLowerInvariant() -in @("1", "true", "yes", "on")

    if (-not [string]::IsNullOrWhiteSpace($PiUser)) {
        $script:EffectivePiUser = $PiUser.Trim()
    }
    else {
        $script:EffectivePiUser = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_PI_USER" -Default "tnt"
    }

    if ($script:EffectivePiUser -notmatch '^[A-Za-z_][A-Za-z0-9_-]*$') {
        throw "Pi user contains unsupported characters: $($script:EffectivePiUser)"
    }
    if ($script:PiPort -notmatch '^\d+$' -or [int]$script:PiPort -lt 1 -or [int]$script:PiPort -gt 65535) {
        throw "RASHKO_PI_PORT must be in 1..65535"
    }
    if ($script:CameraPort -notmatch '^\d+$' -or [int]$script:CameraPort -lt 1 -or [int]$script:CameraPort -gt 65535) {
        throw "RASHKO_CAMERA_PORT must be in 1..65535"
    }

    $adapterSetting = if (-not [string]::IsNullOrWhiteSpace($Adapter)) {
        $Adapter.Trim()
    }
    else {
        Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_ETHERNET_ADAPTER" -Default "auto"
    }
    $script:ActiveAdapter = Select-RashkoEthernetAdapter -AdapterName $adapterSetting
    $script:SshTarget = "$($script:EffectivePiUser)@$($script:ResolvedHost)"
    $script:CameraHealthUrl = "http://$($script:ResolvedHost):$($script:CameraPort)/healthz"
}

function Ensure-ActivePiTarget {
    param(
        [switch]$ForceDiscover,
        [switch]$InteractiveFallback
    )

    $resolution = Resolve-RashkoPiEndpoint `
        -EnvFile $EnvFile `
        -PiHostOverride $PiHost `
        -AdapterOverride $Adapter `
        -ForceDiscover:$ForceDiscover `
        -NoAutoDiscover:$NoAutoDiscover `
        -PersistChanges:$true

    Refresh-SessionConfig
    if (Test-RashkoHostSyntax -HostValue $resolution.PiHost) {
        $script:ResolvedHost = $resolution.PiHost
        $script:SshTarget = "$($script:EffectivePiUser)@$($script:ResolvedHost)"
        $script:CameraHealthUrl = "http://$($script:ResolvedHost):$($script:CameraPort)/healthz"
        return $true
    }

    if ($InteractiveFallback -and $null -ne $resolution.DiscoveryInfo) {
        $candidates = @($resolution.DiscoveryInfo.Candidates)
        if ($candidates.Count -gt 0) {
            Write-Host ""
            Write-Host "Multiple or unverified hosts answered on Ethernet:" -ForegroundColor Yellow
            for ($i = 0; $i -lt $candidates.Count; $i++) {
                $c = $candidates[$i]
                Write-Host ("  [{0}] {1,-16} ({2})" -f ($i + 1), $c.Host, $c.Summary)
            }
            $pick = Read-Host "Select [1..$($candidates.Count)], enter an IP/hostname, or press Enter to cancel"
            if (-not [string]::IsNullOrWhiteSpace($pick)) {
                $trimmedPick = $pick.Trim()
                if ($trimmedPick -match '^\d+$' -and [int]$trimmedPick -ge 1 -and [int]$trimmedPick -le $candidates.Count) {
                    $chosenHost = [string]$candidates[[int]$trimmedPick - 1].Host
                }
                else {
                    $chosenHost = $trimmedPick
                }
                if (Test-RashkoHostSyntax -HostValue $chosenHost) {
                    Set-RashkoDotEnvValues -Path $EnvFile -Updates @{ RASHKO_PI_HOST = $chosenHost }
                    Refresh-SessionConfig
                    Write-Pass "Saved RASHKO_PI_HOST=$chosenHost to .env.windows"
                    return $true
                }
            }
        }
    }

    Write-Failure "No Raspberry Pi host is resolved yet (RASHKO_PI_HOST=$($script:ResolvedHost))." `
        "Plug in the Ethernet cable and run '.\raspberry-console.cmd Discover', or set an IP with '.\raspberry-console.cmd SetHost -PiHost <ip>'."
    return $false
}

function Invoke-DiscoverAction {
    param([switch]$Interactive)

    Refresh-SessionConfig
    $ifDesc = if ($null -ne $script:ActiveAdapter) {
        "$($script:ActiveAdapter.Name) [$($script:ActiveAdapter.Status), $($script:ActiveAdapter.LinkSpeed), $($script:ActiveAdapter.IPAddress)/$($script:ActiveAdapter.PrefixLength)]"
    }
    else {
        "none detected"
    }
    Write-Host ""
    Write-Check "Scanning Ethernet ($ifDesc) for RAshko Raspberry Pi..."
    $piHostname = Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_PI_HOSTNAME" -Default ""
    $adapterSetting = if (-not [string]::IsNullOrWhiteSpace($Adapter)) {
        $Adapter.Trim()
    }
    else {
        Get-RashkoConfigValue -Values $script:EnvironmentValues -Name "RASHKO_ETHERNET_ADAPTER" -Default "auto"
    }

    $result = Find-RashkoPi `
        -CurrentHost $script:ResolvedHost `
        -PiHostname $piHostname `
        -AdapterName $adapterSetting `
        -PiPort ([int]$script:PiPort) `
        -CameraPort ([int]$script:CameraPort) `
        -Secret $script:SharedSecret `
        -AllowInsecure:$script:AllowInsecure

    $candidates = @($result.Candidates)
    if ($candidates.Count -gt 0) {
        Write-Host ""
        Write-Host "Discovered Ethernet hosts:" -ForegroundColor White
        for ($i = 0; $i -lt $candidates.Count; $i++) {
            $item = $candidates[$i]
            $tag = if ($null -ne $result.Selected -and $result.Selected.Host -eq $item.Host) { "*" } else { " " }
            Write-Host ("  {0} [{1}] {2,-16} score={3,-3} {4}" -f $tag, ($i + 1), $item.Host, $item.Score, $item.Summary)
        }
    }

    if ($null -ne $result.Selected) {
        $updates = @{ RASHKO_PI_HOST = [string]$result.Selected.Host }
        if ($null -ne $result.Adapter -and $null -ne $result.Adapter.Subnet) {
            $updates["RASHKO_ETHERNET_SUBNET"] = $result.Adapter.Subnet.Cidr
        }
        Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
        Refresh-SessionConfig
        Write-Pass "Updated .env.windows -> RASHKO_PI_HOST=$($script:ResolvedHost)"
        return $true
    }

    if ($Interactive -and $candidates.Count -gt 0) {
        $choice = Read-Host "No signed controller replied. Pick [1..$($candidates.Count)] to explicitly set a known Pi for SSH, or press Enter to skip"
        if (-not [string]::IsNullOrWhiteSpace($choice) -and $choice.Trim() -match '^\d+$') {
            $idx = [int]$choice.Trim()
            if ($idx -ge 1 -and $idx -le $candidates.Count) {
                $chosen = [string]$candidates[$idx - 1].Host
                Set-RashkoDotEnvValues -Path $EnvFile -Updates @{ RASHKO_PI_HOST = $chosen }
                Refresh-SessionConfig
                Write-Pass "Saved RASHKO_PI_HOST=$chosen to .env.windows"
                return $true
            }
        }
    }

    Write-Failure "Could not automatically verify a Raspberry Pi on $ifDesc." `
        "Check that the Pi is powered and connected via Ethernet, or use option 4 to switch Windows Ethernet between DHCP and Direct-Cable Static IP."
    return $false
}

function Invoke-SetHostAction {
    Refresh-SessionConfig
    $targetHost = $PiHost
    $targetAdapter = $Adapter

    if ([string]::IsNullOrWhiteSpace($targetHost) -and [string]::IsNullOrWhiteSpace($targetAdapter)) {
        Write-Host ""
        Write-Host "Current .env.windows settings:" -ForegroundColor White
        Write-Host "  RASHKO_PI_HOST          = $($script:ResolvedHost)"
        Write-Host "  RASHKO_PI_USER          = $($script:EffectivePiUser)"
        Write-Host "  RASHKO_ETHERNET_ADAPTER = $(Get-RashkoConfigValue -Values $script:EnvironmentValues -Name 'RASHKO_ETHERNET_ADAPTER' -Default 'auto')"
        Write-Host "  RASHKO_PI_HOSTNAME      = $(Get-RashkoConfigValue -Values $script:EnvironmentValues -Name 'RASHKO_PI_HOSTNAME' -Default 'raspberrypi.local')"
        Write-Host ""
        $inputHost = Read-Host "Enter Pi IPv4, hostname (e.g. raspberrypi.local), or 'auto' [leave blank to keep $($script:ResolvedHost)]"
        if (-not [string]::IsNullOrWhiteSpace($inputHost)) {
            $targetHost = $inputHost.Trim()
        }
        $inputUser = Read-Host "Enter Pi SSH username [leave blank to keep $($script:EffectivePiUser)]"
        if (-not [string]::IsNullOrWhiteSpace($inputUser)) {
            $PiUser = $inputUser.Trim()
        }
        $physical = @(Get-RashkoEthernetAdapters)
        if ($physical.Count -gt 0) {
            Write-Host "Available physical adapters: $((@($physical | ForEach-Object { ""$($_.Name) [$($_.IPAddress)]"" })) -join ', ')" -ForegroundColor DarkGray
        }
        $inputAdapter = Read-Host "Enter Windows adapter name or 'auto' [leave blank to keep current]"
        if (-not [string]::IsNullOrWhiteSpace($inputAdapter)) {
            $targetAdapter = $inputAdapter.Trim()
        }
    }

    $updates = @{}
    if (-not [string]::IsNullOrWhiteSpace($targetHost)) {
        if (-not (Test-RashkoHostSyntax -HostValue $targetHost -AllowAuto)) {
            throw "Invalid Pi host: $targetHost"
        }
        $updates["RASHKO_PI_HOST"] = $targetHost.Trim()
    }
    if (-not [string]::IsNullOrWhiteSpace($PiUser)) {
        if ($PiUser.Trim() -notmatch '^[A-Za-z_][A-Za-z0-9_-]*$') {
            throw "Invalid Pi user: $PiUser"
        }
        $updates["RASHKO_PI_USER"] = $PiUser.Trim()
    }
    if (-not [string]::IsNullOrWhiteSpace($targetAdapter)) {
        if ($targetAdapter.Trim().ToLowerInvariant() -ne "auto") {
            [void](Select-RashkoEthernetAdapter -AdapterName $targetAdapter.Trim())
        }
        $updates["RASHKO_ETHERNET_ADAPTER"] = $targetAdapter.Trim()
    }

    if ($updates.Count -eq 0) {
        Write-Host "No changes made." -ForegroundColor Yellow
        return $true
    }

    Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
    Refresh-SessionConfig
    Write-Pass "Updated .env.windows (Pi: $($script:SshTarget), Adapter: $(if ($script:ActiveAdapter) { $script:ActiveAdapter.Name } else { 'auto' }))"
    return $true
}

function Invoke-EthernetConfigAction {
    Refresh-SessionConfig
    $adapters = @(Get-RashkoEthernetAdapters)
    if ($adapters.Count -eq 0) {
        Write-Failure "No physical network adapters detected."
        return $false
    }

    $chosenAdapter = $script:ActiveAdapter
    if (-not [string]::IsNullOrWhiteSpace($EthernetMode)) {
        $targetName = if (-not [string]::IsNullOrWhiteSpace($Adapter)) { $Adapter } elseif ($chosenAdapter) { $chosenAdapter.Name } else { $adapters[0].Name }
        $updated = Set-RashkoEthernetMode -AdapterName $targetName -Mode $EthernetMode -StaticIPAddress $StaticIp -PrefixLength $PrefixLength
        $updates = @{ RASHKO_ETHERNET_ADAPTER = $updated.Name }
        if ($null -ne $updated.Subnet) {
            $updates["RASHKO_ETHERNET_SUBNET"] = $updated.Subnet.Cidr
        }
        Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
        Refresh-SessionConfig
        Write-Pass "Configured $($updated.Name) -> $EthernetMode ($($updated.IPAddress)/$($updated.PrefixLength))"
        return $true
    }

    Write-Host ""
    Write-Host "Physical Windows network adapters:" -ForegroundColor White
    for ($i = 0; $i -lt $adapters.Count; $i++) {
        $a = $adapters[$i]
        $kind = if ($a.IsWiredEthernet) { "Wired Ethernet" } else { "Wireless" }
        Write-Host ("  [{0}] {1,-18} {2,-14} Status={3,-12} Mode={4,-8} IPv4={5}/{6}" -f ($i + 1), $a.Name, "($kind)", $a.Status, $a.Dhcp, $a.IPAddress, $a.PrefixLength)
    }
    Write-Host ""
    Write-Host "Tip: With DHCP enabled, both router LANs and direct PC-to-Pi cables (169.254.x.x link-local + .local mDNS) work automatically." -ForegroundColor DarkGray
    Write-Host "     Use Direct-Cable Static IP only if your Pi has a fixed static Ethernet IP on e.g. 192.168.137.x/24." -ForegroundColor DarkGray

    $pickIdx = Read-Host "Choose adapter [1..$($adapters.Count), Enter=1]"
    $selectedIndex = 0
    if (-not [string]::IsNullOrWhiteSpace($pickIdx)) {
        if ($pickIdx.Trim() -notmatch '^\d+$' -or [int]$pickIdx.Trim() -lt 1 -or [int]$pickIdx.Trim() -gt $adapters.Count) {
            Write-Failure "Invalid adapter choice: $pickIdx"
            return $false
        }
        $selectedIndex = [int]$pickIdx.Trim() - 1
    }
    $targetAdapter = $adapters[$selectedIndex]

    Write-Host ""
    Write-Host "Choose action for '$($targetAdapter.Name)':" -ForegroundColor White
    Write-Host "  1. Use this adapter for RAshko discovery (keep current IPv4 settings)"
    Write-Host "  2. Switch '$($targetAdapter.Name)' to DHCP (Router / Switch / Link-Local automatic)"
    Write-Host "  3. Switch '$($targetAdapter.Name)' to Direct-Cable Static IP (no default gateway; keeps Wi-Fi internet)"
    Write-Host "  0. Cancel"
    $modeChoice = (Read-Host "Choose").Trim()

    switch ($modeChoice) {
        "1" {
            $updates = @{ RASHKO_ETHERNET_ADAPTER = $targetAdapter.Name }
            if ($null -ne $targetAdapter.Subnet) {
                $updates["RASHKO_ETHERNET_SUBNET"] = $targetAdapter.Subnet.Cidr
            }
            Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
            Refresh-SessionConfig
            Write-Pass "Pinned RASHKO_ETHERNET_ADAPTER=$($targetAdapter.Name)"
            return $true
        }
        "2" {
            $updated = Set-RashkoEthernetMode -AdapterName $targetAdapter.Name -Mode "Dhcp"
            $updates = @{ RASHKO_ETHERNET_ADAPTER = $updated.Name; RASHKO_PI_HOST = "auto" }
            if ($null -ne $updated.Subnet) {
                $updates["RASHKO_ETHERNET_SUBNET"] = $updated.Subnet.Cidr
            }
            Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
            Refresh-SessionConfig
            Write-Pass "Switched $($updated.Name) to DHCP ($($updated.IPAddress)/$($updated.PrefixLength))"
            return $true
        }
        "3" {
            $ipInput = Read-Host "Windows static IPv4 on '$($targetAdapter.Name)' [Enter=$StaticIp]"
            $chosenIp = if ([string]::IsNullOrWhiteSpace($ipInput)) { $StaticIp } else { $ipInput.Trim() }
            $prefixInput = Read-Host "Subnet prefix length [Enter=$PrefixLength]"
            $chosenPrefix = if ([string]::IsNullOrWhiteSpace($prefixInput)) { $PrefixLength } else { [int]$prefixInput.Trim() }
            $updated = Set-RashkoEthernetMode -AdapterName $targetAdapter.Name -Mode "Static" -StaticIPAddress $chosenIp -PrefixLength $chosenPrefix
            $updates = @{ RASHKO_ETHERNET_ADAPTER = $updated.Name; RASHKO_PI_HOST = "auto" }
            if ($null -ne $updated.Subnet) {
                $updates["RASHKO_ETHERNET_SUBNET"] = $updated.Subnet.Cidr
            }
            Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
            Refresh-SessionConfig
            Write-Pass "Configured $($updated.Name) static IPv4 -> $($updated.IPAddress)/$($updated.PrefixLength)"
            return $true
        }
        default {
            return $true
        }
    }
}

function Sync-RaspberryAllowlist {
    if (-not (Ensure-ActivePiTarget -InteractiveFallback)) {
        return $false
    }
    if ($null -eq $script:ActiveAdapter -or [string]::IsNullOrWhiteSpace($script:ActiveAdapter.IPAddress)) {
        Write-Failure "Cannot determine active Windows Ethernet IPv4 address to authorize."
        return $false
    }

    $laptopIp = $script:ActiveAdapter.IPAddress
    $subnetCidr = if ($null -ne $script:ActiveAdapter.Subnet) { $script:ActiveAdapter.Subnet.ScanCidr } else { "${laptopIp}/32" }

    Write-Check "Updating RASHKO_ALLOWED_CLIENTS on $($script:SshTarget) for Ethernet $laptopIp ($subnetCidr)"
    Write-Host "       SSH/sudo may prompt for $($script:SshTarget)'s password. The signing key stays on the Pi." -ForegroundColor DarkGray

    $remotePython = @"
import pathlib, subprocess, sys
env_path = pathlib.Path('/etc/rashko-controller.env')
if not env_path.exists():
    print('MISSING_ENV_FILE')
    sys.exit(2)
supports_cidr = False
venv_py = pathlib.Path('/opt/rashko-communicator/.venv/bin/python')
if venv_py.exists():
    probe = subprocess.run(
        [str(venv_py), '-c', "from rashko_communicator.config import _parse_allowed_clients; _parse_allowed_clients('192.168.100.0/24')"],
        capture_output=True, text=True
    )
    supports_cidr = (probe.returncode == 0)
value = '$subnetCidr' if supports_cidr else '$laptopIp'
lines = env_path.read_text(encoding='utf-8').splitlines()
out = []
replaced = False
for line in lines:
    stripped = line.strip()
    if stripped.startswith('RASHKO_ALLOWED_CLIENTS='):
        out.append(f'RASHKO_ALLOWED_CLIENTS={value}')
        replaced = True
    else:
        out.append(line)
if not replaced:
    out.append(f'RASHKO_ALLOWED_CLIENTS={value}')
env_path.write_text('\n'.join(out) + '\n', encoding='utf-8')
subprocess.run(['systemctl', 'restart', 'rashko-controller.service'], check=True)
active = subprocess.run(['systemctl', 'is-active', 'rashko-controller.service'], capture_output=True, text=True, check=True).stdout.strip()
print(f'UPDATED:{value}:SERVICE={active}:CIDR={supports_cidr}')
"@
    $encodedPy = [Convert]::ToBase64String([System.Text.Encoding]::UTF8.GetBytes($remotePython))
    $remoteCmd = "sudo python3 -c `"import base64; exec(base64.b64decode('$encodedPy').decode('utf-8'))`""

    & ssh -t -o StrictHostKeyChecking=accept-new $script:SshTarget $remoteCmd
    if ($LASTEXITCODE -ne 0) {
        Write-Failure "Could not update RASHKO_ALLOWED_CLIENTS over SSH (exit code $LASTEXITCODE)."
        return $false
    }
    Write-Pass "Raspberry Pi controller allowlist synchronized and rashko-controller.service restarted safely (OE HIGH, E-stop latched)."
    return $true
}

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
    Write-Check "SSH authentication and Raspberry Pi services ($($script:SshTarget))"
    Write-Host "       SSH may ask for the password for $($script:SshTarget)." -ForegroundColor DarkGray
    $remoteCommand = "systemctl is-active rashko-controller.service rashko-camera.service"
    $result = Invoke-ExternalCommand -FilePath "ssh" -ArgumentList @(
        "-o", "ConnectTimeout=7",
        "-o", "StrictHostKeyChecking=accept-new",
        "-o", "ServerAliveInterval=5",
        "-o", "ServerAliveCountMax=1",
        $script:SshTarget,
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
    Write-Host "       Use menu option 6 to open the Pi shell and repair the service." -ForegroundColor DarkYellow
    return $false
}

function Test-CameraHealth {
    Write-Check "Camera Module HTTP health endpoint ($($script:CameraHealthUrl))"
    try {
        $health = Invoke-RestMethod -Uri $script:CameraHealthUrl -TimeoutSec 7
        if ($health.status -eq "ok" -and $health.frame_ready -eq $true) {
            Write-Pass "Camera is ready ($($health.backend), $($health.width)x$($health.height) at $($health.fps) FPS)"
            return $true
        }
        Write-Failure "Camera answered but is not ready" ($health | Out-String)
    }
    catch {
        Write-Failure "Camera health request failed: $($script:CameraHealthUrl)" $_.Exception.Message
    }
    return $false
}

function Test-DirectHostUdp {
    Write-Check "Direct Windows-to-Pi authenticated UDP status probe ($($script:ResolvedHost):$($script:PiPort))"
    $ack = Invoke-RashkoUdpRequest `
        -TargetHost $script:ResolvedHost `
        -Port ([int]$script:PiPort) `
        -Secret $script:SharedSecret `
        -AllowInsecure:$script:AllowInsecure `
        -TimeoutMs 1200
    if ($null -ne $ack -and $ack.Code -eq "OK") {
        Write-Pass "Direct UDP status probe received OK ($($ack.State -join ', '))"
        return $true
    }
    Write-Failure "Direct Windows UDP status probe timed out or was rejected"
    return $false
}

function Test-AuthenticatedUdp {
    Write-Check "Docker container authenticated UDP status probe to $($script:ResolvedHost):$($script:PiPort)"
    $result = Invoke-ExternalCommand -FilePath "powershell.exe" -ArgumentList @(
        "-NoProfile",
        "-ExecutionPolicy", "Bypass",
        "-File", $WindowsScript,
        "Ping",
        "-NoAutoDiscover"
    )
    if ($result.ExitCode -eq 0 -and $result.Output -match '"code"\s*:\s*"OK"') {
        Write-Pass "Container UDP status probe received an OK acknowledgement"
        $ackLine = @($result.Output -split "`r?`n" | Where-Object { $_.Trim().StartsWith("{") }) | Select-Object -Last 1
        if ($ackLine) {
            Write-Host "       $ackLine" -ForegroundColor DarkGray
        }
        return $true
    }
    Write-Failure "Container authenticated UDP status probe failed" $result.Output
    return $false
}

function Test-AllConnections {
    if (-not (Ensure-ActivePiTarget -InteractiveFallback)) {
        return $false
    }

    $ethBanner = if ($null -ne $script:ActiveAdapter) {
        "$($script:ActiveAdapter.Name) ($($script:ActiveAdapter.IPAddress)/$($script:ActiveAdapter.PrefixLength))"
    }
    else {
        "none"
    }

    Write-Host ""
    Write-Host "RAshko Raspberry Pi connection gate" -ForegroundColor White
    Write-Host "Windows: $env:COMPUTERNAME [$ethBanner]   Pi: $($script:SshTarget)   UDP: $($script:PiPort)   Camera: $($script:CameraPort)" -ForegroundColor DarkGray
    Write-Host "The UDP check reads status without renewing the motion watchdog." -ForegroundColor Yellow
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
    $directUdpReady = Test-DirectHostUdp

    $udpReady = $false
    if ($containerReady) {
        $udpReady = Test-AuthenticatedUdp
    }
    else {
        Write-Failure "Container UDP status probe skipped" "The RAshko container must be running."
    }

    if (($sshReady -or $cameraReady) -and -not $directUdpReady) {
        Write-Host ""
        Write-Host "[HINT] SSH/Camera reached $($script:ResolvedHost), but UDP :$($script:PiPort) did not answer." -ForegroundColor Yellow
        Write-Host "       If your Windows Ethernet IP changed, run '.\raspberry-console.cmd SyncAllowlist' (Menu option 5)" -ForegroundColor Yellow
        Write-Host "       to update RASHKO_ALLOWED_CLIENTS on the Pi, and check UFW rules if enabled." -ForegroundColor Yellow
    }

    $allReady = $dockerReady -and $containerReady -and $sshReady -and $cameraReady -and $directUdpReady -and $udpReady
    Write-Host ""
    if ($allReady) {
        Write-Host "[READY] Every Windows-to-Raspberry connection passed." -ForegroundColor Green
        Write-Host "You may continue with the Pi shell or the commissioning procedure." -ForegroundColor Green
        return $true
    }

    Write-Host "[BLOCKED] One or more connections failed. Motor commissioning must not continue." -ForegroundColor Red
    Write-Host "Use Discover, SyncAllowlist, Pi shell, or logs from this menu, repair the failure, then run Check again." -ForegroundColor Yellow
    return $false
}

function Open-RaspberryShell {
    if (-not (Ensure-ActivePiTarget -InteractiveFallback)) {
        return $false
    }
    Write-Host ""
    Write-Host "Opening an interactive Linux shell on $($script:SshTarget)" -ForegroundColor Cyan
    Write-Host "Type commands normally. Use Ctrl+C to stop a running command; type exit or press Ctrl+D to return." -ForegroundColor Yellow
    Write-Host "Keep servo power disconnected until the connection gate passes." -ForegroundColor Yellow
    & ssh -o StrictHostKeyChecking=accept-new $script:SshTarget
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
        Write-Failure "SSH session ended with exit code $LASTEXITCODE"
        return $false
    }
    Write-Pass "Returned from Raspberry Pi shell"
    return $true
}

function Open-NewRaspberryShell {
    if (-not (Ensure-ActivePiTarget -InteractiveFallback)) {
        return $false
    }
    $command = "Write-Host 'RAshko Raspberry Pi shell ($($script:SshTarget)): type exit to close.' -ForegroundColor Cyan; ssh -o StrictHostKeyChecking=accept-new $($script:SshTarget)"
    $encodedCommand = [Convert]::ToBase64String([Text.Encoding]::Unicode.GetBytes($command))
    Start-Process -FilePath "powershell.exe" -ArgumentList @(
        "-NoLogo",
        "-NoProfile",
        "-NoExit",
        "-EncodedCommand",
        $encodedCommand
    )
    Write-Pass "Opened a separate PowerShell window for $($script:SshTarget)"
    return $true
}

function Follow-RaspberryLog {
    param(
        [Parameter(Mandatory = $true)]
        [ValidateSet("rashko-controller.service", "rashko-camera.service")]
        [string]$Service
    )

    if (-not (Ensure-ActivePiTarget -InteractiveFallback)) {
        return
    }
    Write-Host "Press Ctrl+C to stop following $Service on $($script:SshTarget) and return." -ForegroundColor Yellow
    & ssh -t -o StrictHostKeyChecking=accept-new $script:SshTarget "sudo journalctl -u $Service -f"
    if ($LASTEXITCODE -ne 0 -and $LASTEXITCODE -ne 130) {
        Write-Failure "Log session ended with exit code $LASTEXITCODE"
    }
}

function Show-Help {
    Write-Host @"
RAshko Raspberry Pi connection console (with automatic Ethernet discovery)

Double-click: raspberry-console.cmd

Execution-policy-safe actions:
  .\raspberry-console.cmd Check            Auto-find Pi on Ethernet if needed, then test Docker, SSH, camera, and UDP
  .\raspberry-console.cmd Discover         Scan active Ethernet adapter (mDNS, ARP/Pi-MAC, /24) and update .env.windows
  .\raspberry-console.cmd SetHost          Set Pi IP/hostname/auto or preferred Ethernet adapter interactively or via flags
  .\raspberry-console.cmd Ethernet         Inspect physical Ethernet adapters or switch DHCP <-> Direct-Cable Static IP
  .\raspberry-console.cmd SyncAllowlist    Authorize current Windows Ethernet IP/subnet in Pi /etc/rashko-controller.env
  .\raspberry-console.cmd Shell            Open the Pi Linux shell in the current window (auto-discovers IP if moved)
  .\raspberry-console.cmd NewShell         Open the Pi Linux shell in a new PowerShell window
  .\raspberry-console.cmd ControllerLogs   Follow the Pi controller journal; Ctrl+C returns
  .\raspberry-console.cmd CameraLogs       Follow the Pi camera journal; Ctrl+C returns

Optional overrides on any command:
  .\raspberry-console.cmd Shell -PiHost raspberrypi.local -PiUser tnt
  .\raspberry-console.cmd Discover -Adapter "Ethernet"
  .\raspberry-console.cmd Ethernet -Adapter "Ethernet" -EthernetMode Static -StaticIp 192.168.137.1 -PrefixLength 24
  .\raspberry-console.cmd Ethernet -Adapter "Ethernet" -EthernetMode Dhcp

Discovery and checks never reset E-stop, enable outputs, or send joint commands.
"@
}

function Show-Menu {
    $lastCheckPassed = $false
    while ($true) {
        Refresh-SessionConfig
        $ethSummary = if ($null -ne $script:ActiveAdapter) {
            "$($script:ActiveAdapter.Name) [$($script:ActiveAdapter.Status), $($script:ActiveAdapter.Dhcp), $($script:ActiveAdapter.IPAddress)/$($script:ActiveAdapter.PrefixLength)]"
        }
        else {
            "No physical adapter detected"
        }

        Write-Host ""
        Write-Host "RAshko Raspberry connection console" -ForegroundColor White
        Write-Host "  Ethernet : $ethSummary" -ForegroundColor DarkCyan
        Write-Host "  Pi Target: $($script:SshTarget)  (UDP :$($script:PiPort), Camera :$($script:CameraPort))" -ForegroundColor DarkCyan
        Write-Host "  ------------------------------------------------------------" -ForegroundColor DarkGray
        Write-Host "  1. Test every connection (auto-discovers Pi if IP changed)"
        Write-Host "  2. Auto-discover Raspberry Pi on Ethernet & save IP"
        Write-Host "  3. Set Pi IP / hostname or Ethernet adapter"
        Write-Host "  4. Adjust Windows Ethernet IPv4 (DHCP <-> Direct-Cable Static IP)"
        Write-Host "  5. Sync Windows Ethernet IP/subnet to Pi allowlist (SSH)"
        Write-Host "  6. Open Pi shell in this window"
        Write-Host "  7. Open Pi shell in a new PowerShell window"
        Write-Host "  8. Follow controller logs"
        Write-Host "  9. Follow camera logs"
        Write-Host "  H. Show help"
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
        switch ($choice.ToUpperInvariant()) {
            "1" { $lastCheckPassed = Test-AllConnections }
            "2" { [void](Invoke-DiscoverAction -Interactive) }
            "3" { [void](Invoke-SetHostAction) }
            "4" { [void](Invoke-EthernetConfigAction) }
            "5" { [void](Sync-RaspberryAllowlist) }
            "6" { [void](Open-RaspberryShell) }
            "7" { [void](Open-NewRaspberryShell) }
            "8" { Follow-RaspberryLog -Service "rashko-controller.service" }
            "9" { Follow-RaspberryLog -Service "rashko-camera.service" }
            "H" { Show-Help }
            "0" { return }
            default { Write-Host "Unknown choice: $choice" -ForegroundColor Yellow }
        }
    }
}

try {
    Refresh-SessionConfig
    switch ($Action) {
        "Menu" { Show-Menu }
        "Check" {
            if (Test-AllConnections) { exit 0 }
            exit 1
        }
        "Discover" {
            if (Invoke-DiscoverAction) { exit 0 }
            exit 1
        }
        "SetHost" {
            if (Invoke-SetHostAction) { exit 0 }
            exit 1
        }
        "Ethernet" {
            if (Invoke-EthernetConfigAction) { exit 0 }
            exit 1
        }
        "SyncAllowlist" {
            if (Sync-RaspberryAllowlist) { exit 0 }
            exit 1
        }
        "Shell" {
            if (Open-RaspberryShell) { exit 0 }
            exit 1
        }
        "NewShell" {
            if (Open-NewRaspberryShell) { exit 0 }
            exit 1
        }
        "ControllerLogs" { Follow-RaspberryLog -Service "rashko-controller.service" }
        "CameraLogs" { Follow-RaspberryLog -Service "rashko-camera.service" }
        "Help" { Show-Help }
    }
}
catch {
    Write-Host "[ERROR] $($_.Exception.Message)" -ForegroundColor Red
    exit 1
}
