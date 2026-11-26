Set-StrictMode -Version 2.0

$script:RaspberryPiMacPrefixes = @(
    "28-CD-C1",
    "2C-CF-67",
    "3A-35-41",
    "88-A2-9E",
    "B8-27-EB",
    "D8-3A-DD",
    "DC-A6-32",
    "E4-5F-01"
)

$script:VirtualAdapterPattern = '(?i)(^vEthernet|^Loopback|WSL|Hyper-V|VMware|VirtualBox|ZeroTier|Radmin|Hamachi|Tailscale|WireGuard|TAP-Windows|Wintun|Bluetooth|NPcap|Terodo|isatap)'
$script:WirelessAdapterPattern = '(?i)(Wi-Fi|Wireless|802\.11|WLAN)'
$script:DefaultPiMdnsNames = @("rashko.local", "raspberrypi.local", "puppypi.local")

function Read-RashkoDotEnv {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path
    )

    $values = @{}
    if (-not (Test-Path -LiteralPath $Path)) {
        return $values
    }

    foreach ($originalLine in @(Get-Content -LiteralPath $Path)) {
        $line = ([string]$originalLine).Trim()
        if (-not $line -or $line.StartsWith("#")) {
            continue
        }
        if ($line.StartsWith("export ")) {
            $line = $line.Substring(7).TrimStart()
        }
        $separator = $line.IndexOf("=")
        if ($separator -le 0) {
            continue
        }
        $key = $line.Substring(0, $separator).Trim()
        $value = $line.Substring($separator + 1).Trim()
        if ($value.Length -ge 2 -and (
            ($value.StartsWith('"') -and $value.EndsWith('"')) -or
            ($value.StartsWith("'") -and $value.EndsWith("'"))
        )) {
            $value = $value.Substring(1, $value.Length - 2)
        }
        $values[$key] = $value
    }
    return $values
}

function Get-RashkoConfigValue {
    param(
        [Parameter(Mandatory = $true)]
        [hashtable]$Values,
        [Parameter(Mandatory = $true)]
        [string]$Name,
        [string]$Default = ""
    )

    if ($Values.ContainsKey($Name) -and -not [string]::IsNullOrWhiteSpace([string]$Values[$Name])) {
        return ([string]$Values[$Name]).Trim()
    }
    return $Default
}

function Test-RashkoHostSyntax {
    param(
        [AllowEmptyString()]
        [string]$HostValue,
        [switch]$AllowAuto
    )

    if ([string]::IsNullOrWhiteSpace($HostValue)) {
        return $false
    }
    $trimmed = $HostValue.Trim()
    if ($AllowAuto -and $trimmed.ToLowerInvariant() -eq "auto") {
        return $true
    }
    if ($trimmed.ToLowerInvariant() -eq "auto") {
        return $false
    }
    return ($trimmed -match '^[A-Za-z0-9][A-Za-z0-9._-]*$')
}

function Set-RashkoDotEnvValues {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Path,
        [Parameter(Mandatory = $true)]
        [hashtable]$Updates
    )

    foreach ($entry in $Updates.GetEnumerator()) {
        $key = [string]$entry.Key
        $val = [string]$entry.Value
        if ($key -notmatch '^[A-Za-z_][A-Za-z0-9_]*$') {
            throw "Invalid .env key name: $key"
        }
        if ($val -match "[\r\n]") {
            throw "Invalid newline in value for $key"
        }
        if ($key -eq "RASHKO_PI_HOST" -and -not (Test-RashkoHostSyntax -HostValue $val -AllowAuto)) {
            throw "Invalid RASHKO_PI_HOST value: $val"
        }
    }

    $newline = "`r`n"
    $lines = New-Object System.Collections.Generic.List[string]
    if (Test-Path -LiteralPath $Path) {
        $rawText = [System.IO.File]::ReadAllText($Path)
        if ($rawText.Contains("`n") -and -not $rawText.Contains("`r`n")) {
            $newline = "`n"
        }
        if ($rawText.Length -gt 0) {
            $splitLines = $rawText -split "\r?\n"
            if ($splitLines.Count -gt 0 -and $splitLines[$splitLines.Count - 1] -eq "" -and $rawText.EndsWith("`n")) {
                for ($i = 0; $i -lt ($splitLines.Count - 1); $i++) {
                    $lines.Add([string]$splitLines[$i])
                }
            }
            else {
                foreach ($item in $splitLines) {
                    $lines.Add([string]$item)
                }
            }
        }
    }

    $updatedKeys = @{}
    for ($i = 0; $i -lt $lines.Count; $i++) {
        $trimmed = $lines[$i].Trim()
        if (-not $trimmed -or $trimmed.StartsWith("#")) {
            continue
        }
        $candidate = $trimmed
        $prefix = ""
        if ($candidate.StartsWith("export ")) {
            $prefix = "export "
            $candidate = $candidate.Substring(7).TrimStart()
        }
        $sep = $candidate.IndexOf("=")
        if ($sep -le 0) {
            continue
        }
        $existingKey = $candidate.Substring(0, $sep).Trim()
        if ($Updates.ContainsKey($existingKey)) {
            $lines[$i] = "${prefix}${existingKey}=$($Updates[$existingKey])"
            $updatedKeys[$existingKey] = $true
        }
    }

    foreach ($key in @($Updates.Keys | Sort-Object)) {
        if (-not $updatedKeys.ContainsKey($key)) {
            $lines.Add("${key}=$($Updates[$key])")
        }
    }

    $finalContent = ($lines -join $newline) + $newline
    $utf8NoBom = New-Object System.Text.UTF8Encoding($false)
    $fullPath = [System.IO.Path]::GetFullPath($Path)
    $tempPath = Join-Path ([System.IO.Path]::GetDirectoryName($fullPath)) (
        "..env.windows." + [Guid]::NewGuid().ToString("N") + ".tmp"
    )
    try {
        [System.IO.File]::WriteAllText($tempPath, $finalContent, $utf8NoBom)
        if ([System.IO.File]::Exists($fullPath)) {
            [System.IO.File]::Replace($tempPath, $fullPath, [NullString]::Value)
        }
        else {
            [System.IO.File]::Move($tempPath, $fullPath)
        }
    }
    finally {
        [System.IO.File]::Delete($tempPath)
    }
}

function ConvertTo-UInt32IPv4 {
    param(
        [Parameter(Mandatory = $true)]
        [string]$IPAddress
    )

    $parsed = $null
    if (-not [System.Net.IPAddress]::TryParse($IPAddress.Trim(), [ref]$parsed)) {
        throw "Not a valid IP address: $IPAddress"
    }
    if ($parsed.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) {
        throw "Not an IPv4 address: $IPAddress"
    }
    $bytes = $parsed.GetAddressBytes()
    return (
        ([uint32]$bytes[0] -shl 24) -bor
        ([uint32]$bytes[1] -shl 16) -bor
        ([uint32]$bytes[2] -shl 8) -bor
        ([uint32]$bytes[3])
    )
}

function ConvertFrom-UInt32IPv4 {
    param(
        [Parameter(Mandatory = $true)]
        [uint32]$Value
    )

    $b0 = [byte](($Value -shr 24) -band 0xFF)
    $b1 = [byte](($Value -shr 16) -band 0xFF)
    $b2 = [byte](($Value -shr 8) -band 0xFF)
    $b3 = [byte]($Value -band 0xFF)
    return "$b0.$b1.$b2.$b3"
}

function Get-IPv4NetworkSummary {
    param(
        [Parameter(Mandatory = $true)]
        [string]$IPAddress,
        [Parameter(Mandatory = $true)]
        [int]$PrefixLength
    )

    if ($PrefixLength -lt 0 -or $PrefixLength -gt 32) {
        throw "PrefixLength must be between 0 and 32"
    }
    $ipInt = ConvertTo-UInt32IPv4 -IPAddress $IPAddress
    $allOnes = [uint64]([uint32]::MaxValue)
    if ($PrefixLength -eq 0) {
        $mask = [uint32]0
    }
    else {
        $mask = [uint32](($allOnes -shl (32 - $PrefixLength)) -band $allOnes)
    }
    $networkInt = [uint32]($ipInt -band $mask)
    $wildcard = [uint32](([uint64](-bnot $mask)) -band $allOnes)
    $broadcastInt = [uint32]($networkInt -bor $wildcard)
    $networkAddress = ConvertFrom-UInt32IPv4 -Value $networkInt
    $broadcastAddress = ConvertFrom-UInt32IPv4 -Value $broadcastInt

    # For large subnets (/16 link-local or /8), bound active TCP host sweeps to the local /24 neighborhood.
    $scanPrefix = if ($PrefixLength -lt 24) { 24 } else { $PrefixLength }
    $scanMask = [uint32](($allOnes -shl (32 - $scanPrefix)) -band $allOnes)
    $scanBase = [uint32]($ipInt -band $scanMask)
    $scanWildcard = [uint32](([uint64](-bnot $scanMask)) -band $allOnes)
    $scanBroadcast = [uint32]($scanBase -bor $scanWildcard)

    $scanHosts = New-Object System.Collections.Generic.List[string]
    if ($scanPrefix -le 30) {
        for ($cur = $scanBase + 1; $cur -lt $scanBroadcast; $cur++) {
            if ($cur -ne $ipInt) {
                $scanHosts.Add((ConvertFrom-UInt32IPv4 -Value ([uint32]$cur)))
            }
        }
    }
    elseif ($scanPrefix -eq 31) {
        for ($cur = $scanBase; $cur -le $scanBroadcast; $cur++) {
            if ($cur -ne $ipInt) {
                $scanHosts.Add((ConvertFrom-UInt32IPv4 -Value ([uint32]$cur)))
            }
        }
    }

    return [pscustomobject]@{
        IPAddress        = $IPAddress
        PrefixLength     = $PrefixLength
        NetworkAddress   = $networkAddress
        BroadcastAddress = $broadcastAddress
        Cidr             = "${networkAddress}/${PrefixLength}"
        ScanCidr         = "$(ConvertFrom-UInt32IPv4 -Value $scanBase)/${scanPrefix}"
        IsLinkLocal      = ($IPAddress.StartsWith("169.254."))
        NetworkInt       = $networkInt
        MaskInt          = $mask
        ScanHosts        = @($scanHosts.ToArray())
    }
}

function Test-IPv4InSubnet {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CandidateIp,
        [Parameter(Mandatory = $true)]
        [pscustomobject]$Subnet
    )

    $parsed = $null
    if (-not [System.Net.IPAddress]::TryParse($CandidateIp.Trim(), [ref]$parsed)) {
        return $false
    }
    if ($parsed.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) {
        return $false
    }
    $candidateInt = ConvertTo-UInt32IPv4 -IPAddress $CandidateIp
    return (($candidateInt -band [uint32]$Subnet.MaskInt) -eq [uint32]$Subnet.NetworkInt)
}

function Test-RaspberryPiMac {
    param(
        [AllowEmptyString()]
        [string]$MacAddress
    )

    if ([string]::IsNullOrWhiteSpace($MacAddress)) {
        return $false
    }
    $normalized = ($MacAddress.Trim().ToUpperInvariant() -replace '[:.]', '-')
    if ($normalized.Length -lt 8) {
        return $false
    }
    $prefix = $normalized.Substring(0, 8)
    return ($script:RaspberryPiMacPrefixes -contains $prefix)
}

function Get-RashkoEthernetAdapters {
    param(
        [switch]$IncludeVirtual
    )

    $adapters = @(Get-NetAdapter -ErrorAction SilentlyContinue)
    $ipConfigs = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue)
    $ipInterfaces = @(Get-NetIPInterface -AddressFamily IPv4 -ErrorAction SilentlyContinue)

    $results = New-Object System.Collections.Generic.List[object]
    foreach ($adapter in $adapters) {
        $name = [string]$adapter.Name
        $desc = [string]$adapter.InterfaceDescription
        $isVirtualProp = $false
        if ($null -ne $adapter.PSObject.Properties["Virtual"]) {
            $isVirtualProp = [bool]$adapter.Virtual
        }
        $isVirtual = $isVirtualProp -or ($name -match $script:VirtualAdapterPattern) -or ($desc -match $script:VirtualAdapterPattern)
        if ($isVirtual -and -not $IncludeVirtual) {
            continue
        }

        $isWireless = ($name -match $script:WirelessAdapterPattern) -or ($desc -match $script:WirelessAdapterPattern)
        $ifIndex = [int]$adapter.ifIndex
        $matchingIps = @(
            $ipConfigs |
                Where-Object { [int]$_.InterfaceIndex -eq $ifIndex -and $_.IPAddress -ne "127.0.0.1" } |
                Sort-Object -Property @{Expression = { if ($_.IPAddress.StartsWith("169.254.")) { 1 } else { 0 } }}
        )
        $ifSettings = $ipInterfaces | Where-Object { [int]$_.InterfaceIndex -eq $ifIndex } | Select-Object -First 1
        $dhcpState = if ($null -ne $ifSettings) { [string]$ifSettings.Dhcp } else { "Unknown" }

        $primaryIp = ""
        $prefixLength = 0
        $prefixOrigin = ""
        $subnetInfo = $null
        if ($matchingIps.Count -gt 0) {
            $primaryIp = [string]$matchingIps[0].IPAddress
            $prefixLength = [int]$matchingIps[0].PrefixLength
            $prefixOrigin = [string]$matchingIps[0].PrefixOrigin
            $subnetInfo = Get-IPv4NetworkSummary -IPAddress $primaryIp -PrefixLength $prefixLength
        }

        $results.Add([pscustomobject]@{
            Name                 = $name
            InterfaceDescription = $desc
            InterfaceIndex       = $ifIndex
            Status               = [string]$adapter.Status
            LinkSpeed            = [string]$adapter.LinkSpeed
            MacAddress           = [string]$adapter.MacAddress
            IsVirtual            = $isVirtual
            IsWireless           = $isWireless
            IsWiredEthernet      = (-not $isVirtual -and -not $isWireless)
            Dhcp                 = $dhcpState
            IPAddress            = $primaryIp
            PrefixLength         = $prefixLength
            PrefixOrigin         = $prefixOrigin
            Subnet               = $subnetInfo
        })
    }

    return @(
        $results.ToArray() | Sort-Object -Property `
            @{Expression = { if ($_.Status -eq "Up") { 0 } else { 1 } }},
            @{Expression = { if ($_.IsWiredEthernet) { 0 } elseif (-not $_.IsVirtual) { 1 } else { 2 } }},
            @{Expression = { if ($_.IPAddress) { 0 } else { 1 } }},
            Name
    )
}

function Select-RashkoEthernetAdapter {
    param(
        [string]$AdapterName = "auto"
    )

    if (-not [string]::IsNullOrWhiteSpace($AdapterName) -and $AdapterName.Trim().ToLowerInvariant() -ne "auto") {
        $wanted = $AdapterName.Trim()
        $allAdapters = @(Get-RashkoEthernetAdapters -IncludeVirtual)
        $exact = @($allAdapters | Where-Object { $_.Name -ieq $wanted })
        if ($exact.Count -gt 0) {
            return $exact[0]
        }
        $wildcard = @($allAdapters | Where-Object { $_.Name -ilike "*$wanted*" -or $_.InterfaceDescription -ilike "*$wanted*" })
        if ($wildcard.Count -eq 1) {
            return $wildcard[0]
        }
        if ($wildcard.Count -gt 1) {
            $names = ($wildcard | ForEach-Object { $_.Name }) -join ", "
            throw "Adapter '$wanted' matched multiple interfaces: $names"
        }
        $available = ($allAdapters | ForEach-Object { "$($_.Name) [$($_.Status)]" }) -join ", "
        throw "Ethernet adapter '$wanted' was not found. Available: $available"
    }

    $physical = @(Get-RashkoEthernetAdapters)
    $upWiredWithIp = @($physical | Where-Object { $_.Status -eq "Up" -and $_.IsWiredEthernet -and $_.IPAddress })
    if ($upWiredWithIp.Count -gt 0) {
        return $upWiredWithIp[0]
    }
    $upAnyPhysicalWithIp = @($physical | Where-Object { $_.Status -eq "Up" -and $_.IPAddress })
    if ($upAnyPhysicalWithIp.Count -gt 0) {
        return $upAnyPhysicalWithIp[0]
    }
    if ($physical.Count -gt 0) {
        return $physical[0]
    }
    return $null
}

function New-RashkoSignedRequestBytes {
    param(
        [Parameter(Mandatory = $true)]
        [string]$SessionId,
        [Parameter(Mandatory = $true)]
        [int]$Sequence,
        [Parameter(Mandatory = $true)]
        [int64]$SentAtMs,
        [ValidateSet("status", "estop")]
        [string]$MessageType,
        [AllowEmptyString()]
        [string]$Secret = "",
        [switch]$AllowInsecure
    )

    if ($SessionId -notmatch '^[0-9a-f]{32}$') {
        throw "SessionId must be 32 lowercase hex characters"
    }
    $secretBytes = [System.Text.Encoding]::UTF8.GetBytes($Secret)
    if ($secretBytes.Length -gt 0 -and $secretBytes.Length -lt 32) {
        throw "RASHKO_SHARED_SECRET must contain at least 32 bytes"
    }
    if ($secretBytes.Length -eq 0 -and -not $AllowInsecure) {
        throw "RASHKO_SHARED_SECRET is required unless AllowInsecure is set"
    }

    $unsignedJson = '{"sent_at_ms":' + $SentAtMs + ',"seq":' + $Sequence + ',"session":"' + $SessionId + '","type":"' + $MessageType + '","v":1}'
    if ($secretBytes.Length -eq 0) {
        return [System.Text.Encoding]::ASCII.GetBytes($unsignedJson)
    }

    $hmac = New-Object System.Security.Cryptography.HMACSHA256
    try {
        $hmac.Key = $secretBytes
        $hashBytes = $hmac.ComputeHash([System.Text.Encoding]::ASCII.GetBytes($unsignedJson))
    }
    finally {
        $hmac.Dispose()
    }
    $authHex = (($hashBytes | ForEach-Object { $_.ToString("x2") }) -join "")
    $signedJson = '{"auth":"' + $authHex + '","sent_at_ms":' + $SentAtMs + ',"seq":' + $Sequence + ',"session":"' + $SessionId + '","type":"' + $MessageType + '","v":1}'
    return [System.Text.Encoding]::ASCII.GetBytes($signedJson)
}

function Test-RashkoSignedAckBytes {
    param(
        [Parameter(Mandatory = $true)]
        [byte[]]$Datagram,
        [Parameter(Mandatory = $true)]
        [string]$ExpectedSessionId,
        [Parameter(Mandatory = $true)]
        [int]$ExpectedSequence,
        [AllowEmptyString()]
        [string]$Secret = "",
        [switch]$AllowInsecure,
        [int]$FreshnessMs = 10000
    )

    if ($null -eq $Datagram -or $Datagram.Length -eq 0 -or $Datagram.Length -gt 4096) {
        return $null
    }
    $rawJson = [System.Text.Encoding]::UTF8.GetString($Datagram)
    $secretBytes = [System.Text.Encoding]::UTF8.GetBytes($Secret)

    if ($secretBytes.Length -gt 0) {
        if ($rawJson -notmatch '"auth":"([0-9a-f]{64})"') {
            return $null
        }
        $suppliedAuth = $Matches[1]
        $unsignedJson = $rawJson -replace ',"auth":"[0-9a-f]{64}"', ''
        if ($unsignedJson -eq $rawJson) {
            $unsignedJson = $rawJson -replace '"auth":"[0-9a-f]{64}",', ''
        }
        $hmac = New-Object System.Security.Cryptography.HMACSHA256
        try {
            $hmac.Key = $secretBytes
            $expectedHash = $hmac.ComputeHash([System.Text.Encoding]::ASCII.GetBytes($unsignedJson))
        }
        finally {
            $hmac.Dispose()
        }
        $expectedAuth = (($expectedHash | ForEach-Object { $_.ToString("x2") }) -join "")
        if ($suppliedAuth -cne $expectedAuth) {
            return $null
        }
    }
    elseif (-not $AllowInsecure) {
        return $null
    }

    try {
        $parsed = $rawJson | ConvertFrom-Json
    }
    catch {
        return $null
    }

    if (
        $parsed.v -ne 1 -or
        $parsed.type -ne "ack" -or
        $parsed.session -cne $ExpectedSessionId -or
        [int]$parsed.seq -ne $ExpectedSequence
    ) {
        return $null
    }

    $nowMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()
    if ([Math]::Abs($nowMs - [int64]$parsed.sent_at_ms) -gt $FreshnessMs) {
        return $null
    }

    $stateMask = [int]$parsed.state
    $stateNames = New-Object System.Collections.Generic.List[string]
    if ($stateMask -band 1)  { $stateNames.Add("ESTOP_LATCHED") }
    if ($stateMask -band 2)  { $stateNames.Add("WATCHDOG_TRIPPED") }
    if ($stateMask -band 4)  { $stateNames.Add("OUTPUTS_ENABLED") }
    if ($stateMask -band 8)  { $stateNames.Add("HARDWARE_READY") }
    if ($stateMask -band 16) { $stateNames.Add("INSECURE_MODE") }

    $codeMap = @{
        0 = "OK"
        1 = "BAD_PACKET"
        2 = "AUTH_FAILED"
        3 = "STALE_SEQUENCE"
        4 = "INVALID_COMMAND"
        5 = "ESTOP_LATCHED"
        6 = "HARDWARE_ERROR"
        7 = "STALE_TIMESTAMP"
        8 = "CONTROL_BUSY"
    }
    $codeInt = [int]$parsed.code
    $codeName = if ($codeMap.ContainsKey($codeInt)) { $codeMap[$codeInt] } else { "UNKNOWN_$codeInt" }

    return [pscustomobject]@{
        Code      = $codeName
        CodeValue = $codeInt
        State     = @($stateNames.ToArray())
        AnglesDeg = @($parsed.angles_deg | ForEach-Object { [double]$_ })
        Detail    = [string]$parsed.detail
        SentAtMs  = [int64]$parsed.sent_at_ms
    }
}

function Invoke-RashkoUdpRequest {
    param(
        [Parameter(Mandatory = $true)]
        [string]$TargetHost,
        [int]$Port = 5005,
        [ValidateSet("status", "estop")]
        [string]$MessageType = "status",
        [AllowEmptyString()]
        [string]$Secret = "",
        [switch]$AllowInsecure,
        [int]$TimeoutMs = 450
    )

    $secretBytes = [System.Text.Encoding]::UTF8.GetBytes($Secret)
    if (($secretBytes.Length -lt 32 -and -not $AllowInsecure) -or $Secret -like "replace-with-*") {
        return $null
    }

    $randBytes = New-Object byte[] 16
    $rng = [System.Security.Cryptography.RandomNumberGenerator]::Create()
    try { $rng.GetBytes($randBytes) } finally { $rng.Dispose() }
    $sessionId = (($randBytes | ForEach-Object { $_.ToString("x2") }) -join "")
    $nowMs = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()

    $payload = New-RashkoSignedRequestBytes `
        -SessionId $sessionId `
        -Sequence 0 `
        -SentAtMs $nowMs `
        -MessageType $MessageType `
        -Secret $Secret `
        -AllowInsecure:$AllowInsecure

    $udp = New-Object System.Net.Sockets.UdpClient
    try {
        $udp.Client.ReceiveTimeout = $TimeoutMs
        $udp.Client.SendTimeout = $TimeoutMs
        $udp.Connect($TargetHost, $Port)
        [void]$udp.Send($payload, $payload.Length)
        $remoteEp = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)
        $responseBytes = $udp.Receive([ref]$remoteEp)
        return Test-RashkoSignedAckBytes `
            -Datagram $responseBytes `
            -ExpectedSessionId $sessionId `
            -ExpectedSequence 0 `
            -Secret $Secret `
            -AllowInsecure:$AllowInsecure
    }
    catch {
        return $null
    }
    finally {
        $udp.Close()
    }
}

function Test-RashkoCameraHealth {
    param(
        [Parameter(Mandatory = $true)]
        [string]$TargetHost,
        [int]$CameraPort = 8080,
        [int]$TimeoutMs = 600
    )

    $uri = "http://${TargetHost}:${CameraPort}/healthz"
    $request = [System.Net.HttpWebRequest]::Create($uri)
    $request.Method = "GET"
    $request.Timeout = $TimeoutMs
    $request.ReadWriteTimeout = $TimeoutMs
    $request.KeepAlive = $false
    try {
        $response = [System.Net.HttpWebResponse]$request.GetResponse()
        try {
            if ([int]$response.StatusCode -ne 200) {
                return $null
            }
            $reader = New-Object System.IO.StreamReader($response.GetResponseStream(), [System.Text.Encoding]::UTF8)
            try {
                $body = $reader.ReadToEnd()
            }
            finally {
                $reader.Dispose()
            }
            $json = $body | ConvertFrom-Json
            if ($null -ne $json -and ($json.backend -eq "picamera2" -or $json.backend -eq "synthetic")) {
                return [pscustomobject]@{
                    Status     = [string]$json.status
                    Backend    = [string]$json.backend
                    FrameReady = [bool]$json.frame_ready
                    Width      = [int]$json.width
                    Height     = [int]$json.height
                    Fps        = [int]$json.fps
                }
            }
        }
        finally {
            $response.Close()
        }
    }
    catch {
        return $null
    }
    return $null
}

function Test-RashkoSshBanner {
    param(
        [Parameter(Mandatory = $true)]
        [string]$TargetHost,
        [int]$Port = 22,
        [int]$TimeoutMs = 400
    )

    $client = New-Object System.Net.Sockets.TcpClient
    try {
        $async = $client.BeginConnect($TargetHost, $Port, $null, $null)
        if (-not $async.AsyncWaitHandle.WaitOne($TimeoutMs, $false)) {
            return $null
        }
        $client.EndConnect($async)
        $stream = $client.GetStream()
        $stream.ReadTimeout = $TimeoutMs
        $buffer = New-Object byte[] 128
        $read = $stream.Read($buffer, 0, $buffer.Length)
        if ($read -le 0) {
            return "SSH_OPEN"
        }
        $banner = ([System.Text.Encoding]::ASCII.GetString($buffer, 0, $read)).Trim()
        if ($banner.StartsWith("SSH-")) {
            return $banner
        }
        return "SSH_OPEN"
    }
    catch {
        return $null
    }
    finally {
        $client.Close()
    }
}

function Invoke-ParallelTcpPortSweep {
    param(
        [string[]]$Hosts,
        [int[]]$Ports = @(8080, 22),
        [int]$TimeoutMs = 260,
        [int]$BatchSize = 96
    )

    $openByHost = @{}
    if ($null -eq $Hosts -or @($Hosts).Count -eq 0) {
        return $openByHost
    }

    $workItems = New-Object System.Collections.Generic.List[object]
    foreach ($ip in $Hosts) {
        foreach ($port in $Ports) {
            $workItems.Add([pscustomobject]@{ Host = $ip; Port = $port })
        }
    }

    for ($offset = 0; $offset -lt $workItems.Count; $offset += $BatchSize) {
        $end = [Math]::Min($offset + $BatchSize, $workItems.Count)
        $pending = New-Object System.Collections.Generic.List[object]
        for ($i = $offset; $i -lt $end; $i++) {
            $item = $workItems[$i]
            $tcp = New-Object System.Net.Sockets.TcpClient
            try {
                $task = $tcp.ConnectAsync($item.Host, $item.Port)
                $pending.Add([pscustomobject]@{
                    Host   = $item.Host
                    Port   = $item.Port
                    Client = $tcp
                    Task   = $task
                })
            }
            catch {
                $tcp.Close()
            }
        }

        if ($pending.Count -gt 0) {
            $tasks = [System.Threading.Tasks.Task[]]@($pending | ForEach-Object { $_.Task })
            try {
                [void][System.Threading.Tasks.Task]::WaitAll($tasks, $TimeoutMs)
            }
            catch {
                # Individual faulted/refused tasks are expected during subnet scans.
            }

            foreach ($probe in $pending) {
                try {
                    if ($probe.Task.Status -eq [System.Threading.Tasks.TaskStatus]::RanToCompletion -and $probe.Client.Connected) {
                        if (-not $openByHost.ContainsKey($probe.Host)) {
                            $openByHost[$probe.Host] = New-Object System.Collections.Generic.List[int]
                        }
                        $openByHost[$probe.Host].Add([int]$probe.Port)
                    }
                }
                finally {
                    $probe.Client.Close()
                }
            }
        }
    }

    return $openByHost
}

function Send-EthernetNeighborWarmup {
    param(
        [Parameter(Mandatory = $true)]
        [pscustomobject]$Adapter
    )

    if (-not $Adapter.IPAddress -or $null -eq $Adapter.Subnet) {
        return
    }
    $localEp = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Parse($Adapter.IPAddress), 0)
    $udp = New-Object System.Net.Sockets.UdpClient($localEp)
    try {
        $udp.EnableBroadcast = $true
        $probeBytes = [byte[]]@(0x00)
        foreach ($dest in @($Adapter.Subnet.BroadcastAddress, "224.0.0.1")) {
            try {
                [void]$udp.Send($probeBytes, $probeBytes.Length, $dest, 9)
            }
            catch {
                # Best-effort ARP warmup on direct links.
            }
        }
    }
    finally {
        $udp.Close()
    }
}

function Resolve-CandidateHostnames {
    param(
        [string[]]$Hostnames,
        [pscustomobject]$PreferredSubnet = $null
    )

    $resolved = New-Object System.Collections.Generic.List[object]
    foreach ($name in @($Hostnames)) {
        if ([string]::IsNullOrWhiteSpace($name)) {
            continue
        }
        $cleanName = $name.Trim()
        try {
            $addresses = @([System.Net.Dns]::GetHostAddresses($cleanName) | Where-Object {
                $_.AddressFamily -eq [System.Net.Sockets.AddressFamily]::InterNetwork
            })
            foreach ($addr in $addresses) {
                $ipStr = $addr.IPAddressToString
                $onPreferred = ($null -ne $PreferredSubnet) -and (Test-IPv4InSubnet -CandidateIp $ipStr -Subnet $PreferredSubnet)
                $resolved.Add([pscustomobject]@{
                    Hostname          = $cleanName
                    IPAddress         = $ipStr
                    OnPreferredSubnet = $onPreferred
                })
            }
        }
        catch {
            # Hostname / mDNS did not resolve on this network.
        }
    }

    return @(
        $resolved.ToArray() | Sort-Object -Property `
            @{Expression = { if ($_.OnPreferredSubnet) { 0 } else { 1 } }},
            Hostname,
            IPAddress
    )
}

function Test-RashkoPiCandidate {
    param(
        [Parameter(Mandatory = $true)]
        [string]$CandidateHost,
        [int]$PiPort = 5005,
        [int]$CameraPort = 8080,
        [AllowEmptyString()]
        [string]$Secret = "",
        [switch]$AllowInsecure,
        [AllowEmptyString()]
        [string]$MacAddress = "",
        [AllowEmptyString()]
        [string]$Source = "",
        [int]$TimeoutMs = 450
    )

    $udpAck = Invoke-RashkoUdpRequest `
        -TargetHost $CandidateHost `
        -Port $PiPort `
        -Secret $Secret `
        -AllowInsecure:$AllowInsecure `
        -TimeoutMs $TimeoutMs
    $cameraHealth = Test-RashkoCameraHealth `
        -TargetHost $CandidateHost `
        -CameraPort $CameraPort `
        -TimeoutMs $TimeoutMs
    $sshBanner = Test-RashkoSshBanner `
        -TargetHost $CandidateHost `
        -Port 22 `
        -TimeoutMs $TimeoutMs

    $isPiMac = Test-RaspberryPiMac -MacAddress $MacAddress
    $udpOk = ($null -ne $udpAck -and $udpAck.Code -eq "OK")
    $cameraOk = ($null -ne $cameraHealth)
    $sshOk = (-not [string]::IsNullOrWhiteSpace($sshBanner))
    $isDebianSsh = ($sshOk -and $sshBanner -match '(?i)(Debian|Raspbian)')

    $score = 0
    if ($udpOk)       { $score += 100 }
    if ($cameraOk)    { $score += 90 }
    if ($isPiMac)     { $score += 55 }
    if ($isDebianSsh) { $score += 25 }
    elseif ($sshOk)   { $score += 10 }
    if ($Source -like "mdns:*") { $score += 30 }

    $verifiedRashko = ($udpOk -and $Secret.Length -gt 0)
    $likelyRaspberryPi = ($verifiedRashko -or ($isPiMac -and $sshOk) -or ($Source -like "mdns:*" -and $sshOk))

    $reasons = New-Object System.Collections.Generic.List[string]
    if ($udpOk)    { $reasons.Add("UDP:$($udpAck.Code)") }
    if ($cameraOk) { $reasons.Add("Camera:$($cameraHealth.Backend)") }
    if ($sshOk)    { $reasons.Add("SSH:22") }
    if ($isPiMac)  { $reasons.Add("PiMAC:$MacAddress") }
    if ($Source)   { $reasons.Add($Source) }

    return [pscustomobject]@{
        Host              = $CandidateHost
        Score             = $score
        VerifiedRashko    = $verifiedRashko
        LikelyRaspberryPi = $likelyRaspberryPi
        UdpOk             = $udpOk
        UdpAck            = $udpAck
        CameraOk          = $cameraOk
        CameraHealth      = $cameraHealth
        SshOk             = $sshOk
        SshBanner         = [string]$sshBanner
        MacAddress        = $MacAddress
        IsRaspberryPiMac  = $isPiMac
        Source            = $Source
        Summary           = ($reasons -join ", ")
    }
}

function Find-RashkoPi {
    param(
        [string]$CurrentHost = "",
        [string]$PiHostname = "",
        [string]$AdapterName = "auto",
        [int]$PiPort = 5005,
        [int]$CameraPort = 8080,
        [AllowEmptyString()]
        [string]$Secret = "",
        [switch]$AllowInsecure,
        [switch]$QuickOnly
    )

    $adapter = Select-RashkoEthernetAdapter -AdapterName $AdapterName
    $preferredSubnet = if ($null -ne $adapter) { $adapter.Subnet } else { $null }

    $candidatesByIp = @{}
    $evaluated = New-Object System.Collections.Generic.List[object]

    # 1. Fast check of current host (if on active Ethernet subnet or non-IP hostname) and mDNS names.
    $quickTargets = New-Object System.Collections.Generic.List[object]
    if (Test-RashkoHostSyntax -HostValue $CurrentHost) {
        $parsedCurrent = $null
        $isIp = [System.Net.IPAddress]::TryParse($CurrentHost.Trim(), [ref]$parsedCurrent)
        $onActiveSubnet = (-not $isIp) -or ($null -eq $preferredSubnet) -or (Test-IPv4InSubnet -CandidateIp $CurrentHost -Subnet $preferredSubnet)
        if ($onActiveSubnet) {
            if ($isIp) {
                $quickTargets.Add([pscustomobject]@{ Host = $parsedCurrent.IPAddressToString; Source = "configured" })
            }
            else {
                foreach ($entry in @(Resolve-CandidateHostnames -Hostnames @($CurrentHost) -PreferredSubnet $preferredSubnet)) {
                    if ($null -eq $preferredSubnet -or $entry.OnPreferredSubnet) {
                        $quickTargets.Add([pscustomobject]@{ Host = $entry.IPAddress; Source = "configured" })
                    }
                }
            }
        }
    }

    $mdnsNames = New-Object System.Collections.Generic.List[string]
    if (-not [string]::IsNullOrWhiteSpace($PiHostname)) {
        $mdnsNames.Add($PiHostname.Trim())
    }
    foreach ($defaultName in $script:DefaultPiMdnsNames) {
        if (-not ($mdnsNames -contains $defaultName)) {
            $mdnsNames.Add($defaultName)
        }
    }
    foreach ($resolved in @(Resolve-CandidateHostnames -Hostnames @($mdnsNames.ToArray()) -PreferredSubnet $preferredSubnet)) {
        if ($null -ne $preferredSubnet -and -not $resolved.OnPreferredSubnet) {
            continue
        }
        $quickTargets.Add([pscustomobject]@{
            Host   = $resolved.IPAddress
            Source = "mdns:$($resolved.Hostname)"
        })
    }

    foreach ($target in $quickTargets) {
        if ($candidatesByIp.ContainsKey($target.Host)) {
            continue
        }
        $probe = Test-RashkoPiCandidate `
            -CandidateHost $target.Host `
            -PiPort $PiPort `
            -CameraPort $CameraPort `
            -Secret $Secret `
            -AllowInsecure:$AllowInsecure `
            -Source $target.Source
        $candidatesByIp[$target.Host] = $probe
        if ($probe.Score -gt 0) {
            $evaluated.Add($probe)
        }
        if ($probe.VerifiedRashko) {
            return [pscustomobject]@{
                Adapter    = $adapter
                Selected   = $probe
                Candidates = @($evaluated.ToArray() | Sort-Object -Property Score -Descending)
            }
        }
    }

    if ($QuickOnly -or $null -eq $adapter -or -not $adapter.IPAddress -or $null -eq $adapter.Subnet) {
        return [pscustomobject]@{
            Adapter    = $adapter
            Selected   = $null
            Candidates = @($evaluated.ToArray() | Sort-Object -Property Score -Descending)
        }
    }

    # 2. Warm up ARP/neighbors on the Ethernet interface (especially important for 169.254.x.x direct cable!).
    Send-EthernetNeighborWarmup -Adapter $adapter
    $neighborMacByIp = @{}
    $neighbors = @(
        Get-NetNeighbor -InterfaceIndex $adapter.InterfaceIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object {
                $_.IPAddress -and
                $_.IPAddress -ne $adapter.IPAddress -and
                $_.IPAddress -ne $adapter.Subnet.BroadcastAddress -and
                -not $_.IPAddress.StartsWith("224.") -and
                -not $_.IPAddress.StartsWith("239.") -and
                $_.IPAddress -ne "255.255.255.255" -and
                $_.State -notin @("Unreachable", "Permanent")
            }
    )
    foreach ($neighbor in $neighbors) {
        $ip = [string]$neighbor.IPAddress
        $mac = [string]$neighbor.LinkLayerAddress
        if ($mac -and $mac -ne "00-00-00-00-00-00") {
            $neighborMacByIp[$ip] = $mac
        }
    }

    # Check Raspberry Pi MAC neighbors first for near-instant discovery.
    foreach ($ip in @($neighborMacByIp.Keys)) {
        $mac = $neighborMacByIp[$ip]
        if ((Test-RaspberryPiMac -MacAddress $mac) -and -not $candidatesByIp.ContainsKey($ip)) {
            $probe = Test-RashkoPiCandidate `
                -CandidateHost $ip `
                -PiPort $PiPort `
                -CameraPort $CameraPort `
                -Secret $Secret `
                -AllowInsecure:$AllowInsecure `
                -MacAddress $mac `
                -Source "arp:raspberry-mac"
            $candidatesByIp[$ip] = $probe
            if ($probe.Score -gt 0) {
                $evaluated.Add($probe)
            }
            if ($probe.VerifiedRashko) {
                return [pscustomobject]@{
                    Adapter    = $adapter
                    Selected   = $probe
                    Candidates = @($evaluated.ToArray() | Sort-Object -Property Score -Descending)
                }
            }
        }
    }

    # 3. Parallel asynchronous TCP sweep on the Ethernet /24 + remaining ARP neighbors.
    $openHosts = Invoke-ParallelTcpPortSweep `
        -Hosts $adapter.Subnet.ScanHosts `
        -Ports @($CameraPort, 22) `
        -TimeoutMs 250

    # Refresh neighbor MACs after TCP SYN sweep populated the Ethernet ARP table!
    $freshNeighbors = @(
        Get-NetNeighbor -InterfaceIndex $adapter.InterfaceIndex -AddressFamily IPv4 -ErrorAction SilentlyContinue |
            Where-Object { $_.LinkLayerAddress -and $_.LinkLayerAddress -ne "00-00-00-00-00-00" }
    )
    foreach ($neighbor in $freshNeighbors) {
        $neighborMacByIp[[string]$neighbor.IPAddress] = [string]$neighbor.LinkLayerAddress
    }

    $ipsToInspect = New-Object System.Collections.Generic.List[string]
    foreach ($ip in @($openHosts.Keys)) {
        if (-not $candidatesByIp.ContainsKey($ip)) {
            $ipsToInspect.Add($ip)
        }
    }
    foreach ($ip in @($neighborMacByIp.Keys)) {
        if (-not $candidatesByIp.ContainsKey($ip) -and -not ($ipsToInspect -contains $ip)) {
            $ipsToInspect.Add($ip)
        }
    }

    foreach ($ip in $ipsToInspect) {
        $mac = if ($neighborMacByIp.ContainsKey($ip)) { $neighborMacByIp[$ip] } else { "" }
        $source = if ($openHosts.ContainsKey($ip)) { "ethernet-scan" } else { "arp-neighbor" }
        $probe = Test-RashkoPiCandidate `
            -CandidateHost $ip `
            -PiPort $PiPort `
            -CameraPort $CameraPort `
            -Secret $Secret `
            -AllowInsecure:$AllowInsecure `
            -MacAddress $mac `
            -Source $source
        $candidatesByIp[$ip] = $probe
        if ($probe.Score -gt 0) {
            $evaluated.Add($probe)
        }
    }

    $sorted = @($evaluated.ToArray() | Sort-Object -Property @{Expression = "Score"; Descending = $true}, Host)
    $selected = $null
    $verified = @($sorted | Where-Object { $_.VerifiedRashko })
    if ($verified.Count -gt 0) {
        $selected = $verified[0]
    }

    return [pscustomobject]@{
        Adapter    = $adapter
        Selected   = $selected
        Candidates = $sorted
    }
}

function Resolve-RashkoPiEndpoint {
    param(
        [Parameter(Mandatory = $true)]
        [string]$EnvFile,
        [string]$PiHostOverride = "",
        [string]$AdapterOverride = "",
        [switch]$ForceDiscover,
        [switch]$NoAutoDiscover,
        [switch]$PersistChanges = $true,
        [switch]$Quiet
    )

    $envValues = Read-RashkoDotEnv -Path $EnvFile
    $configuredHost = Get-RashkoConfigValue -Values $envValues -Name "RASHKO_PI_HOST" -Default "auto"
    $piHostname = Get-RashkoConfigValue -Values $envValues -Name "RASHKO_PI_HOSTNAME" -Default ""
    $configuredAdapter = Get-RashkoConfigValue -Values $envValues -Name "RASHKO_ETHERNET_ADAPTER" -Default "auto"
    $autoDiscoverRaw = Get-RashkoConfigValue -Values $envValues -Name "RASHKO_PI_AUTO_DISCOVER" -Default "true"
    $autoDiscoverEnabled = (-not $NoAutoDiscover) -and ($autoDiscoverRaw.ToLowerInvariant() -in @("1", "true", "yes", "on"))

    $piPort = [int](Get-RashkoConfigValue -Values $envValues -Name "RASHKO_PI_PORT" -Default "5005")
    $cameraPort = [int](Get-RashkoConfigValue -Values $envValues -Name "RASHKO_CAMERA_PORT" -Default "8080")
    $secret = Get-RashkoConfigValue -Values $envValues -Name "RASHKO_SHARED_SECRET" -Default ""
    $allowInsecure = (Get-RashkoConfigValue -Values $envValues -Name "RASHKO_ALLOW_INSECURE" -Default "false").ToLowerInvariant() -in @("1", "true", "yes", "on")

    $effectiveAdapterName = if (-not [string]::IsNullOrWhiteSpace($AdapterOverride)) { $AdapterOverride.Trim() } else { $configuredAdapter }
    $adapter = Select-RashkoEthernetAdapter -AdapterName $effectiveAdapterName
    $ethernetSubnet = if ($null -ne $adapter -and $null -ne $adapter.Subnet) { $adapter.Subnet.Cidr } else { "" }

    # Explicit CLI override wins immediately.
    if (-not [string]::IsNullOrWhiteSpace($PiHostOverride) -and $PiHostOverride.Trim().ToLowerInvariant() -ne "auto") {
        $cleanOverride = $PiHostOverride.Trim()
        if (-not (Test-RashkoHostSyntax -HostValue $cleanOverride)) {
            throw "Unsupported PiHost override: $cleanOverride"
        }
        if ($PersistChanges -and (Test-Path -LiteralPath $EnvFile)) {
            $updates = @{ RASHKO_PI_HOST = $cleanOverride }
            if ($ethernetSubnet) {
                $updates["RASHKO_ETHERNET_SUBNET"] = $ethernetSubnet
            }
            Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
        }
        return [pscustomobject]@{
            PiHost         = $cleanOverride
            PiPort         = $piPort
            CameraPort     = $cameraPort
            Adapter        = $adapter
            EthernetSubnet = $ethernetSubnet
            Discovered     = $false
            DiscoveryInfo  = $null
        }
    }

    $needsDiscovery = $ForceDiscover -or ($PiHostOverride.Trim().ToLowerInvariant() -eq "auto") -or ($configuredHost.ToLowerInvariant() -eq "auto") -or (-not (Test-RashkoHostSyntax -HostValue $configuredHost))
    if (-not $needsDiscovery -and $autoDiscoverEnabled) {
        $parsedIp = $null
        $isLiteralIp = [System.Net.IPAddress]::TryParse($configuredHost, [ref]$parsedIp)
        if (-not $isLiteralIp) {
            $needsDiscovery = $true
        }
        elseif ($null -ne $adapter -and $null -ne $adapter.Subnet -and -not (Test-IPv4InSubnet -CandidateIp $configuredHost -Subnet $adapter.Subnet)) {
            if (-not $Quiet) {
                Write-Host "[AUTO] Configured Pi IP $configuredHost is outside active $($adapter.Name) subnet $($adapter.Subnet.Cidr); scanning Ethernet..." -ForegroundColor Yellow
            }
            $needsDiscovery = $true
        }
        else {
            $quickCheck = Test-RashkoPiCandidate `
                -CandidateHost $configuredHost `
                -PiPort $piPort `
                -CameraPort $cameraPort `
                -Secret $secret `
                -AllowInsecure:$allowInsecure `
                -Source "configured" `
                -TimeoutMs 400
            if (-not $quickCheck.VerifiedRashko) {
                if (-not $Quiet) {
                    Write-Host "[AUTO] Configured Pi host $configuredHost did not respond; scanning for the Raspberry Pi..." -ForegroundColor Yellow
                }
                $needsDiscovery = $true
            }
        }
    }

    $finalHost = $configuredHost
    $discovered = $false
    $discoveryResult = $null
    if ($needsDiscovery -and ($ForceDiscover -or $autoDiscoverEnabled)) {
        $discoveryResult = Find-RashkoPi `
            -CurrentHost $configuredHost `
            -PiHostname $piHostname `
            -AdapterName $effectiveAdapterName `
            -PiPort $piPort `
            -CameraPort $cameraPort `
            -Secret $secret `
            -AllowInsecure:$allowInsecure
        if ($null -ne $discoveryResult.Selected) {
            $finalHost = [string]$discoveryResult.Selected.Host
            $discovered = $true
            if (-not $Quiet) {
                Write-Host "[AUTO] Found Raspberry Pi at $finalHost ($($discoveryResult.Selected.Summary))" -ForegroundColor Green
            }
        }
        elseif (-not $Quiet) {
            $ifLabel = if ($null -ne $adapter) { "$($adapter.Name) ($($adapter.IPAddress)/$($adapter.PrefixLength), $($adapter.Status))" } else { "no active Ethernet adapter" }
            Write-Host "[WARN] Automatic Raspberry Pi discovery found no verified match on $ifLabel." -ForegroundColor DarkYellow
        }
    }

    if ($PersistChanges -and (Test-Path -LiteralPath $EnvFile)) {
        $updates = @{}
        if ($discovered -and $finalHost -ne $configuredHost) {
            $updates["RASHKO_PI_HOST"] = $finalHost
        }
        if ($ethernetSubnet -and (Get-RashkoConfigValue -Values $envValues -Name "RASHKO_ETHERNET_SUBNET") -ne $ethernetSubnet) {
            $updates["RASHKO_ETHERNET_SUBNET"] = $ethernetSubnet
        }
        if (-not [string]::IsNullOrWhiteSpace($AdapterOverride) -and $AdapterOverride.Trim() -ne $configuredAdapter) {
            $updates["RASHKO_ETHERNET_ADAPTER"] = $AdapterOverride.Trim()
        }
        if ($updates.Count -gt 0) {
            Set-RashkoDotEnvValues -Path $EnvFile -Updates $updates
        }
    }

    return [pscustomobject]@{
        PiHost         = $finalHost
        PiPort         = $piPort
        CameraPort     = $cameraPort
        Adapter        = $adapter
        EthernetSubnet = $ethernetSubnet
        Discovered     = $discovered
        DiscoveryInfo  = $discoveryResult
    }
}

function Test-IsWindowsAdministrator {
    $identity = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($identity)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Set-RashkoEthernetMode {
    param(
        [Parameter(Mandatory = $true)]
        [string]$AdapterName,
        [Parameter(Mandatory = $true)]
        [ValidateSet("Dhcp", "Static")]
        [string]$Mode,
        [string]$StaticIPAddress = "192.168.137.1",
        [int]$PrefixLength = 24
    )

    $adapter = Select-RashkoEthernetAdapter -AdapterName $AdapterName
    if ($null -eq $adapter) {
        throw "No matching Ethernet adapter found for '$AdapterName'."
    }
    if ($adapter.IsVirtual) {
        throw "Refusing to modify virtual adapter '$($adapter.Name)'."
    }

    if ($Mode -eq "Static") {
        [void](ConvertTo-UInt32IPv4 -IPAddress $StaticIPAddress)
        if ($PrefixLength -lt 8 -or $PrefixLength -gt 30) {
            throw "PrefixLength for direct Ethernet static mode must be in 8..30"
        }
    }

    $escapedAlias = $adapter.Name.Replace("'", "''")
    if ($Mode -eq "Dhcp") {
        $elevatedScript = @"
`$ErrorActionPreference = 'Stop'
Set-NetIPInterface -InterfaceAlias '$escapedAlias' -AddressFamily IPv4 -Dhcp Enabled
Set-DnsClientServerAddress -InterfaceAlias '$escapedAlias' -ResetServerAddresses
ipconfig /renew '$escapedAlias' | Out-Null
"@
    }
    else {
        $elevatedScript = @"
`$ErrorActionPreference = 'Stop'
Get-NetIPAddress -InterfaceAlias '$escapedAlias' -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { `$_.PrefixOrigin -eq 'Manual' } |
    Remove-NetIPAddress -Confirm:`$false -ErrorAction SilentlyContinue
Set-NetIPInterface -InterfaceAlias '$escapedAlias' -AddressFamily IPv4 -Dhcp Disabled
New-NetIPAddress -InterfaceAlias '$escapedAlias' -AddressFamily IPv4 -IPAddress '$StaticIPAddress' -PrefixLength $PrefixLength | Out-Null
"@
    }

    if (Test-IsWindowsAdministrator) {
        Invoke-Expression $elevatedScript
    }
    else {
        Write-Host "Requesting Windows Administrator (UAC) permission to configure '$($adapter.Name)' -> $Mode..." -ForegroundColor Cyan
        $encoded = [Convert]::ToBase64String([System.Text.Encoding]::Unicode.GetBytes($elevatedScript))
        $proc = Start-Process -FilePath "powershell.exe" -Verb RunAs -PassThru -Wait -ArgumentList @(
            "-NoLogo", "-NoProfile", "-ExecutionPolicy", "Bypass", "-EncodedCommand", $encoded
        )
        if ($proc.ExitCode -ne 0) {
            throw "Elevated Ethernet configuration exited with code $($proc.ExitCode)"
        }
    }

    Start-Sleep -Seconds 2
    return Select-RashkoEthernetAdapter -AdapterName $adapter.Name
}
