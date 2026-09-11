# Fixed public deployment helpers. Dot-sourcing this file only defines functions;
# probing is read-only and never starts/stops a service.
function ConvertFrom-EturnalScalar([string]$Value) {
    $text = ([string]$Value).Trim()
    if (!$text) { return '' }
    if ($text.Length -ge 2 -and $text.StartsWith('"') -and $text.EndsWith('"')) {
        return ($text.Substring(1, $text.Length - 2) -replace '\\"', '"')
    }
    if ($text.Length -ge 2 -and $text.StartsWith("'") -and $text.EndsWith("'")) {
        return $text.Substring(1, $text.Length - 2).Replace("''", "'")
    }
    $comment = $text.IndexOf(' #')
    if ($comment -ge 0) { $text = $text.Substring(0, $comment).TrimEnd() }
    return $text
}

function Read-EturnalSettings([string]$Path) {
    if (!$Path -or !(Test-Path -LiteralPath $Path -PathType Leaf)) { return $null }
    $result = @{ Secret = ''; RelayIpv4 = ''; ListenPort = 0; RelayMinPort = 0; RelayMaxPort = 0 }
    $inListen = $false
    $listenIndent = -1
    foreach ($raw in @(Get-Content -LiteralPath $Path -ErrorAction Stop)) {
        $line = [string]$raw
        if ($line -match '^\s*(#|$)') { continue }
        $indent = $line.Length - $line.TrimStart().Length
        $trimmed = $line.Trim()
        if ($inListen -and $indent -le $listenIndent) { $inListen = $false }
        if ($trimmed -match '^listen\s*:\s*$') {
            $inListen = $true
            $listenIndent = $indent
            continue
        }
        if ($inListen) {
            if ($result.ListenPort -eq 0 -and $trimmed -match '^port\s*:\s*([0-9]+)\s*(?:#.*)?$') {
                $result.ListenPort = [int]$Matches[1]
            }
            continue
        }
        if ($trimmed -match '^secret\s*:\s*(.*)$') {
            $result.Secret = ConvertFrom-EturnalScalar $Matches[1]
        } elseif ($trimmed -match '^relay_ipv4_addr\s*:\s*(.*)$') {
            $result.RelayIpv4 = ConvertFrom-EturnalScalar $Matches[1]
        } elseif ($trimmed -match '^relay_min_port\s*:\s*([0-9]+)') {
            $result.RelayMinPort = [int]$Matches[1]
        } elseif ($trimmed -match '^relay_max_port\s*:\s*([0-9]+)') {
            $result.RelayMaxPort = [int]$Matches[1]
        }
    }
    if ($result.ListenPort -eq 0) { $result.ListenPort = 3478 }
    return $result
}

function Test-PublicIPv4([string]$Value) {
    if (!$Value) { return $false }
    $address = $null
    if (![Net.IPAddress]::TryParse($Value.Trim(), [ref]$address)) { return $false }
    if ($address.AddressFamily -ne [Net.Sockets.AddressFamily]::InterNetwork) { return $false }
    $bytes = $address.GetAddressBytes()
    if ($bytes[0] -eq 0 -or $bytes[0] -eq 10 -or $bytes[0] -eq 127 -or $bytes[0] -ge 224) { return $false }
    if ($bytes[0] -eq 169 -and $bytes[1] -eq 254) { return $false }
    if ($bytes[0] -eq 172 -and $bytes[1] -ge 16 -and $bytes[1] -le 31) { return $false }
    if ($bytes[0] -eq 192 -and $bytes[1] -eq 168) { return $false }
    if ($bytes[0] -eq 100 -and $bytes[1] -ge 64 -and $bytes[1] -le 127) { return $false }
    if ($bytes[0] -eq 198 -and ($bytes[1] -eq 18 -or $bytes[1] -eq 19)) { return $false }
    return $true
}

function Get-FixedPublicIp {
    if ($env:SPACE_SIM_PUBLIC_IP) {
        $forced = [string]$env:SPACE_SIM_PUBLIC_IP
        if (!(Test-PublicIPv4 $forced)) { throw 'SPACE_SIM_PUBLIC_IP must be a public IPv4 address.' }
        return $forced.Trim()
    }
    foreach ($uri in @('https://api.ipify.org', 'https://icanhazip.com', 'https://ifconfig.me/ip')) {
        try {
            try {
                $response = Invoke-WebRequest -Uri $uri -TimeoutSec 10 -MaximumRedirection 0 -NoProxy -ErrorAction Stop
            } catch {
                $response = Invoke-WebRequest -Uri $uri -TimeoutSec 10 -MaximumRedirection 0 -ErrorAction Stop
            }
            $value = ([string]$response.Content).Trim()
            if (Test-PublicIPv4 $value) { return $value }
        } catch { }
    }
    return $null
}

function Get-FixedDeploymentCandidate {
    [CmdletBinding()]
    param([string]$ProjectRoot, [switch]$NoNetwork)

    $configPath = if ($env:SPACE_SIM_ETURNAL_CONFIG) {
        [IO.Path]::GetFullPath([string]$env:SPACE_SIM_ETURNAL_CONFIG, $ProjectRoot)
    } else {
        Join-Path $env:ProgramFiles 'eturnal\etc\eturnal.yml'
    }
    $eturnal = Read-EturnalSettings $configPath
    if (!$eturnal -or [string]::IsNullOrWhiteSpace($eturnal.Secret)) { return $null }

    # If the local service exists but is stopped, a fixed deployment would
    # advertise a TURN endpoint that cannot work. Fall back to the tunnel mode.
    $eturnalServices = @(Get-Service -Name 'eturnal*' -ErrorAction SilentlyContinue)
    if ($eturnalServices.Count -gt 0 -and !($eturnalServices | Where-Object Status -eq 'Running')) {
        return $null
    }

    $publicIp = [string]$eturnal.RelayIpv4
    if (!(Test-PublicIPv4 $publicIp)) {
        if ($NoNetwork) { return $null }
        $publicIp = Get-FixedPublicIp
    }
    if (!(Test-PublicIPv4 $publicIp)) { return $null }
    $publicIp = $publicIp.Trim()

    $hostName = ([string]$env:SPACE_SIM_FIXED_PUBLIC_HOST).Trim().TrimEnd('.')
    if (!$hostName) { $hostName = "$publicIp.sslip.io" }
    if ($hostName.Length -gt 253 -or $hostName -notmatch '^[A-Za-z0-9](?:[A-Za-z0-9.-]*[A-Za-z0-9])?$' -or
        $hostName.Contains('..') -or $hostName -match '^\d{1,3}(?:\.\d{1,3}){3}$') {
        throw 'SPACE_SIM_FIXED_PUBLIC_HOST must be a DNS hostname, not an IP address.'
    }

    $turnPort = [int]$eturnal.ListenPort
    if ($turnPort -lt 1 -or $turnPort -gt 65535) { throw 'eturnal listen port is not valid.' }
    $turnUrls = @(
        "turn:${publicIp}:${turnPort}?transport=udp",
        "turn:${publicIp}:${turnPort}?transport=tcp"
    )
    $placeholder = $eturnal.Secret -match '(?i)^(replace_with|change_?me|default|random|test)' -or
        $eturnal.Secret -eq 'replace_with_a_long_random_secret'

    return [pscustomobject]@{
        PublicIp = $publicIp
        PublicHost = $hostName
        PublicUrl = "https://$hostName"
        TurnHost = $publicIp
        TurnPort = $turnPort
        TurnUrls = $turnUrls
        StunUrl = "stun:${publicIp}:${turnPort}"
        TurnSecret = [string]$eturnal.Secret
        TurnSecretIsPlaceholder = [bool]$placeholder
        EturnalConfigPath = $configPath
        RelayMinPort = [int]$eturnal.RelayMinPort
        RelayMaxPort = [int]$eturnal.RelayMaxPort
    }
}

function Initialize-FixedConfig {
    [CmdletBinding()]
    param([string]$ConfigPath, [string]$ProjectRoot, $Candidate,
          [ValidateSet('acme', 'tunnel', 'ip-acme')][string]$TlsMode = 'acme',
          [string]$PublicUrl = '')

    $directory = Split-Path -Parent $ConfigPath
    $null = New-Item -ItemType Directory -Path $directory -Force
    $config = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/deployment.example.json') | ConvertFrom-Json -AsHashtable
    if (Test-Path -LiteralPath $ConfigPath -PathType Leaf) {
        $existing = Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json -AsHashtable
        if ([string]$existing.tls_mode -eq $TlsMode) {
            # Keep machine-local roots and the chosen admin name across restarts.
            foreach ($key in @('admin_username', 'require_access_key', 'show_access_window', 'adapter_root', 'model_root', 'unreal_root')) {
                if ($existing.ContainsKey($key)) { $config[$key] = $existing[$key] }
            }
        }
    }
    $config.tls_mode = $TlsMode
    if ($TlsMode -eq 'tunnel') {
        # The tunnel launcher replaces this placeholder with the allocated
        # trycloudflare.com hostname before the gateway opens.
        $config.public_url = 'https://unassigned.trycloudflare.com'
    } elseif ($PublicUrl) {
        # ip-acme pins the entry point to the public IP itself, so the address
        # never changes even though the certificate is short-lived.
        $config.public_url = [string]$PublicUrl
    } else {
        $config.public_url = [string]$Candidate.PublicUrl
    }
    $config.certificate_file = ''
    $config.certificate_key_file = ''
    $config.ice_servers = @(@{ urls = [string]$Candidate.StunUrl })
    $config.turn_urls = @($Candidate.TurnUrls | ForEach-Object { [string]$_ })

    $temporary = Join-Path $directory ('.fixed-' + [guid]::NewGuid().ToString('N') + '.local.json')
    try {
        $config | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $temporary -Encoding utf8NoBOM
        $null = Get-DeploymentSettings $temporary $ProjectRoot
        Move-Item -LiteralPath $temporary -Destination $ConfigPath -Force
    } finally {
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary -Force }
    }
}

function Wait-FixedGateway([string]$Url, [int]$TimeoutSeconds = 180) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $lastError = 'no response yet'
    do {
        $probe = $Url + '/api/health?fixed_probe=' + [guid]::NewGuid().ToString('N')
        try {
            try {
                $response = Invoke-WebRequest -Uri $probe -TimeoutSec 10 -MaximumRedirection 0 -NoProxy -ErrorAction Stop
            } catch {
                $response = Invoke-WebRequest -Uri $probe -TimeoutSec 10 -MaximumRedirection 0 -ErrorAction Stop
            }
            if ($response.StatusCode -eq 200 -and ($response.Content | ConvertFrom-Json).ok -eq $true) { return }
            $lastError = "unexpected HTTP status $($response.StatusCode)"
        } catch {
            $lastError = $_.Exception.Message
        }
        Start-Sleep -Seconds 3
    } while ([DateTime]::UtcNow -lt $deadline)
    throw "Fixed public HTTPS health probe failed for $Url. Check that 80/443 reach this host, that the hostname resolves to the public IP, and review logs/deployment-proxy.*.log. Last error: $lastError"
}

function Show-FixedAccess([string]$ProjectRoot, [string]$ConfigPath, [switch]$NonInteractive) {
    $report = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/public-access.json') | ConvertFrom-Json
    $parsedHost = $null
    $isIpEntry = [Net.IPAddress]::TryParse(([uri]$report.public_url).Host, [ref]$parsedHost)
    Write-Host "固定公网 HTTPS 入口已验证：$($report.public_url)"
    if ($isIpEntry) {
        Write-Host '该地址是公网 IP 本身，由 Let''s Encrypt IP 证书（shortlived，约 6 天）自动续期；重启部署不会更换入口。'
        Write-Host '证书续期依赖本机 Caddy 持续运行；长时间关机可能导致证书过期，重新启动部署会自动补签。'
    } else {
        Write-Host '该地址使用固定主机名；重启部署不会更换入口。任何客户端 IP 都可访问，但仍需登录和访问密钥。'
    }
    Write-Host 'STUN/TURN 已自动接入本机 eturnal；部分用户网络下的 WebRTC 视频仍需实际客户端验收。'
    if ($report.PSObject.Properties['require_access_key'] -and !$report.require_access_key) {
        Write-Warning '本次部署已关闭访问密钥：入口仅剩登录一道防线，任何知道网址的人都能打开登录页。'
    }
    # Read the config first so a restart honours an edited value immediately;
    # only fall back to the report from the previous run.
    $showWindow = $true
    try {
        $configured = Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json
        if ($configured.PSObject.Properties['show_access_window']) { $showWindow = [bool]$configured.show_access_window }
        elseif ($report.PSObject.Properties['show_access_window']) { $showWindow = [bool]$report.show_access_window }
    } catch {
        if ($report.PSObject.Properties['show_access_window']) { $showWindow = [bool]$report.show_access_window }
    }
    if ($showWindow) {
        Write-Output '访问链接和登录资料在本机凭据窗口查看，不会写入命令行或日志。'
    } else {
        Write-Output '已按配置跳过凭据窗口（show_access_window=false）。需要时运行 show_deployment_access.cmd。'
    }
    if ($showWindow -and !$NonInteractive) {
        try {
            $pwsh = (Get-Process -Id $PID).Path
            $null = Start-Process -FilePath $pwsh -ArgumentList @('-NoLogo', '-NoProfile', '-ExecutionPolicy', 'RemoteSigned', '-STA', '-File', ('"' + (Join-Path $ProjectRoot 'scripts/show_public_access.ps1') + '"'), '-ConfigPath', ('"' + $ConfigPath + '"')) -WindowStyle Hidden
        } catch { Write-Warning 'Credential window could not open. Run show_deployment_access.cmd to retry; services remain running.' }
    }
}

