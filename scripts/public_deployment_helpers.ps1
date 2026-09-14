# Public deployment helpers. Dot-sourcing has no service/network side effects.
function Initialize-PublicConfig([string]$ConfigPath, [string]$ProjectRoot) {
    if (Test-Path -LiteralPath $ConfigPath -PathType Leaf) { return }
    $config = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/deployment.example.json') | ConvertFrom-Json -AsHashtable
    $config.public_url = 'https://unassigned.trycloudflare.com'
    $config.tls_mode = 'tunnel'
    $config.ice_servers = @(@{urls='stun:stun.cloudflare.com:3478'})
    $config | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ConfigPath -Encoding utf8NoBOM
}

function Resolve-PublicTunnel([string]$ProjectRoot) {
    if ($env:SPACE_SIM_CLOUDFLARED_EXE) {
        $file = Get-Command $env:SPACE_SIM_CLOUDFLARED_EXE -CommandType Application -ErrorAction Stop
        return $file.Source
    }
    if ([Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString() -ne 'X64') {
        throw 'Automatic public tunnel download currently supports Windows x64. Set SPACE_SIM_CLOUDFLARED_EXE for another supported platform.'
    }
    $manifest = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/cloudflared-release.json') | ConvertFrom-Json
    $version = [string]$manifest.version
    if ($version -notmatch '^\d{4}\.\d+\.\d+$' -or $manifest.sha256.amd64 -notmatch '^[a-f0-9]{64}$') { throw 'Invalid cloudflared pin manifest.' }
    $directory = Join-Path $ProjectRoot "run/deployment-tools/cloudflared-$version"
    $file = Join-Path $directory 'cloudflared.exe'
    if ((Test-Path -LiteralPath $file) -and (Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -eq $manifest.sha256.amd64) { return $file }
    $null = New-Item -ItemType Directory -Path $directory -Force
    $temporary = Join-Path $directory ([guid]::NewGuid().ToString('N') + '.download')
    Write-Host "正在下载并校验公网隧道工具 cloudflared $version……"
    try {
        Receive-DeploymentFile "https://github.com/cloudflare/cloudflared/releases/download/$version/cloudflared-windows-amd64.exe" $temporary
        if ((Get-FileHash -LiteralPath $temporary -Algorithm SHA256).Hash -ne $manifest.sha256.amd64) { throw 'cloudflared checksum mismatch.' }
        Move-Item -LiteralPath $temporary -Destination $file -Force
    } finally {
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary -Force }
    }
    return $file
}

function Get-PublicLoopbackPort {
    $listener = [Net.Sockets.TcpListener]::new([Net.IPAddress]::Loopback, 0)
    try { $listener.Start(); return ([Net.IPEndPoint]$listener.LocalEndpoint).Port }
    finally { $listener.Stop() }
}

function Get-PublicProxyConfig($Settings, [int]$Port, [int]$AdminPort, [string]$Nonce, [switch]$Holding) {
    $routes = 'header X-Space-Sim-Preparing ' + $Nonce + "`n" + 'respond "Public deployment is preparing; no application is exposed yet." 503'
    if (!$Holding) {
        $hostName = ([uri]$Settings.PublicUrl).DnsSafeHost
        if ($hostName -notmatch '^[a-z0-9]+(?:-[a-z0-9]+)*\.trycloudflare\.com$') { throw 'Expected an assigned trycloudflare.com hostname.' }
        $routes = @"
@public host $hostName
handle @public {
    header {
        X-Space-Sim-Deployment $Nonce
        Cache-Control no-store
        X-Content-Type-Options nosniff
        Referrer-Policy no-referrer
        X-Frame-Options DENY
        Permissions-Policy "gamepad=(self)"
    }
    @signalling path /stream
    handle @signalling {
        reverse_proxy 127.0.0.1:$($Settings.Ports.player_port) {
            header_up X-Forwarded-Proto https
            header_up X-Forwarded-For {http.request.header.CF-Connecting-IP}
        }
    }
    handle {
        reverse_proxy 127.0.0.1:$($Settings.Ports.api_port) {
            header_up X-Forwarded-Proto https
            header_up X-Forwarded-For {http.request.header.CF-Connecting-IP}
        }
    }
}
respond "Unknown site" 421
"@
    }
    return @"
{
    admin 127.0.0.1:$AdminPort
    auto_https off
}
http://:$Port {
    bind 127.0.0.1
    $routes
}
"@
}

function Start-PublicChild([string]$Executable, [string[]]$Arguments, [string]$LogBase, [string]$ProjectRoot) {
    # A TLS/tunnel child has no need to inherit platform passwords or unrelated CF tokens/configuration.
    $previous = @{}
    foreach ($item in Get-ChildItem Env: | Where-Object { $_.Name -like 'SPACE_SIM_*' -or $_.Name -like 'TUNNEL_*' }) {
        $previous[$item.Name] = $item.Value
        [Environment]::SetEnvironmentVariable($item.Name, $null, 'Process')
    }
    try {
        return Start-Process -FilePath $Executable -ArgumentList $Arguments -WorkingDirectory $ProjectRoot -PassThru -WindowStyle Hidden `
            -RedirectStandardOutput ($LogBase + '.out.log') -RedirectStandardError ($LogBase + '.err.log')
    } finally {
        foreach ($name in $previous.Keys) { [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process') }
    }
}

function Save-PublicPending([string]$Path, $Proxy, $Tunnel, [string]$Url = '') {
    $record = @{mode='public'; public_url=$Url; public_verified=$false}
    if ($Proxy) { $record.proxy_pid=$Proxy.Id; $record.proxy_start=$Proxy.StartTime.ToUniversalTime().Ticks }
    if ($Tunnel) { $record.tunnel_pid=$Tunnel.Id; $record.tunnel_start=$Tunnel.StartTime.ToUniversalTime().Ticks }
    $record | ConvertTo-Json | Set-Content -LiteralPath $Path -Encoding utf8NoBOM
}

function Stop-PublicPending([string]$ProjectRoot) {
    $path = Join-Path $ProjectRoot 'run/public-pending.json'
    if (!(Test-Path -LiteralPath $path)) { return }
    $record = Get-Content -Raw -LiteralPath $path | ConvertFrom-Json
    foreach ($prefix in @('tunnel', 'proxy')) {
        $processId = [int]$record."${prefix}_pid"
        $ticks = [long]$record."${prefix}_start"
        if (Test-LauncherProcess $processId $ticks) { Stop-Process -Id $processId -Force -ErrorAction SilentlyContinue }
    }
    Remove-Item -LiteralPath $path -Force
}

function Wait-PublicTunnelUrl($Tunnel, [string]$LogBase, [int]$TimeoutSeconds = 90) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    while ([DateTime]::UtcNow -lt $deadline) {
        if ($Tunnel.HasExited) { throw 'Public tunnel exited before allocating an address. Check its local log (redact URLs before sharing).' }
        foreach ($suffix in @('.out.log', '.err.log')) {
            if (!(Test-Path -LiteralPath ($LogBase + $suffix))) { continue }
            $content = Get-Content -Raw -LiteralPath ($LogBase + $suffix) -ErrorAction SilentlyContinue
            # Redirected files exist before cloudflared emits its first line; -Raw returns $null then.
            if ([string]::IsNullOrWhiteSpace([string]$content)) { continue }
            $match = [regex]::Match([string]$content, 'https://[a-z0-9]+(?:-[a-z0-9]+)*\.trycloudflare\.com(?=[\s|/]|$)')
            if ($match.Success) { return $match.Value }
        }
        Start-Sleep -Milliseconds 500
    }
    throw 'Public tunnel did not allocate an address in time. No domain input is required; check outbound connectivity and retry.'
}

function Wait-PublicGateway([string]$Url, [string]$Nonce, $Proxy, $Tunnel, [int]$TimeoutSeconds = 75) {
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    do {
        if ($Proxy.HasExited -or $Tunnel.HasExited) { throw 'Public gateway process exited.' }
        try {
            $response = Invoke-WebRequest -Uri ($Url + '/api/health?deployment_probe=' + $Nonce) -TimeoutSec 8 -MaximumRedirection 0 -ErrorAction Stop
            if ($response.StatusCode -eq 200 -and ($response.Headers['X-Space-Sim-Deployment'] -join '') -eq $Nonce -and
                ($response.Content | ConvertFrom-Json).ok -eq $true) { return }
        } catch { } # DNS/edge registration can lag behind tunnel allocation.
        Start-Sleep -Seconds 2
    } while ([DateTime]::UtcNow -lt $deadline)
    throw 'The public HTTPS health probe failed. Local process startup is not proof of public availability; cleanup will close this attempted deployment.'
}

function Invoke-PublicAuth([string]$ProjectRoot, [string]$Username, [switch]$Apply) {
    $arguments = @((Join-Path $ProjectRoot 'tools/prepare_public_auth.py'), '--database', (Join-Path $ProjectRoot 'data/auth.sqlite3'), '--username', $Username)
    if ($Apply) { $arguments += @('--apply', '--backup-directory', (Join-Path $ProjectRoot 'run/auth-backups')) }
    $result = & python @arguments
    if ($LASTEXITCODE -ne 0) { throw 'Public authentication preflight/migration failed. Custom passwords were not reset.' }
    return ($result | Out-String | ConvertFrom-Json)
}

function Show-PublicAccess([string]$ProjectRoot, [string]$ConfigPath, [switch]$NonInteractive) {
    $report = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/public-access.json') | ConvertFrom-Json
    Write-Host "公网 HTTPS 入口已验证：$($report.public_url)"
    Write-Host '这是临时公网发布：重启隧道后网址会变化，不是稳定域名/生产 SLA。'
    $turnConfigured = $false
    try {
        $turnConfigured = @((Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json).turn_urls).Count -gt 0
    } catch { }
    if ($turnConfigured) {
        Write-Host '本机 STUN/TURN 已自动接入；UE 视频仍建议在实际用户网络和浏览器中验收。场景请从网页启动。'
    } else {
        Write-Host '网页入口已通；UE 视频还需 WebRTC 连通，部分网络需要 TURN。场景请从网页启动。'
    }
    $showWindow = $true
    try {
        $configured = Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json
        if ($configured.PSObject.Properties['show_access_window']) { $showWindow = [bool]$configured.show_access_window }
        elseif ($report.PSObject.Properties['show_access_window']) { $showWindow = [bool]$report.show_access_window }
    } catch {
        if ($report.PSObject.Properties['show_access_window']) { $showWindow = [bool]$report.show_access_window }
    }
    if ($showWindow) {
        Write-Host '访问链接和自动生成的登录信息在本机凭据窗口中查看，不会写入命令行/普通报告文件。'
    } else {
        Write-Host '已按配置跳过凭据窗口（show_access_window=false）。需要时运行 show_deployment_access.cmd。'
    }
    if ($showWindow -and !$NonInteractive) {
        try {
            $pwsh = (Get-Process -Id $PID).Path
            $null = Start-Process -FilePath $pwsh -ArgumentList @('-NoLogo', '-NoProfile', '-ExecutionPolicy', 'RemoteSigned', '-STA', '-File', ('"' + (Join-Path $ProjectRoot 'scripts/show_public_access.ps1') + '"'), '-ConfigPath', ('"' + $ConfigPath + '"')) -WindowStyle Hidden
        } catch { Write-Warning 'Credential window could not open. Run show_deployment_access.cmd to retry; services remain running.' }
    }
}

