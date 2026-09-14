# Called under deployment_launcher.ps1's shared lifecycle lock.
function Start-PublicDeployment([string]$ProjectRoot, [string]$ConfigPath, [string]$SecretPath, [string]$CondaRoot,
    [switch]$ValidateOnly, [switch]$NonInteractive, [switch]$Restart, [System.Collections.IDictionary]$Overrides = $null) {
    $previous = @{}
    $proxy = $null; $tunnel = $null; $switched = $false; $complete = $false
    $pending = Join-Path $ProjectRoot 'run/public-pending.json'
    try {
        if (!$ValidateOnly) { Initialize-PublicConfig $ConfigPath $ProjectRoot }
        $settings = Get-DeploymentSettings $ConfigPath $ProjectRoot
        if ($settings.TlsMode -ne 'tunnel') { throw 'Public automatic mode requires its own tunnel configuration; direct HTTPS settings were not overwritten.' }
        $values = Read-LauncherSecrets $SecretPath $settings -ReadOnly:$ValidateOnly -GenerateAdmin -NonInteractive -Overrides $Overrides
        foreach ($name in $values.Keys) {
            $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
            [Environment]::SetEnvironmentVariable($name, $values[$name], 'Process')
        }
        Assert-DeploymentSecrets $settings
        if ($ValidateOnly) {
            Write-Host 'Public settings and saved secrets validated. No downloads, files, services or network changes. This does not verify a live public endpoint.'
            return
        }
        if (!$Restart -and (Test-LauncherRunning $ProjectRoot $settings.PublicUrl $ConfigPath $SecretPath)) {
            Show-PublicAccess $ProjectRoot $ConfigPath -NonInteractive:$NonInteractive
            return
        }
        Save-LauncherSecrets $SecretPath $values
        Enable-LauncherRuntime $CondaRoot
        $null = Invoke-PublicAuth $ProjectRoot $settings.AdminUsername
        $caddy = Resolve-LauncherCaddy $settings.CaddyExecutable $ProjectRoot
        $cloudflared = Resolve-PublicTunnel $ProjectRoot
        Stop-PublicPending $ProjectRoot
        $run = Join-Path $ProjectRoot 'run/public-deployment'
        $logs = Join-Path $ProjectRoot 'logs'
        $null = New-Item -ItemType Directory -Path $run, $logs -Force
        do { $port = Get-PublicLoopbackPort } while ($port -in $settings.Ports.Values)
        do { $adminPort = Get-PublicLoopbackPort } while ($adminPort -eq $port -or $adminPort -in $settings.Ports.Values)
        $nonce = New-LauncherSecret 16
        $configFile = Join-Path $run 'Caddyfile'
        Get-PublicProxyConfig $settings $port $adminPort $nonce -Holding | Set-Content -LiteralPath $configFile -Encoding utf8NoBOM
        & $caddy validate --config $configFile --adapter caddyfile
        if ($LASTEXITCODE -ne 0) { throw 'Public gateway holding configuration is invalid; old services were not stopped.' }
        $proxy = Start-PublicChild $caddy @('run','--config',('"'+$configFile+'"'),'--adapter','caddyfile') (Join-Path $logs 'public-proxy') $ProjectRoot
        Save-PublicPending $pending $proxy $null
        # Confirm our own closed gateway bound successfully before starting any public tunnel.
        $ready = $false
        for ($i=0; $i -lt 30; $i++) {
            if ($proxy.HasExited) { throw 'Loopback gateway failed to bind; nothing was published.' }
            try {
                $response = Invoke-WebRequest -Uri "http://127.0.0.1:$port/" -SkipHttpErrorCheck -TimeoutSec 2
                if ($response.StatusCode -eq 503 -and ($response.Headers['X-Space-Sim-Preparing'] -join '') -eq $nonce -and $response.Content -eq 'Public deployment is preparing; no application is exposed yet.') { $ready=$true; break }
            } catch { }
            Start-Sleep -Milliseconds 200
        }
        if (!$ready) { throw 'Closed gateway readiness check failed; nothing was published.' }
        $emptyConfig = Join-Path $run 'cloudflared-empty.yml'
        '# Intentionally empty: do not inherit another tunnel configuration.' | Set-Content -LiteralPath $emptyConfig -Encoding utf8NoBOM
        $tunnelArguments = @('tunnel','--config',('"'+$emptyConfig+'"'),'--no-autoupdate','--url',"http://127.0.0.1:$port",'--metrics','127.0.0.1:0','--loglevel','info')
        # Quick Tunnel allocation can fail on a transient API timeout. Retry only
        # while our gateway is closed, with a fresh child/log and bounded attempts.
        for ($attempt = 1; $attempt -le 3; $attempt++) {
            if ($proxy.HasExited) { throw 'Closed gateway exited before public tunnel allocation.' }
            $tunnelLogs = Join-Path $logs ('public-tunnel-' + [guid]::NewGuid().ToString('N'))
            $tunnel = Start-PublicChild $cloudflared $tunnelArguments $tunnelLogs $ProjectRoot
            Save-PublicPending $pending $proxy $tunnel
            try {
                $url = Wait-PublicTunnelUrl $tunnel $tunnelLogs
                break
            } catch {
                if (!$tunnel.HasExited) { $tunnel.Kill(); $null = $tunnel.WaitForExit(5000) }
                $tunnel = $null
                Save-PublicPending $pending $proxy $null
                if ($attempt -eq 3) {
                    throw "Public tunnel allocation failed after 3 attempts; no application was exposed. Check outbound connectivity. Local diagnostic log: $tunnelLogs.err.log (redact URLs before sharing)."
                }
                Write-Warning "公网隧道申请暂未成功（第 $attempt/3 次），正在自动重试；应用网关尚未开放。诊断日志：$tunnelLogs.err.log"
                Start-Sleep -Seconds (2 * $attempt)
            }
        }
        $config = Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json -AsHashtable
        $config.public_url = $url
        $config | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $ConfigPath -Encoding utf8NoBOM
        $settings = Get-DeploymentSettings $ConfigPath $ProjectRoot
        Save-PublicPending $pending $proxy $tunnel $url
        Write-Host "已分配临时公网地址：$url；网关仍关闭，正在切换安全后端。"
        # Public traffic still receives only 503 here. Rotate only known default
        # administrator passwords after stopping the old backend; preserve custom accounts.
        $switched = $true
        & (Join-Path $ProjectRoot 'scripts/stop_platform.ps1') -Quiet -KeepPendingPublic
        $auth = Invoke-PublicAuth $ProjectRoot $settings.AdminUsername -Apply
        if (@($auth.rotated_default_users).Count -gt 0 -and $auth.rotated_default_users) {
            Write-Host '已有管理员的默认密码已更换为自动生成的强密码；旧登录会话已撤销，数据库已备份。自定义密码保持不变。'
        }
        $previous['SPACE_SIM_FORWARDED_ALLOW_IPS'] = $env:SPACE_SIM_FORWARDED_ALLOW_IPS
        $env:SPACE_SIM_FORWARDED_ALLOW_IPS = '127.0.0.1,::1'
        $arguments = @{
            RemoteAccess=$true; PublicOperatorUrl=$url; ApiHost='127.0.0.1'; PixelPlayerHost='127.0.0.1'
            SecureCookies=$true; NoAccessLog=$true; NoBrowser=$true; KeepPendingPublic=$true
            AllowedOriginsJson=(ConvertTo-Json -InputObject @($url) -Compress)
            PixelPlayerPublicUrl=($url -replace '^https:', 'wss:') + '/stream'
            AdminUsername=$settings.AdminUsername; AdminPassword=$env:SPACE_SIM_ADMIN_PASSWORD
            StreamJwtSecret=$env:SPACE_SIM_STREAM_JWT_SECRET
            StreamAccessKey=$(if ($settings.RequireAccessKey) { $env:SPACE_SIM_STREAM_ACCESS_KEY } else { '' })
            RequireAccessKey=$settings.RequireAccessKey
            IceServersJson=$settings.IceServersJson; TurnUrlsJson=$settings.TurnUrlsJson; TurnAuthSecret=$env:SPACE_SIM_TURN_AUTH_SECRET
            ApiPort=$settings.Ports.api_port; PixelPlayerPort=$settings.Ports.player_port; PixelStreamerPort=$settings.Ports.streamer_port
            ControlPort=$settings.Ports.control_port; CapturePort=$settings.Ports.capture_port; RenderPort=$settings.Ports.render_port
        }
        foreach ($entry in @{AdapterRoot='adapter_root'; ModelRoot='model_root'; UnrealRoot='unreal_root'}.GetEnumerator()) {
            if ($settings.Paths[$entry.Value]) { $arguments[$entry.Key]=$settings.Paths[$entry.Value] }
        }
        & (Join-Path $ProjectRoot 'scripts/run_platform.ps1') @arguments
        # Open the gateway only after the authenticated backend/signalling have started.
        Get-PublicProxyConfig $settings $port $adminPort $nonce | Set-Content -LiteralPath $configFile -Encoding utf8NoBOM
        & $caddy reload --config $configFile --adapter caddyfile --address "127.0.0.1:$adminPort"
        if ($LASTEXITCODE -ne 0) { throw 'Public gateway activation failed.' }
        Wait-PublicGateway $url $nonce $proxy $tunnel
        $record = Get-Content -Raw -LiteralPath $pending | ConvertFrom-Json -AsHashtable
        $record.public_verified=$true
        $record.config_path=$configFile
        $record | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $ProjectRoot 'run/deployment.json') -Encoding utf8NoBOM
        Remove-Item -LiteralPath $pending -Force
        @{public_url=$url; config_path=$ConfigPath; generated_password_users=@($auth.generated_password_users);
            existing_admin_users=@($auth.existing_admin_users); require_access_key=[bool]$settings.RequireAccessKey;
            show_access_window=[bool]$settings.ShowAccessWindow} |
            ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $ProjectRoot 'run/public-access.json') -Encoding utf8NoBOM
        Save-LauncherRunMarker $ProjectRoot $ConfigPath $SecretPath
        $complete=$true
        Show-PublicAccess $ProjectRoot $ConfigPath -NonInteractive:$NonInteractive
    } finally {
        if (!$complete -and ($proxy -or $tunnel)) {
            foreach ($child in @($tunnel,$proxy)) { if ($child -and !$child.HasExited) { $child.Kill(); $null=$child.WaitForExit(5000) } }
            if (Test-Path -LiteralPath $pending) { Remove-Item -LiteralPath $pending -Force }
            if ($switched) { & (Join-Path $ProjectRoot 'scripts/stop_platform.ps1') -Quiet }
        }
        foreach ($name in $previous.Keys) { [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process') }
    }
}

