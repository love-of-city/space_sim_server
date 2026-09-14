param(
    [string]$ConfigPath = (Join-Path (Split-Path -Parent $PSScriptRoot) 'deploy/deployment.local.json'),
    [switch]$ValidateOnly,
    [switch]$Start
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7) { throw 'Use PowerShell 7 (pwsh) for deployment.' }
if ($Start -and $ValidateOnly) { throw 'Choose either -Start or -ValidateOnly.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'deployment_config.ps1')
$settings = Get-DeploymentSettings $ConfigPath $projectRoot
Assert-DeploymentSecrets $settings
$caddyfile = Get-DeploymentCaddyfile $settings $projectRoot
if (!$Start) {
    Write-Output "Deployment settings and secret presence validated: $($settings.PublicUrl)"
    Write-Output 'No files, services, firewall, certificates or running scenes were changed.'
    Write-Output 'This is not a network/TLS/GPU connectivity check. Use -Start only when ready to stop the existing platform.'
    return
}

# All deterministic checks precede run_platform.ps1, which stops the old platform.
$caddy = (Get-Command $settings.CaddyExecutable -ErrorAction Stop).Source
foreach ($command in @('python', 'conda', 'npm.cmd', 'node.exe')) { $null = Get-Command $command -ErrorAction Stop }
& python (Join-Path $projectRoot 'tools/check_deployment_auth.py') --database (Join-Path $projectRoot 'data/auth.sqlite3')
if ($LASTEXITCODE -ne 0) { throw 'Authentication preflight failed; existing platform was not stopped.' }

$runDirectory = Join-Path $projectRoot 'run/deployment'
$logDirectory = Join-Path $projectRoot 'logs'
New-Item -ItemType Directory -Path $runDirectory, $logDirectory -Force | Out-Null
$caddyConfigPath = Join-Path $runDirectory 'Caddyfile'
$caddyfile | Set-Content -LiteralPath $caddyConfigPath -Encoding utf8NoBOM
& $caddy validate --config $caddyConfigPath --adapter caddyfile
if ($LASTEXITCODE -ne 0) { throw 'Caddy configuration validation failed; existing platform was not stopped.' }

# Do not log secrets or pass them through flattened native-process arguments.
$environmentNames = @('SPACE_SIM_FORWARDED_ALLOW_IPS', 'SPACE_SIM_STREAM_JWT_SECRET', 'SPACE_SIM_STREAM_ACCESS_KEY',
    'SPACE_SIM_ADMIN_USERNAME', 'SPACE_SIM_ADMIN_PASSWORD', 'SPACE_SIM_SECURE_COOKIES', 'SPACE_SIM_ALLOWED_ORIGINS',
    'SPACE_SIM_NO_ACCESS_LOG', 'SPACE_SIM_RUNTIME_ADAPTER_ROOT', 'SPACE_SIM_RUNTIME_MODEL_ROOT',
    'SPACE_SIM_RUNTIME_UNREAL_ROOT', 'SPACE_SIM_RUNTIME_POWERSHELL_EXE')
$previous = @{}
foreach ($name in $environmentNames) { $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process') }
$platformStarted = $false
$proxy = $null
try {
    $env:SPACE_SIM_FORWARDED_ALLOW_IPS = '127.0.0.1,::1'
    $arguments = @{
        RemoteAccess = $true; PublicOperatorUrl = $settings.PublicUrl
        ApiHost = '127.0.0.1'; PixelPlayerHost = '127.0.0.1'
        SecureCookies = $true; NoAccessLog = $true; NoBrowser = $true
        AllowedOriginsJson = ConvertTo-Json -InputObject @($settings.PublicUrl) -Compress
        PixelPlayerPublicUrl = ($settings.PublicUrl -replace '^https:', 'wss:') + '/stream'
        AdminUsername = $settings.AdminUsername; AdminPassword = $env:SPACE_SIM_ADMIN_PASSWORD
        StreamJwtSecret = $env:SPACE_SIM_STREAM_JWT_SECRET
        StreamAccessKey = $(if ($settings.RequireAccessKey) { $env:SPACE_SIM_STREAM_ACCESS_KEY } else { '' })
        RequireAccessKey = $settings.RequireAccessKey
        IceServersJson = $settings.IceServersJson; TurnUrlsJson = $settings.TurnUrlsJson
        TurnAuthSecret = $env:SPACE_SIM_TURN_AUTH_SECRET
        ApiPort = $settings.Ports.api_port; PixelPlayerPort = $settings.Ports.player_port
        PixelStreamerPort = $settings.Ports.streamer_port; ControlPort = $settings.Ports.control_port
        CapturePort = $settings.Ports.capture_port; RenderPort = $settings.Ports.render_port
    }
    foreach ($entry in @{AdapterRoot='adapter_root'; ModelRoot='model_root'; UnrealRoot='unreal_root'}.GetEnumerator()) {
        if ($settings.Paths[$entry.Value]) { $arguments[$entry.Key] = $settings.Paths[$entry.Value] }
    }
    & (Join-Path $PSScriptRoot 'run_platform.ps1') @arguments
    $platformStarted = $true
    $proxy = Start-Process -FilePath $caddy -ArgumentList @('run', '--config', ('"' + $caddyConfigPath + '"'), '--adapter', 'caddyfile') `
        -WorkingDirectory $projectRoot -PassThru -WindowStyle Hidden `
        -RedirectStandardOutput (Join-Path $logDirectory 'deployment-proxy.out.log') `
        -RedirectStandardError (Join-Path $logDirectory 'deployment-proxy.err.log')
    @{
        proxy_pid = $proxy.Id; proxy_start = $proxy.StartTime.ToUniversalTime().Ticks
        public_url = $settings.PublicUrl; config_path = $caddyConfigPath
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $projectRoot 'run/deployment.json') -Encoding utf8
    Start-Sleep -Seconds 2
    if ($proxy.HasExited) { throw 'HTTPS proxy exited. Check logs/deployment-proxy.err.log.' }
    if ($settings.TlsMode -in @('acme', 'ip-acme')) {
        . (Join-Path $PSScriptRoot 'fixed_deployment_helpers.ps1')
        Wait-FixedGateway $settings.PublicUrl 180
        $showConfigPath = (Resolve-Path -LiteralPath $ConfigPath).Path
        @{
            mode = $(if ($settings.TlsMode -eq 'ip-acme') { 'ip' } else { 'fixed' })
            public_url = $settings.PublicUrl
            config_path = $showConfigPath
            generated_password_users = @()
            existing_admin_users = @($settings.AdminUsername)
            require_access_key = [bool]$settings.RequireAccessKey
            show_access_window = [bool]$settings.ShowAccessWindow
        } | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath (Join-Path $projectRoot 'run/public-access.json') -Encoding utf8NoBOM
    }
    Write-Output "Deployment processes started: $($settings.PublicUrl)"
    Write-Output 'Certificate issuance and remote WebRTC connectivity still require verification from your LOCAL browser.'
    Write-Output 'Open the HTTPS origin with /?access_key=<your SPACE_SIM_STREAM_ACCESS_KEY>, then log in.'
    Write-Output 'Secrets are intentionally not printed. Stop with scripts/stop_platform.ps1.'
} catch {
    # Also clean up our own child if writing its PID record failed.
    if ($proxy -and !$proxy.HasExited) { $proxy.Kill() }
    if ($platformStarted) { & (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet }
    throw
} finally {
    foreach ($name in $environmentNames) { [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process') }
}

