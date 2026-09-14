param(
    [ValidateSet('Start', 'Stop')][string]$Action = 'Start',
    [ValidateSet('Auto', 'Fixed', 'Ip', 'Turn', 'Public', 'Direct')][string]$Mode = 'Auto',
    [string]$ConfigPath = '',
    [string]$CondaRoot = '',
    [switch]$ValidateOnly,
    [switch]$NonInteractive,
    [switch]$Restart
)

$ErrorActionPreference = 'Stop'
if ($PSVersionTable.PSVersion.Major -lt 7 -or !$IsWindows) { throw 'Use PowerShell 7 on Windows.' }
# Keep Chinese diagnostics readable in redirected logs and non-UTF-8 Windows consoles.
[Console]::OutputEncoding = [Text.UTF8Encoding]::new($false)
$OutputEncoding = [Console]::OutputEncoding
if ($Action -eq 'Stop' -and ($ValidateOnly -or $Restart)) { throw 'Stop cannot be combined with ValidateOnly or Restart.' }
if ($ValidateOnly -and $Restart) { throw 'ValidateOnly cannot be combined with Restart.' }
$projectRoot = Split-Path -Parent $PSScriptRoot
$fixedHelpers = Join-Path $PSScriptRoot 'fixed_deployment_helpers.ps1'

function Get-FixedCandidateSafely([string]$Root, [string]$Helpers) {
    if (!(Test-Path -LiteralPath $Helpers -PathType Leaf)) { return $null }
    . $Helpers
    return Get-FixedDeploymentCandidate -ProjectRoot $Root
}

if ($Mode -eq 'Auto') {
    if ($Action -eq 'Stop') {
        # Stop never depends on deployment settings.
        $Mode = 'Direct'
    } elseif ($ConfigPath) {
        $Mode = 'Direct'
        $candidatePath = [IO.Path]::GetFullPath($ConfigPath, $projectRoot)
        if ((Test-Path -LiteralPath $candidatePath -PathType Leaf) -and
            (Get-Content -Raw -LiteralPath $candidatePath | ConvertFrom-Json).tls_mode -eq 'tunnel') { $Mode = 'Public' }
    } else {
        $hasDirect = Test-Path -LiteralPath (Join-Path $projectRoot 'deploy/deployment.local.json') -PathType Leaf
        $hasIp = Test-Path -LiteralPath (Join-Path $projectRoot 'deploy/ip.local.json') -PathType Leaf
        $hasTurn = Test-Path -LiteralPath (Join-Path $projectRoot 'deploy/turn.local.json') -PathType Leaf
        if ($hasDirect) {
            # Never overwrite a hand-configured deployment.
            $Mode = 'Direct'
        } elseif ($hasIp) {
            # A stable public-IP entry point beats a rotating tunnel URL.
            $Mode = 'Ip'
        } elseif ($hasTurn) {
            $Mode = 'Turn'
        } else {
            $candidate = $null
            try { $candidate = Get-FixedCandidateSafely $projectRoot $fixedHelpers }
            catch { Write-Warning ("TURN 入口探测失败，将回退到临时公网隧道：" + $_.Exception.Message) }
            $Mode = if ($candidate) { 'Turn' } else { 'Public' }
        }
    }
}
if (!$ConfigPath) {
    $ConfigPath = Join-Path $projectRoot $(switch ($Mode) {
        'Public' { 'deploy/public.local.json' }
        'Turn'   { 'deploy/turn.local.json' }
        'Fixed'  { 'deploy/fixed.local.json' }
        'Ip'     { 'deploy/ip.local.json' }
        default  { 'deploy/deployment.local.json' }
    })
}
$ConfigPath = [IO.Path]::GetFullPath($ConfigPath, $projectRoot)
. (Join-Path $PSScriptRoot 'deployment_config.ps1')
. (Join-Path $PSScriptRoot 'deployment_bootstrap.ps1')

# Both desktop launchers share a lock. A second click must not race startup/cleanup.
$hash = [Security.Cryptography.SHA256]::Create()
try {
    $projectId = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($projectRoot.ToLowerInvariant()))).Replace('-', '')
    $configId = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($ConfigPath.ToLowerInvariant()))).Replace('-', '')
} finally { $hash.Dispose() }
$secretPath = Join-Path $projectRoot "deploy/secrets/$configId.clixml"
$mutex = [Threading.Mutex]::new($false, "Local\SpaceSimDeployment-$projectId")
$locked = $false
$previous = @{}
Push-Location -LiteralPath $projectRoot
try {
    try { $locked = $mutex.WaitOne(0) } catch [Threading.AbandonedMutexException] { $locked = $true }
    if (!$locked) { throw 'Another start/stop operation is still running. Let it finish before clicking again.' }
    if ($Action -eq 'Stop') {
        & (Join-Path $PSScriptRoot 'stop_platform.ps1')
        return
    }
    if ($Mode -in @('Public', 'Turn')) {
        . (Join-Path $PSScriptRoot 'public_deployment_helpers.ps1')
        . (Join-Path $PSScriptRoot 'public_deployment.ps1')
        $turnOverrides = @{}
        if ($Mode -eq 'Turn') {
            . $fixedHelpers
            $turnCandidate = Get-FixedDeploymentCandidate -ProjectRoot $projectRoot
            if (!$turnCandidate) {
                throw 'TURN mode is unavailable: the local eturnal configuration or public IPv4 could not be detected. Use -Mode Public to fall back to the temporary tunnel.'
            }
            if ($turnCandidate.TurnSecretIsPlaceholder) {
                Write-Warning 'TURN 仍在使用 eturnal 安装默认占位 secret；当前部署可运行，但请让管理员尽快更换为随机 secret。'
            }
            if ($ValidateOnly -and !(Test-Path -LiteralPath $ConfigPath -PathType Leaf)) {
                Write-Host "TURN 模式配置将在首次启动时自动生成；入口由隧道分配，TURN=$($turnCandidate.TurnUrls -join ', ')"
                Write-Host '本次验证没有创建文件，也没有启停服务。'
                return
            }
            if (!$ValidateOnly) {
                Initialize-FixedConfig -ConfigPath $ConfigPath -ProjectRoot $projectRoot -Candidate $turnCandidate -TlsMode tunnel
            }
            $turnOverrides['SPACE_SIM_TURN_AUTH_SECRET'] = [string]$turnCandidate.TurnSecret
        }
        Start-PublicDeployment $projectRoot $ConfigPath $secretPath $CondaRoot -ValidateOnly:$ValidateOnly `
            -NonInteractive:$NonInteractive -Restart:$Restart -Overrides $turnOverrides
        return
    }

    $turnOverrides = @{}
    if ($Mode -in @('Fixed', 'Ip')) {
        . $fixedHelpers
        $fixedCandidate = Get-FixedDeploymentCandidate -ProjectRoot $projectRoot
        if (!$fixedCandidate) {
            throw "$Mode deployment is unavailable: the local eturnal configuration or public IPv4 could not be detected. Use -Mode Public to fall back to the temporary tunnel."
        }
        if ($fixedCandidate.TurnSecretIsPlaceholder) {
            Write-Warning 'TURN 仍在使用 eturnal 安装默认占位 secret；当前部署可运行，但请让管理员尽快更换为随机 secret。'
        }
        $ipUrl = "https://$($fixedCandidate.PublicIp)"
        if ($ValidateOnly -and !(Test-Path -LiteralPath $ConfigPath -PathType Leaf)) {
            if ($Mode -eq 'Ip') {
                Write-Host "公网 IP 证书入口将在首次启动时自动申请：$ipUrl"
                Write-Host '该模式不需要域名、证书或密码，也不需要 80/443 之外的任何入站端口；本次验证没有创建文件，也没有启停服务。'
            } else {
                Write-Host "固定公网配置将在首次启动时自动生成：$($fixedCandidate.PublicUrl)"
                Write-Host '固定模式不需要输入域名、IP、证书或密码；本次验证没有创建文件，也没有启停服务。'
            }
            return
        }
        if (!$ValidateOnly) {
            if ($Mode -eq 'Ip') {
                Initialize-FixedConfig -ConfigPath $ConfigPath -ProjectRoot $projectRoot -Candidate $fixedCandidate -TlsMode 'ip-acme' -PublicUrl $ipUrl
            } else {
                Initialize-FixedConfig -ConfigPath $ConfigPath -ProjectRoot $projectRoot -Candidate $fixedCandidate
            }
        }
        $turnOverrides['SPACE_SIM_TURN_AUTH_SECRET'] = [string]$fixedCandidate.TurnSecret
    }

    if (!$ValidateOnly) {
        Initialize-LauncherConfig $ConfigPath $projectRoot -NonInteractive:$NonInteractive
    }
    $settings = Get-DeploymentSettings $ConfigPath $projectRoot
    $values = Read-LauncherSecrets $secretPath $settings -NonInteractive:$NonInteractive -ReadOnly:$ValidateOnly -GenerateAdmin:($Mode -in @('Fixed', 'Ip')) -Overrides $turnOverrides
    # Never put credentials on a native command line or in console output.
    foreach ($name in $values.Keys) {
        $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
        [Environment]::SetEnvironmentVariable($name, $values[$name], 'Process')
    }
    Assert-DeploymentSecrets $settings
    if ($ValidateOnly) {
        & (Join-Path $PSScriptRoot 'deploy_platform.ps1') -ConfigPath $ConfigPath -ValidateOnly
        return
    }
    if (!$Restart -and (Test-LauncherRunning $projectRoot $settings.PublicUrl $ConfigPath $secretPath)) {
        Write-Host '已记录的部署进程仍在运行，本次不会停止场景或重复启动。需要重启时使用 -Restart。'
        # Do not hold the lifecycle lock while the user copies a link.
        $mutex.ReleaseMutex(); $locked = $false
        if ($Mode -in @('Fixed', 'Ip')) { Show-FixedAccess $projectRoot $ConfigPath -NonInteractive:$NonInteractive }
        else { Show-LauncherAccess $settings -NonInteractive:$NonInteractive }
        return
    }
    Save-LauncherSecrets $secretPath $values
    Write-Host '正在加载现有仿真环境……'
    Enable-LauncherRuntime $CondaRoot
    # Keep the default-password gate read-only. Never silently reset an existing account.
    & python (Join-Path $projectRoot 'tools/check_deployment_auth.py') --database (Join-Path $projectRoot 'data/auth.sqlite3')
    if ($LASTEXITCODE -ne 0) {
        throw 'Authentication preflight failed; no running services were stopped. If an existing admin still uses the default password, change it in the current local webpage, then double-click Start again.'
    }
    $caddy = Resolve-LauncherCaddy $settings.CaddyExecutable $projectRoot
    # Keep the user's config unchanged. A cached caddy.exe is resolved through this
    # launcher's child-process PATH, so deleting run/ does not leave a stale config path.
    $previous['PATH'] = $env:PATH
    $env:PATH = (Split-Path -Parent $caddy) + [IO.Path]::PathSeparator + $env:PATH
    Write-Host '准备切换平台：将停止旧平台/场景并重新构建前端。'
    & (Join-Path $PSScriptRoot 'deploy_platform.ps1') -ConfigPath $ConfigPath -Start
    Save-LauncherRunMarker $projectRoot $ConfigPath $secretPath
    $mutex.ReleaseMutex(); $locked = $false
    if ($Mode -in @('Fixed', 'Ip')) { Show-FixedAccess $projectRoot $ConfigPath -NonInteractive:$NonInteractive }
    else { Show-LauncherAccess $settings -NonInteractive:$NonInteractive }
} finally {
    foreach ($name in $previous.Keys) { [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process') }
    if ($locked) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
    Pop-Location
}




