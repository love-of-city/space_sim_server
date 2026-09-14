# Bootstrap helpers only. Dot-sourcing this file never starts/stops the platform.
function Read-LauncherText([string]$Prompt, [switch]$NonInteractive) {
    if ($NonInteractive) { throw "First-run input required: $Prompt. Run start_deployment.cmd interactively once." }
    $value = Read-Host $Prompt
    if ([string]::IsNullOrWhiteSpace($value)) { throw 'No value entered; existing services were not changed.' }
    return $value.Trim()
}

function Initialize-LauncherConfig([string]$ConfigPath, [string]$ProjectRoot, [switch]$NonInteractive) {
    if (Test-Path -LiteralPath $ConfigPath -PathType Leaf) { return }
    Write-Host '首次配置：填写平台对外提供的统一网址，只需一次。这里不是访问者 IP 白名单。'
    Write-Host '不同 IP 的用户访问同一网址，无需登记每个人的 IP；仍需登录和操作授权。'
    Write-Host '公网发布请使用已配置的公开域名和公网入口；脚本不会自动购买域名、分配公网 IP 或修改 DNS/防火墙。'
    $url = Read-LauncherText '请输入平台统一 HTTPS 网址（使用你实际配置的域名；不是用户电脑的 IP）' -NonInteractive:$NonInteractive
    if ($url -notmatch '^[A-Za-z][A-Za-z0-9+.-]*://') { $url = 'https://' + $url }
    $mode = Read-LauncherText '证书方式：面向公众的域名输入 acme；已有可信证书输入 manual；仅内网测试输入 internal' -NonInteractive:$NonInteractive
    $config = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/deployment.example.json') | ConvertFrom-Json -AsHashtable
    $config.public_url = $url
    $config.tls_mode = $mode.ToLowerInvariant()
    if ($config.tls_mode -eq 'manual') {
        $config.certificate_file = Read-LauncherText '证书文件完整路径' -NonInteractive:$NonInteractive
        $config.certificate_key_file = Read-LauncherText '证书私钥文件完整路径' -NonInteractive:$NonInteractive
    }
    # Validate a temporary sibling first; invalid/aborted input must not become saved settings.
    $directory = Split-Path -Parent $ConfigPath
    $null = New-Item -ItemType Directory -Path $directory -Force
    $temporary = Join-Path $directory ('.setup-' + [guid]::NewGuid().ToString('N') + '.local.json')
    try {
        $config | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $temporary -Encoding utf8NoBOM
        $null = Get-DeploymentSettings $temporary $ProjectRoot
        Move-Item -LiteralPath $temporary -Destination $ConfigPath -ErrorAction Stop
    } finally {
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary -Force }
    }
    Write-Host "已保存首次配置：$ConfigPath"
}

function Enable-LauncherRuntime([string]$CondaRoot) {
    $candidates = @($CondaRoot, $env:SPACE_SIM_CONDA_ROOT)
    if ($env:CONDA_EXE) { $candidates += Split-Path -Parent (Split-Path -Parent $env:CONDA_EXE) }
    $command = Get-Command conda.exe -ErrorAction SilentlyContinue
    if ($command) { $candidates += Split-Path -Parent (Split-Path -Parent $command.Source) }
    foreach ($base in @($env:USERPROFILE, $env:ProgramData)) {
        if ($base) {
            foreach ($name in @('miniconda3', 'anaconda3', 'miniforge3')) { $candidates += Join-Path $base $name }
        }
    }
    $root = $null
    foreach ($candidate in $candidates | Where-Object { $_ } | Select-Object -Unique) {
        if (Test-Path -LiteralPath ([IO.Path]::Combine($candidate, 'shell', 'condabin', 'Conda.psm1')) -PathType Leaf) {
            $root = [IO.Path]::GetFullPath($candidate)
            break
        }
    }
    if (!$root) { throw 'Cannot find the existing Conda installation. Set SPACE_SIM_CONDA_ROOT or use -CondaRoot; no new simulation environment will be installed.' }
    $env:CONDA_EXE = Join-Path $root 'Scripts/conda.exe'
    $env:_CONDA_ROOT = $root
    $env:_CONDA_EXE = $env:CONDA_EXE
    Import-Module (Join-Path $root 'shell/condabin/Conda.psm1') -Global -Force
    conda activate space-sim-server
    if ($LASTEXITCODE -ne 0 -or !$env:CONDA_PREFIX) { throw 'Unable to activate the existing space-sim-server Conda environment.' }
    $python = Join-Path $env:CONDA_PREFIX 'python.exe'
    $actualPython = Get-Command python -ErrorAction Stop
    if (!(Test-Path -LiteralPath $python -PathType Leaf) -or $actualPython.Source -ne $python) {
        throw 'Conda activation did not select the expected Python; refusing to use another interpreter.'
    }
    foreach ($name in @('npm.cmd', 'node.exe')) { $null = Get-Command $name -ErrorAction Stop }
    & $python -c 'import fastapi, pydantic, uvicorn'
    if ($LASTEXITCODE -ne 0) { throw 'The existing space-sim-server environment is missing backend dependencies.' }
}

function New-LauncherSecret([int]$Bytes) {
    $buffer = [byte[]]::new($Bytes)
    $random = [Security.Cryptography.RandomNumberGenerator]::Create()
    try { $random.GetBytes($buffer) } finally { $random.Dispose() }
    return [BitConverter]::ToString($buffer).Replace('-', '')
}

function Read-LauncherSecrets([string]$SecretPath, $Settings, [switch]$NonInteractive, [switch]$ReadOnly, [switch]$GenerateAdmin, [System.Collections.IDictionary]$Overrides = $null) {
    $saved = @{}
    if (Test-Path -LiteralPath $SecretPath) {
        try {
            $saved = Import-Clixml -LiteralPath $SecretPath -ErrorAction Stop
            if ($saved -isnot [System.Collections.IDictionary]) { throw 'Invalid secret container.' }
            foreach ($value in $saved.Values) {
                if ($value -isnot [securestring] -or $value.Length -eq 0) { throw 'Plaintext or empty secrets are not accepted.' }
            }
            foreach ($name in @('SPACE_SIM_ADMIN_PASSWORD', 'SPACE_SIM_STREAM_JWT_SECRET', 'SPACE_SIM_STREAM_ACCESS_KEY')) {
                if (!$saved.Contains($name)) { throw 'Incomplete saved secret store; do not silently rotate missing keys.' }
            }
        } catch {
            throw 'Cannot decrypt saved deployment secrets. Use the same Windows account on the same computer. The file was not overwritten; restore a backup rather than silently regenerating keys.'
        }
    }
    $values = @{}
    $names = @('SPACE_SIM_ADMIN_PASSWORD', 'SPACE_SIM_STREAM_JWT_SECRET', 'SPACE_SIM_STREAM_ACCESS_KEY')
    if ($Settings.HasTurn -or $saved.Contains('SPACE_SIM_TURN_AUTH_SECRET')) { $names += 'SPACE_SIM_TURN_AUTH_SECRET' }
    foreach ($name in $names) {
        if ($Overrides -and $Overrides.Contains($name) -and $Overrides[$name]) {
            # Auto-detected values (for example the current eturnal shared
            # secret) must win over a stale saved value; they are re-encrypted
            # by the caller unless the run is read-only validation.
            $values[$name] = [string]$Overrides[$name]
            continue
        }
        if ($saved.Contains($name)) {
            $values[$name] = [pscredential]::new('deployment', $saved[$name]).GetNetworkCredential().Password
        } else {
            $values[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
        }
        if ($values[$name]) { continue }
        if ($ReadOnly) { throw "Missing saved secret or environment variable: $name. Run start_deployment.cmd once to configure it." }
        if ($GenerateAdmin -and $name -eq 'SPACE_SIM_ADMIN_PASSWORD') {
            $values[$name] = New-LauncherSecret 24
            continue
        }
        switch ($name) {
            'SPACE_SIM_STREAM_JWT_SECRET' { $values[$name] = New-LauncherSecret 48 }
            'SPACE_SIM_STREAM_ACCESS_KEY' { $values[$name] = New-LauncherSecret 24 }
            default {
                if ($NonInteractive) { throw "First-run secret required: $name. Run start_deployment.cmd interactively once." }
                $prompt = if ($name -eq 'SPACE_SIM_ADMIN_PASSWORD') {
                    '管理员初始化密码（12～256 字符；不会替换已有账号密码）'
                } else { 'TURN 服务的 REST shared secret（必须与 TURN 服务一致）' }
                $secret = Read-Host $prompt -AsSecureString
                $confirm = Read-Host '请再次输入确认' -AsSecureString
                $plain = [pscredential]::new('deployment', $secret).GetNetworkCredential().Password
                if ($plain -cne [pscredential]::new('deployment', $confirm).GetNetworkCredential().Password) {
                    throw 'Secret confirmation did not match; saved secrets and services were not changed.'
                }
                $values[$name] = $plain
            }
        }
    }
    if ($values.SPACE_SIM_ADMIN_PASSWORD.Length -gt 256) { throw 'Administrator password must not exceed 256 characters.' }
    return $values
}

function Save-LauncherSecrets([string]$SecretPath, [System.Collections.IDictionary]$Values) {
    if (!$IsWindows) { throw 'This launcher requires Windows DPAPI; refusing to save secrets on another OS.' }
    $directory = Split-Path -Parent $SecretPath
    $null = New-Item -ItemType Directory -Path $directory -Force
    $protected = @{}
    foreach ($name in $Values.Keys) { $protected[$name] = ConvertTo-SecureString -String $Values[$name] -AsPlainText -Force }
    $temporary = Join-Path $directory ('.secrets-' + [guid]::NewGuid().ToString('N') + '.clixml')
    try {
        $protected | Export-Clixml -LiteralPath $temporary -Depth 4 -Encoding utf8
        Move-Item -LiteralPath $temporary -Destination $SecretPath -Force
    } finally {
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary -Force }
    }
}

function Install-LauncherCaddyArchive([string]$ArchivePath, [string]$ExpectedSha512, [string]$Destination) {
    if ($ExpectedSha512 -notmatch '^[a-fA-F0-9]{128}$' -or
        (Get-FileHash -LiteralPath $ArchivePath -Algorithm SHA512).Hash -ne $ExpectedSha512) {
        throw 'Caddy archive checksum mismatch. The download will not be executed.'
    }
    $archive = [IO.Compression.ZipFile]::OpenRead($ArchivePath)
    $temporary = $Destination + '.' + [guid]::NewGuid().ToString('N') + '.tmp'
    try {
        $entry = $archive.GetEntry('caddy.exe')
        if (!$entry) { throw 'Verified Caddy archive does not contain caddy.exe.' }
        # Extract only the named executable, never arbitrary archive paths.
        [IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $temporary, $false)
        Move-Item -LiteralPath $temporary -Destination $Destination -Force
    } finally {
        $archive.Dispose()
        if (Test-Path -LiteralPath $temporary) { Remove-Item -LiteralPath $temporary -Force }
    }
}

function Resolve-LauncherCaddy([string]$RequestedExecutable, [string]$ProjectRoot) {
    $requested = $RequestedExecutable
    if ($requested.Contains('/') -or $requested.Contains('\')) { $requested = [IO.Path]::GetFullPath($requested, $ProjectRoot) }
    $existing = Get-Command $requested -CommandType Application -ErrorAction SilentlyContinue
    if ($existing) { return $existing.Source }
    if ($RequestedExecutable -notin @('caddy', 'caddy.exe')) {
        throw 'The explicitly configured caddy_executable was not found. Correct that path in deployment.local.json.'
    }
    $manifest = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/caddy-release.json') | ConvertFrom-Json -AsHashtable
    $architecture = switch ([Runtime.InteropServices.RuntimeInformation]::OSArchitecture.ToString()) {
        'X64' { 'amd64' }
        'Arm64' { 'arm64' }
        default { throw 'Automatic Caddy setup supports only Windows x64/ARM64. Supply caddy_executable manually.' }
    }
    $version = [string]$manifest.version
    if ($version -notmatch '^\d+\.\d+\.\d+$') { throw 'Invalid pinned Caddy version.' }
    $asset = "caddy_${version}_windows_${architecture}.zip"
    $directory = Join-Path $ProjectRoot "run/deployment-tools/caddy-$version-$architecture"
    $executable = Join-Path $directory 'caddy.exe'
    if (Test-Path -LiteralPath $executable -PathType Leaf) { return $executable }
    $null = New-Item -ItemType Directory -Path $directory -Force
    $download = Join-Path $directory ([guid]::NewGuid().ToString('N') + '.download.zip')
    Write-Host "未找到 Caddy，正在下载固定版本 $version（首次联网，校验 SHA-512；不安装系统服务）……"
    try {
        Receive-DeploymentFile "https://github.com/caddyserver/caddy/releases/download/v$version/$asset" $download
        Install-LauncherCaddyArchive $download ([string]$manifest.sha512[$architecture]) $executable
    } catch {
        throw 'Caddy setup failed (network/download/checksum). Existing services were not stopped. Retry, or put an official caddy.exe on PATH / set caddy_executable in deployment.local.json.'
    } finally {
        if (Test-Path -LiteralPath $download) { Remove-Item -LiteralPath $download -Force }
    }
    return $executable
}

function Test-LauncherProcess([int]$ProcessId, [long]$StartTicks) {
    if ($ProcessId -le 0 -or $StartTicks -le 0) { return $false }
    $process = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    return $null -ne $process -and $process.StartTime.ToUniversalTime().Ticks -eq $StartTicks
}

function Test-LauncherRunning([string]$ProjectRoot, [string]$PublicUrl, [string]$ConfigPath, [string]$SecretPath) {
    try {
        $proxy = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/deployment.json') -ErrorAction Stop | ConvertFrom-Json
        $backend = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/platform.json') -ErrorAction Stop | ConvertFrom-Json
        $stream = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/pixel_streaming.json') -ErrorAction Stop | ConvertFrom-Json
        $marker = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/deployment-launcher.json') -ErrorAction Stop | ConvertFrom-Json
        if ($proxy.mode -eq 'public' -and (!(Test-LauncherProcess $proxy.tunnel_pid $proxy.tunnel_start) -or !$proxy.public_verified)) { return $false }
        return $marker.config_sha256 -eq (Get-FileHash -LiteralPath $ConfigPath -Algorithm SHA256 -ErrorAction Stop).Hash -and
            $marker.secrets_sha256 -eq (Get-FileHash -LiteralPath $SecretPath -Algorithm SHA256 -ErrorAction Stop).Hash -and
            $marker.proxy_pid -eq $proxy.proxy_pid -and $marker.proxy_start -eq $proxy.proxy_start -and
            $proxy.public_url -eq $PublicUrl -and
            (Test-LauncherProcess $proxy.proxy_pid $proxy.proxy_start) -and
            (Test-LauncherProcess $backend.backend_service_pid $backend.backend_service_start) -and
            (Test-LauncherProcess $stream.pid $stream.start_ticks)
    } catch { return $false }
}

function Save-LauncherRunMarker([string]$ProjectRoot, [string]$ConfigPath, [string]$SecretPath) {
    $proxy = Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'run/deployment.json') | ConvertFrom-Json
    @{
        config_sha256 = (Get-FileHash -LiteralPath $ConfigPath -Algorithm SHA256).Hash
        secrets_sha256 = (Get-FileHash -LiteralPath $SecretPath -Algorithm SHA256).Hash
        proxy_pid = $proxy.proxy_pid
        proxy_start = $proxy.proxy_start
    } | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $ProjectRoot 'run/deployment-launcher.json') -Encoding utf8NoBOM
}

function Show-LauncherAccess($Settings, [switch]$NonInteractive) {
    Write-Host "平台统一网址：$($Settings.PublicUrl)"
    Write-Host '用户不需要固定 IP，也不需要登记 IP；需通过该网址访问并登录。'
    if ($Settings.TlsMode -eq 'internal') {
        Write-Host '内网证书：必须在本地操作电脑信任 Caddy 的 CA 根证书；不要忽略浏览器证书警告。'
        $dataDirectory = if ($env:XDG_DATA_HOME) { Join-Path $env:XDG_DATA_HOME 'caddy' } else { Join-Path $env:APPDATA 'Caddy' }
        Write-Host ('默认本地 CA 根证书：' + (Join-Path $dataDirectory 'pki/authorities/local/root.crt') + '；不要分发 root.key。')
    }
    Write-Host '在本地电脑浏览器访问；登录后再从网页启动场景。证书、网络和 WebRTC 视频仍需实际验证。'
    if (!$Settings.RequireAccessKey) {
        Write-Warning '本次部署已关闭访问密钥：入口仅剩登录一道防线。'
    }
    if (!$NonInteractive) {
        $choice = Read-Host '输入 C 复制访问链接到剪贴板（含密钥，勿分享/记录）；直接回车结束'
        if ($choice -ieq 'c') {
            try {
                $linkValue = if ($Settings.RequireAccessKey) { $Settings.PublicUrl + '/?access_key=' + $env:SPACE_SIM_STREAM_ACCESS_KEY } else { $Settings.PublicUrl + '/' }
                Set-Clipboard -Value $linkValue -ErrorAction Stop
                Write-Host '访问链接已复制。请粘贴到本地浏览器地址栏；用完后清理剪贴板/历史。'
            } catch { Write-Warning 'Clipboard is unavailable; secrets were not printed. The platform is still running.' }
        }
    }
}

# HttpClient cancellation covers the response BODY as well as the connection.
# Invoke-WebRequest -TimeoutSec alone can stall indefinitely during a slow body transfer.
function Receive-DeploymentFile([string]$Uri, [string]$Destination, [int]$TimeoutSeconds = 300) {
    if (![uri]::IsWellFormedUriString($Uri, [UriKind]::Absolute) -or ([uri]$Uri).Scheme -ne 'https') { throw 'Tool downloads require HTTPS.' }
    $handler = [Net.Http.HttpClientHandler]::new()
    $client = [Net.Http.HttpClient]::new($handler)
    $cancel = [Threading.CancellationTokenSource]::new([TimeSpan]::FromSeconds($TimeoutSeconds))
    $response=$null; $inputStream=$null; $outputStream=$null
    try {
        $client.DefaultRequestHeaders.UserAgent.ParseAdd('SpaceSim-Deployment/1.0')
        $response = $client.GetAsync($Uri, [Net.Http.HttpCompletionOption]::ResponseHeadersRead, $cancel.Token).GetAwaiter().GetResult()
        $null=$response.EnsureSuccessStatusCode()
        $inputStream=$response.Content.ReadAsStreamAsync($cancel.Token).GetAwaiter().GetResult()
        $outputStream=[IO.File]::Open($Destination,[IO.FileMode]::CreateNew,[IO.FileAccess]::Write,[IO.FileShare]::None)
        $inputStream.CopyToAsync($outputStream,81920,$cancel.Token).GetAwaiter().GetResult()
    } finally {
        if($outputStream){$outputStream.Dispose()}; if($inputStream){$inputStream.Dispose()}; if($response){$response.Dispose()}
        $cancel.Dispose(); $client.Dispose(); $handler.Dispose()
    }
}

