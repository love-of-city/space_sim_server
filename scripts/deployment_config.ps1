# Pure validation/rendering helpers; dot-sourcing does not touch services or files.
function Get-DeploymentSettings([string]$ConfigPath, [string]$ProjectRoot) {
    $config = Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json -AsHashtable
    if ($config -isnot [System.Collections.IDictionary]) { throw 'Deployment config must be a JSON object.' }
    $allowed = @('public_url', 'tls_mode', 'certificate_file', 'certificate_key_file', 'caddy_executable',
        'api_port', 'player_port', 'streamer_port', 'control_port', 'capture_port', 'render_port',
        'admin_username', 'require_access_key', 'show_access_window', 'adapter_root', 'model_root', 'unreal_root', 'ice_servers', 'turn_urls')
    foreach ($key in $config.Keys) { if ($key -notin $allowed) { throw "Unknown deployment setting: $key" } }
    $url = [uri][string]$config.public_url
    if (!$url.IsAbsoluteUri -or $url.Scheme -ne 'https' -or !$url.Host -or $url.UserInfo -or
        $url.Query -or $url.Fragment -or $url.AbsolutePath -ne '/' -or $url.Host.Contains('*')) {
        throw 'public_url must be an HTTPS origin without credentials, path, query or fragment.'
    }
    if ($url.Host -eq 'sim.example.com') { throw 'Replace sim.example.com with your real hostname before deployment.' }
    $publicUrl = $url.GetLeftPart([UriPartial]::Authority)
    $tlsMode = [string]$config.tls_mode
    if ($tlsMode -notin @('acme', 'ip-acme', 'internal', 'manual', 'tunnel')) { throw 'tls_mode must be acme, ip-acme, internal, manual or tunnel.' }
    if ($tlsMode -eq 'ip-acme') {
        $parsedIp = $null
        if (![Net.IPAddress]::TryParse($url.Host, [ref]$parsedIp)) { throw 'tls_mode ip-acme requires public_url to use a literal IP address.' }
        if ($url.Port -ne 443) { throw 'tls_mode ip-acme requires the default HTTPS port 443.' }
    }
    $ports = @{}
    foreach ($key in @('api_port', 'player_port', 'streamer_port', 'control_port', 'capture_port', 'render_port')) {
        $value = $config[$key]
        if (($value -isnot [int] -and $value -isnot [long]) -or $value -lt 1024 -or $value -gt 65535) {
            throw "$key must be an integer in [1024, 65535]."
        }
        $ports[$key] = [int]$value
    }
    if (($ports.Values | Sort-Object -Unique).Count -ne $ports.Count -or $url.Port -in $ports.Values) {
        throw 'Internal ports must be distinct and must not equal the HTTPS listener port.'
    }
    $paths = @{}
    foreach ($key in @('adapter_root', 'model_root', 'unreal_root', 'certificate_file', 'certificate_key_file')) {
        $value = [string]$config[$key]
        if ($value) {
            $value = [IO.Path]::GetFullPath($value, $ProjectRoot)
            if (!(Test-Path -LiteralPath $value)) { throw "Configured path does not exist: $key" }
        }
        $paths[$key] = $value
    }
    if ($tlsMode -eq 'manual' -and (!$paths.certificate_file -or !$paths.certificate_key_file)) {
        throw 'manual TLS requires certificate_file and certificate_key_file.'
    }
    if ($tlsMode -eq 'manual' -and (!(Test-Path -LiteralPath $paths.certificate_file -PathType Leaf) -or
        !(Test-Path -LiteralPath $paths.certificate_key_file -PathType Leaf))) { throw 'TLS certificate paths must be files.' }
    foreach ($key in @('ice_servers', 'turn_urls')) {
        if ($config[$key] -isnot [array]) { throw "$key must be a JSON array." }
    }
    foreach ($server in $config.ice_servers) {
        if ($server -isnot [System.Collections.IDictionary] -or !$server.urls) { throw 'Every ICE server requires urls.' }
        foreach ($key in $server.Keys) { if ($key -notin @('urls')) { throw 'Put TURN credentials in the secret environment, not ice_servers.' } }
        foreach ($endpoint in @($server.urls)) {
            if ($endpoint -isnot [string] -or $endpoint -notmatch '^stuns?:[^\s]+$') { throw 'ice_servers only accepts STUN URLs; use turn_urls for TURN.' }
        }
    }
    foreach ($endpoint in $config.turn_urls) {
        if ($endpoint -isnot [string] -or $endpoint -notmatch '^turns?:[^\s]+$') { throw 'turn_urls must contain turn:/turns: URLs.' }
    }
    if ([string]$config.admin_username -notmatch '^[A-Za-z0-9_.-]{3,64}$') { throw 'Invalid admin_username.' }
    if (![string]$config.caddy_executable) { throw 'caddy_executable is required.' }
    # Absent means enabled: configs written before this option keep the secure default.
    $requireAccessKey = $true
    if ($config.ContainsKey('require_access_key')) {
        if ($config.require_access_key -isnot [bool]) { throw 'require_access_key must be true or false.' }
        $requireAccessKey = [bool]$config.require_access_key
    }
    # Absent means show: existing configs keep opening the credential window.
    $showAccessWindow = $true
    if ($config.ContainsKey('show_access_window')) {
        if ($config.show_access_window -isnot [bool]) { throw 'show_access_window must be true or false.' }
        $showAccessWindow = [bool]$config.show_access_window
    }
    return @{
        PublicUrl = $publicUrl; TlsMode = $tlsMode; Ports = $ports; Paths = $paths
        CaddyExecutable = [string]$config.caddy_executable; AdminUsername = [string]$config.admin_username
        RequireAccessKey = $requireAccessKey; ShowAccessWindow = $showAccessWindow
        IceServersJson = ConvertTo-Json -InputObject $config.ice_servers -Depth 8 -Compress
        TurnUrlsJson = ConvertTo-Json -InputObject $config.turn_urls -Compress
        HasTurn = $config.turn_urls.Count -gt 0
    }
}

function Assert-DeploymentSecrets($Settings) {
    if (!$env:SPACE_SIM_ADMIN_PASSWORD -or $env:SPACE_SIM_ADMIN_PASSWORD.Length -lt 12 -or
        $env:SPACE_SIM_ADMIN_PASSWORD -eq 'ChangeMe123!') { throw 'Set SPACE_SIM_ADMIN_PASSWORD to a non-default password of at least 12 characters.' }
    if (!$env:SPACE_SIM_STREAM_JWT_SECRET -or $env:SPACE_SIM_STREAM_JWT_SECRET.Length -lt 32) {
        throw 'Set SPACE_SIM_STREAM_JWT_SECRET to a random secret of at least 32 characters.'
    }
    # Only demanded when the deployment still gates the console behind the key.
    if ($Settings.RequireAccessKey -and
        (!$env:SPACE_SIM_STREAM_ACCESS_KEY -or $env:SPACE_SIM_STREAM_ACCESS_KEY -notmatch '^[A-Za-z0-9_-]{24,}$')) {
        throw 'Set SPACE_SIM_STREAM_ACCESS_KEY to a random URL-safe secret of at least 24 characters.'
    }
    if ($Settings.HasTurn -and (!$env:SPACE_SIM_TURN_AUTH_SECRET -or $env:SPACE_SIM_TURN_AUTH_SECRET.Length -lt 16)) {
        throw 'TURN is configured: set SPACE_SIM_TURN_AUTH_SECRET to the matching TURN REST shared secret.'
    }
}

function Get-DeploymentCaddyfile($Settings, [string]$ProjectRoot) {
    if ($Settings.TlsMode -eq 'tunnel') { throw 'Tunnel TLS is managed by the public launcher, not the direct Caddy deployment.' }
    $tls = ''
    $globalExtra = ''
    if ($Settings.TlsMode -eq 'internal') { $tls = 'tls internal' }
    elseif ($Settings.TlsMode -eq 'ip-acme') {
        # Let's Encrypt only issues IP certificates from the short-lived profile.
        # Clients never send SNI for an IP literal, so without default_sni Caddy
        # cannot pick a certificate and every handshake fails.
        $storage = ConvertTo-Json -InputObject (Join-Path $ProjectRoot 'deploy/certs/acme') -Compress
        $globalExtra = "`n    storage file_system $storage`n    default_sni $(([uri]$Settings.PublicUrl).Host)"
        $tls = "tls {`n        issuer acme {`n            profile shortlived`n        }`n    }"
    }
    elseif ($Settings.TlsMode -eq 'manual') {
        # Caddy quoted strings accept JSON escaping; never interpolate raw paths.
        $cert = ConvertTo-Json -InputObject $Settings.Paths.certificate_file -Compress
        $key = ConvertTo-Json -InputObject $Settings.Paths.certificate_key_file -Compress
        $tls = "tls $cert $key"
    }
    return (Get-Content -Raw -LiteralPath (Join-Path $ProjectRoot 'deploy/Caddyfile.template')).
        Replace('__GLOBAL_EXTRA__', $globalExtra).
        Replace('__PUBLIC_URL__', $Settings.PublicUrl).
        Replace('__TLS__', $tls).
        Replace('__API_PORT__', [string]$Settings.Ports.api_port).
        Replace('__PLAYER_PORT__', [string]$Settings.Ports.player_port)
}
