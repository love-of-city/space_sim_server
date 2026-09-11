"""Public orchestration tests use copied helpers and fake external effects only.

No actual public tunnel, live backend, or UE connection is created.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or r"C:\Program Files\PowerShell\7\pwsh.exe"
pytestmark = pytest.mark.skipif(os.name != "nt" or not Path(PWSH).is_file(), reason="Windows PowerShell 7 required")


def invoke(script, *args, **env):
    environment = {key: value for key, value in os.environ.items() if not key.startswith("SPACE_SIM_")}
    environment.update(env)
    return subprocess.run([PWSH, "-NoProfile", "-File", str(script), *map(str, args)],
        env=environment, cwd=script.parent, text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=30)


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / "public launcher repo"
    for directory in ("scripts", "deploy", "run", "tools"):
        (root / directory).mkdir(parents=True)
    for name in ("deployment_launcher.ps1", "deployment_bootstrap.ps1", "deployment_config.ps1",
                 "public_deployment.ps1", "public_deployment_helpers.ps1"):
        shutil.copyfile(ROOT / "scripts" / name, root / "scripts" / name)
    for name in ("deployment.example.json", "Caddyfile.template", "cloudflared-release.json"):
        shutil.copyfile(ROOT / "deploy" / name, root / "deploy" / name)
    (root / "scripts/run_platform.ps1").write_text("""
param([switch]$KeepPendingPublic)
$root=Split-Path -Parent $PSScriptRoot
if (!$KeepPendingPublic) { throw 'Pending gateway must survive platform replacement.' }
'platform' | Add-Content (Join-Path $root 'run/events.txt')
if ($env:TEST_PLATFORM_FAIL) { throw 'Fake platform failed.' }
""", encoding="utf-8")
    (root / "scripts/stop_platform.ps1").write_text("""
param([switch]$Quiet,[switch]$KeepPendingPublic)
$root=Split-Path -Parent $PSScriptRoot
if ($KeepPendingPublic) { 'stop-old' | Add-Content (Join-Path $root 'run/events.txt') }
else { 'stop-all' | Add-Content (Join-Path $root 'run/events.txt') }
""", encoding="utf-8")
    # A script command substitutes Caddy; no binary is launched.
    (root / "fake-caddy.ps1").write_text("""
'caddy' | Add-Content (Join-Path $PSScriptRoot 'run/events.txt')
$global:LASTEXITCODE=0
""", encoding="utf-8")
    with (root / "scripts/public_deployment_helpers.ps1").open("a", encoding="utf-8") as file:
        file.write("""
$script:testRoot = Split-Path -Parent $PSScriptRoot
function Event($Name) { $Name | Add-Content (Join-Path $script:testRoot 'run/events.txt') }
function Read-Host { throw 'PUBLIC MODE MUST NEVER PROMPT' }
function Start-Sleep { param($Seconds,$Milliseconds) Event 'wait' }
function Enable-LauncherRuntime { param($CondaRoot) Event 'runtime' }
function Resolve-LauncherCaddy { param($RequestedExecutable,$ProjectRoot) return (Join-Path $ProjectRoot 'fake-caddy.ps1') }
function Resolve-PublicTunnel { param($ProjectRoot) return 'fake-cloudflared.exe' }
function Invoke-PublicAuth { param($ProjectRoot,$Username,[switch]$Apply)
    if ($env:TEST_AUTH_FAIL) { throw 'Fake auth failure.' }
    Event $(if ($Apply) {'auth-apply'} else {'auth-check'})
    return @{generated_password_users=@('admin');existing_admin_users=@();rotated_default_users=@()}
}
function Stop-PublicPending { param($ProjectRoot) Event 'pending-cleanup' }
function Start-PublicChild { param($Executable,$Arguments,$LogBase,$ProjectRoot)
    Event $(if ($Executable -like '*cloudflared*') {'tunnel'} else {'closed-proxy'})
    $process=[pscustomobject]@{Id=123456;StartTime=[DateTime]::UtcNow;HasExited=$false}
    $process | Add-Member ScriptMethod Kill { $this.HasExited=$true; Event 'kill' }
    $process | Add-Member ScriptMethod WaitForExit { param($Timeout) return $true }
    return $process
}
function Invoke-WebRequest { param($Uri,[switch]$SkipHttpErrorCheck,$TimeoutSec)
    $config=Get-Content -Raw (Join-Path $script:testRoot 'run/public-deployment/Caddyfile')
    $nonce=[regex]::Match($config,'X-Space-Sim-Preparing ([A-F0-9]+)').Groups[1].Value
    return @{StatusCode=503;Headers=@{'X-Space-Sim-Preparing'=$nonce};Content='Public deployment is preparing; no application is exposed yet.'}
}
function Wait-PublicTunnelUrl { param($Tunnel,$LogBase)
    if ($env:TEST_TUNNEL_FAIL) { throw 'Fake tunnel allocation failure.' }
    Event 'address'
    return 'https://automatic-public-example.trycloudflare.com'
}
function Wait-PublicGateway { param($Url,$Nonce,$Proxy,$Tunnel)
    Event 'public-check'
    if ($env:TEST_PUBLIC_FAIL) { throw 'Fake public check failed.' }
}
function Save-LauncherRunMarker { param($ProjectRoot,$ConfigPath,$SecretPath) Event 'marker' }
function Test-LauncherRunning { return [bool]$env:TEST_ALREADY_RUNNING }
function Show-PublicAccess { param($ProjectRoot,$ConfigPath,[switch]$NonInteractive) Event 'show-access' }
""")
    return root


def events(repo):
    path = repo / "run/events.txt"
    return path.read_text(encoding="utf-8-sig").splitlines() if path.exists() else []


def launch(repo, *args, **env):
    return invoke(repo / "scripts/deployment_launcher.ps1", "-NonInteractive", *args, **env)


def test_default_launch_is_public_without_any_configuration_or_input(repo):
    result = launch(repo)
    assert result.returncode == 0, result.stdout + result.stderr
    config = json.loads((repo / "deploy/public.local.json").read_text(encoding="utf-8"))
    assert config["tls_mode"] == "tunnel"
    assert config["public_url"] == "https://automatic-public-example.trycloudflare.com"
    assert config["ice_servers"] == [{"urls": "stun:stun.cloudflare.com:3478"}]
    assert not (repo / "deploy/deployment.local.json").exists()
    assert list((repo / "deploy/secrets").glob("*.clixml"))
    order = events(repo)
    assert order.index("auth-check") < order.index("tunnel") < order.index("address") < order.index("stop-old")
    assert order.index("stop-old") < order.index("auth-apply") < order.index("platform") < order.index("public-check") < order.index("show-access")
    assert "stop-all" not in order
    assert not (repo / "run/public-pending.json").exists()
    state = json.loads((repo / "run/deployment.json").read_text(encoding="utf-8"))
    assert state["public_verified"] and state["tunnel_pid"] == 123456
    report = json.loads((repo / "run/public-access.json").read_text(encoding="utf-8"))
    assert "password" not in report and "access_key" not in report
    assert "PUBLIC MODE MUST NEVER PROMPT" not in result.stdout + result.stderr


@pytest.mark.parametrize("failure,old_stopped", [
    ("TEST_AUTH_FAIL", False), ("TEST_TUNNEL_FAIL", False),
    ("TEST_PLATFORM_FAIL", True), ("TEST_PUBLIC_FAIL", True),
])
def test_failed_public_bootstrap_cleans_its_children_without_false_success(repo, failure, old_stopped):
    result = launch(repo, **{failure:"1"})
    assert result.returncode != 0
    order = events(repo)
    assert ("stop-old" in order) is old_stopped
    assert ("stop-all" in order) is old_stopped
    if failure != "TEST_AUTH_FAIL":
        assert order.count("kill") == (4 if failure == "TEST_TUNNEL_FAIL" else 2)
    assert "show-access" not in order
    assert not (repo / "run/public-pending.json").exists()
    assert not (repo / "run/public-access.json").exists()


def test_tunnel_allocation_retries_without_opening_gateway_or_stopping_old_platform(repo):
    with (repo / "scripts/public_deployment_helpers.ps1").open("a", encoding="utf-8") as file:
        file.write("""
$script:allocatedLogs=@()
function Wait-PublicTunnelUrl { param($Tunnel,$LogBase)
    if ($LogBase -in $script:allocatedLogs) { throw 'A retry must not parse a stale log.' }
    $script:allocatedLogs += $LogBase
    Event 'allocate-attempt'
    if ($script:allocatedLogs.Count -lt 3) { throw 'Transient allocation timeout.' }
    Event 'address'
    return 'https://automatic-public-example.trycloudflare.com'
}
""")
    result = launch(repo)
    assert result.returncode == 0, result.stdout + result.stderr
    order = events(repo)
    assert order.count("tunnel") == 3
    assert order.count("allocate-attempt") == 3
    assert order.count("kill") == 2
    assert order.count("wait") == 2
    assert order.count("closed-proxy") == 1
    assert order.index("address") < order.index("stop-old") < order.index("platform")
    assert order[-1] == "show-access"
    assert "stop-all" not in order
    assert not (repo / "run/public-pending.json").exists()


def test_tunnel_retry_is_bounded_and_does_not_report_success(repo):
    result = launch(repo, TEST_TUNNEL_FAIL="1")
    assert result.returncode != 0
    assert "failed after 3 attempts" in result.stderr
    order = events(repo)
    assert order.count("tunnel") == 3
    assert order.count("wait") == 2
    assert order.count("kill") == 4  # Three failed tunnels plus the closed gateway.
    assert "stop-old" not in order and "platform" not in order
    assert "show-access" not in order
    assert not (repo / "run/public-pending.json").exists()


def test_public_validate_only_is_readonly_and_reuses_encrypted_store(repo):
    assert launch(repo).returncode == 0
    before = {p: p.read_bytes() for p in repo.rglob("*") if p.is_file()}
    result = launch(repo, "-ValidateOnly")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "No downloads" in result.stdout
    assert before == {p: p.read_bytes() for p in repo.rglob("*") if p.is_file()}


def test_repeat_click_never_restarts_a_running_public_deployment(repo):
    assert launch(repo).returncode == 0
    before = len(events(repo))
    result = launch(repo, TEST_ALREADY_RUNNING="1")
    assert result.returncode == 0, result.stdout + result.stderr
    assert events(repo)[before:] == ["show-access"]


def test_gateway_render_keeps_network_bind_loopback_and_origin_checks(tmp_path):
    harness = tmp_path / "render.ps1"
    harness.write_text("""
param($Helpers)
. $Helpers
$settings=@{PublicUrl='https://automatic-public-example.trycloudflare.com';Ports=@{api_port=8000;player_port=8080}}
Get-PublicProxyConfig $settings 19090 19091 'TESTNONCE' -Holding
Get-PublicProxyConfig $settings 19090 19091 'TESTNONCE'
""", encoding="utf-8")
    result = invoke(harness, ROOT / "scripts/public_deployment_helpers.ps1")
    assert result.returncode == 0, result.stderr
    closed, active = result.stdout.split('admin 127.0.0.1:19091', 2)[1:]
    assert "503" in closed and "reverse_proxy" not in closed
    assert "bind 127.0.0.1" in active
    assert "@public host automatic-public-example.trycloudflare.com" in active
    assert "reverse_proxy 127.0.0.1:8080" in active
    assert "reverse_proxy 127.0.0.1:8000" in active
    assert "CF-Connecting-IP" in active
    assert "X-Forwarded-Proto https" in active


def test_tunnel_url_parser_rejects_suffix_confusion(tmp_path):
    harness=tmp_path / 'parse.ps1'
    harness.write_text("""
param($Helpers,$Logs)
. $Helpers
$process=[pscustomobject]@{HasExited=$false}
'https://spoof.trycloudflare.com.evil.test https://valid-sample.trycloudflare.com |' | Set-Content ($Logs+'.err.log')
Wait-PublicTunnelUrl $process $Logs 2
""", encoding='utf-8')
    result=invoke(harness, ROOT / 'scripts/public_deployment_helpers.ps1', tmp_path / 'logs')
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == 'https://valid-sample.trycloudflare.com'

@pytest.mark.parametrize("initial_content", ["", " \r\n\t"])
@pytest.mark.parametrize("suffix", [".out.log", ".err.log"])
def test_tunnel_url_waits_for_initially_empty_redirected_logs(tmp_path, initial_content, suffix):
    logs = tmp_path / "tunnel logs"
    for extension in (".out.log", ".err.log"):
        Path(str(logs) + extension).write_text(initial_content, encoding="utf-8")
    harness = tmp_path / "wait-for-log.ps1"
    harness.write_text("""
param($Helpers,$Logs,$Suffix)
$ErrorActionPreference='Stop'
. $Helpers
$script:waits=0
# Deterministically emit the first tunnel line only after a polling iteration.
function Start-Sleep { param($Milliseconds)
    $script:waits++
    'https://delayed-log.trycloudflare.com |' | Set-Content -LiteralPath ($Logs+$Suffix)
}
$process=[pscustomobject]@{HasExited=$false}
$url=Wait-PublicTunnelUrl $process $Logs 5
if ($script:waits -ne 1) { throw 'Must wait once for initially empty logs.' }
$url
""", encoding="utf-8")
    result = invoke(harness, ROOT / "scripts/public_deployment_helpers.ps1", logs, suffix)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.strip() == "https://delayed-log.trycloudflare.com"
    assert not result.stderr


def test_tunnel_empty_logs_fail_with_timeout_not_null_regex_error(tmp_path):
    logs = tmp_path / "empty logs"
    for suffix in (".out.log", ".err.log"):
        Path(str(logs) + suffix).touch()
    harness = tmp_path / "empty-log-timeout.ps1"
    harness.write_text("""
param($Helpers,$Logs)
$ErrorActionPreference='Stop'
. $Helpers
$process=[pscustomobject]@{HasExited=$false}
Wait-PublicTunnelUrl $process $Logs 1
""", encoding="utf-8")
    result = invoke(harness, ROOT / "scripts/public_deployment_helpers.ps1", logs)
    assert result.returncode != 0
    assert "did not allocate an address in time" in result.stderr
    assert "Value cannot be null" not in result.stderr


@pytest.mark.parametrize("keep_pending", [False, True])
def test_stop_cleans_only_matching_proxy_and_tunnel_identities(tmp_path, keep_pending):
    root = tmp_path / 'stopping repo'
    (root / 'scripts').mkdir(parents=True)
    (root / 'run').mkdir()
    shutil.copyfile(ROOT / 'scripts/stop_platform.ps1', root / 'scripts/stop_platform.ps1')
    (root / 'scripts/stop_pixel_streaming.ps1').write_text('param([switch]$Quiet)\n', encoding='utf-8')
    (root / 'run/deployment.json').write_text(json.dumps({'proxy_pid':101,'proxy_start':1,'tunnel_pid':102,'tunnel_start':2}),encoding='utf-8')
    (root / 'run/public-pending.json').write_text(json.dumps({'proxy_pid':103,'proxy_start':3,'tunnel_pid':104,'tunnel_start':999}),encoding='utf-8')
    harness=tmp_path / 'stop.ps1'
    harness.write_text("""
param($Root,[string]$Keep)
$ErrorActionPreference='Stop'
function Get-Process { param($Id,$ErrorAction)
    $ticks=@{101=1L;102=2L;103=3L;104=4L}[[int]$Id]
    return [pscustomobject]@{StartTime=[DateTime]::new($ticks,[DateTimeKind]::Utc)}
}
function Get-CimInstance { param($ClassName,$Filter,$ErrorAction) return @() }
function Stop-Process { param($Id,[switch]$Force,$ErrorAction) Write-Output ('KILLED-'+$Id) }
& (Join-Path $Root 'scripts/stop_platform.ps1') -Quiet -KeepPendingPublic:($Keep -eq 'yes')
""", encoding='utf-8')
    result=invoke(harness, root, 'yes' if keep_pending else 'no')
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'KILLED-101' in result.stdout and 'KILLED-102' in result.stdout
    assert ('KILLED-103' in result.stdout) is (not keep_pending)
    assert 'KILLED-104' not in result.stdout
    assert not (root / 'run/deployment.json').exists()
    assert (root / 'run/public-pending.json').exists() is keep_pending


def test_actual_platform_start_preserves_closed_gateway_but_not_failure_cleanup():
    text=(ROOT / 'scripts/run_platform.ps1').read_text(encoding='utf-8')
    assert "& (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet -KeepPendingPublic:$KeepPendingPublic" in text
    assert "try { & (Join-Path $PSScriptRoot 'stop_platform.ps1') -Quiet }" in text


@pytest.mark.parametrize('matching', [True, False])
def test_public_health_requires_per_launch_marker_and_json_ok(tmp_path, matching):
    harness=tmp_path / 'health.ps1'
    harness.write_text("""
param($Helpers,$Matching)
$ErrorActionPreference='Stop'
. $Helpers
function Invoke-WebRequest { param($Uri,$TimeoutSec,$MaximumRedirection,$ErrorAction)
    $marker=if($Matching -eq 'yes'){'EXPECTED'}else{'WRONG'}
    return @{StatusCode=200;Headers=@{'X-Space-Sim-Deployment'=$marker};Content='{"ok":true}'}
}
$child=[pscustomobject]@{HasExited=$false}
Wait-PublicGateway 'https://test.trycloudflare.com' 'EXPECTED' $child $child 0
""",encoding='utf-8')
    result=invoke(harness, ROOT / 'scripts/public_deployment_helpers.ps1', 'yes' if matching else 'no')
    assert (result.returncode == 0) is matching, result.stdout + result.stderr


@pytest.mark.parametrize('valid', [True, False])
def test_cloudflared_download_checksum_is_enforced_without_executing_file(tmp_path, valid):
    import hashlib
    root=tmp_path / 'download repo'
    (root / 'deploy').mkdir(parents=True)
    payload=b'FAKE cloudflared payload - NOT executed'
    digest=hashlib.sha256(payload).hexdigest() if valid else '0'*64
    (root / 'deploy/cloudflared-release.json').write_text(json.dumps({'version':'2026.9.0','sha256':{'amd64':digest}}),encoding='utf-8')
    harness=tmp_path / 'download.ps1'
    harness.write_text("""
param($Helpers,$Root)
$ErrorActionPreference='Stop'
. $Helpers
function Receive-DeploymentFile { param($Uri,$Destination)
    if ($Uri -ne 'https://github.com/cloudflare/cloudflared/releases/download/2026.9.0/cloudflared-windows-amd64.exe') { throw 'Unexpected source.' }
    [IO.File]::WriteAllBytes($Destination,[Text.Encoding]::UTF8.GetBytes('FAKE cloudflared payload - NOT executed'))
}
$file=Resolve-PublicTunnel $Root
function Receive-DeploymentFile { throw 'Must reuse verified download.' }
$again=Resolve-PublicTunnel $Root
if($file -ne $again){throw 'Wrong cache path.'}
""",encoding='utf-8')
    result=invoke(harness, ROOT / 'scripts/public_deployment_helpers.ps1', root)
    assert (result.returncode == 0) is valid, result.stdout + result.stderr
    files=list(root.rglob('cloudflared.exe'))
    assert bool(files) is valid
    assert not list(root.rglob('*.download'))

@pytest.mark.parametrize("generated", [True, False])
def test_real_access_window_builds_offscreen_without_disclosing_credentials(tmp_path, generated):
    root = tmp_path / "credential-window repo"
    (root / "scripts").mkdir(parents=True)
    (root / "run").mkdir()
    (root / "deploy/secrets").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/show_public_access.ps1", root / "scripts/show_public_access.ps1")
    config = root / "deploy/public.local.json"
    config.write_text('{}', encoding='utf-8')
    (root / "run/public-access.json").write_text(json.dumps({
        "public_url":"https://offscreen-test.trycloudflare.com", "config_path":str(config),
        "generated_password_users":["admin"] if generated else [],
        "existing_admin_users":[] if generated else ["custom-admin"],
    }), encoding='utf-8')
    harness = tmp_path / "offscreen.ps1"
    harness.write_text("""
param($Root,$Helpers)
$ErrorActionPreference='Stop'
. $Helpers
$config=Join-Path $Root 'deploy/public.local.json'
$hash=[Security.Cryptography.SHA256]::Create()
try{$id=[BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($config.ToLowerInvariant()))).Replace('-','')}
finally{$hash.Dispose()}
Save-LauncherSecrets (Join-Path $Root ('deploy/secrets/'+$id+'.clixml')) @{
    SPACE_SIM_ADMIN_PASSWORD='Offscreen-Test-Password-Do-Not-Print';
    SPACE_SIM_STREAM_ACCESS_KEY='Offscreen-Access-Do-Not-Print-12345'
}
& (Join-Path $Root 'scripts/show_public_access.ps1') -ConfigPath $config -ValidateOnly
""", encoding='utf-8')
    result = invoke(harness, root, ROOT / "scripts/deployment_bootstrap.ps1")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "validated; no window" in result.stdout
    assert "Offscreen-Test-Password-Do-Not-Print" not in result.stdout + result.stderr
    assert "Offscreen-Access-Do-Not-Print-12345" not in result.stdout + result.stderr

def test_access_dialog_overrides_hidden_console_first_show():
    """Offscreen controls alone cannot catch STARTUPINFO hiding the first dialog.

    The visible-window path is also exercised manually by the no-argument launcher.
    Keep the explicit first-show override separate from read-only validation.
    """
    source = (ROOT / "scripts/show_public_access.ps1").read_text(encoding="utf-8")
    read_only = source.index("if ($ValidateOnly)")
    override = source.index("[SpaceSimAccessWindow]::ShowWindow($form.Handle, 0)")
    display = source.index("$form.ShowDialog()")
    assert read_only < override < display
    assert '[DllImport("user32.dll")]' in source
    helper = (ROOT / "scripts/public_deployment_helpers.ps1").read_text(encoding="utf-8")
    window_launch = helper[helper.index("function Show-PublicAccess"):]
    assert "-WindowStyle Hidden" in window_launch
