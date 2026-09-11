"""Desktop launcher regression tests in throwaway repositories.

The real platform start/stop scripts and the real Caddy download are NEVER called.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or r"C:\Program Files\PowerShell\7\pwsh.exe"
pytestmark = pytest.mark.skipif(
    os.name != "nt" or not Path(PWSH).is_file(), reason="Windows and PowerShell 7 required"
)
SECRET_NAMES = (
    "SPACE_SIM_ADMIN_PASSWORD", "SPACE_SIM_STREAM_JWT_SECRET",
    "SPACE_SIM_STREAM_ACCESS_KEY", "SPACE_SIM_TURN_AUTH_SECRET",
)
SECRETS = {
    "SPACE_SIM_ADMIN_PASSWORD": "Launcher-Test-Password!",
    "SPACE_SIM_STREAM_JWT_SECRET": "test-jwt-" + "j" * 48,
    "SPACE_SIM_STREAM_ACCESS_KEY": "test-access-" + "k" * 32,
}


def environment(*, secrets=True, **extra):
    result = {key: value for key, value in os.environ.items() if key not in SECRET_NAMES}
    if secrets:
        result.update(SECRETS)
    result["PATH"] = str(Path(sys.executable).parent) + os.pathsep + result.get("PATH", "")
    result.update(extra)
    return result


def run_ps(script, *args, env=None):
    return subprocess.run(
        [PWSH, "-NoProfile", "-File", str(script), *map(str, args)],
        cwd=script.parent, env=env or environment(),
        text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=30,
    )


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / "launcher repo with spaces"
    for name in ("scripts", "deploy", "tools"):
        (root / name).mkdir(parents=True)
    for name in ("deployment_launcher.ps1", "deployment_bootstrap.ps1", "deployment_config.ps1"):
        shutil.copyfile(ROOT / "scripts" / name, root / "scripts" / name)
    for name in ("deployment.example.json", "Caddyfile.template", "caddy-release.json"):
        shutil.copyfile(ROOT / "deploy" / name, root / "deploy" / name)
    settings = json.loads((root / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    settings.update(public_url="https://sim.test:8443", tls_mode="internal")
    (root / "deploy/deployment.local.json").write_text(json.dumps(settings), encoding="utf-8")
    # Override only external effects, within this throwaway copy. No production test hooks.
    with (root / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("""
function Enable-LauncherRuntime { param($CondaRoot)
    if ($env:TEST_RUNTIME_FAIL) { throw 'Fake missing runtime.' }
}
function Resolve-LauncherCaddy { param($RequestedExecutable, $ProjectRoot)
    if ($env:TEST_CADDY_FAIL) { throw 'Fake download failure.' }
    return (Join-Path $ProjectRoot 'fake tools/caddy.exe')
}
function Show-LauncherAccess { param($Settings, [switch]$NonInteractive)
    Write-Host ('ACCESS-ORIGIN: ' + $Settings.PublicUrl)
}
""")
    (root / "tools/check_deployment_auth.py").write_text(
        "import os, sys\nsys.exit(1 if os.getenv('TEST_AUTH_FAIL') else 0)\n", encoding="utf-8"
    )
    (root / "scripts/deploy_platform.ps1").write_text("""
param([string]$ConfigPath, [switch]$ValidateOnly, [switch]$Start)
if ($ValidateOnly) { Write-Host 'VALIDATION-ONLY'; return }
if (!$Start) { throw 'Expected explicit -Start.' }
$root = Split-Path -Parent $PSScriptRoot
$null = New-Item -ItemType Directory -Path (Join-Path $root 'run') -Force
$hash = [Security.Cryptography.SHA256]::Create()
try {
    $record = @{}
    foreach ($name in @('SPACE_SIM_ADMIN_PASSWORD','SPACE_SIM_STREAM_JWT_SECRET','SPACE_SIM_STREAM_ACCESS_KEY','SPACE_SIM_TURN_AUTH_SECRET')) {
        $record[$name] = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes([string][Environment]::GetEnvironmentVariable($name)))).Replace('-','')
    }
    $record | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'run/fake-start.json') -Encoding utf8NoBOM
    @{proxy_pid=123; proxy_start=456; public_url='https://sim.test:8443'} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $root 'run/deployment.json') -Encoding utf8NoBOM
} finally { $hash.Dispose() }
""", encoding="utf-8")
    (root / "scripts/stop_platform.ps1").write_text(
        "'stopped' | Set-Content -LiteralPath (Join-Path (Split-Path -Parent $PSScriptRoot) 'fake-stop.txt')\n",
        encoding="utf-8",
    )
    return root


def launch(repo, *args, env=None, interactive=False):
    return run_ps(repo / "scripts/deployment_launcher.ps1", *([] if interactive else ["-NonInteractive"]), *args, env=env)


def snapshot(repo):
    return {path.relative_to(repo): path.read_bytes() for path in repo.rglob("*") if path.is_file()}


def saved_secret(repo):
    return next((repo / "deploy/secrets").glob("*.clixml"))


def assert_no_secret_output(result):
    for secret in SECRETS.values():
        assert secret not in result.stdout + result.stderr


def test_first_start_saves_encrypted_secrets_and_next_start_reuses_them(repo):
    original_config = (repo / "deploy/deployment.local.json").read_bytes()
    first = launch(repo)
    assert first.returncode == 0, first.stdout + first.stderr
    assert_no_secret_output(first)
    stored = saved_secret(repo).read_bytes()
    for value in SECRETS.values():
        assert value.encode() not in stored
    expected = json.loads((repo / "run/fake-start.json").read_text(encoding="utf-8"))
    for name, value in SECRETS.items():
        assert expected[name] == hashlib.sha256(value.encode()).hexdigest().upper()
    second = launch(repo, env=environment(secrets=False))
    assert second.returncode == 0, second.stdout + second.stderr
    assert_no_secret_output(second)
    assert json.loads((repo / "run/fake-start.json").read_text(encoding="utf-8")) == expected
    assert (repo / "deploy/deployment.local.json").read_bytes() == original_config


def test_saved_keys_take_precedence_over_incidental_shell_variables(repo):
    assert launch(repo).returncode == 0
    expected = json.loads((repo / "run/fake-start.json").read_text(encoding="utf-8"))
    result = launch(repo, env=environment(SPACE_SIM_STREAM_ACCESS_KEY="different-" + "z" * 32))
    assert result.returncode == 0, result.stderr
    assert json.loads((repo / "run/fake-start.json").read_text(encoding="utf-8")) == expected


@pytest.mark.parametrize("saved", [False, True])
def test_validate_only_never_bootstraps_writes_or_starts(repo, saved):
    if saved:
        assert launch(repo).returncode == 0
    before = snapshot(repo)
    result = launch(repo, "-ValidateOnly", env=environment(
        secrets=not saved, TEST_RUNTIME_FAIL="1", TEST_CADDY_FAIL="1", TEST_AUTH_FAIL="1"
    ))
    assert result.returncode == 0, result.stdout + result.stderr
    assert "VALIDATION-ONLY" in result.stdout
    assert snapshot(repo) == before
    assert_no_secret_output(result)


def test_validate_only_does_not_generate_missing_secrets(repo):
    before = snapshot(repo)
    result = launch(repo, "-ValidateOnly", env=environment(secrets=False))
    assert result.returncode != 0
    assert snapshot(repo) == before


@pytest.mark.parametrize("failure", ["TEST_RUNTIME_FAIL", "TEST_AUTH_FAIL", "TEST_CADDY_FAIL"])
def test_preflight_failure_never_calls_start_or_stop(repo, failure):
    result = launch(repo, env=environment(**{failure: "1"}))
    assert result.returncode != 0
    assert not (repo / "run/fake-start.json").exists()
    assert not (repo / "fake-stop.txt").exists()
    assert_no_secret_output(result)


def test_noninteractive_missing_config_does_not_create_it(repo):
    (repo / "deploy/deployment.local.json").unlink()
    before = snapshot(repo)
    result = launch(repo, "-Mode", "Direct")
    assert result.returncode != 0
    assert "First-run input required" in result.stderr
    assert snapshot(repo) == before


def test_missing_admin_secret_fails_without_persisting_partial_keys(repo):
    result = launch(repo, env=environment(secrets=False))
    assert result.returncode != 0
    assert "SPACE_SIM_ADMIN_PASSWORD" in result.stderr
    assert not (repo / "deploy/secrets").exists()
    assert not (repo / "run").exists()


@pytest.mark.parametrize("address,tls_mode", [
    ("192.0.2.10:8443", "internal"),
    ("sim.public.test", "acme"),
])
def test_interactive_first_config_is_saved_once(repo, address, tls_mode):
    (repo / "deploy/deployment.local.json").unlink()
    with (repo / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("""
$script:answers = [Collections.Generic.Queue[string]]::new()
$script:answers.Enqueue('__ADDRESS__')
$script:answers.Enqueue('__TLS_MODE__')
function Read-Host { param($Prompt) Write-Host $Prompt; return $script:answers.Dequeue() }
""".replace('__ADDRESS__', address).replace('__TLS_MODE__', tls_mode))
    result = launch(repo, "-Mode", "Direct", interactive=True)
    assert result.returncode == 0, result.stdout + result.stderr
    config = json.loads((repo / "deploy/deployment.local.json").read_text(encoding="utf-8"))
    assert config["public_url"] == f"https://{address}"
    assert config["tls_mode"] == tls_mode
    assert "不是访问者 IP 白名单" in result.stdout
    assert "平台统一 HTTPS 网址" in result.stdout
    assert_no_secret_output(result)


@pytest.mark.parametrize("corruption", ["not xml", '<Objs Version="1.1.0.1" xmlns="http://schemas.microsoft.com/powershell/2004/04"><S>plaintext</S></Objs>'])
def test_corrupt_secret_store_is_not_silently_replaced(repo, corruption):
    assert launch(repo).returncode == 0
    secret_path = saved_secret(repo)
    secret_path.write_text(corruption, encoding="utf-8")
    before = snapshot(repo)
    result = launch(repo)
    assert result.returncode != 0
    assert "Cannot decrypt saved deployment secrets" in result.stderr
    assert snapshot(repo) == before


def test_stop_needs_no_config_runtime_caddy_or_credentials(repo):
    (repo / "deploy/deployment.local.json").unlink()
    result = launch(repo, "-Action", "Stop", env=environment(
        secrets=False, TEST_RUNTIME_FAIL="1", TEST_CADDY_FAIL="1", TEST_AUTH_FAIL="1"
    ))
    assert result.returncode == 0, result.stdout + result.stderr
    assert (repo / "fake-stop.txt").exists()
    assert not (repo / "deploy/secrets").exists()
    assert not (repo / "run").exists()


def test_already_running_is_noop_unless_restart_requested(repo):
    assert launch(repo).returncode == 0
    with (repo / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("\nfunction Test-LauncherRunning { return $true }\n")
    before = snapshot(repo)
    result = launch(repo, env=environment(secrets=False, TEST_RUNTIME_FAIL="1"))
    assert result.returncode == 0, result.stdout + result.stderr
    assert snapshot(repo) == before
    restart = launch(repo, "-Restart", env=environment(secrets=False, TEST_RUNTIME_FAIL="1"))
    assert restart.returncode != 0
    assert "Fake missing runtime" in restart.stderr


def test_turn_secret_is_saved_and_required_when_configured(repo):
    path = repo / "deploy/deployment.local.json"
    config = json.loads(path.read_text(encoding="utf-8"))
    config["turn_urls"] = ["turn:relay.test:3478?transport=udp"]
    path.write_text(json.dumps(config), encoding="utf-8")
    missing = launch(repo)
    assert missing.returncode != 0
    assert "SPACE_SIM_TURN_AUTH_SECRET" in missing.stderr
    value = "turn-shared-test-" + "t" * 32
    result = launch(repo, env=environment(SPACE_SIM_TURN_AUTH_SECRET=value))
    assert result.returncode == 0, result.stderr
    assert value.encode() not in saved_secret(repo).read_bytes()
    assert launch(repo, env=environment(secrets=False)).returncode == 0


def test_real_running_check_uses_pid_start_time_config_and_secret_fingerprints(repo, tmp_path):
    harness = tmp_path / "identity.ps1"
    harness.write_text("""
param($Root, $Helpers)
. $Helpers
$run = Join-Path $Root 'run'
$null = New-Item -ItemType Directory -Path $run -Force
$start = (Get-Process -Id $PID).StartTime.ToUniversalTime().Ticks
@{proxy_pid=$PID; proxy_start=$start; public_url='https://sim.test:8443'} | ConvertTo-Json | Set-Content (Join-Path $run 'deployment.json')
@{backend_service_pid=$PID; backend_service_start=$start} | ConvertTo-Json | Set-Content (Join-Path $run 'platform.json')
@{pid=$PID; start_ticks=$start} | ConvertTo-Json | Set-Content (Join-Path $run 'pixel_streaming.json')
$config = Join-Path $Root 'deploy/deployment.local.json'
$secret = Join-Path $Root 'test-secret-file'
'opaque fixture' | Set-Content $secret
Save-LauncherRunMarker $Root $config $secret
if (!(Test-LauncherRunning $Root 'https://sim.test:8443' $config $secret)) { throw 'Valid identity rejected.' }
if (Test-LauncherRunning $Root 'https://other.test:8443' $config $secret) { throw 'Wrong URL accepted.' }
if (Test-LauncherProcess $PID ($start - 1)) { throw 'Reused PID accepted.' }
'changed config' | Add-Content $config
if (Test-LauncherRunning $Root 'https://sim.test:8443' $config $secret) { throw 'Changed config accepted.' }
""", encoding="utf-8")
    result = run_ps(harness, repo, ROOT / "scripts/deployment_bootstrap.ps1")
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("valid", [True, False])
def test_downloaded_archive_requires_pinned_checksum_and_extracts_only_caddy(tmp_path, valid):
    archive = tmp_path / "caddy.zip"
    payload = b"FAKE executable; never run by this test"
    with zipfile.ZipFile(archive, "w") as file:
        file.writestr("caddy.exe", payload)
        file.writestr("../unexpected.txt", "must never extract")
    checksum = hashlib.sha512(archive.read_bytes()).hexdigest() if valid else "0" * 128
    destination = tmp_path / "caddy.exe"
    destination.write_bytes(b"old executable")
    harness = tmp_path / "archive.ps1"
    harness.write_text("param($Helpers, $Archive, $Hash, $Destination)\n$ErrorActionPreference='Stop'\n"
        ". $Helpers\nInstall-LauncherCaddyArchive $Archive $Hash $Destination\n", encoding="utf-8")
    result = run_ps(harness, ROOT / "scripts/deployment_bootstrap.ps1", archive, checksum, destination)
    assert (result.returncode == 0) is valid, result.stdout + result.stderr
    assert destination.read_bytes() == (payload if valid else b"old executable")
    assert not (tmp_path / "unexpected.txt").exists()
    assert not (tmp_path.parent / "unexpected.txt").exists()
    assert not list(tmp_path.glob("*.tmp"))


def test_missing_explicit_caddy_path_does_not_download(tmp_path):
    harness = tmp_path / "no-download.ps1"
    harness.write_text("""
param($Helpers, $Root)
$ErrorActionPreference='Stop'
. $Helpers
function Receive-DeploymentFile { throw 'NETWORK-MUST-NOT-BE-USED' }
Resolve-LauncherCaddy './nonexistent-custom/caddy.exe' $Root
""", encoding="utf-8")
    result = run_ps(harness, ROOT / "scripts/deployment_bootstrap.ps1", tmp_path)
    assert result.returncode != 0
    assert "explicitly configured" in result.stderr
    assert "NETWORK-MUST-NOT-BE-USED" not in result.stderr


def test_desktop_wrappers_are_independent_of_cwd_and_preserve_exit_status():
    for action in ("Start", "Stop"):
        text = (ROOT / f"{action.lower()}_deployment.cmd").read_text(encoding="ascii")
        assert '"%~dp0scripts\\deployment_launcher.ps1"' in text
        assert f"-Action {action} %*" in text
        assert '-NoProfile -ExecutionPolicy RemoteSigned' in text
        assert 'exit /b %RESULT%' in text
        assert "run_platform.ps1" not in text

@pytest.mark.parametrize("matching", [True, False])
def test_interactive_secret_confirmation_and_automatic_random_keys(repo, matching):
    with (repo / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("""
$script:passwordInputs = 0
function Read-Host { param($Prompt, [switch]$AsSecureString)
    $script:passwordInputs++
    $value = if ($env:TEST_CONFIRM_MISMATCH -and $script:passwordInputs -eq 2) { 'Different-Test-Password!' } else { 'Confirmed-Test-Password!' }
    if (!$AsSecureString) { throw 'Expected masked password input.' }
    return ConvertTo-SecureString $value -AsPlainText -Force
}
""")
    env = environment(secrets=False, **({} if matching else {"TEST_CONFIRM_MISMATCH": "1"}))
    result = launch(repo, env=env, interactive=True)
    assert (result.returncode == 0) is matching, result.stdout + result.stderr
    assert "Confirmed-Test-Password!" not in result.stdout + result.stderr
    if matching:
        assert b"Confirmed-Test-Password!" not in saved_secret(repo).read_bytes()
        record = json.loads((repo / "run/fake-start.json").read_text(encoding="utf-8"))
        assert record["SPACE_SIM_ADMIN_PASSWORD"] == hashlib.sha256(b"Confirmed-Test-Password!").hexdigest().upper()
        assert record["SPACE_SIM_STREAM_JWT_SECRET"] != record["SPACE_SIM_STREAM_ACCESS_KEY"]
        assert launch(repo, env=environment(secrets=False)).returncode == 0
    else:
        assert not (repo / "deploy/secrets").exists()
        assert not (repo / "run").exists()


def test_auto_caddy_download_flow_is_verified_cached_and_offline_tested(repo, tmp_path):
    archive = tmp_path / "fake release.zip"
    payload = b"FAKE Caddy; never executed"
    with zipfile.ZipFile(archive, "w") as file:
        file.writestr("caddy.exe", payload)
    checksum = hashlib.sha512(archive.read_bytes()).hexdigest()
    (repo / "deploy/caddy-release.json").write_text(json.dumps({
        "version": "1.2.3", "sha512": {"amd64": checksum, "arm64": checksum}
    }), encoding="utf-8")
    harness = tmp_path / "download.ps1"
    harness.write_text(r"""
param($Helpers, $Root, $Archive)
$ErrorActionPreference = 'Stop'
. $Helpers
function Get-Command { param($Name, $CommandType, $ErrorAction) return $null }
function Receive-DeploymentFile { param($Uri, $OutFile, $TimeoutSec)
    if ($Uri -notmatch '^https://github.com/caddyserver/caddy/releases/download/v1\.2\.3/caddy_1\.2\.3_windows_(amd64|arm64)\.zip$') { throw 'Unexpected download source.' }
    Copy-Item -LiteralPath $Archive -Destination $OutFile
}
$exe = Resolve-LauncherCaddy 'caddy.exe' $Root
if (!(Test-Path -LiteralPath $exe -PathType Leaf)) { throw 'Executable missing.' }
function Receive-DeploymentFile { throw 'Cache must not download again.' }
$cached = Resolve-LauncherCaddy 'caddy.exe' $Root
if ($exe -ne $cached) { throw 'Cache was not reused.' }
""", encoding="utf-8")
    result = run_ps(harness, ROOT / "scripts/deployment_bootstrap.ps1", repo, archive)
    assert result.returncode == 0, result.stdout + result.stderr
    executables = list((repo / "run/deployment-tools").rglob("caddy.exe"))
    assert len(executables) == 1 and executables[0].read_bytes() == payload
    assert not list((repo / "run").rglob("*.download.zip"))


def test_invalid_first_run_config_does_not_persist_a_broken_file(repo):
    (repo / "deploy/deployment.local.json").unlink()
    with (repo / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("""
$script:answers = [Collections.Generic.Queue[string]]::new()
$script:answers.Enqueue('http://sim.test')
$script:answers.Enqueue('internal')
function Read-Host { param($Prompt) return $script:answers.Dequeue() }
""")
    result = launch(repo, "-Mode", "Direct", interactive=True)
    assert result.returncode != 0
    assert not (repo / "deploy/deployment.local.json").exists()
    assert not list((repo / "deploy").glob(".setup-*"))
    assert not (repo / "run").exists()

@pytest.mark.parametrize("action,flags", [("start", "-ValidateOnly"), ("stop", "")])
def test_actual_cmd_entry_in_throwaway_repo_from_another_directory(repo, action, flags):
    batch = repo / f"{action}_deployment.cmd"
    shutil.copyfile(ROOT / batch.name, batch)
    command = f'"{os.environ["COMSPEC"]}" /d /s /c ""{batch}" {flags}"'
    before = snapshot(repo)
    result = subprocess.run(command, cwd=repo.parent, env=environment(), input="",
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=30)
    assert result.returncode == 0, result.stdout + result.stderr
    assert_no_secret_output(result)
    if action == "start":
        assert "VALIDATION-ONLY" in result.stdout
        assert snapshot(repo) == before
    else:
        assert (repo / "fake-stop.txt").exists()
        assert not (repo / "run").exists()


def test_validation_restores_callers_environment_and_working_directory(repo, tmp_path):
    assert launch(repo).returncode == 0
    harness = tmp_path / "restore.ps1"
    harness.write_text("""
param($Launcher)
$ErrorActionPreference='Stop'
$env:SPACE_SIM_STREAM_JWT_SECRET = 'caller-value-must-survive'
$before = (Get-Location).Path
& $Launcher -ValidateOnly -NonInteractive
if ($env:SPACE_SIM_STREAM_JWT_SECRET -ne 'caller-value-must-survive') { throw 'Leaked secret override to caller.' }
if ((Get-Location).Path -ne $before) { throw 'Changed caller directory.' }
""", encoding="utf-8")
    result = run_ps(harness, repo / "scripts/deployment_launcher.ps1")
    assert result.returncode == 0, result.stdout + result.stderr


def test_incomplete_encrypted_store_is_not_automatically_rekeyed(repo, tmp_path):
    assert launch(repo).returncode == 0
    secret = saved_secret(repo)
    harness = tmp_path / "incomplete.ps1"
    harness.write_text("""
param($Helpers, $Path)
$ErrorActionPreference='Stop'
. $Helpers
Save-LauncherSecrets $Path @{SPACE_SIM_ADMIN_PASSWORD='Saved-Test-Password!'}
""", encoding="utf-8")
    result = run_ps(harness, ROOT / "scripts/deployment_bootstrap.ps1", secret)
    assert result.returncode == 0, result.stderr
    before = snapshot(repo)
    result = launch(repo)
    assert result.returncode != 0
    assert "Cannot decrypt saved deployment secrets" in result.stderr
    assert snapshot(repo) == before


def test_start_stop_mutex_rejects_concurrent_lifecycle_operations(repo, tmp_path):
    harness = tmp_path / "locked.ps1"
    harness.write_text(r"""
param($Root, $Pwsh)
$ErrorActionPreference='Stop'
$hash = [Security.Cryptography.SHA256]::Create()
try { $id = [BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($Root.ToLowerInvariant()))).Replace('-','') }
finally { $hash.Dispose() }
$mutex = [Threading.Mutex]::new($false, ('Local\SpaceSimDeployment-' + $id))
$null = $mutex.WaitOne(0)
try {
    & $Pwsh -NoProfile -File (Join-Path $Root 'scripts/deployment_launcher.ps1') -Action Stop -NonInteractive
    if ($LASTEXITCODE -eq 0) { throw 'Concurrent stop was allowed.' }
    if (Test-Path -LiteralPath (Join-Path $Root 'fake-stop.txt')) { throw 'Stop reached the platform during startup.' }
} finally { $mutex.ReleaseMutex(); $mutex.Dispose() }
exit 0
""", encoding="utf-8")
    result = run_ps(harness, repo, PWSH)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "Another start/stop operation" in result.stderr
    assert not (repo / "run").exists()
