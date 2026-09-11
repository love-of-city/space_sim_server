"""Isolated fixed-public deployment tests; no services, tunnel or real network are used."""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or r"C:\Program Files\PowerShell\7\pwsh.exe"
pytestmark = pytest.mark.skipif(
    os.name != "nt" or not Path(PWSH).is_file(), reason="Windows PowerShell 7 required"
)

TEST_TURN_SECRET = "unit-test-turn-shared-secret-123456"
TEST_IP = "203.0.113.42"


def clean_environment(**extra):
    environment = {key: value for key, value in os.environ.items() if not key.startswith("SPACE_SIM_")}
    environment["PATH"] = str(Path(sys.executable).parent) + os.pathsep + environment.get("PATH", "")
    environment.update(extra)
    return environment


def run_ps(script, *args, env=None):
    return subprocess.run(
        [PWSH, "-NoProfile", "-File", str(script), *map(str, args)],
        cwd=script.parent, env=env or clean_environment(), text=True, encoding="utf-8",
        errors="replace", capture_output=True, timeout=60,
    )


def write_eturnal(path: Path, *, relay: str = TEST_IP, port: int = 3478, secret: str = TEST_TURN_SECRET):
    path.write_text(f"""eturnal:
  secret: "{secret}"
  listen:
    -
      ip: "0.0.0.0"
      port: {port}
      transport: udp
  relay_ipv4_addr: "{relay}"
  relay_min_port: 49152
  relay_max_port: 65535
""", encoding="utf-8")


def test_probe_builds_fixed_candidate_without_echoing_secret(tmp_path):
    repo = tmp_path / "fixed probe repo"
    (repo / "scripts").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/fixed_deployment_helpers.ps1", repo / "scripts/fixed_deployment_helpers.ps1")
    config = repo / "eturnal.yml"
    write_eturnal(config)
    harness = tmp_path / "probe.ps1"
    harness.write_text("""
param($Helpers, $Root)
$ErrorActionPreference='Stop'
. $Helpers
$candidate = Get-FixedDeploymentCandidate -ProjectRoot $Root
if (!$candidate) { throw 'No fixed candidate.' }
[pscustomobject]@{
    url = $candidate.PublicUrl
    stun = $candidate.StunUrl
    turn_urls = @($candidate.TurnUrls)
    port = $candidate.TurnPort
    placeholder = $candidate.TurnSecretIsPlaceholder
} | ConvertTo-Json -Depth 5 -Compress
""", encoding="utf-8")
    env = clean_environment(SPACE_SIM_ETURNAL_CONFIG=str(config), SPACE_SIM_PUBLIC_IP=TEST_IP)
    result = run_ps(harness, ROOT / "scripts/fixed_deployment_helpers.ps1", repo, env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    payload = json.loads(result.stdout.strip().splitlines()[-1])
    assert payload["url"] == f"https://{TEST_IP}.sslip.io"
    assert payload["stun"] == f"stun:{TEST_IP}:3478"
    assert payload["turn_urls"] == [
        f"turn:{TEST_IP}:3478?transport=udp",
        f"turn:{TEST_IP}:3478?transport=tcp",
    ]
    assert payload["port"] == 3478
    assert payload["placeholder"] is False
    assert TEST_TURN_SECRET not in result.stdout + result.stderr

def test_probe_returns_none_when_eturnal_configuration_is_absent(tmp_path):
    repo = tmp_path / "missing eturnal repo"
    (repo / "scripts").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/fixed_deployment_helpers.ps1", repo / "scripts/fixed_deployment_helpers.ps1")
    harness = tmp_path / "probe-missing.ps1"
    harness.write_text("""
param($Helpers, $Root)
$ErrorActionPreference='Stop'
. $Helpers
if (Get-FixedDeploymentCandidate -ProjectRoot $Root) { throw 'Missing eturnal must not produce a candidate.' }
'MISSING-OK'
""", encoding="utf-8")
    env = clean_environment(
        SPACE_SIM_ETURNAL_CONFIG=str(repo / "deploy/no-such-eturnal.yml"),
        SPACE_SIM_PUBLIC_IP=TEST_IP,
    )
    result = run_ps(harness, ROOT / "scripts/fixed_deployment_helpers.ps1", repo, env=env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "MISSING-OK" in result.stdout


def test_fixed_config_generation_is_idempotent_and_flags_placeholder_secret(tmp_path):
    repo = tmp_path / "fixed init repo"
    (repo / "scripts").mkdir(parents=True)
    (repo / "deploy").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/fixed_deployment_helpers.ps1", repo / "scripts/fixed_deployment_helpers.ps1")
    shutil.copyfile(ROOT / "scripts/deployment_config.ps1", repo / "scripts/deployment_config.ps1")
    shutil.copyfile(ROOT / "deploy/deployment.example.json", repo / "deploy/deployment.example.json")
    config = repo / "eturnal.yml"
    write_eturnal(config, secret="replace_with_a_long_random_secret")
    harness = tmp_path / "init.ps1"
    harness.write_text("""
param($Root)
$ErrorActionPreference='Stop'
. (Join-Path $Root 'scripts/deployment_config.ps1')
. (Join-Path $Root 'scripts/fixed_deployment_helpers.ps1')
$candidate = Get-FixedDeploymentCandidate -ProjectRoot $Root
if (!$candidate) { throw 'No fixed candidate.' }
if (!$candidate.TurnSecretIsPlaceholder) { throw 'Placeholder secret was not detected.' }
Initialize-FixedConfig -ConfigPath (Join-Path $Root 'deploy/fixed.local.json') -ProjectRoot $Root -Candidate $candidate
""", encoding="utf-8")
    env = clean_environment(SPACE_SIM_ETURNAL_CONFIG=str(config), SPACE_SIM_PUBLIC_IP=TEST_IP)
    first = run_ps(harness, repo, env=env)
    assert first.returncode == 0, first.stdout + first.stderr
    path = repo / "deploy/fixed.local.json"
    original = path.read_bytes()
    payload = json.loads(original.decode("utf-8"))
    assert payload["tls_mode"] == "acme"
    assert payload["public_url"] == f"https://{TEST_IP}.sslip.io"
    assert payload["ice_servers"] == [{"urls": f"stun:{TEST_IP}:3478"}]
    assert payload["turn_urls"] == [
        f"turn:{TEST_IP}:3478?transport=udp",
        f"turn:{TEST_IP}:3478?transport=tcp",
    ]
    second = run_ps(harness, repo, env=env)
    assert second.returncode == 0, second.stdout + second.stderr
    assert path.read_bytes() == original
    assert not list((repo / "deploy").glob(".fixed-*"))

@pytest.fixture
def launcher_repo(tmp_path):
    root = tmp_path / "fixed launcher repo"
    for directory in ("scripts", "deploy", "tools"):
        (root / directory).mkdir(parents=True)
    for name in ("deployment_launcher.ps1", "deployment_bootstrap.ps1", "deployment_config.ps1",
                 "fixed_deployment_helpers.ps1"):
        shutil.copyfile(ROOT / "scripts" / name, root / "scripts" / name)
    for name in ("deployment.example.json", "Caddyfile.template"):
        shutil.copyfile(ROOT / "deploy" / name, root / "deploy" / name)
    (root / "tools/check_deployment_auth.py").write_text("import sys\nsys.exit(0)\n", encoding="utf-8")
    (root / "scripts/run_platform.ps1").write_text("exit 0\n", encoding="utf-8")
    (root / "scripts/public_deployment_helpers.ps1").write_text("# Fake public helpers.\n", encoding="utf-8")
    (root / "scripts/public_deployment.ps1").write_text("""
function Start-PublicDeployment {
    param([string]$ProjectRoot,[string]$ConfigPath,[string]$SecretPath,[string]$CondaRoot,
          [switch]$ValidateOnly,[switch]$NonInteractive,[switch]$Restart,[System.Collections.IDictionary]$Overrides)
    $settings=Get-DeploymentSettings $ConfigPath $ProjectRoot
    $values=Read-LauncherSecrets $SecretPath $settings -GenerateAdmin -NonInteractive -Overrides $Overrides
    $config=Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json
    $hash=[Security.Cryptography.SHA256]::Create()
    try {
        $secret=[string]$values['SPACE_SIM_TURN_AUTH_SECRET']
        $fingerprint=[BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($secret))).Replace('-','')
    } finally { $hash.Dispose() }
    $run=Join-Path $ProjectRoot 'run'
    $null=New-Item -ItemType Directory -Path $run -Force
    @{turn_secret_sha256=$fingerprint;tls_mode=$config.tls_mode;turn_urls=@($config.turn_urls)} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $run 'turn-start.json') -Encoding utf8NoBOM
}
""", encoding="utf-8")
    (root / "scripts/stop_platform.ps1").write_text("exit 0\n", encoding="utf-8")
    (root / "scripts/deploy_platform.ps1").write_text("""
param([string]$ConfigPath,[switch]$ValidateOnly,[switch]$Start)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
if ($ValidateOnly) { Write-Output 'FIXED-VALIDATION'; return }
$config=Get-Content -Raw -LiteralPath $ConfigPath | ConvertFrom-Json
$hash=[Security.Cryptography.SHA256]::Create()
try {
    $secret=[string]$env:SPACE_SIM_TURN_AUTH_SECRET
    $fingerprint=[BitConverter]::ToString($hash.ComputeHash([Text.Encoding]::UTF8.GetBytes($secret))).Replace('-','')
} finally { $hash.Dispose() }
$run=Join-Path $root 'run'
$null=New-Item -ItemType Directory -Path $run -Force
@{turn_secret_sha256=$fingerprint;public_url=$config.public_url;config_path=$ConfigPath} |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $run 'fixed-start.json') -Encoding utf8NoBOM
@{mode='fixed';public_url=$config.public_url;config_path=(Resolve-Path -LiteralPath $ConfigPath).Path} |
    ConvertTo-Json | Set-Content -LiteralPath (Join-Path $run 'public-access.json') -Encoding utf8NoBOM
""", encoding="utf-8")
    with (root / "scripts/deployment_bootstrap.ps1").open("a", encoding="utf-8") as file:
        file.write("""
function Enable-LauncherRuntime { param($CondaRoot) }
function Resolve-LauncherCaddy { param($RequestedExecutable,$ProjectRoot) return (Join-Path $ProjectRoot 'fake-caddy.exe') }
function Show-LauncherAccess { param($Settings,[switch]$NonInteractive) Write-Output ('DIRECT:' + $Settings.PublicUrl) }
function Test-LauncherRunning { return $false }
function Save-LauncherRunMarker { param($ProjectRoot,$ConfigPath,$SecretPath) }
""")
    config = root / "eturnal.yml"
    write_eturnal(config)
    return root


def launch(root, *args, **env):
    values = {
        "SPACE_SIM_ETURNAL_CONFIG": str(root / "eturnal.yml"),
        "SPACE_SIM_PUBLIC_IP": TEST_IP,
        "SPACE_SIM_ADMIN_PASSWORD": "Fixed-Test-Password!",
        "SPACE_SIM_STREAM_JWT_SECRET": "j" * 48,
        "SPACE_SIM_STREAM_ACCESS_KEY": "k" * 32,
    }
    values.update(env)
    return run_ps(
        root / "scripts/deployment_launcher.ps1", "-NonInteractive", *args,
        env=clean_environment(**values),
    )


def test_launcher_auto_selects_turn_tunnel_and_injects_detected_secret(launcher_repo):
    result = launch(launcher_repo)
    assert result.returncode == 0, result.stdout + result.stderr
    config = json.loads((launcher_repo / "deploy/turn.local.json").read_text(encoding="utf-8"))
    assert config["tls_mode"] == "tunnel"
    assert config["turn_urls"] == [
        f"turn:{TEST_IP}:3478?transport=udp",
        f"turn:{TEST_IP}:3478?transport=tcp",
    ]
    recorded = json.loads((launcher_repo / "run/turn-start.json").read_text(encoding="utf-8"))
    expected = hashlib.sha256(TEST_TURN_SECRET.encode()).hexdigest().upper()
    assert recorded["turn_secret_sha256"] == expected
    assert recorded["tls_mode"] == "tunnel"
    assert recorded["turn_urls"] == config["turn_urls"]
    assert TEST_TURN_SECRET not in result.stdout + result.stderr


def test_fixed_mode_still_supports_acme_public_hostname(launcher_repo):
    result = launch(launcher_repo, "-Mode", "Fixed")
    assert result.returncode == 0, result.stdout + result.stderr
    fixed = json.loads((launcher_repo / "deploy/fixed.local.json").read_text(encoding="utf-8"))
    assert fixed["tls_mode"] == "acme"
    assert fixed["public_url"] == f"https://{TEST_IP}.sslip.io"
    recorded = json.loads((launcher_repo / "run/fixed-start.json").read_text(encoding="utf-8"))
    expected = hashlib.sha256(TEST_TURN_SECRET.encode()).hexdigest().upper()
    assert recorded["turn_secret_sha256"] == expected
    assert TEST_TURN_SECRET not in result.stdout + result.stderr


def test_fixed_validate_only_reports_but_does_not_create_config(launcher_repo):
    before = {path: path.read_bytes() for path in launcher_repo.rglob("*") if path.is_file()}
    result = launch(launcher_repo, "-Mode", "Fixed", "-ValidateOnly")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "首次启动时自动生成" in result.stdout
    assert not (launcher_repo / "deploy/fixed.local.json").exists()
    assert before == {path: path.read_bytes() for path in launcher_repo.rglob("*") if path.is_file()}


def _write_ip_config(path: Path, public_url: str) -> None:
    config = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    config.update(public_url=public_url, tls_mode="ip-acme")
    path.write_text(json.dumps(config), encoding="utf-8")


def _render_repo(tmp_path, name):
    repo = tmp_path / name
    (repo / "scripts").mkdir(parents=True)
    (repo / "deploy").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/deployment_config.ps1", repo / "scripts/deployment_config.ps1")
    shutil.copyfile(ROOT / "deploy/Caddyfile.template", repo / "deploy/Caddyfile.template")
    shutil.copyfile(ROOT / "deploy/deployment.example.json", repo / "deploy/deployment.example.json")
    harness = tmp_path / (name + ".ps1")
    harness.write_text("""
param($Root)
$ErrorActionPreference='Stop'
. (Join-Path $Root 'scripts/deployment_config.ps1')
$settings = Get-DeploymentSettings (Join-Path $Root 'deploy/ip.local.json') $Root
Get-DeploymentCaddyfile $settings $Root
""", encoding="utf-8")
    return repo, harness


def test_ip_acme_renders_short_lived_profile_and_default_sni(tmp_path):
    repo, harness = _render_repo(tmp_path, "ip acme render repo")
    _write_ip_config(repo / "deploy/ip.local.json", f"https://{TEST_IP}")
    result = run_ps(harness, repo)
    assert result.returncode == 0, result.stdout + result.stderr
    rendered = result.stdout
    # Let's Encrypt only issues IP certificates from the short-lived profile,
    # and clients omit SNI for an IP literal, so default_sni is mandatory.
    assert f"https://{TEST_IP} {{" in rendered
    assert f"default_sni {TEST_IP}" in rendered
    assert "profile shortlived" in rendered
    assert "storage file_system" in rendered
    assert "admin off" in rendered
    assert "reverse_proxy 127.0.0.1:" in rendered


def test_ip_acme_rejects_a_hostname_entry_point(tmp_path):
    repo, harness = _render_repo(tmp_path, "ip acme reject repo")
    _write_ip_config(repo / "deploy/ip.local.json", "https://sim.test")
    result = run_ps(harness, repo)
    assert result.returncode != 0
    assert "literal IP address" in result.stdout + result.stderr


def test_ip_mode_pins_the_public_ip_and_injects_turn_secret(launcher_repo):
    result = launch(launcher_repo, "-Mode", "Ip")
    assert result.returncode == 0, result.stdout + result.stderr
    config = json.loads((launcher_repo / "deploy/ip.local.json").read_text(encoding="utf-8"))
    assert config["tls_mode"] == "ip-acme"
    assert config["public_url"] == f"https://{TEST_IP}"
    recorded = json.loads((launcher_repo / "run/fixed-start.json").read_text(encoding="utf-8"))
    expected = hashlib.sha256(TEST_TURN_SECRET.encode()).hexdigest().upper()
    assert recorded["turn_secret_sha256"] == expected
    assert recorded["public_url"] == f"https://{TEST_IP}"
    assert TEST_TURN_SECRET not in result.stdout + result.stderr


def test_ip_mode_validate_only_reports_but_does_not_create_config(launcher_repo):
    before = {path: path.read_bytes() for path in launcher_repo.rglob("*") if path.is_file()}
    result = launch(launcher_repo, "-Mode", "Ip", "-ValidateOnly")
    assert result.returncode == 0, result.stdout + result.stderr
    assert "公网 IP 证书入口将在首次启动时自动申请" in result.stdout
    assert not (launcher_repo / "deploy/ip.local.json").exists()
    assert before == {path: path.read_bytes() for path in launcher_repo.rglob("*") if path.is_file()}


def _gate_repo(tmp_path, name, require_access_key):
    repo = tmp_path / name
    (repo / "scripts").mkdir(parents=True)
    (repo / "deploy").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/deployment_config.ps1", repo / "scripts/deployment_config.ps1")
    config = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    config.update(public_url=f"https://{TEST_IP}", tls_mode="ip-acme")
    if require_access_key is None:
        config.pop("require_access_key", None)
    else:
        config["require_access_key"] = require_access_key
    (repo / "deploy/ip.local.json").write_text(json.dumps(config), encoding="utf-8")
    harness = tmp_path / (name + ".ps1")
    harness.write_text("""
param($Root)
$ErrorActionPreference='Stop'
. (Join-Path $Root 'scripts/deployment_config.ps1')
$settings = Get-DeploymentSettings (Join-Path $Root 'deploy/ip.local.json') $Root
Write-Output ('REQUIRE=' + $settings.RequireAccessKey)
try { Assert-DeploymentSecrets $settings; Write-Output 'ASSERT=ok' }
catch { Write-Output ('ASSERT=fail: ' + $_.Exception.Message) }
""", encoding="utf-8")
    return repo, harness


def _gate_env():
    # Deliberately omits SPACE_SIM_STREAM_ACCESS_KEY so the assertion outcome
    # depends only on require_access_key.
    return clean_environment(
        SPACE_SIM_ADMIN_PASSWORD="Unit-Test-Password!",
        SPACE_SIM_STREAM_JWT_SECRET="j" * 48,
    )


def test_disabling_access_key_drops_the_secret_requirement(tmp_path):
    repo, harness = _gate_repo(tmp_path, "opt out gate", False)
    result = run_ps(harness, repo, env=_gate_env())
    assert result.returncode == 0, result.stdout + result.stderr
    assert "REQUIRE=False" in result.stdout
    assert "ASSERT=ok" in result.stdout


def test_enabling_access_key_still_requires_the_secret(tmp_path):
    repo, harness = _gate_repo(tmp_path, "opt in gate", True)
    result = run_ps(harness, repo, env=_gate_env())
    assert result.returncode == 0, result.stdout + result.stderr
    assert "REQUIRE=True" in result.stdout
    assert "ASSERT=fail" in result.stdout
    assert "SPACE_SIM_STREAM_ACCESS_KEY" in result.stdout


def test_missing_access_key_setting_defaults_to_required(tmp_path):
    # Configs written before this option must keep the safe default.
    repo, harness = _gate_repo(tmp_path, "default gate", None)
    result = run_ps(harness, repo, env=_gate_env())
    assert result.returncode == 0, result.stdout + result.stderr
    assert "REQUIRE=True" in result.stdout
    assert "ASSERT=fail" in result.stdout


def _window_repo(tmp_path, name, value):
    repo = tmp_path / name
    (repo / "scripts").mkdir(parents=True)
    (repo / "deploy").mkdir(parents=True)
    shutil.copyfile(ROOT / "scripts/deployment_config.ps1", repo / "scripts/deployment_config.ps1")
    config = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    config.update(public_url=f"https://{TEST_IP}", tls_mode="ip-acme")
    if value is _ABSENT:
        config.pop("show_access_window", None)
    else:
        config["show_access_window"] = value
    (repo / "deploy/ip.local.json").write_text(json.dumps(config), encoding="utf-8")
    harness = tmp_path / (name + ".ps1")
    harness.write_text("""
param($Root)
$ErrorActionPreference='Stop'
. (Join-Path $Root 'scripts/deployment_config.ps1')
$settings = Get-DeploymentSettings (Join-Path $Root 'deploy/ip.local.json') $Root
Write-Output ('WINDOW=' + $settings.ShowAccessWindow)
""", encoding="utf-8")
    return repo, harness


_ABSENT = object()


def test_access_window_defaults_to_shown_when_absent(tmp_path):
    repo, harness = _window_repo(tmp_path, "window default", _ABSENT)
    result = run_ps(harness, repo)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "WINDOW=True" in result.stdout


def test_access_window_can_be_disabled(tmp_path):
    repo, harness = _window_repo(tmp_path, "window off", False)
    result = run_ps(harness, repo)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "WINDOW=False" in result.stdout


def test_access_window_rejects_non_boolean(tmp_path):
    repo, harness = _window_repo(tmp_path, "window invalid", "no")
    result = run_ps(harness, repo)
    assert result.returncode != 0
    assert "show_access_window must be true or false" in result.stdout + result.stderr
