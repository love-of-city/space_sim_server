"""Configuration-only tests: never run -Start or touch the real platform."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh") or (r"C:\Program Files\PowerShell\7\pwsh.exe" if os.name == "nt" else None)
pytestmark = pytest.mark.skipif(not PWSH or not Path(PWSH).is_file(), reason="PowerShell 7 not available")


def config():
    value = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    value.update(public_url="https://sim.test:8443", tls_mode="internal")
    return value


def environment():
    return {**os.environ, "SPACE_SIM_ADMIN_PASSWORD": "Deployment-Test-Password!",
        "SPACE_SIM_STREAM_JWT_SECRET": "j" * 48, "SPACE_SIM_STREAM_ACCESS_KEY": "k" * 32,
        "SPACE_SIM_TURN_AUTH_SECRET": ""}


def invoke(tmp_path, value, env=None, flags=("-ValidateOnly",)):
    file = tmp_path / "deployment test.json"
    file.write_text(json.dumps(value), encoding="utf-8")
    return subprocess.run([PWSH, "-NoProfile", "-File", str(ROOT / "scripts/deploy_platform.ps1"),
        "-ConfigPath", str(file), *flags], cwd=ROOT, env=env or environment(),
        text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=20)


def test_validate_only_and_default_mode_have_no_runtime_side_effects(tmp_path):
    paths = [ROOT / "run/platform.json", ROOT / "run/scene_runtime.json", ROOT / "run/deployment.json", ROOT / "run/deployment/Caddyfile"]
    before = {p: p.read_bytes() if p.exists() else None for p in paths}
    for flags in [("-ValidateOnly",), ()]:
        result = invoke(tmp_path, config(), flags=flags)
        assert result.returncode == 0, result.stderr
        assert "No files, services" in result.stdout
        assert "Deployment-Test-Password!" not in result.stdout + result.stderr
        assert "j" * 48 not in result.stdout + result.stderr
    assert before == {p: p.read_bytes() if p.exists() else None for p in paths}


@pytest.mark.parametrize("key,value", [
    ("public_url", "http://sim.test"), ("public_url", "https://sim.test/path"),
    ("public_url", "https://sim.test/?key=secret"), ("public_url", "https://user@sim.test"),
    ("public_url", "https://sim.example.com"), ("tls_mode", "disabled"),
    ("api_port", 8080), ("api_port", 8443), ("api_port", "8000"), ("api_port", 0),
    ("ice_servers", {}), ("ice_servers", [{"urls": "turn:relay.test", "credential": "secret"}]),
    ("turn_urls", ["https://relay.test"]), ("unrecognized_option", True),
])
def test_invalid_configuration_fails_before_any_service_change(tmp_path, key, value):
    settings = config(); settings[key] = value
    assert invoke(tmp_path, settings).returncode != 0


@pytest.mark.parametrize("name,value", [
    ("SPACE_SIM_ADMIN_PASSWORD", "ChangeMe123!"), ("SPACE_SIM_STREAM_JWT_SECRET", ""),
    ("SPACE_SIM_STREAM_ACCESS_KEY", "not URL safe"),
])
def test_invalid_secrets_are_rejected_without_echoing_them(tmp_path, name, value):
    env = environment(); env[name] = value
    result = invoke(tmp_path, config(), env=env)
    assert result.returncode != 0
    assert name in result.stderr
    if value:
        assert value not in result.stdout + result.stderr


def test_turn_requires_matching_secret_and_stun_accepts_arrays(tmp_path):
    settings = config(); settings["turn_urls"] = ["turn:relay.test:3478?transport=udp"]
    settings["ice_servers"] = [{"urls": ["stun:relay.test:3478"]}]
    assert invoke(tmp_path, settings).returncode != 0
    env = environment(); env["SPACE_SIM_TURN_AUTH_SECRET"] = "t" * 32
    result = invoke(tmp_path, settings, env=env)
    assert result.returncode == 0, result.stderr


def test_rendered_proxy_keeps_control_and_signalling_same_origin(tmp_path):
    file = tmp_path / "config.json"; file.write_text(json.dumps(config()), encoding="utf-8")
    # Paths passed as process arguments, not interpolated shell code.
    helper = tmp_path / "render.ps1"
    helper.write_text("param($Root, $Config)\n. (Join-Path $Root 'scripts/deployment_config.ps1')\n"
        "$settings = Get-DeploymentSettings $Config $Root\nGet-DeploymentCaddyfile $settings $Root\n", encoding="utf-8")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(helper), str(ROOT), str(file)],
        capture_output=True, text=True, encoding="utf-8", timeout=20)
    assert result.returncode == 0, result.stderr
    assert "https://sim.test:8443" in result.stdout
    assert "tls internal" in result.stdout
    assert "reverse_proxy 127.0.0.1:8000" in result.stdout
    assert "reverse_proxy 127.0.0.1:8080" in result.stdout
    assert "path /stream" in result.stdout
    assert "__TLS__" not in result.stdout


def test_all_deployment_powershell_files_parse_without_execution(tmp_path):
    helper = tmp_path / "parse.ps1"
    helper.write_text("param($Root)\n$failed=$false\n"
        "Get-ChildItem -LiteralPath (Join-Path $Root 'scripts') -Filter '*.ps1' | ForEach-Object {\n"
        "$tokens=$null; $errors=$null; $null=[System.Management.Automation.Language.Parser]::ParseFile($_.FullName,[ref]$tokens,[ref]$errors)\n"
        "if($errors.Count){$errors | Out-String | Write-Output; $failed=$true}\n}\nif($failed){exit 1}\n", encoding="utf-8")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(helper), str(ROOT)], capture_output=True, text=True, timeout=20)
    assert result.returncode == 0, result.stdout + result.stderr
