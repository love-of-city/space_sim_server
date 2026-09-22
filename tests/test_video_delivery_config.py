"""Video FPS/quality configuration: isolated checks never start the platform or UE."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
PWSH = shutil.which("pwsh")

@pytest.mark.skipif(not PWSH, reason="PowerShell 7 required")
@pytest.mark.parametrize("fps", [None, 60, 90, 120, 0, 121, True, "90", 90.5])
def test_deployment_preview_fps_validation(tmp_path, fps):
    config = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    config["public_url"] = "https://sim.test"
    if fps is None:
        config.pop("preview_fps", None)
    else:
        config["preview_fps"] = fps
    path = tmp_path / "settings.json"
    path.write_text(json.dumps(config))
    helper = tmp_path / "check.ps1"
    helper.write_text("param($Root,$Config)\n$ErrorActionPreference='Stop'\n"
        ". (Join-Path $Root 'scripts/deployment_config.ps1')\n"
        "(Get-DeploymentSettings $Config $Root).PreviewFps\n")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(helper), str(ROOT), str(path)],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=15)
    valid = fps is None or type(fps) is int and 1 <= fps <= 120
    if valid:
        assert result.returncode == 0, result.stderr
        assert int(result.stdout.strip()) == (90 if fps is None else fps)
    else:
        assert result.returncode != 0
        assert "preview_fps" in result.stderr


def test_rate_plumbed_through_both_deployment_modes():
    for filename in ("deploy_platform.ps1", "public_deployment.ps1"):
        text = (ROOT / "scripts" / filename).read_text(encoding="utf-8").replace(" ", "")
        assert "PreviewRate=$settings.PreviewFps" in text
    text = (ROOT / "scripts/start_scene_instance.ps1").read_text(encoding="utf-8")
    assert "PixelStreamingCameraFps = [int][Math]::Round($PreviewRate)" in text
    assert "[Math]::Min(30" not in text
    assert "[Math]::Min(60" not in text


def test_no_change_to_physics_or_camera_resolution():
    text = (ROOT / "simulation/teleop_grasp_unreal.py").read_text(encoding="utf-8")
    from space_arm_platform.sampling import DYNAMICS_HZ, RENDER_HZ
    # The rational clock replaced the old rounded frame_period_ns literal.
    # Preview tuning must still leave native scheduling and output rates intact.
    assert (DYNAMICS_HZ, RENDER_HZ) == (240, 30)
    assert "frame_rate_hz=RENDER_HZ" in text
    assert "native.TIME_STEP = 1.0 / DYNAMICS_HZ" in text
    assert "camera_pip_resolution=(640, 360)" in text
    text = (ROOT / "scripts/start_scene_instance.ps1").read_text(encoding="utf-8")
    assert "if ($datasetCapture)" in text


def test_project_caps_adaptive_video_bitrate_without_forcing_a_high_floor():
    config = ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/Config/DefaultEngine.ini"
    if not config.is_file():
        pytest.skip("Sibling UE adapter is not checked out")
    text = config.read_text(encoding="utf-8-sig")
    settings = text.split("[SystemSettings]", 1)[1].split("[", 1)[0]
    assert "PixelStreaming2.WebRTC.MaxBitrate=8000000" in settings
    assert "PixelStreaming2.Encoder.TargetBitrate=" not in settings
    assert "PixelStreaming2.WebRTC.MinBitrate=" not in settings


@pytest.mark.skipif(os.name != "nt" or not PWSH, reason="Windows PowerShell required")
@pytest.mark.parametrize("quality", [None, 0, 60, 75, 100, -1, 101])
def test_renderer_arguments_are_valid_and_do_not_start_ue(tmp_path, quality):
    adapter = ROOT.parent / "space_sim_UE_Adapter"
    source = adapter / "Unreal/BskUnrealRenderer/scripts/start_renderer.ps1"
    if not source.is_file():
        pytest.skip("Paired UE adapter checkout not present")
    (tmp_path / "start_renderer.ps1").write_text(source.read_text(encoding="utf-8-sig"), encoding="utf-8")
    (tmp_path / "common.ps1").write_text("$ProjectFile=Join-Path $PSScriptRoot 'fake.uproject'\n"
        "$ProjectRoot=$PSScriptRoot\nfunction Resolve-UnrealRoot($Requested){ return $PSScriptRoot }\n", encoding="utf-8")
    quality_arg = "" if quality is None else f" -EncoderMinQuality {quality}"
    wrapper = tmp_path / "wrapper.ps1"
    wrapper.write_text("$ErrorActionPreference='Stop'\n"
        "function Start-Process { param($FilePath,$ArgumentList,[switch]$PassThru,$WindowStyle)\n"
        " @{args=@($ArgumentList);window=$WindowStyle} | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'args.json')\n"
        " [pscustomobject]@{Id=12345}\n}\n"
        "& (Join-Path $PSScriptRoot 'start_renderer.ps1') -PixelStreamingURL ws://127.0.0.1:1 -PixelStreamingCameraIds overview -PixelStreamingFps 90 -PixelStreamingCameraFps 90" + quality_arg + "\n", encoding="utf-8")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(wrapper)], capture_output=True,
                            text=True, encoding="utf-8", errors="replace", timeout=15)
    if quality in (-1, 101):
        assert result.returncode != 0
        assert "EncoderMinQuality" in result.stderr
        assert not (tmp_path / "args.json").exists(), "invalid settings must not start UE"
        return
    assert result.returncode == 0, result.stderr
    actual = json.loads((tmp_path / "args.json").read_text(encoding="utf-8-sig"))
    assert actual["window"] == "Hidden"
    for expected in ("-PixelStreamingWebRTCFps=90", "-BskPixelStreamingCameraFps=90",
                     "-PixelStreamingUseMediaCapture=false", "-PixelStreamingDecoupleFramerate=false",
                     '-ExecCmds="t.MaxFPS 90,r.VSync 0"', "-ResX=1280", "-ResY=720"):
        assert expected in actual["args"]

    assert f"-PixelStreamingEncoderMinQuality={60 if quality is None else quality}" in actual["args"]
    assert not any(arg.startswith(("-PixelStreamingEncoderTargetBitrate=", "-PixelStreamingWebRTCMinBitrate=",
                                  "-PixelStreamingWebRTCMaxBitrate=")) for arg in actual["args"])


@pytest.mark.skipif(not PWSH, reason="PowerShell 7 required")
@pytest.mark.parametrize("quality", ["absent", 0, 60, 75, 100, -1, 101, True, False, "60", 60.5, None])
def test_deployment_encoder_min_quality_validation(tmp_path, quality):
    config = json.loads((ROOT / "deploy/deployment.example.json").read_text(encoding="utf-8"))
    config["public_url"] = "https://sim.test"
    if quality == "absent":
        config.pop("encoder_min_quality", None)
    else:
        config["encoder_min_quality"] = quality
    path = tmp_path / "settings.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    helper = tmp_path / "check.ps1"
    helper.write_text("param($Root,$Config)\n$ErrorActionPreference='Stop'\n"
        ". (Join-Path $Root 'scripts/deployment_config.ps1')\n"
        "(Get-DeploymentSettings $Config $Root).EncoderMinQuality\n", encoding="utf-8")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(helper), str(ROOT), str(path)],
        capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=15)
    valid = quality == "absent" or type(quality) is int and 0 <= quality <= 100
    if valid:
        assert result.returncode == 0, result.stderr
        assert int(result.stdout.strip()) == (60 if quality == "absent" else quality)
    else:
        assert result.returncode != 0
        assert "encoder_min_quality" in result.stderr


def test_quality_plumbed_through_all_launch_scripts():
    for filename in ("deploy_platform.ps1", "public_deployment.ps1"):
        text = (ROOT / "scripts" / filename).read_text(encoding="utf-8").replace(" ", "")
        assert "EncoderMinQuality=$settings.EncoderMinQuality" in text
    for filename in ("run_platform.ps1", "run_backend.ps1", "start_scene_instance.ps1"):
        text = (ROOT / "scripts" / filename).read_text(encoding="utf-8")
        assert "[ValidateRange(0, 100)]\n    [int]$EncoderMinQuality = 60" in text
    text = (ROOT / "scripts/run_platform.ps1").read_text(encoding="utf-8")
    assert "'-EncoderMinQuality', $EncoderMinQuality" in text
    text = (ROOT / "scripts/run_backend.ps1").read_text(encoding="utf-8")
    assert "'--runtime-encoder-min-quality', $EncoderMinQuality" in text
    text = (ROOT / "scripts/start_scene_instance.ps1").read_text(encoding="utf-8")
    assert "EncoderMinQuality = $EncoderMinQuality" in text
    assert "encoder_min_quality = $EncoderMinQuality" in text


@pytest.mark.parametrize("quality", [None, 0, 75, 100])
def test_quality_flows_from_api_to_scene_launcher(tmp_path, monkeypatch, quality):
    from types import SimpleNamespace
    from fastapi.testclient import TestClient
    from space_arm_platform.app import PlatformConfig, create_app
    from space_arm_platform.models import SceneInstanceCreate

    project = tmp_path / "project"
    (project / "frontend").mkdir(parents=True)
    (project / "scripts").mkdir()
    (project / "scripts/start_scene_instance.ps1").touch()
    powershell = project / "fake-pwsh.exe"
    powershell.touch()
    commands = []

    def fake_popen(command, **kwargs):
        commands.append(command)
        return SimpleNamespace(pid=12345, returncode=0, poll=lambda: 0)

    monkeypatch.setattr("space_arm_platform.scene_runtime.subprocess.Popen", fake_popen)
    overrides = {} if quality is None else {"runtime_encoder_min_quality": quality}
    app = create_app(PlatformConfig(
        project_root=project, data_root=tmp_path / "episodes", simulation_port=0, capture_port=0,
        runtime_adapter_root=project, runtime_model_root=ROOT / "model/SARM/platform", runtime_unreal_root=project,
        runtime_powershell_exe=powershell, **overrides,
    ))
    expected = 60 if quality is None else quality
    with TestClient(app) as client:
        assert client.post("/api/auth/login", json={"username": "admin", "password": "ChangeMe123!"}).status_code == 200
        settings = client.get("/api/client-config").json()
        assert settings["pixel_streaming_encoder_min_quality"] == expected
        assert settings["pixel_streaming_fps"] == 90
        assert app.state.scenes.launch.encoder_min_quality == expected
        app.state.scenes.start(SceneInstanceCreate())
        assert len(commands) == 1
        command = commands[0]
        assert command[command.index("-EncoderMinQuality") + 1] == str(expected)
        assert command[command.index("-PreviewRate") + 1] == "90.0"


@pytest.mark.parametrize("quality", [-1, 101, True, "60", 60.5, None])
def test_platform_config_rejects_invalid_quality(tmp_path, quality):
    from space_arm_platform.app import PlatformConfig
    with pytest.raises(ValueError, match="runtime_encoder_min_quality"):
        PlatformConfig(project_root=tmp_path, data_root=tmp_path, runtime_encoder_min_quality=quality)


@pytest.mark.parametrize("quality", [None, "0", "75", "100", "-1", "101", "60.5", "true"])
def test_backend_cli_quality_validation_and_propagation(monkeypatch, quality):
    import sys
    from space_arm_platform import main as entry
    argv = ["space_arm_platform.main"]
    if quality is not None:
        argv += ["--runtime-encoder-min-quality", quality]
    monkeypatch.setattr(sys, "argv", argv)
    monkeypatch.setenv("SPACE_SIM_ALLOWED_ORIGINS", "[]")
    captured = []
    monkeypatch.setattr(entry, "create_app", lambda config: captured.append(config))
    monkeypatch.setattr(entry.uvicorn, "run", lambda *args, **kwargs: None)
    if quality in ("-1", "101", "60.5", "true"):
        with pytest.raises(SystemExit) as exc:
            entry.main()
        assert exc.value.code == 2
        assert not captured
    else:
        entry.main()
        assert captured[0].runtime_encoder_min_quality == (60 if quality is None else int(quality))
