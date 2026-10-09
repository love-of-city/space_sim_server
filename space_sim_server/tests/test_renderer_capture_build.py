"""Prevent the exact stale-DLL / -NoLink capture failure before launching UE."""
import os
from pathlib import Path
import shutil
import subprocess
import pytest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = Path(os.environ.get("SPACE_SIM_RESET_ADAPTER", str(ROOT.parents[1] / "space_sim_UE_adapter/space_sim_UE_Adapter")))
SCRIPTS = ADAPTER / "Unreal/BskUnrealRenderer/scripts"
PWSH = shutil.which("pwsh")
pytestmark = pytest.mark.skipif(not PWSH or not (SCRIPTS / "runtime_build.ps1").exists(), reason="matching UE adapter and PowerShell required")


@pytest.mark.parametrize("state", ["fresh", "missing", "stale_cpp", "stale_header", "compile_only"])
def test_capture_requires_a_linked_up_to_date_plugin(tmp_path, state):
    plugin = tmp_path / "Plugins/BskUnrealRuntime"
    source = plugin / "Source/BskUnrealRuntime/Public/BskLeRobotSampling.h"
    source.parent.mkdir(parents=True)
    source.write_text("// sampler")
    cpp = source.parent.parent / "Private/BskSceneController.cpp"
    cpp.parent.mkdir(); cpp.write_text("// capture")
    stamp = 1_700_000_000
    for p in (source, cpp): os.utime(p, (stamp, stamp))
    dll = plugin / "Binaries/Win64/UnrealEditor-BskUnrealRuntime.dll"
    dll.parent.mkdir(parents=True)
    if state != "missing":
        dll.write_bytes(b"fixture DLL")
        os.utime(dll, (stamp + 10, stamp + 10))
    if state in ("stale_cpp", "stale_header", "compile_only"):
        changed = cpp if state == "stale_cpp" else source
        os.utime(changed, (stamp + 20, stamp + 20))
    if state == "compile_only":
        obj = plugin / "Intermediate/BskSceneController.obj"
        obj.parent.mkdir(); obj.write_bytes(b"new compile, no link")
        os.utime(obj, (stamp + 30, stamp + 30))
    wrapper = tmp_path / "check.ps1"
    wrapper.write_text("param($Helper,$Project)\n$ErrorActionPreference='Stop'\n. $Helper\nAssert-BskCaptureRuntimeBuild $Project\n'fresh'\n")
    result = subprocess.run([PWSH, "-NoProfile", "-File", str(wrapper), str(SCRIPTS / "runtime_build.ps1"), str(tmp_path)], capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=20)
    if state == "fresh":
        assert result.returncode == 0, result.stdout + result.stderr
    else:
        assert result.returncode != 0
        assert "without -NoLink" in result.stderr


def test_capture_launcher_checks_freshness_before_starting_a_process():
    source = (SCRIPTS / "start_renderer.ps1").read_text(encoding="utf-8-sig")
    assert source.index("Assert-BskCaptureRuntimeBuild $ProjectRoot") < source.index("$process = Start-Process")
    assert "$normalizedCaptureProducts.Count -gt 0 -or $CaptureNetworkPort -gt 0 -or $CaptureDirectory" in source
