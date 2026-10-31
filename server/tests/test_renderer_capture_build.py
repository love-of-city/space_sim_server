"""The Windows launcher must refuse stale source, shader and unlinked builds."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = Path(os.environ.get('SPACE_SIM_RESET_ADAPTER', str(ROOT.parent / 'ue-adapter')))
SCRIPTS = ADAPTER / 'Unreal/BskUnrealRenderer/scripts'
PWSH = shutil.which('pwsh')
pytestmark = pytest.mark.skipif(not PWSH or not (SCRIPTS / 'runtime_build.ps1').exists(),
                               reason='matching UE adapter and PowerShell required')


@pytest.mark.parametrize('state', ['fresh', 'missing', 'stale_cpp', 'stale_header', 'stale_shader', 'compile_only'])
def test_capture_requires_a_linked_up_to_date_plugin(tmp_path, state):
    adapter = tmp_path / 'adapter'
    project = adapter / 'Unreal/BskUnrealRenderer'
    engine = tmp_path / 'engine'
    plugin = project / 'Plugins/BskUnrealRuntime'
    source = plugin / 'Source/BskUnrealRuntime/Public/BskLeRobotSampling.h'
    source.parent.mkdir(parents=True)
    source.write_text('// sampler')
    cpp = source.parent.parent / 'Private/BskSceneController.cpp'
    cpp.parent.mkdir(); cpp.write_text('// capture')
    shader = plugin / 'Shaders/Camera.usf'
    shader.parent.mkdir(); shader.write_text('// camera')
    (plugin / 'BskUnrealRuntime.uplugin').write_text('{}')
    (project / 'BskUnrealRenderer.uproject').write_text('{}')
    (engine / 'Engine/Build').mkdir(parents=True)
    (engine / 'Engine/Build/Build.version').write_text(json.dumps(
        dict(MajorVersion=5, MinorVersion=6, PatchVersion=1, Changelist=44394996)))
    for directory, modules in [(engine / 'Engine/Binaries/Win64', {}),
            (project / 'Binaries/Win64', {'BskUnrealRenderer': 'project.dll'}),
            (plugin / 'Binaries/Win64', {'BskUnrealRuntime': 'runtime.dll', 'BskCameraShaders': 'shader.dll'})]:
        directory.mkdir(parents=True)
        (directory / 'UnrealEditor.modules').write_text(json.dumps(dict(BuildId='fixture', Modules=modules)))
        for name in modules.values():
            (directory / name).write_bytes(b'linked fixture')
    identity_source = ADAPTER / 'Adapters/bsk_render_adapter/build_identity.py'
    spec = importlib.util.spec_from_file_location('test_build_identity', identity_source)
    identity = importlib.util.module_from_spec(spec); spec.loader.exec_module(identity)
    identity.write_build_manifest(project, engine, platform='Win64')
    for relative in ('scripts/write_ue_build_manifest.py', 'Adapters/bsk_render_adapter/build_identity.py'):
        destination = adapter / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ADAPTER / relative, destination)
    if state == 'missing':
        (plugin / 'Binaries/Win64/runtime.dll').unlink()
    if state.startswith('stale_') or state == 'compile_only':
        changed = {'stale_cpp': cpp, 'stale_shader': shader}.get(state, source)
        changed.write_text('// changed without linking')
        # Old mtimes must not hide changed content.
        os.utime(changed, (1_600_000_000, 1_600_000_000))
    if state == 'compile_only':
        obj = plugin / 'Intermediate/BskSceneController.obj'
        obj.parent.mkdir(); obj.write_bytes(b'new object, no link')
    wrapper = tmp_path / 'check.ps1'
    wrapper.write_text("param($Helper,$Project,$Engine,$Python)\n$ErrorActionPreference='Stop'\n"
                       "function Resolve-UnrealRoot { param($RequestedRoot) return $Engine }\n"
                       "function Resolve-BskPython { param($RequiredModules) return $Python }\n"
                       ". $Helper\nAssert-BskCaptureRuntimeBuild $Project $Engine\n'fresh'\n")
    result = subprocess.run([PWSH, '-NoProfile', '-File', str(wrapper), str(SCRIPTS / 'runtime_build.ps1'),
                             str(project), str(engine), sys.executable], capture_output=True, text=True,
                            encoding='utf-8', errors='replace', timeout=20)
    assert (result.returncode == 0) == (state == 'fresh'), result.stdout + result.stderr
    if state != 'fresh':
        assert 'full build' in result.stderr


def test_capture_launcher_checks_freshness_before_starting_a_process():
    source = (SCRIPTS / 'start_renderer.ps1').read_text(encoding='utf-8-sig')
    assert source.index('Assert-BskCaptureRuntimeBuild $ProjectRoot') < source.index('$process = Start-Process')
    assert '$normalizedCaptureProducts.Count -gt 0 -or $CaptureNetworkPort -gt 0 -or $CaptureDirectory' in source
