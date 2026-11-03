"""Layout preparation must use the available platform's runtime and editor."""
import importlib
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock


def test_geometry_probe_can_load_windows_plugin_without_linux_shared_object(tmp_path,monkeypatch):
    runtime=SimpleNamespace(__file__=str(tmp_path/'mujoco/__init__.py'),mj_version=lambda:370,
                            mj_loadPluginLibrary=Mock())
    monkeypatch.setitem(__import__('sys').modules,'mujoco',runtime)
    spec=importlib.util.spec_from_file_location('layout_probe_under_test',
        Path(__file__).resolve().parents[1]/'tools/diagnostics/check_independent_task_box_layout.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    (tmp_path/'run/native_acceleration').mkdir(parents=True)
    (tmp_path/'run/native_acceleration/sarm_thread.json').write_text(json.dumps(dict(runtime_version=370,library='plugin.dll')))
    module.load_geometry_plugin(runtime,tmp_path)
    runtime.mj_loadPluginLibrary.assert_called_once_with(str(tmp_path/'plugin.dll'))


def test_layout_import_uses_windows_editor_and_unattended_flags(tmp_path,monkeypatch):
    monkeypatch.setattr('space_arm_platform.unreal_runtime.sys.platform','win32')
    from tools.assets.prepare_independent_layout_catalog import unreal_command
    editor=tmp_path/'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
    editor.parent.mkdir(parents=True);editor.touch();editor.chmod(0o755)
    project=tmp_path/'project'
    command=unreal_command(tmp_path,project)
    assert command[:2]==[str(editor.resolve()),str(project/'BskUnrealRenderer.uproject')]
    assert '-unattended' in command and '-NullRHI' in command
