"""Frozen visual assets must match this checkout and their imported mesh bytes."""
import hashlib
import json
from pathlib import Path

import pytest

from space_arm_platform.benchmark_vision_resources import (
    new_renderer_user_directory, physical_graphics_identity, receipt_dependencies,
    selected_graphics_identity, validate_catalog,
)


@pytest.fixture
def assets(tmp_path):
    server,project,engine=(tmp_path/name for name in ('server','project','engine'))
    for path in (server,project/'Content/BSK',engine):
        path.mkdir(parents=True)
    mesh=server/'mesh.obj'
    mesh.write_bytes(b'original authored mesh')
    imported=project/'Content/BSK/mesh.uasset'
    imported.write_bytes(b'imported UE mesh')
    catalog=project/'catalog.json'
    catalog.write_text(json.dumps(dict(schema='bsk-render-asset-catalog/1',assets={str(mesh):dict(
        source_sha256=hashlib.sha256(mesh.read_bytes()).hexdigest(),asset_path='/Game/BSK/mesh.mesh')})))
    return server,project,engine,catalog,mesh,imported


@pytest.mark.parametrize('defect',['changed_source','missing_asset','foreign_checkout','escaping_asset'])
def test_imported_mesh_identity_is_not_just_a_valid_catalogue_filename(assets,defect):
    server,project,_,catalog,mesh,imported=assets
    assert validate_catalog(server,project,catalog)==[mesh]
    data=json.loads(catalog.read_text())
    if defect=='changed_source':
        mesh.write_bytes(b'another geometry')
    elif defect=='missing_asset':
        imported.unlink()
    elif defect=='foreign_checkout':
        data['assets'][str(project/'foreign.obj')]=data['assets'].pop(str(mesh))
    else:
        data['assets'][str(mesh)]['asset_path']='/Game/../outside.mesh'
    catalog.write_text(json.dumps(data))
    with pytest.raises((ValueError,FileNotFoundError)):
        validate_catalog(server,project,catalog)


@pytest.mark.parametrize('path',['$(UnknownDir)/module.dll','$(ProjectDir)/../outside.dll','$(EngineDir)/missing.dll'])
def test_receipt_cannot_silently_drop_unresolved_missing_or_external_runtime_inputs(assets,path):
    _,project,engine,*_=assets
    receipt=project/'target.json'
    receipt.write_text(json.dumps(dict(RuntimeDependencies=[dict(Path=path,Type='NonUFS')],BuildProducts=[])))
    with pytest.raises((ValueError,FileNotFoundError)):
        list(receipt_dependencies(receipt,project,engine))


def test_receipt_includes_actual_runtime_binaries_but_does_not_require_debug_symbols(assets):
    _,project,engine,*_=assets
    library=engine/'renderer.dll'
    library.write_bytes(b'loaded renderer')
    receipt=project/'target.json'
    receipt.write_text(json.dumps(dict(RuntimeDependencies=[dict(Path='$(EngineDir)/renderer.dll',Type='NonUFS')],
        BuildProducts=[dict(Path='$(EngineDir)/absent.pdb',Type='SymbolFile')])))
    assert list(receipt_dependencies(receipt,project,engine))==[library]


def test_image_method_registry_import_does_not_load_cv2_or_basilisk():
    import subprocess
    import sys
    root=Path(__file__).resolve().parents[1]
    script="""
import sys
from space_arm_platform.benchmark_contract import load_task, SPEC_DIRECTORY
from space_arm_platform.benchmark_registry import method_classes
for name in ('d23-marked-mono-pose-v1','d24-marked-stereo-pose-v1','d27-stereo-gyro-spin-v1','d37-stereo-body-approach-v1'):
    task=load_task(SPEC_DIRECTORY/(name+'.json'))
    assert len(method_classes(task))==(4 if task.source_id=='D37' else 3)
assert 'cv2' not in sys.modules and 'Basilisk' not in sys.modules
"""
    result=subprocess.run([sys.executable,'-c',script],cwd=root,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr


GPU=dict(Name='NVIDIA GeForce RTX 4090',DriverVersion='32.0.15.9174',
         PNPDeviceID='PCI\\VEN_10DE&DEV_2684\\3&13C0B0C5&0&18',AdapterCompatibility='NVIDIA')
RDP=dict(Name='Microsoft Remote Display Adapter',DriverVersion='10.0.20348.3451',
         PNPDeviceID='SWD\\REMOTEDISPLAYENUM\\RDPIDD_INDIRECTDISPLAY')
RHI_LOG='''[2026.11.06-14.08.03:922][  0]LogRHI: RHI Adapter Info:
[2026.11.06-14.08.03:922][  0]LogRHI:             Name: NVIDIA GeForce RTX 4090
[2026.11.06-14.08.03:922][  0]LogRHI:   Driver Version: 591.74 (internal:32.0.15.9174, unified:591.74)
[2026.11.06-14.08.03:922][  0]LogRHI:      Driver Date: 12-30-2025
'''


def test_remote_desktop_disconnect_does_not_change_gpu_identity_but_a_driver_change_does():
    frozen=physical_graphics_identity([GPU,RDP])
    assert frozen==physical_graphics_identity(GPU)
    assert frozen!=physical_graphics_identity(dict(GPU,DriverVersion='32.0.15.9999'))
    assert frozen!=physical_graphics_identity(dict(GPU,PNPDeviceID='PCI\\another_device'))
    assert selected_graphics_identity(RHI_LOG,frozen)==dict(Name=GPU['Name'],DriverVersion=GPU['DriverVersion'])


@pytest.mark.parametrize('log',[RHI_LOG.replace('NVIDIA GeForce RTX 4090','Microsoft Basic Render Driver'),
    RHI_LOG.replace('32.0.15.9174','32.0.15.9999'),RHI_LOG.replace('internal:','missing:'),''])
def test_actual_renderer_must_match_physical_model_and_driver(log):
    with pytest.raises(ValueError):
        selected_graphics_identity(log,physical_graphics_identity([GPU,RDP]))


@pytest.mark.parametrize('devices',[None,[],[RDP],[dict(GPU,DriverVersion=None)]])
def test_software_display_or_incomplete_hardware_identity_cannot_be_frozen(devices):
    with pytest.raises(ValueError):
        physical_graphics_identity(devices)


def test_deep_evidence_paths_get_distinct_short_user_dirs_without_inherited_settings(tmp_path_factory,monkeypatch):
    root=tmp_path_factory.mktemp('ue')
    monkeypatch.setenv('SPACESIM_BENCHMARK_RUNTIME_ROOT',str(root))
    episode=Path('batch')/('long-output-'*20)/'episodes/123'
    first=new_renderer_user_directory(episode)
    (first/'Saved').mkdir()
    (first/'Saved/GameUserSettings.ini').write_text('old session setting')
    second=new_renderer_user_directory(episode)
    assert first!=second and len(str(second))<=120
    assert not (second/'Saved').exists()
    assert json.loads((second/'owner.json').read_text('utf-8'))['episode_directory']==str(episode)
    assert (first/'Saved/GameUserSettings.ini').read_text()=='old session setting'
    monkeypatch.setenv('SPACESIM_BENCHMARK_RUNTIME_ROOT',str(root/('too-deep-'*12)))
    with pytest.raises(ValueError,match='short SPACESIM_BENCHMARK_RUNTIME_ROOT'):
        new_renderer_user_directory(episode)
