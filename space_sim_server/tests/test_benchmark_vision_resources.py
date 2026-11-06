"""Frozen visual assets must match this checkout and their imported mesh bytes."""
import hashlib
import json
from pathlib import Path

import pytest

from space_arm_platform.benchmark_vision_resources import receipt_dependencies, validate_catalog


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
for name in ('d23-marked-mono-pose-v1','d24-marked-stereo-pose-v1'):
    task=load_task(SPEC_DIRECTORY/(name+'.json'))
    assert len(method_classes(task))==3
assert 'cv2' not in sys.modules and 'Basilisk' not in sys.modules
"""
    result=subprocess.run([sys.executable,'-c',script],cwd=root,capture_output=True,text=True,timeout=30)
    assert result.returncode==0,result.stderr
