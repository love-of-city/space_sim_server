"""The two published scenes must remain byte-identical and self-contained."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'model/dual_stowed_service'


@pytest.mark.parametrize('version',[1,2])
def test_frozen_model_hash_and_mesh_dependencies(version):
    versions=json.loads((ASSETS/'camera_versions.json').read_text())['versions']
    record=versions[str(version)]
    model=ASSETS/record['model']
    design=json.loads((model.parent/'design.json').read_text())
    assert hashlib.sha256(model.read_bytes()).hexdigest()==record['model_sha256']==design['model_sha256']
    root=ET.parse(model).getroot()
    meshes=root.findall('./asset/mesh[@file]')
    assert meshes
    for mesh in meshes:
        path=(model.parent/mesh.get('file')).resolve()
        assert path.is_relative_to(ROOT/'model')
        assert path.is_file(),path
        assert not path.read_bytes().startswith(b'version https://git-lfs.github.com/spec/'),path
    cameras=root.findall('.//camera')
    assert {camera.get('name') for camera in cameras}=={'spacecraft_overview','left_sarm_wrist_cam','right_sarm_wrist_cam'}
    assert (ASSETS/record['preview_image']).is_file()
