"""The stage journal must advance through multiple successful subprocesses."""
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

from tools import prepare_dual_taskbox_pool as pipeline
from tools.build_dual_taskbox_layout import unique_asset_filenames


def test_vfs_distinguishes_same_basename_and_preserves_shared_file(tmp_path):
    root = ET.fromstring('<mujoco><asset/></mujoco>')
    sources = []
    for role in ('target', 'service'):
        folder = tmp_path/role
        folder.mkdir()
        path = folder/'base.obj'
        path.write_text(role)
        sources.append(path)
        ET.SubElement(root.find('asset'), 'mesh', name=role, file=str(path))
    ET.SubElement(root.find('asset'), 'mesh', name='shared', file=str(sources[0]))
    aliases = unique_asset_filenames(root, tmp_path)
    nodes = root.findall('./asset/mesh')
    assert nodes[0].get('file') == nodes[2].get('file')
    assert Path(nodes[0].get('file')).name != Path(nodes[1].get('file')).name
    assert len(aliases) == 2
    for entry in aliases:
        assert Path(entry['path']).read_bytes() == Path(entry['source']).read_bytes()


def test_existing_donors_advance_and_register_after_both_checks(tmp_path, monkeypatch):
    monkeypatch.setattr(pipeline, 'ManagerLease', lambda path: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(pipeline, 'compose', lambda base, target, service, case: case.mkdir())
    calls = []
    monkeypatch.setattr(pipeline.subprocess, 'run', lambda args, **kwargs: calls.append(args))
    def register(index, cases, donors, output):
        journal = json.loads((tmp_path/'out/pipeline.json').read_text())
        assert journal['stages'] == {'combined-0': 'passed', 'native-load-0': 'passed', 'combined-1': 'passed', 'native-load-1': 'passed'}
        assert len(cases) == 2 and len(calls) == 4
        return {'variants': 2, 'tasks': 8}
    monkeypatch.setattr(pipeline, 'register', register)
    args = SimpleNamespace(count=2, seed=10, output=tmp_path/'out',
        existing_donor=[tmp_path/'a', tmp_path/'b'], adapter=tmp_path/'adapter',
        base=tmp_path/'base', index=tmp_path/'index.json', base_catalog=Path('unused'),
        unreal=Path('unused'), platform_python=Path('unused'))
    assert pipeline.prepare(args)['tasks'] == 8
    result = json.loads((args.output/'pipeline.json').read_text())
    assert result['completed'] is True
    assert result['full_task_accepted'] is False
