"""Task-box ownership and integration contracts (no physics DLL in this process)."""
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'model/SARM/platform/sarm_task_box_module.xml'


def test_owned_components_and_upstream_robot_are_preserved():
    old = ET.parse(SCENE.with_name('sarm_ground_target_self_collision.xml')).getroot()
    new = ET.parse(SCENE).getroot()
    for selector in ('.//body[@name="link1"]', './/body[@name="capture_target"]',
                     './actuator', './contact', './custom'):
        before = [line.rstrip() for line in ET.tostring(old.find(selector)).strip().splitlines()]
        after = [line.rstrip() for line in ET.tostring(new.find(selector)).strip().splitlines()]
        assert before == after
    config = json.loads((ROOT / 'model/task_box/module/assembly.json').read_text(encoding='utf-8'))
    assert {p['id'] for p in config['parts']} == {1, 8, 9, 10, 11, 12, 13, 14, 15}
    assert len(new.findall('.//flexcomp')) == 10
    for contact in new.findall('.//flexcomp/contact'):
        assert contact.get('contype') == '0'
        assert contact.get('conaffinity') == '3'
    flag = new.find('./option/flag')
    assert flag.get('contact') == 'enable'
    assert flag.get('constraint') == 'enable'
    assert flag.get('override') == 'disable'


def test_module_is_opt_in_and_dependencies_are_checked():
    from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, capture_target
    import sys
    sys.path.insert(0, str(ROOT / 'tools'))
    from select_sarm_scene import selected_scene
    assert DEFAULT_TEMPLATE != 'sarm-task-box-contacts'
    assert capture_target('sarm-task-box-contacts').resolve_model(SCENE.parent) == SCENE
    assert selected_scene(SCENE.parent, 'sarm-task-box-contacts', check=True) == SCENE


def test_missing_manifest_is_not_silently_accepted(monkeypatch):
    import sys
    sys.path.insert(0, str(ROOT / 'tools'))
    from select_sarm_scene import selected_scene
    read = Path.read_text
    is_file = Path.is_file
    def missing(path, *args, **kwargs):
        if path == SCENE.with_suffix('.manifest.json'):
            raise FileNotFoundError(path)
        return read(path, *args, **kwargs)
    monkeypatch.setattr(Path, 'read_text', missing)
    monkeypatch.setattr(Path, 'is_file', lambda p: False if p == SCENE.with_suffix('.manifest.json') else is_file(p))
    with pytest.raises(FileNotFoundError):
        selected_scene(SCENE.parent, 'sarm-task-box-contacts', check=True)


def test_authoritative_upstream_changes_require_rebuild(monkeypatch):
    import sys
    sys.path.insert(0, str(ROOT / 'tools'))
    import select_sarm_scene as selector
    original = selector.fingerprint
    authoritative = ROOT / 'model/SARM/platform/sarm_platform.xml'
    monkeypatch.setattr(selector, 'fingerprint', lambda p: '0' * 64 if p == authoritative else original(p))
    with pytest.raises(ValueError, match='Stale/modified'):
        selector.selected_scene(SCENE.parent, 'sarm-task-box-contacts', check=True)
    with pytest.raises(ValueError, match='Stale/modified'):
        selector.verified_source_tree(ROOT, SCENE.with_name('sarm_ground_target_self_collision.xml'))
