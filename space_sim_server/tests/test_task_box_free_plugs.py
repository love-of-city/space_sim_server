"""Acceptance contract for removable, unlatched local CAD plugs."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


def test_free_plugs_have_no_fake_latch_and_keep_upstream_robot():
    fixed = ET.parse(ROOT / 'model/SARM/platform/sarm_task_box_module.xml').getroot()
    moving = ET.parse(ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml').getroot()
    for name, number in [('guide_plug', 9), ('aviation_plug', 13)]:
        body = moving.find(f'./worldbody/body[@name="{name}"]')
        assert body is not None and body.find('freejoint') is not None
        assert float(body.find('inertial').get('mass')) > 0
        contact = body.find('flexcomp/contact')
        assert int(contact.get('contype')) == 4
        assert int(contact.get('conaffinity')) & 1
        assert moving.find(f'.//body[@name="cubesat_bus"]/flexcomp[@name="task_box_{number:03}_contact"]') is None
        assert not any(name in (e.get('body1'), e.get('body2')) for e in moving.findall('./equality/*'))
    for name in ('link1', 'finger1', 'finger2'):
        a = fixed.find(f'.//body[@name="{name}"]')
        b = moving.find(f'.//body[@name="{name}"]')
        assert a is not None and ET.tostring(a) == ET.tostring(b)
    assert ET.tostring(fixed.find('actuator')) == ET.tostring(moving.find('actuator'))


def test_free_plugs_receive_orbital_translation_and_velocity():
    from space_arm_platform.task_box import initialize_free_plugs

    class Body:
        def setPosition(self, value):
            self.position = list(value)

        def setVelocity(self, value):
            self.velocity = list(value)

    class Scene:
        bodies = {n: Body() for n in ('guide_plug', 'aviation_plug')}

        def getBodyNames(self):
            return list(self.bodies)

        def getBody(self, name):
            return self.bodies[name]

    scene = Scene()
    path = ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml'
    initialize_free_plugs(scene, path, [7000000, 0, 0], [0, 7500, 0])
    for body in scene.bodies.values():
        assert body.position[0] > 7000000
        assert body.velocity == [0, 7500, 0]


def test_recentered_collision_meshes_keep_cad_dimensions_and_faces():
    def mesh(path):
        lines = path.read_text(encoding='utf-8').splitlines()
        vertices = [list(map(float, s.split()[1:4])) for s in lines if s.startswith('v ')]
        extent = [max(v[i] for v in vertices)-min(v[i] for v in vertices) for i in range(3)]
        return extent, sum(s.startswith('f ') for s in lines)

    metadata = json.loads((ROOT/'model/SARM/platform/sarm_task_box_plugs.manifest.json').read_text())
    for item, number in zip(metadata['plugs'], (9, 13), strict=True):
        original = mesh(ROOT/f'model/task_box/module/task_box_{number:03}_contact.obj')
        moved = mesh(ROOT/f'model/task_box/free_plugs/{item["name"]}.obj')
        assert original[1] == moved[1]
        assert all(abs(a-b) < 1e-7 for a, b in zip(original[0], moved[0], strict=True))
