"""Contact load bench using unchanged upstream wrist/finger meshes and motors.

Wrist is bolted to a test stand; this is NOT a full-arm reach/IK/platform test.
No grasp welds, pose playback, or artificial plug position constraints.
"""
import copy
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml'


def build(name):
    root = ET.parse(SCENE).getroot()
    wrist = copy.deepcopy(root.find('.//body[@name="link6"]'))
    wrist.set('pos', '0 0 0')
    wrist.set('quat', '1 0 0 0')
    wrist.remove(wrist.find('joint'))
    plug = copy.deepcopy(root.find(f'./worldbody/body[@name="{name}"]'))
    metadata = json.loads(SCENE.with_suffix('.manifest.json').read_text())['plugs']
    item = next(v for v in metadata if v['name'] == name)
    origin = np.array([.28, .275, 0]) if name == 'guide_plug' else np.array([.37, .22, 0])
    center = np.asarray(item['bus_center_m'])
    pos = np.array([0., 0., .255]) + np.diag([1, -1, -1]) @ (center - origin)
    plug.set('pos', ' '.join(map(str, pos)))
    plug.set('quat', '0 1 0 0')
    world = root.find('worldbody')
    world.clear()
    world.extend([wrist, plug])
    actuator = root.find('actuator')
    for child in list(actuator):
        if child.get('joint') not in ('joint_finger1', 'joint_finger2'):
            actuator.remove(child)
    for tag in ('keyframe', 'contact', 'sensor', 'equality'):
        e = root.find(tag)
        if e is not None:
            root.remove(e)
    # Fixing the wrist to world changes MuJoCo's automatic parent filtering.
    # Restore the upstream moving-wrist parent/child exclusions on this stand.
    pairs = ET.SubElement(root, 'contact')
    for finger in ('finger1', 'finger2'):
        ET.SubElement(pairs, 'exclude', body1='link6', body2=finger)
    used = {g.get('mesh') for g in root.findall('.//geom')}
    asset = root.find('asset')
    for e in list(asset):
        if e.tag == 'mesh' and e.get('name') not in used:
            asset.remove(e)
    for e in list(asset.findall('mesh')) + root.findall('.//flexcomp'):
        if e.get('file'):
            e.set('file', str((SCENE.parent/e.get('file')).resolve()))
    return mj.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))


def main():
    report = dict(scope=__doc__, cases={})
    for name in ('guide_plug', 'aviation_plug'):
        model = build(name)
        data = mj.MjData(model)
        for finger in ('joint_finger1', 'joint_finger2'):
            data.qpos[model.jnt_qposadr[model.joint(finger).id]] = .0375
        mj.mj_forward(model, data)
        body = model.body(name).id
        initial = data.xpos[body].copy()
        finger_bodies = [model.body(f).id for f in ('finger1', 'finger2')]
        rows = []
        physics_wall = 0.
        start = time.perf_counter()
        for step in range(1500):
            data.ctrl[:] = -.1  # 0.1 N per finger, inside unchanged upstream 0.5 N limit.
            # Third second: finite tensile disturbance, not a prescribed plug path.
            data.xfrc_applied[body, 2] = .05 if step >= 1000 else 0
            step_start = time.perf_counter()
            mj.mj_step(model, data)
            physics_wall += time.perf_counter()-step_start
            fingers, peak = set(), 0.
            for i, c in enumerate(data.contact):
                if not any(k >= 0 for k in c.flex):
                    continue
                for g in c.geom:
                    if g >= 0 and int(model.geom_bodyid[g]) in finger_bodies:
                        f = np.zeros(6)
                        mj.mj_contactForce(model, data, i, f)
                        if f[0] > 1e-6:
                            fingers.add(int(model.geom_bodyid[g]))
                            peak = max(peak, float(f[0]))
            rows.append((len(fingers), data.ncon, peak))
        wall = time.perf_counter() - start
        loaded = rows[500:]
        result = dict(simulated_s=float(data.time), wall_s=wall, rtf=float(data.time/wall),
            physics_step_wall_s=physics_wall, physics_only_rtf=float(data.time/physics_wall),
            diagnostics_and_loop_wall_s=wall-physics_wall,
            both_fingers_contact_fraction=sum(r[0] == 2 for r in loaded)/len(loaded),
            peak_contact_count=max(r[1] for r in rows), peak_normal_force_N=max(r[2] for r in rows),
            displacement_m=float(np.linalg.norm(data.xpos[body]-initial)),
            warnings=data.warning.number.tolist(), finite=bool(np.isfinite(data.qpos).all()),
            finger_command_N=-.1, third_second_axial_load_N=.05)
        report['cases'][name] = result
        print(name, result, flush=True)
    path = ROOT/'reports/task-box-grip-load.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    for result in report['cases'].values():
        assert result['finite'] and not any(result['warnings'])
        assert result['both_fingers_contact_fraction'] >= .95, 'Lost bilateral grasp'
        assert result['displacement_m'] < .005, 'Plug slipped more than 5 mm'


if __name__ == '__main__':
    main()
