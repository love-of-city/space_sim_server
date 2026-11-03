"""Compile and check a randomized two-box initial state; no task acceptance."""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'tools')]
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.scene_library.hull_closure import audit_hull_closure
from simulation.scene_library.manifest import digest, write_json


def audit(case):
    path = case/'preheld-dual.xml'
    root = ET.parse(path).getroot()
    initial = json.loads((case/'initial-scene.json').read_text(encoding='utf-8'))['randomization']
    print('Compiling randomized dual model', flush=True)
    model = load_geometry_model(path)
    print('Compiled; checking initial state', flush=True)
    data = mj.MjData(model)
    for name, value in zip([*(f'joint{i}' for i in range(1, 7)), 'ee_link11_joint', 'ee_link12_joint'], initial['arm_joint_position_rad']):
        data.qpos[model.joint(name).qposadr[0]] = value
    for (_, name, _), value in zip(INITIAL_JOINTS, initial['preheld_capture_joint_position']):
        data.qpos[model.joint(name).qposadr[0]] = value
    address = model.joint('capture_target_free').qposadr[0]
    data.qpos[address:address+3] = initial['target_position_m']
    data.qpos[address+3:address+7] = initial['target_orientation_wxyz']
    mj.mj_fwdPosition(model, data)
    print('Initial forward pass complete', flush=True)
    def descendants(names):
        return {b.get('name') for name in names for b in root.find(f'.//body[@name="{name}"]').iter('body')}
    target = descendants(('task_box_base', 'drawer_base', 'aviation_plug', 'guide_plug', 'cylindrical_plug', 'bolt'))
    from simulation.scene_library.taskbox_mount import LOOSE_BODIES
    complete = all(root.find(f'.//body[@name="{prefix+name}"]') is not None
                   for prefix in ('', 'service_') for name in LOOSE_BODIES)
    service_names = ('service_task_box_base', 'service_drawer_base') + tuple(
        'service_'+name for name in LOOSE_BODIES if root.find(f'.//body[@name="service_{name}"]') is not None)
    service = descendants(service_names)
    arm = descendants(('link1', 'bottom_mount'))
    body = [model.body(int(model.geom_bodyid[g])).name for g in range(model.ngeom)]
    cross = []
    for contact in data.contact:
        a, b = body[contact.geom1], body[contact.geom2]
        forbidden = ((a in target|service and b in arm) or (b in target|service and a in arm)
                     or (a in target and b in service) or (b in target and a in service))
        if forbidden and contact.dist < -1e-6:
            cross.append(dict(geoms=[model.geom(contact.geom1).name, model.geom(contact.geom2).name], distance_m=float(contact.dist)))
    # Explicit distance tests also cover pairs suppressed by owner exclusions.
    active = [g for g in range(model.ngeom) if model.geom_contype[g] or model.geom_conaffinity[g]]
    rotation = data.geom_xmat.reshape(-1, 3, 3)
    centers = data.geom_xpos + np.einsum('nij,nj->ni', rotation, model.geom_aabb[:, :3])
    extents = np.einsum('nij,nj->ni', abs(rotation), model.geom_aabb[:, 3:])
    low, high = centers-extents, centers+extents
    panels = [g for g in active if model.geom(g).name.startswith('hull_closure_')]
    penetration, unsupported = [], []
    for a in panels:
        for b in active:
            if body[b] == 'capture_target' or np.any(high[a]+.005 < low[b]) or np.any(high[b]+.005 < low[a]):
                continue
            names = [model.geom(a).name, model.geom(b).name]
            if model.geom_type[b] == mj.mjtGeom.mjGEOM_SDF:
                unsupported.append(names)
                continue
            distance = float(mj.mj_geomDistance(model, data, a, b, .005, None))
            if distance < -1e-6:
                penetration.append(dict(geoms=names, distance_m=distance))
    closure = audit_hull_closure(root, json.loads((case/'hull-closure.json').read_text(encoding='utf-8')))
    checks = dict(compiled=True, complete_parts_on_both_spacecraft=complete,
                  finite_state=bool(np.isfinite(data.qpos).all()),
                  closed_hull=closure['passed'], no_panel_penetrations=not penetration,
                  no_unsupported_panel_pairs=not unsupported, no_arm_or_cross_box_penetrations=not cross)
    report = dict(schema='dual-taskbox-initial-geometry/1', model_sha256=digest(path),
                  scene_sha256=digest(case/'initial-scene.json'), checks=checks,
                  passed=all(checks.values()), closure_penetrations=penetration,
                  unsupported=unsupported, cross_contacts=cross, full_task_accepted=False,
                  dataset_capture_allowed=False)
    write_json(case/'initial-geometry.json', report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', type=Path, required=True)
    result = audit(parser.parse_args().case)
    print(json.dumps(result))
    raise SystemExit(0 if result['passed'] else 1)
