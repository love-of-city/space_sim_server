"""Explicit initial assembly distances for an independently held target.

Run in the separate geometry interpreter; this does not execute dynamics.
Intentional capture-interface and mounting-support contacts are not a strength
test. Full native contact traces are checked by verify_preheld_capture.py.
"""
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.components.preheld_capture import validate_preheld
from simulation.scene_library.manifest import digest, write_json


def audit(model_path, scene_path, output):
    scene = json.loads(Path(scene_path).read_text(encoding='utf-8-sig'))
    initial = scene['randomization']
    references = validate_preheld(model_path, initial['preheld_capture_joint_position'])
    root = ET.parse(model_path).getroot()
    model = load_geometry_model(model_path)
    data = mj.MjData(model)
    for (_, name, _), value in zip(INITIAL_JOINTS, references, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    for name, value in zip([*(f'joint{i}' for i in range(1, 7)), 'ee_link11_joint', 'ee_link12_joint'],
                           initial['arm_joint_position_rad'], strict=True):
        data.qpos[model.joint(name).qposadr[0]] = value
    target = model.body('capture_target')
    address = model.jnt_qposadr[target.jntadr[0]]
    data.qpos[address:address+3] = initial['target_position_m']
    data.qpos[address+3:address+7] = initial['target_orientation_wxyz']
    mj.mj_forward(model, data)

    def descendants(name):
        node = root.find(f'.//body[@name="{name}"]')
        return set() if node is None else {n.get('name') for n in node.iter('body')}

    hardware = descendants('task_box_base') | descendants('drawer_base')
    capture = descendants('bottom_mount')
    operating = descendants('link1')
    target_bodies = descendants('capture_target')
    service = descendants('cubesat_bus') - capture - operating
    groups = {key: [] for key in ('target_hull', 'hardware', 'target_all', 'service')}
    for gid in range(model.ngeom):
        if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        body = model.body(int(model.geom_bodyid[gid])).name
        if body == 'capture_target':
            groups['target_hull'].append(gid)
        if body in hardware:
            groups['hardware'].append(gid)
        if body in target_bodies:
            groups['target_all'].append(gid)
        if body in service:
            groups['service'].append(gid)
    checks, unsupported = [], []
    for left, right in [('target_hull', 'hardware'), ('target_all', 'service')]:
        tested, hits = 0, []
        for a in groups[left]:
            for b in groups[right]:
                if any(int(model.geom_type[g]) == int(mj.mjtGeom.mjGEOM_SDF) for g in (a, b)):
                    unsupported.append([model.geom(a).name, model.geom(b).name])
                    continue
                distance = float(mj.mj_geomDistance(model, data, a, b, .005, None))
                tested += 1
                if distance < 0:
                    hits.append(dict(first=model.geom(a).name, second=model.geom(b).name,
                                     separation_m=distance))
        checks.append(dict(groups=[left, right], tested_pairs=tested, penetrations=hits))
    complete = all(groups.values()) and not unsupported
    report = dict(schema='space-arm-preheld-inspection/1', model_sha256=digest(model_path),
        scene_sha256=digest(scene_path), validator_sha256=digest(__file__),
        geometry_counts={k: len(v) for k, v in groups.items()}, checks=checks,
        coverage_complete=bool(complete), unsupported_pairs=unsupported,
        passed=bool(complete and not any(c['penetrations'] for c in checks)),
        scope='initial_explicit_pair_distances_only', task_accepted=False,
        limitation='No support strength or friction-only capture acceptance; requires full native holding audit.')
    write_json(output, report)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = audit(args.model, args.scene, args.output)
    print(json.dumps(report))
    raise SystemExit(0 if report['passed'] else 1)
