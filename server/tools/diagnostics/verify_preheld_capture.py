"""Independently check recorded dual-arm holding poses and interface contacts."""
import argparse
import gzip
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from preview_scene_model import load_geometry_model
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.components.preheld_capture import validate_preheld
from simulation.scene_library.manifest import digest, write_json
from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix as rotation


def verify(model_path, scene_path, native, output):
    scene = json.loads(scene_path.read_text(encoding='utf-8-sig'))
    summary = json.loads((native / 'run/summary.json').read_text(encoding='utf-8-sig'))
    launch = json.loads((native / 'launch.json').read_text(encoding='utf-8-sig'))
    scene_hash = digest(scene_path)
    if launch.get('source_sha256', {}).get(str(scene_path.resolve())) != scene_hash:
        raise ValueError('Scene does not match the native launch')
    if digest(model_path) != summary['model_sha256']:
        raise ValueError('Model does not match the native trace')
    reference = validate_preheld(model_path, scene['randomization']['preheld_capture_joint_position'])
    root = ET.parse(model_path).getroot()
    rel = np.fromstring(root.find('./equality/weld[@name="preheld_capture_lock"]').get('relpose'), sep=' ')
    expected_r = rotation(rel[3:])
    model = load_geometry_model(model_path)
    data = mj.MjData(model)
    addresses = [model.joint(joint).qposadr[0] for _, joint, _ in INITIAL_JOINTS]
    max_position = max_angle = max_joint = max_drift = 0.
    baseline = None
    times = []
    trace = native / 'states-200hz.jsonl.gz'
    with gzip.open(trace, 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            q = np.asarray(row['qpos'])
            if q.shape != (model.nq,) or not np.isfinite(q).all():
                raise ValueError('Invalid native qpos')
            data.qpos[:] = q
            mj.mj_kinematics(model, data)
            grip, target, bus = [data.body(n) for n in ('gripper', 'capture_target', 'cubesat_bus')]
            rg, rt, rb = [b.xmat.reshape(3, 3) for b in (grip, target, bus)]
            delta = rg.T @ (target.xpos - grip.xpos) - rel[:3]
            angle = np.arccos(np.clip((np.trace(expected_r.T @ rg.T @ rt) - 1) / 2, -1., 1.))
            relative = rb.T @ (target.xpos - bus.xpos)
            if baseline is None:
                baseline = relative.copy()
            max_position = max(max_position, float(np.linalg.norm(delta)))
            max_angle = max(max_angle, float(angle))
            max_joint = max(max_joint, float(np.max(np.abs(q[addresses[:6]] - reference[:6]))))
            max_drift = max(max_drift, float(np.linalg.norm(relative - baseline)))
            times.append(int(row['sim_time_ns']))
    names = json.loads((native / 'names.json').read_text(encoding='utf-8-sig'))
    capture = {n.get('name') for n in root.find('.//body[@name="bottom_mount"]').iter('body')}
    target = {n.get('name') for n in root.find('./worldbody/body[@name="capture_target"]').iter('body')}
    service = {n.get('name') for n in root.find('./worldbody/body[@name="cubesat_bus"]').iter('body')}
    operating = {n.get('name') for n in root.find('.//body[@name="link1"]').iter('body')}
    service -= capture | operating
    body_names = {int(i): n for i, n in names['bodies'].items()}
    geom_names = {int(i): n for i, n in names['geoms'].items()}
    capture_ids = {i for i, n in body_names.items() if n in capture}
    target_ids = {i for i, n in body_names.items() if n in target}
    service_ids = {i for i, n in body_names.items() if n in service}
    forbidden = {}
    star_collisions = {}
    interface_contacts = {}
    contact_times = []
    with gzip.open(native / 'contacts-400hz.jsonl.gz', 'rt') as stream:
        for line in stream:
            row = json.loads(line)
            contact_times.append(row['interval_end_ns'])
            for ga, gb, ba, bb, distance in row['contacts']:
                if distance >= 0:
                    continue
                if (ba in target_ids and bb in service_ids) or (bb in target_ids and ba in service_ids):
                    key = '|'.join(sorted((geom_names[ga], geom_names[gb])))
                    star_collisions[key] = min(star_collisions.get(key, 0.), distance)
                if (ba in capture_ids) == (bb in capture_ids):
                    continue
                g1, g2 = geom_names[ga], geom_names[gb]
                allowed = ((g1.startswith('capture_x_piece_') and g2.startswith(('collision_ring_convex_', 'collision_jaw_')))
                           or (g2.startswith('capture_x_piece_') and g1.startswith(('collision_ring_convex_', 'collision_jaw_'))))
                key = '|'.join(sorted((g1, g2)))
                if allowed:
                    interface_contacts[key] = min(interface_contacts.get(key, 0.), distance)
                else:
                    forbidden[key] = min(forbidden.get(key, 0.), distance)
    from simulation.plug_retention import model_retention_sockets
    expected_owners = {(plug, s.name): s.owner_body for plug, seats in model_retention_sockets(model_path, owned=True).items() for s in seats}
    actual_owners = {(plug.split('@')[0], s['name']): s.get('owner_body')
        for plug, seats in summary.get('plug_retention', {}).get('sockets', {}).items() for s in seats}
    checks = dict(pose_coverage_200hz=bool(len(times) > 1 and times[0] == 0 and np.all(np.diff(times) == 5_000_000)),
        contact_coverage_400hz=bool(len(contact_times) > 1 and contact_times[0] == 0
                                  and np.all(np.diff(contact_times) == 2_500_000)
                                  and contact_times[-1] >= times[-1]),
        grasp_position_error_under_5mm=max_position <= .005, grasp_angle_error_under_0p02rad=max_angle <= .02,
        capture_joint_error_under_0p05rad=max_joint <= .05, target_relative_drift_under_5mm=max_drift <= .005,
        no_unintended_capture_external_contact=not forbidden,
        expected_interface_penetration_under_1mm=all(d >= -.001 for d in interface_contacts.values()),
        no_target_service_hardware_collision=not star_collisions,
        retention_reactions_bound_to_actual_owners=bool(expected_owners and actual_owners == expected_owners),
        capture_controller_installed=summary.get('preheld_capture', {}).get('mode') == 'ideal_locked_grasp')
    report = dict(schema='dual-arm-preheld-validation/1', model_sha256=digest(model_path),
        scene_sha256=scene_hash, validator_sha256=digest(__file__),
        trace_sha256=digest(trace), contact_trace_sha256=digest(native / 'contacts-400hz.jsonl.gz'),
        mode='ideal_locked_grasp', duration_s=times[-1] / 1e9 if times else 0,
        max_grasp_position_error_m=max_position, max_grasp_angle_error_rad=max_angle,
        max_capture_joint_error_rad=max_joint, max_target_relative_drift_m=max_drift,
        forbidden_contacts=forbidden, star_hardware_collisions=star_collisions,
        expected_interface_contacts=interface_contacts,
        maximum_expected_interface_penetration_m=-min(interface_contacts.values(), default=0.),
        interface_penetration_limit_m=.001, checks=checks, passed=all(checks.values()),
        full_operation_accepted=False, limitation='Holding interval only; not friction-only capture or full operation acceptance')
    write_json(output, report)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--scene', type=Path, required=True)
    p.add_argument('--native', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    print(json.dumps(verify(a.model, a.scene, a.native, a.output)))
