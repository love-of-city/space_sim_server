"""Separate Python-MuJoCo process: static collision screen of an arm candidate.

Never import Basilisk here. This does not simulate deployment, torque, braking
or margins; it cannot certify a flight arm trajectory. No scene file is edited.
"""
import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scene-model',type=Path,required=True)
    p.add_argument('--plugin',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args = p.parse_args()
    if mujoco.mj_version() != 3007000:
        raise RuntimeError('this offline screen requires the authored model MuJoCo 3.7.0 ABI')
    mujoco.mj_loadPluginLibrary(str(args.plugin.resolve()))
    model = mujoco.MjModel.from_xml_path(str(args.scene_model.resolve()))
    data = mujoco.MjData(model)
    # Distant target for a service self-collision screen. Only this offline
    # model's initial configuration changes; no running simulation is touched.
    target = model.body('capture_target').id
    free = int(model.body_jntadr[target])
    data.qpos[int(model.jnt_qposadr[free]):int(model.jnt_qposadr[free])+3] = [1000.,0.,0.]
    capture_root = model.body('bottom_link1').id
    capture = set()
    for body in range(1,model.nbody):
        ancestor = body
        while ancestor:
            if ancestor == capture_root:
                capture.add(body)
                break
            ancestor = int(model.body_parentid[ancestor])
    def contacts():
        result = {}
        for contact in data.contact:
            a,b = [int(model.geom_bodyid[g]) for g in (contact.geom1,contact.geom2)]
            if contact.dist >= -1e-5 or not ({a,b}&capture):
                continue
            # Different bodies within the capture mechanism can intentionally
            # touch. Retain arm links versus body/other arm as relevant pairs.
            names = [model.body(i).name for i in (a,b)]
            if not any(n.startswith('bottom_link') for n in names):
                continue
            pair = tuple(sorted(names))
            result[pair] = min(result.get(pair,0.),float(contact.dist))
        return result
    samples = []
    baseline = None
    new = {}
    for degree in range(0,-91,-1):
        data.qpos[model.joint('bottom_joint1').qposadr] = np.deg2rad(degree)
        mujoco.mj_forward(model,data)
        current = contacts()
        if baseline is None:
            baseline = current
        for pair,distance in current.items():
            if pair not in baseline:
                new.setdefault(pair,[]).append(dict(joint1_deg=degree,penetration_m=-distance))
        samples.append(dict(joint1_deg=degree,contacts=[dict(bodies=list(pair),penetration_m=-dist)
                                                     for pair,dist in current.items()]))
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    summary = dict(scope='static model self-collision screen, not deployment certification',
        scene_sha256=hashlib.sha256(args.scene_model.read_bytes()).hexdigest(),
        plugin_sha256=hashlib.sha256(args.plugin.read_bytes()).hexdigest(),runtime=mujoco.mj_version(),
        candidate='bottom_joint1: 0 to -90 degrees; all other arm joints at authored zero',
        baseline_contacts=[dict(bodies=list(pair),penetration_m=-dist) for pair,dist in baseline.items()],
        introduced_contacts=[dict(bodies=list(pair),first=values[0],
                                 worst=max(values,key=lambda v:v['penetration_m'])) for pair,values in new.items()],
        sampled_configurations=len(samples),no_new_penetration_pairs=not new,
        deployment_validated=False)
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    (output/'samples.jsonl').write_text(''.join(json.dumps(s)+'\n' for s in samples),encoding='utf-8')
    print(json.dumps(summary,indent=2))
    return 0 if not new else 2


if __name__ == '__main__':
    raise SystemExit(main())
