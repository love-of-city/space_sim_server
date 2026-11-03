"""Inspect cross-module geometry even when rigid-body collision filtering skips it."""
import argparse
from pathlib import Path
import json

import mujoco
from preview_scene_model import load_geometry_model
from simulation.scene_library.manifest import digest, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    model = load_geometry_model(args.model)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    roots = {'mounted_capture_target': 'target', 'task_box_base': 'hardware',
             'drawer_base': 'hardware', 'service_receiver_mount': 'receiver'}
    groups = {key: [] for key in ('target', 'hardware', 'receiver', 'service')}
    for gid in range(model.ngeom):
        if not (model.geom_contype[gid] or model.geom_conaffinity[gid]):
            continue
        geom_name = model.geom(gid).name or ''
        if 'standoff' in geom_name or 'pedestal' in geom_name:
            continue  # Intentional support interfaces need a separate structural check.
        bid = int(model.geom_bodyid[gid])
        if model.body(bid).name == 'cubesat_bus':
            groups['service'].append(gid)
            continue
        while bid:
            name = model.body(bid).name
            if name in roots:
                groups[roots[name]].append(gid)
                break
            bid = int(model.body_parentid[bid])
    checks, unsupported = [], []
    for left, right in [('target', 'service'), ('target', 'hardware'),
                        ('target', 'receiver'), ('receiver', 'service')]:
        rows, tested = [], 0
        for a in groups[left]:
            for b in groups[right]:
                if any(int(model.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_SDF) for g in (a, b)):
                    unsupported.append([model.geom(a).name, model.geom(b).name])
                    continue
                distance = float(mujoco.mj_geomDistance(model, data, a, b, .005, None))
                tested += 1
                if distance < .005:
                    rows.append(dict(first=model.geom(a).name, second=model.geom(b).name,
                                     separation_m=distance))
        checks.append(dict(groups=[left, right], tested_pairs=tested,
                           penetrations=[row for row in rows if row['separation_m'] < 0.],
                           pairs_within_5mm=rows))
    empty_groups = [name for name, ids in groups.items() if not ids and name != 'receiver']
    result = dict(schema='space-arm-mount-inspection/2', model_sha256=digest(args.model),
                  coverage_complete=not empty_groups and not unsupported, empty_required_groups=empty_groups,
                  geometry_counts={name: len(ids) for name, ids in groups.items()}, checks=checks,
                  unsupported_pairs=unsupported, task_accepted=False, assembly_accepted=False,
                  scope='Explicit pair distances at authored joint zero pose; no rigid-body filter; no strength or support-interface proof')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_json(args.output, result)
    print(json.dumps(dict(report=str(args.output), tested_pairs=sum(r['tested_pairs'] for r in checks),
        penetration_pairs=sum(len(r['penetrations']) for r in checks), unsupported_pairs=len(unsupported))))


if __name__ == '__main__':
    main()
