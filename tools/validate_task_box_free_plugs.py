"""Actual CAD connector surface tests and repeatable MuJoCo timings."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / 'model/SARM/platform/sarm_task_box_plugs.xml'


def name(model, kind, index):
    return mj.mj_id2name(model, kind, int(index))


def contacts(model, data, flexid):
    rows = []
    for i, c in enumerate(data.contact):
        if flexid not in c.flex:
            continue
        f = np.zeros(6)
        mj.mj_contactForce(model, data, i, f)
        rows.append(dict(distance_m=float(c.dist), force_N=float(f[0]),
                         pair=[name(model, mj.mjtObj.mjOBJ_GEOM, g) if g >= 0 else
                               name(model, mj.mjtObj.mjOBJ_FLEX, k) for g, k in zip(c.geom, c.flex)]))
    return rows


def pose(model, data, item, target, angle, lift):
    j = model.joint(item['name'] + '_free').id
    a = model.jnt_qposadr[j]
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    old = np.array([.28, .275, 0]) if item['name'] == 'guide_plug' else np.array([.37, .22, 0])
    center = np.asarray(item['bus_center_m'])
    data.qpos[a:a+3] = np.asarray(target) + rotation @ (center - old) + [0, 0, lift]
    data.qpos[a+3:a+7] = [np.cos(angle/2), 0, 0, np.sin(angle/2)]


def main(output):
    assert mj.__version__ == '3.7.0'
    model = mj.MjModel.from_xml_path(str(SCENE))
    root = ET.parse(SCENE).getroot()
    expected_meshes = [g.get('mesh', '') for g in root.find('worldbody').iter('geom')]
    compiled_meshes = [model.mesh(int(model.geom_dataid[i])).name
                       if model.geom_type[i] == mj.mjtGeom.mjGEOM_MESH else ''
                       for i in range(model.ngeom)]
    assert expected_meshes == compiled_meshes, 'UE geometry mapping differs'
    data = mj.MjData(model)
    metadata = json.loads(SCENE.with_suffix('.manifest.json').read_text())['plugs']
    mj.mj_forward(model, data)
    report = dict(initial_contacts=int(data.ncon), nq=model.nq, nv=model.nv,
                  xml_compiled_geom_mapping_equal=True, cases={})
    angle = np.arctan2(.5469441386039887, .8371691043312224)
    for item in metadata:
        number = 9 if item['name'] == 'guide_plug' else 13
        f = mj.mj_name2id(model, mj.mjtObj.mjOBJ_FLEX, f'task_box_{number:03}_contact')
        target = [.535, .275 if number == 9 else .195, 0]
        for label, rotation, lift, offset in [('aligned_above', angle, .081, 0),
                                               ('aligned_partial', angle, .02, 0),
                                               ('aligned_seated', angle, .0001, 0),
                                               ('wrong_angle', angle + .4, .0001, 0),
                                               ('wrong_position', angle, .0001, .004)]:
            mj.mj_resetData(model, data)
            pose(model, data, item, np.array(target)+[offset, 0, 0], rotation, lift)
            mj.mj_forward(model, data)
            rows = contacts(model, data, f)
            report['cases'][item['name'] + '/' + label] = dict(contacts=len(rows),
                minimum_distance_m=min((r['distance_m'] for r in rows), default=None),
                peak_force_N=max((r['force_N'] for r in rows), default=0), sample=rows[:2])
    report['warnings'] = data.warning.number.tolist()
    report['dynamic_insertion'] = {}
    for item in metadata:
        number = 9 if item['name'] == 'guide_plug' else 13
        f = mj.mj_name2id(model, mj.mjtObj.mjOBJ_FLEX, f'task_box_{number:03}_contact')
        target = [.535, .275 if number == 9 else .195, 0]
        j = model.joint(item['name'] + '_free').id
        a, v = model.jnt_qposadr[j], model.jnt_dofadr[j]
        for label, rotation in [('aligned', angle), ('misaligned', angle + .4)]:
            mj.mj_resetData(model, data)
            pose(model, data, item, target, rotation, .081)
            data.qvel[v+2] = -.03
            first, peak, max_contacts = None, 0., 0
            physics_wall = 0.
            start = time.perf_counter()
            for _ in range(2000):
                step_start = time.perf_counter()
                mj.mj_step(model, data)
                physics_wall += time.perf_counter()-step_start
                rows = contacts(model, data, f)
                active = [r for r in rows if r['force_N'] > 1e-6]
                if active and first is None:
                    first = dict(time_s=float(data.time), center_z_m=float(data.qpos[a+2]))
                max_contacts = max(max_contacts, len(rows))
                peak = max(peak, max((r['force_N'] for r in active), default=0.))
            wall = time.perf_counter()-start
            report['dynamic_insertion'][item['name']+'/'+label] = dict(
                simulated_s=float(data.time), wall_s=wall, rtf=float(data.time/wall),
                physics_step_wall_s=physics_wall, physics_only_rtf=float(data.time/physics_wall),
                diagnostics_and_loop_wall_s=wall-physics_wall,
                first_contact=first, peak_normal_force_N=peak, max_contacts=max_contacts,
                final_center_m=data.qpos[a:a+3].tolist(), warnings=data.warning.number.tolist(),
                finite=bool(np.isfinite(data.qpos).all()),
                seating_depth_reached=bool(abs(data.qpos[a+2]-item['bus_center_m'][2]) < .0005),
                method='Initial approach velocity 0.03 m/s, no pose overwrite or insertion force; not gripper-driven insertion.')
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    assert report['initial_contacts'] == 0
    for result in report['dynamic_insertion'].values():
        assert result['finite'] and not any(result['warnings'])
    assert report['cases']['guide_plug/aligned_seated']['contacts'] == 0
    for item in metadata:
        prefix = item['name'] + '/'
        assert report['cases'][prefix+'aligned_partial']['contacts'] == 0
        assert report['cases'][prefix+'wrong_angle']['peak_force_N'] > 0
        assert report['cases'][prefix+'wrong_position']['peak_force_N'] > 0
    print(json.dumps(report, indent=2), flush=True)
    # Timings are reported separately from geometry scan and do not imply gripping.
    for file in ('sarm_ground_target_self_collision.xml', 'sarm_task_box_module.xml', 'sarm_task_box_plugs.xml'):
        m = mj.MjModel.from_xml_path(str(SCENE.with_name(file)))
        results = []
        for _ in range(3):
            d = mj.MjData(m)
            start = time.perf_counter()
            mj.mj_step(m, d, nstep=1000)
            wall = time.perf_counter() - start
            results.append(dict(wall_s=wall, rtf=float(d.time/wall), final_contacts=int(d.ncon),
                                finite=bool(np.isfinite(d.qpos).all()), warnings=d.warning.number.tolist()))
            assert results[-1]['finite'] and not any(results[-1]['warnings'])
        report.setdefault('unloaded_benchmark', {})[file] = results
        output.write_text(json.dumps(report, indent=2), encoding='utf-8')
        print(file, results, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'reports/task-box-free-plugs.json')
    main(parser.parse_args().output)
