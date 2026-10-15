"""Offline MuJoCo 3.7.0 dynamics/performance acceptance (never import Basilisk).

Run in a separate Python process with the optional posture dependencies installed.
The same-version SARM plugin must have been built by tools/build_sarm_plugin.py.
Applied torque is a test fixture, not a claim of end-to-end robot grasp control.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics
import time

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
H = .016 / (2*np.pi)


def checked_step(model, data):
    mj.mj_step(model, data)
    if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all() or data.warning.number.any():
        raise AssertionError('Non-finite state or MuJoCo warning; no automatic reset allowed')


def address(model, name):
    joint = model.joint(name)
    return int(joint.qposadr[0]), int(joint.dofadr[0])


def hook_contact_force(model, data):
    ids = {model.body('fastener').id, model.body('hook').id}
    total = 0.
    force = np.zeros(6)
    for i, contact in enumerate(data.contact):
        if set(model.geom_bodyid[contact.geom]) == ids:
            mj.mj_contactForce(model, data, i, force)
            total += max(0., force[0])
    return total


def benchmark(models):
    samples = {name: [] for name in models}
    data = {name: mj.MjData(model) for name, model in models.items()}
    # Alternate model order to reduce scheduler/cache ordering bias.
    for batch in range(7):
        names = list(models) if batch % 2 == 0 else list(reversed(models))
        for name in names:
            model, d = models[name], data[name]
            elapsed = 0.
            for _ in range(100):
                mj.mj_resetData(model, d)
                mj.mj_forward(model, d)
                start = time.perf_counter()
                mj.mj_step(model, d)
                elapsed += time.perf_counter()-start
                assert not d.warning.number.any()
            samples[name].append(elapsed*10)  # milliseconds per step
    return {name: {'median_ms': statistics.median(values), 'batches_ms': values}
            for name, values in samples.items()}


def motion_benchmark(models):
    """Identical external torque schedule; trajectories need not be equivalent."""
    results = {}
    for name, model in models.items():
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        nut, rod = model.body('fastener').id, model.body('locking_rod').id
        timing = []
        max_contacts = 0
        for torque, steps in [(.06, 960), (0., 240), (-.15, 1440), (0., 240)]:
            for i in range(steps):
                # Pure equal/opposite moments; no net external wrench.
                axis = data.xmat[nut].reshape(3, 3)[:, 2]
                applied = torque*min(1., (i+1)/48)*axis
                data.xfrc_applied[:] = 0
                data.xfrc_applied[nut, 3:] = applied
                data.xfrc_applied[rod, 3:] = -applied
                start = time.perf_counter()
                checked_step(model, data)
                timing.append((time.perf_counter()-start)*1000)
                max_contacts = max(max_contacts, data.ncon)
        results[name] = {'mean_ms': statistics.mean(timing),
                         'median_ms': statistics.median(timing),
                         'p95_ms': float(np.quantile(timing, .95)),
                         'max_contacts': max_contacts, 'simulated_s': float(data.time)}
    return results


def validate(model, output):
    d = mj.MjData(model)
    z, zd = address(model, 'fastener_slide')
    theta, td = address(model, 'fastener_spin')
    stroke = float(model.joint('fastener_slide').range[1])
    rows, ends = [], []
    for cycle in range(3):
        for phase, torque, steps in [('tighten', .06, 960), ('hold_tight', 0., 240),
                                     ('loosen', -.15, 1440), ('hold_loose', 0., 240)]:
            for i in range(steps):
                d.qfrc_applied[:] = 0
                d.qfrc_applied[td] = torque * min(1., (i+1)/48)
                checked_step(model, d)
                rows.append({'time_s': float(d.time), 'cycle': cycle, 'phase': phase,
                    'torque_nm': float(d.qfrc_applied[td]), 'travel_m': float(d.qpos[z]),
                    'angle_rad': float(d.qpos[theta]), 'speed_rad_s': float(d.qvel[td]),
                    'helix_error_m': float(d.qpos[z]-H*d.qpos[theta]),
                    'hook_normal_force_n': hook_contact_force(model, d)})
            ends.append(dict(rows[-1]))
            assert abs(ends[-1]['helix_error_m']) < 5e-6, ends[-1]
            if phase == 'tighten':
                assert .004 < d.qpos[z] < .007, ends[-1]
                assert ends[-1]['hook_normal_force_n'] > 1., ends[-1]
                assert abs(d.qvel[td]) < .01, ends[-1]
            if phase == 'loosen':
                assert abs(d.qpos[z]) < 10e-6, ends[-1]
            if phase == 'hold_loose':
                assert abs(d.qvel[td]) < 1e-4, ends[-1]
                assert ends[-1]['hook_normal_force_n'] < .1, ends[-1]
    with (output/'cycles.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    # Soft constraints at 240 Hz permit a brief 0.15-mm release transient;
    # require 5-um closure at every settled phase above. Report both, not just pass.
    assert max(abs(row['helix_error_m']) for row in rows) < 1.5e-4, max(rows, key=lambda row: abs(row['helix_error_m']))
    assert min(row['travel_m'] for row in rows) > -1e-4
    assert max(row['travel_m'] for row in rows) < stroke+1e-4

    # Below breakaway torque, a stationary mid-stroke fastener stays put to 2 um.
    mj.mj_resetData(model, d)
    d.qpos[z] = .0015; d.qpos[theta] = .0015/H
    for _ in range(240):
        d.qfrc_applied[td] = .02
        checked_step(model, d)
    creep = float(abs(d.qpos[z]-.0015))
    assert creep < 2e-6, creep

    # The rod can swing without detaching its child; preserve coaxial geometry.
    mj.mj_resetData(model, d)
    rj, _ = address(model, 'locking_rod_hinge')
    d.qpos[rj] = -.5
    d.qpos[z] = .0015; d.qpos[theta] = .0015/H
    mj.mj_forward(model, d)
    rod, nut = model.body('locking_rod').id, model.body('fastener').id
    for _ in range(960):
        d.qfrc_applied[td] = .06
        checked_step(model, d)
        relative = d.xmat[rod].reshape(3, 3).T @ (d.xpos[nut]-d.xpos[rod])
        assert np.hypot(relative[0], relative[2]) < 2e-6
    swung_travel = float(d.qpos[z])
    assert swung_travel > .007 and hook_contact_force(model, d) < .1

    # Equal/opposite axial forces at the same world point test captive retention.
    mj.mj_resetData(model, d)
    mj.mj_forward(model, d)
    for _ in range(240):
        d.qfrc_applied[:] = 0
        force = -50*d.xmat[nut].reshape(3, 3)[:, 2]
        point = d.xpos[nut].copy()
        mj.mj_applyFT(model, d, force, np.zeros(3), point, nut, d.qfrc_applied)
        mj.mj_applyFT(model, d, -force, np.zeros(3), point, rod, d.qfrc_applied)
        checked_step(model, d)
    retention = float(d.qpos[z])
    assert abs(retention) < 20e-6, retention
    return {'cycles': 3, 'ends': ends, 'max_helix_error_m': max(abs(x['helix_error_m']) for x in rows),
            'max_settled_helix_error_m': max(abs(x['helix_error_m']) for x in ends),
            'min_travel_m': min(x['travel_m'] for x in rows),
            'max_travel_m': max(x['travel_m'] for x in rows),
            'subthreshold_creep_m_per_s': creep, 'retention_50n_travel_m': retention,
            'swung_away_travel_m': swung_travel,
            'scope': 'applied internal torque, rod swing and axial retention; no robot-grasp acceptance'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'run/captive_fastener_validation')
    args = parser.parse_args()
    if mj.__version__ != '3.7.0':
        raise RuntimeError('This validation requires MuJoCo 3.7.0')
    manifest = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text())
    plugin = ROOT/manifest['library']
    if hashlib.sha256(plugin.read_bytes()).hexdigest() != manifest['plugin_sha256']:
        raise RuntimeError('Plugin hash mismatch')
    mj.mj_loadPluginLibrary(str(plugin))
    paths = {'captive': ROOT/'model/SARM/platform/sarm_20261012.xml',
             'sdf': ROOT/'model/SARM/platform/sarm_20261012_sdf.xml'}
    models = {name: mj.MjModel.from_xml_path(str(path)) for name, path in paths.items()}
    for model in models.values():
        model.opt.timestep = 1/240
        model.opt.disableflags |= mj.mjtDisableBit.mjDSBL_AUTORESET
    # All source body poses, masses and inertia tensors must survive reparenting.
    states = {name: mj.MjData(model) for name, model in models.items()}
    for name, model in models.items():
        mj.mj_forward(model, states[name])
    for b in range(1, models['sdf'].nbody):
        name = models['sdf'].body(b).name
        i = models['captive'].body(name).id
        for field in ('xpos', 'xmat'):
            assert np.allclose(getattr(states['sdf'], field)[b], getattr(states['captive'], field)[i], atol=1e-10, rtol=0), (name, field)
        for field in ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat'):
            assert np.allclose(getattr(models['sdf'], field)[b], getattr(models['captive'], field)[i], atol=1e-10, rtol=0), (name, field)
    args.output.mkdir(parents=True, exist_ok=True)
    report = {'runtime': mj.__version__, 'dt_s': 1/240,
              'models': {name: hashlib.sha256(path.read_bytes()).hexdigest() for name, path in paths.items()},
              'functional': validate(models['captive'], args.output), 'performance': benchmark(models),
              'motion_performance': motion_benchmark(models)}
    report['speedup'] = report['performance']['sdf']['median_ms']/report['performance']['captive']['median_ms']
    (args.output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
