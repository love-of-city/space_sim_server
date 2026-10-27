"""Independent native fixtures for locked retention, elastic clip seating and lift.

External joint torque is a labelled test fixture, not robot demonstration data.
Run separately from Basilisk. No qpos/qvel writes occur during integration.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'backend'))

from simulation.scripted_tasks.randomization import write_model_variant
from tools.locking_cycle_contacts import apply_locking_cycle_contacts

def fixture(model, data, *, seconds, controller):
    hinge = model.joint('locking_rod_hinge')
    q, v = int(hinge.qposadr[0]), int(hinge.dofadr[0])
    rod, clip, nut, base, hook = (model.body(name).id for name in
                                  ('locking_rod', 'limit_clip', 'fastener', 'task_box_base', 'hook'))
    force = np.zeros(6)
    rows, maxima = [], {'clip_contact_force_n': 0., 'base_penetration_m': 0.}
    for tick in range(round(seconds/model.opt.timestep)+1):
        phase, torque = controller(float(data.time), float(data.qpos[q]), float(data.qvel[v]))
        data.qfrc_applied[:] = 0
        data.qfrc_applied[v] = torque
        clip_hits, hook_hits = [], []
        for index, contact in enumerate(data.contact):
            bodies = set(model.geom_bodyid[contact.geom])
            if bodies == {rod, clip} or bodies == {nut, hook}:
                mj.mj_contactForce(model, data, index, force)
                hit = dict(separation_m=float(contact.dist), normal_force_n=float(force[0]))
                (clip_hits if bodies == {rod, clip} else hook_hits).append(hit)
                if bodies == {rod, clip}:
                    maxima['clip_contact_force_n'] = max(maxima['clip_contact_force_n'], float(force[0]))
            if bodies == {nut, base}:
                maxima['base_penetration_m'] = max(maxima['base_penetration_m'], -float(contact.dist))
        if tick % 100 == 0:
            rows.append(dict(time_s=float(data.time), phase=phase, hinge_rad=float(data.qpos[q]),
                             hinge_speed_rad_s=float(data.qvel[v]), torque_nm=float(torque),
                             clip_contacts=clip_hits, hook_contacts=hook_hits))
        if tick == round(seconds/model.opt.timestep):
            break
        mj.mj_step(model, data)
        if data.warning.number.any() or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
            raise AssertionError('Native warning or non-finite state; no automatic reset allowed')
    return {'samples': rows, 'maxima': maxima, 'warnings': data.warning.number.tolist()}


def validate(source, output):
    output.mkdir(parents=True, exist_ok=True)
    locked_path = write_model_variant(source, output/'locked.xml', {'scripted_task_name':'locking_rod_cycle'})
    loose_path = write_model_variant(source, output/'loose.xml', {'scripted_task_name':'aviation_bolt_insert'})
    loose_tree = ET.parse(loose_path)
    apply_locking_cycle_contacts(loose_tree.getroot())
    loose_tree.find('.//joint[@name="locking_rod_hinge"]').set('range', '-1.75 .15')
    loose_tree.write(loose_path)
    keep = ctypes.CDLL(str(next(Path(mj.__file__).parent.glob('libmujoco.so*'))), mode=ctypes.RTLD_GLOBAL)
    manifest = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text())
    if mj.mj_version() != manifest['runtime_version']:
        raise ValueError('Probe and production MuJoCo versions differ')
    mj.mj_loadPluginLibrary(str(ROOT/manifest['library']))
    models = {name: mj.MjModel.from_xml_path(str(path)) for name, path in
              (('locked', locked_path), ('loose', loose_path))}
    for model in models.values():
        model.opt.timestep = .0025
        model.opt.disableflags |= mj.mjtDisableBit.mjDSBL_AUTORESET
    locked = models['locked']
    data = mj.MjData(locked)
    mj.mj_forward(locked, data)
    retain = fixture(locked, data, seconds=8., controller=lambda t, q, v:
                     ('unloaded_warmup' if t < 2 else 'locked_load', 0. if t < 2 else -.035*min(1., t-2)))
    assert abs(retain['samples'][-1]['hinge_rad']) < .04
    assert sum(hit['normal_force_n'] for hit in retain['samples'][-1]['hook_contacts']) > .1

    def cycle_controller(t, q, v):
        phase = 'lower' if t < 6 else 'released_hold' if t < 9 else 'raise'
        target = -min(1.53, t*.4) if phase == 'lower' else min(0., -1.53+(t-9)*.4)
        torque = 0. if phase == 'released_hold' else float(np.clip(20*(target-q)-.35*v, -3., 3.))
        return phase, torque

    loose = models['loose']
    data = mj.MjData(loose)
    mj.mj_forward(loose, data)
    cycle = fixture(loose, data, seconds=16., controller=cycle_controller)
    held = [row for row in cycle['samples'] if 8 < row['time_s'] < 9]
    assert held and all(-1.56 < row['hinge_rad'] < -1.505 for row in held)
    assert all(abs(row['hinge_speed_rad_s']) < .003 for row in held)
    assert any(row['clip_contacts'] for row in held)
    assert abs(cycle['samples'][-1]['hinge_rad']) < .01
    assert cycle['maxima']['base_penetration_m'] < .0001
    _ = keep
    report = dict(passed=True, runtime=mj.__version__, timestep_s=.0025,
                  source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                  locked_retention=retain, clip_cycle=cycle, runtime_state_writes=False,
                  scope='external joint-torque physics fixtures for unloaded initial settle, geometric lock, elastic clip seat/hold and lift; no robot-grasp acceptance',
                  clip_compliance='nominal uncalibrated elastic snap fit')
    (output/'validation.json').write_text(json.dumps(report, indent=2)+'\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=ROOT/'model/SARM/platform/sarm_20261012.xml')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps({key: value for key, value in validate(args.model.resolve(), args.output.resolve()).items()
                      if key not in {'locked_retention', 'clip_cycle'}}, indent=2))
