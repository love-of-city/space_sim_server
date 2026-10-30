"""Independent native SDF contact fixture, not an end-to-end robot task.

Run in a separate standalone MuJoCo process, never after importing Basilisk.
Equal/opposite wrenches act at the same world point, with a compliant alignment
fixture. All motion comes from mj_step; no qpos or qvel is assigned after reset.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

import argparse
import ctypes
import hashlib
import json
import math
from pathlib import Path

import mujoco as mj
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def validate(model_path):
    keep = ctypes.CDLL(str(next(Path(mj.__file__).parent.glob('libmujoco.so*'))),
                       mode=ctypes.RTLD_GLOBAL)
    manifest = json.loads((ROOT/'run/native_acceleration/sarm_thread.json').read_text())
    if mj.mj_version() != manifest['runtime_version']:
        raise ValueError('Probe and production MuJoCo versions differ')
    mj.mj_loadPluginLibrary(str(ROOT/manifest['library']))
    model = mj.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = .0025  # production 400-Hz dynamics, one native substep
    model.opt.disableflags |= mj.mjtDisableBit.mjDSBL_AUTORESET
    bolt, bus = (model.body(name).id for name in ('bolt', 'cubesat_bus'))
    results = {}
    for label, seconds in (('idle', 1.), ('pull_without_turning', 2.), ('unscrew_positive_z', 4.)):
        data = mj.MjData(model)
        mj.mj_forward(model, data)
        base_r = data.xmat[bus].reshape(3, 3)
        initial_p = base_r.T @ (data.xpos[bolt]-data.xpos[bus])
        initial_r = base_r.T @ data.xmat[bolt].reshape(3, 3)
        angle, last_yaw = 0., 0.
        rows = []
        for tick in range(round(seconds/model.opt.timestep)+1):
            base_r = data.xmat[bus].reshape(3, 3)
            p = base_r.T @ (data.xpos[bolt]-data.xpos[bus])
            r = base_r.T @ data.xmat[bolt].reshape(3, 3)
            rr = r @ initial_r.T
            yaw = math.atan2(rr[1, 0], rr[0, 0])
            angle += (yaw-last_yaw+math.pi) % (2*math.pi)-math.pi
            last_yaw = yaw
            if tick % 200 == 0:
                rows.append(dict(time_s=float(data.time), displacement_m=(p-initial_p).tolist(),
                                 angle_rad=angle, helix_error_m=float(p[2]-initial_p[2]-.016*angle/(2*math.pi))))
            if tick == round(seconds/model.opt.timestep):
                break
            data.qfrc_applied[:] = 0
            if label != 'idle':
                velocity = base_r.T @ (data.cvel[bolt, 3:]-data.cvel[bus, 3:])
                omega = base_r.T @ (data.cvel[bolt, :3]-data.cvel[bus, :3])
                theta = .5*data.time if label == 'unscrew_positive_z' else 0.
                c, s = math.cos(theta), math.sin(theta)
                desired = np.array([[c,-s,0],[s,c,0],[0,0,1]]) @ initial_r
                error = desired @ r.T
                angular_error = .5*np.array([error[2,1]-error[1,2], error[0,2]-error[2,0], error[1,0]-error[0,1]])
                force = -500*(p-initial_p)-3*velocity
                force[2] = 1. if label == 'unscrew_positive_z' else 5.
                torque = .5*angular_error-.02*omega
                point = data.xpos[bolt].copy()
                mj.mj_applyFT(model, data, base_r@force, base_r@torque, point, bolt, data.qfrc_applied)
                mj.mj_applyFT(model, data, -base_r@force, -base_r@torque, point, bus, data.qfrc_applied)
            mj.mj_step(model, data)
            if data.warning.number.any() or not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise AssertionError('Native warning or non-finite state; no automatic reset allowed')
        results[label] = dict(samples=rows, final=rows[-1], warnings=data.warning.number.tolist())
    assert np.linalg.norm(results['idle']['final']['displacement_m']) < 1e-8
    assert abs(results['pull_without_turning']['final']['displacement_m'][2]) < .001
    screw = results['unscrew_positive_z']['final']
    assert screw['angle_rad'] > 1.5 and screw['displacement_m'][2] > .004
    assert abs(screw['helix_error_m']) < .001
    _ = keep
    return dict(passed=True, runtime=mj.__version__, model_sha256=hashlib.sha256(model_path.read_bytes()).hexdigest(),
                timestep_s=float(model.opt.timestep), results=results, runtime_state_writes=False,
                scope='native storage-bore stability, same-point 5-N pull fixture and positive-Z unscrew fixture; no robot-grasp or full transfer acceptance')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = validate(args.model.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({key: value for key, value in report.items() if key != 'results'}, indent=2))
