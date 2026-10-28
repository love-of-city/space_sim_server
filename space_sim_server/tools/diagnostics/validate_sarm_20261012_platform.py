"""Offline native integration probe using the same Basilisk DLL as the platform.

Run with the platform Python. No Python MuJoCo package is imported. An integration
warning, automatic reset or non-finite state fails the run in LocalMujocoStepper.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))

from pathlib import Path
import argparse
import hashlib
import json
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent/'space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples'))

def validate_cycles(samples):
    """Functional bounds, not a claim of microradian physical accuracy.

    Require >=95% stroke each way, <=0.025 rad tracking error (one .02-rad
    command increment plus .005 rad), and <1e-6 rad opposed-link closure error.
    """
    if max(abs(s['positions'][6]-s['target_rad']) for s in samples) > .025:
        raise AssertionError('tendon target tracking failed')
    if max(s['coupling_error_rad'] for s in samples) > 1e-6:
        raise AssertionError('opposed linkage equality failed')
    for start in (10,20,30):
        opening=[s['positions'][6] for s in samples if start < s['time_s'] <= start+5]
        closing=[s['positions'][6] for s in samples if start+5 < s['time_s'] <= start+10]
        if not opening or not closing or max(opening) < .95 or closing[-1] > .05:
            raise AssertionError('full gripper stroke not observed')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=ROOT/'run/sarm_20261012_validation.json')
    parser.add_argument('--local-substeps', type=int, default=None,
        help='Override native substeps; otherwise use the platform environment/default.')
    args=parser.parse_args()
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    n=load_native_grasp_module(ROOT/'model/SARM/platform')
    from simulation.sarm_model_profile import load_model_profile
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from Basilisk.utilities import macros
    n.MODEL_PATH=ROOT/'model/SARM/platform/sarm_20261012.xml'
    p=load_model_profile(n.MODEL_PATH)
    n.JOINTS=p.joints; n.ACTUATORS=p.arm_actuators
    n.KP=np.full(6,500.); n.KD=np.full(6,50.);n.TORQUE_LIMITS=np.full(6,100.);n.TIME_STEP=1/240
    q=np.zeros(8)
    ref=HeldJointReferencePublisher(lambda t:(q.copy(),np.zeros(8)),8)
    simulation,scene,keep,_=n._build_simulation(external_reference=ref,dynamics_backend='local',local_substeps=args.local_substeps,
        record_history=False,attitude_control_enabled=False)
    simulation.AddModelToTask('graspTask',ref,9000)
    simulation.InitializeSimulation()
    # Exercise Galilean common drift, which exposed source free-joint world
    # damping during the real platform startup (zero velocity concealed it).
    from space_arm_platform.task_box import initialize_free_plugs
    initialize_free_plugs(scene,n.MODEL_PATH,np.zeros(3),n.COMMON_VELOCITY)
    for name in ('cubesat_bus','capture_target'):
        scene.getBody(name).setVelocity(n.COMMON_VELOCITY)
    reads=[scene.getBody(b).getScalarJoint(j).stateOutMsg.addSubscriber() for b,j in p.joints]
    samples=[]
    started=time.perf_counter()
    for i in range(1,401):
        seconds=i/10
        # Ten seconds at rest, then three full 0..1..0 cycles at 0.2 rad/s.
        phase=max(0.,seconds-10)%10
        q[6]=.2*min(phase,10-phase);q[7]=-q[6]
        simulation.ConfigureStopTime(macros.sec2nano(seconds));simulation.ExecuteSimulation()
        stepper=simulation.local_stepper
        if not np.isfinite(stepper.qpos).all() or not np.isfinite(stepper.qvel).all():
            raise RuntimeError('Non-finite physical state')
        positions=[float(read().state) for read in reads]
        samples.append(dict(time_s=seconds,target_rad=float(q[6]),positions=positions,
            coupling_error_rad=abs(positions[6]+positions[7]),max_qacc=float(np.abs(stepper.qacc).max())))
        if i%10==0: print(json.dumps(samples[-1]),flush=True)
    elapsed=time.perf_counter()-started
    validate_cycles(samples)
    report=dict(model_sha256=hashlib.sha256(n.MODEL_PATH.read_bytes()).hexdigest(),runtime='Basilisk MuJoCo 3.7.0',
        substeps=stepper.substeps,wall_seconds=elapsed,real_time_factor=40/elapsed,
        nq=stepper.nq,nv=stepper.nv,samples=samples,
        max_coupling_error_rad=max(s['coupling_error_rad'] for s in samples),
        scope='zero arm pose with common free-body drift, 10 seconds idle and three full tendon cycles; no grasp/plug insertion acceptance')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(f'PASS {args.output}',flush=True)
    _=keep

if __name__=='__main__': main()
