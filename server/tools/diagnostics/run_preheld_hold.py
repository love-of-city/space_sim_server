"""Record a fixed-reference preheld interval using the production native runner.

No task trajectory, planner, renderer, camera or joint-state override is installed.
The process-local adapters below replace only the task/controller and its reader;
BSK dynamics, capture-arm servos, retention, sensors and cleanup remain the runner's.
Run verify_preheld_capture.py separately in the geometry Python environment.
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import dataclass
from datetime import datetime, timezone
import gzip
import hashlib
import json
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch
import xml.etree.ElementTree as ET

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT/'backend'), str(ROOT.parent/'ue-adapter/Adapters'),
               str(ROOT.parent/'ue-adapter/Unreal/BskUnrealRenderer/examples')]


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


@dataclass(frozen=True)
class HoldConfig:
    duration_s: float
    force_limit_n: float = 40.
    torque_limit_nm: float = 4.
    overload_hold_s: float = .1


class HoldMonitor:
    """Keep the requested reference, including after measured servo deflection."""
    def __init__(self, reference, duration_s):
        self.reference = np.array(reference, dtype=float, copy=True)
        if self.reference.shape != (8,) or not np.isfinite(self.reference).all():
            raise ValueError('Hold requires eight finite initial joint references')
        ticks=duration_s/.005
        if not np.isfinite(duration_s) or duration_s <= 0 or not np.isclose(ticks, round(ticks), atol=1e-8, rtol=0):
            raise ValueError('Hold duration must be a positive multiple of 0.005 seconds')
        self.config = HoldConfig(float(duration_s))
        self.velocity = np.zeros(8)
        self.stage, self.result, self.reason = 'holding', None, ''
        self.events = []
        self.max_force_n = self.max_torque_nm = 0.
        self._last_ns = self._overload_start_ns = None

    def fail(self, reason, sample):
        self.stage, self.result, self.reason = 'failed', 'failed', reason
        self.events.append(dict(stage='failed', sim_time_s=float(sample.time_s), reason=reason))

    def step(self, sample):
        if self.result is not None:
            return self.reference.copy(), self.velocity.copy()
        stamp=round(sample.time_s*1e9)
        wrench=np.asarray(sample.wrench, dtype=float)
        if self._last_ns is None and stamp != 0 or self._last_ns is not None and stamp-self._last_ns != 5_000_000:
            self.fail('missing_or_nonconsecutive_200hz_hold_sample',sample)
        elif wrench.shape != (6,) or not np.isfinite(wrench).all():
            self.fail('invalid_wrist_measurement',sample)
        elif not sample.wrench_valid and not (stamp==0 and np.all(wrench==0)):
            self.fail('wrist_sensor_unavailable',sample)
        else:
            force,torque=float(np.linalg.norm(wrench[:3])),float(np.linalg.norm(wrench[3:]))
            self.max_force_n=max(self.max_force_n,force);self.max_torque_nm=max(self.max_torque_nm,torque)
            if force>self.config.force_limit_n or torque>self.config.torque_limit_nm:
                if self._overload_start_ns is None:self._overload_start_ns=stamp
                if stamp-self._overload_start_ns >= round(self.config.overload_hold_s*1e9):
                    self.fail('wrist_load_limit',sample)
            else:self._overload_start_ns=None
            if self.result is None and stamp >= round(self.config.duration_s*1e9):
                self.stage,self.result,self.reason='hold_completed','success','requested_hold_interval_completed'
                self.events.append(dict(stage='hold_completed',sim_time_s=float(sample.time_s)))
        self._last_ns=stamp
        return self.reference.copy(), self.velocity.copy()

    def metrics(self, sample):
        return dict(hold_only=True, full_task_accepted=False,
                    max_operating_joint_reference_error_rad=float(np.max(np.abs(sample.joints-self.reference))))


def run(args):
    model,scene_path,output=args.model.resolve(),args.scene.resolve(),args.output.resolve()
    initial_scene=json.loads(scene_path.read_text(encoding='utf-8-sig'))
    if not initial_scene.get('preheld_capture'):
        raise ValueError('Hold recording requires an explicit preheld_capture initial scene')
    initial=np.asarray(initial_scene['randomization']['arm_joint_position_rad'],dtype=float)
    HoldMonitor(initial,args.seconds)  # Validate before importing native libraries.
    from simulation.scripted_tasks.randomization import options_for_scene
    options=options_for_scene(initial_scene).model_dump()
    if any(isinstance(value,dict) and value.get('enabled') for value in options.values()):
        raise ValueError('Fixed hold forbids collection randomization and disturbances')
    output.mkdir(parents=True,exist_ok=False)
    sources=sorted(set(ROOT.joinpath('simulation').rglob('*.py')) |
                   {Path(__file__).resolve(),ROOT/'collection/run_scripted_task.py',model,scene_path,
                    model.with_name('attitude_control.json')})
    launch=dict(schema='preheld-fixed-reference-recording/1',started_utc=datetime.now(timezone.utc).isoformat(),
                model=str(model),scene=str(scene_path),duration_s=args.seconds,
                source_sha256={str(path.resolve()):digest(path) for path in sources},
                fixed_operating_joint_reference_rad=initial.tolist(),force_limit_n=40.,torque_limit_nm=4.,
                continuous_overload_hold_s=.1,native_substeps=1,UE_started=False,
                task_trajectory_executed=False,full_task_accepted=False,
                adapter='process-local fixed-reference controller and read-only recorder; production dynamics unchanged')
    write_json(output/'launch.json',launch)
    # Preserve the same DLL load order as run_scripted_task before importing the stepper.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    load_native_grasp_module(ROOT/'model/SARM/platform')
    from simulation.local_mujoco_stepper import LocalMujocoStepper
    from simulation.scripted_tasks import drawer_lift, native_state
    from simulation.serial_chain_kinematics import SerialChainKinematics
    from collection import run_scripted_task
    names={};previous_contact_ns=None

    class HoldTask(HoldMonitor):
        def __init__(self, model_path, _ignored_task_config):
            super().__init__(initial,args.seconds)
            self.name='preheld_hold'
            self.model_path=model_path
            self.chain=SerialChainKinematics.from_mjcf(model_path,base_body='cubesat_bus',
                joint_names=tuple(f'joint{i}' for i in range(1,7)),tool_site='sarm_ee')
            root=ET.parse(model_path).getroot()
            self.lower,self.upper=np.array([np.fromstring(root.find(f'.//joint[@name="joint{i}"]').get('range'),sep=' ')
                                          for i in range(1,7)]).T

    original_update=LocalMujocoStepper.UpdateState
    with ExitStack() as stack:
        contacts=stack.enter_context(gzip.open(output/'contacts-400hz.jsonl.gz','wt',encoding='utf-8',compresslevel=1))
        states=stack.enter_context(gzip.open(output/'states-200hz.jsonl.gz','wt',encoding='utf-8',compresslevel=1))

        def update(stepper,now):
            nonlocal previous_contact_ns
            result=original_update(stepper,now)
            if not names:
                names.update(bodies={str(i):stepper._name(1,i) for i in range(stepper.nbody)},
                             geoms={})
            snapshot=stepper.contact_snapshot()
            for row in snapshot:
                for geom_id in (int(row[0]),int(row[1])):
                    key=str(geom_id)
                    if key not in names['geoms']:
                        names['geoms'][key]=stepper._name(5,geom_id) if geom_id>=0 else ''
            contacts.write(json.dumps(dict(interval_start_ns=previous_contact_ns,interval_end_ns=int(now),
                stage='holding',native_step_count=int(stepper.step_count),contacts=snapshot.tolist()))+'\n')
            previous_contact_ns=int(now)
            return result

        class HoldState:
            def __init__(self,task,_scene,stepper):
                self.task,self.stepper=task,stepper

            def sample(self,time_s,q,v,wrench):
                states.write(json.dumps(dict(sim_time_ns=round(time_s*1e9),stage='holding',
                    joint_position=q.tolist(),joint_velocity=v.tolist(),
                    joint_reference=self.task.reference.tolist(),
                    joint_reference_velocity=[0.]*8,wrist_wrench=wrench,
                    qpos=self.stepper.qpos.tolist(),qvel=self.stepper.qvel.tolist()))+'\n')
                return SimpleNamespace(time_s=time_s,joints=q,joint_velocities=v,
                    wrench=np.r_[wrench['force_n'],wrench['torque_nm']],wrench_valid=bool(wrench['valid']))

            def metrics(self,sample):
                return self.task.metrics(sample)

        stack.enter_context(patch.object(LocalMujocoStepper,'UpdateState',update))
        stack.enter_context(patch.object(drawer_lift,'DrawerLiftTask',HoldTask))
        stack.enter_context(patch.dict(native_state.STATE_READERS,{'drawer_lift':HoldState}))
        argv=['--task','drawer_lift','--model',str(model),'--scene',str(scene_path),'--output',str(output/'run'),
              '--max-sim-seconds',str(args.seconds+.01),'--substeps','1','--force-limit','40','--scene-expert',
              '--no-environment-planning','--no-save-trace']
        launch['runner_argv']=argv
        write_json(output/'launch.json',launch)
        code=2
        try:
            code=run_scripted_task.main(argv=argv)
        finally:
            write_json(output/'names.json',names)
            launch.update(native_returncode=code,finished_utc=datetime.now(timezone.utc).isoformat())
            write_json(output/'launch.json',launch)
            summary_path=output/'run/summary.json'
            if summary_path.exists():
                summary=json.loads(summary_path.read_text())
                summary.update(task_name='preheld_hold',full_task_accepted=False,
                    hold_interval_complete=summary['result']=='success',
                    scope='fixed operating reference and native preheld capture interval only')
                if summary['result']=='success':summary['result']='hold_completed'
                write_json(summary_path,summary)
    return code


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('model','scene','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=2.)
    args=parser.parse_args(argv)
    return run(args)


if __name__=='__main__':
    sys.exit(main())
