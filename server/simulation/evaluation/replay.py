"""Adapter into the production task runtime; the donor controller never steps."""
from copy import deepcopy
import json
import numpy as np
from .fixed_trajectory import FrozenTrajectory, sha256
from .scoring import PhysicalScorer
from simulation.policy_task_success import DrawerCycleSuccess, EVALUATION_PERIOD_NS


def same_task_configuration(frozen, current):
    """Current artifacts must identify the exact effective configuration."""
    return frozen == current


def scorer_for(task, trajectory, *, model=None):
    kwargs={}
    if hasattr(task, 'hole'):
        kwargs=dict(hole=task.hole,hole_rotation=task.hole_rotation,
                    tip=task.config.shaft_tip_local_m,flange=task.config.flange_local_m,keyed=task.profile.keyed,
                    moving_hole=getattr(task, 'hole_owner', 'cubesat_bus') != 'cubesat_bus',
                    pickup=task.pickup,pickup_rotation=task.pickup_rotation,
                    moving_pickup=getattr(task, 'pickup_owner', 'cubesat_bus') != 'cubesat_bus')
    final=trajectory.document['samples'][-1]['position_m']
    if trajectory.document['task']=='drawer_cycle':
        # Share the policy monitor's full geometry/dwell criteria; no script flags.
        if model is None:raise ValueError('Drawer benchmark scoring requires the immutable model, not controller targets')
        monitor=DrawerCycleSuccess(model,sample_period_ns=EVALUATION_PERIOD_NS,mode="benchmark")
        final=monitor.grip_origin+monitor.axis*monitor.config.clearance_m
        kwargs['drawer_spec']=monitor.specification()
    return PhysicalScorer(trajectory.document['task'], final, **kwargs)


def physical_row(sample, task, reference):
    p,rotation=task.chain.forward(sample.joints[:6]);ref,_=task.chain.forward(reference[:6])
    row=dict(time_s=float(sample.time_s),tool_position_m=p.tolist(),reference_position_m=ref.tolist(),
             joint_position=sample.joints.tolist(),wrench=sample.wrench.tolist(),wrench_valid=bool(sample.wrench_valid))
    if hasattr(sample,'drawer_m'):
        row.update(drawer_m=float(sample.drawer_m),drawer_velocity_m_s=float(sample.drawer_velocity_m_s),
                   buckle_rad=float(sample.buckle_rad),contacts=sample.finger_drawer_contacts,
                   tool_rotation=rotation.tolist(),joint_velocity=sample.joint_velocities.tolist())
        if getattr(sample, 'drawer_mount', None) is not None:
            origin, drawer_rotation = sample.drawer_mount
            row.update(drawer_origin_body_m=origin.tolist(), drawer_rotation_body=drawer_rotation.tolist())
    else:
        row.update(plug_position_m=sample.plug_position_body_m.tolist(),plug_rotation=sample.plug_rotation_body.tolist(),
                   plug_velocity_m_s=sample.plug_velocity_body_m_s.tolist(),plug_angular_velocity_rad_s=sample.plug_angular_velocity_body_rad_s.tolist(),
                   contacts=sample.finger_plug_contacts)
        if getattr(sample, 'site_poses', None):
            hole, hole_r = sample.site_poses['hole']
            row.update(hole_position_m=hole.tolist(), hole_rotation=hole_r.tolist())
            pickup, pickup_r = sample.site_poses['pickup']
            row.update(pickup_position_m=pickup.tolist(), pickup_rotation=pickup_r.tolist())
    return row


class FixedReplayTask:
    version='fixed-replay/2-local-expert'
    terminal_hold_s=2.5

    def __init__(self, donor, path, model, initial, server_root):
        from dataclasses import asdict
        self.donor=donor
        self.trajectory=FrozenTrajectory(json.loads(path.read_text(encoding='utf-8')))
        self.trajectory.validate_runtime(model,initial,server_root)
        if self.trajectory.document['task']!=donor.name:raise ValueError('Trajectory task mismatch')
        frozen_config=self.trajectory.document.get('donor_config')
        if frozen_config is not None and not same_task_configuration(frozen_config,json.loads(json.dumps(asdict(donor.config)))):
            raise ValueError('Trajectory task configuration mismatch')
        for q in self.trajectory.commands:
            if np.any(q[:6]<donor.lower) or np.any(q[:6]>donor.upper) or not 0<=q[6]<=1.01 or not -1.01<=q[7]<=0:
                raise ValueError('Frozen reference violates actuator limits')
        self.scorer=scorer_for(donor,self.trajectory,model=model)
        self.name=donor.name;self.reference=np.asarray(initial).copy();self.velocity=np.zeros(8)
        self.stage='fixed_playback';self.result=None;self.reason='';self.events=[]
        self.max_force_n=0.;self.max_torque_nm=0.;self.last_physical_row=None
        self.trajectory_sha256=sha256(path)
        self._initial_drawer = 0.
        self._initial_position = None

    def __getattr__(self,name):
        return getattr(self.donor,name)

    def step(self,sample):
        if self.result is not None:return self.reference.copy(),np.zeros(8)
        if self._initial_position is None and hasattr(sample, 'plug_position_body_m'):
            self._initial_position = sample.plug_position_body_m.copy()
        self.reference,self.velocity=self.trajectory.reference(sample.time_s)
        self.last_physical_row=physical_row(sample,self,self.reference)
        self.scorer.update(self.last_physical_row)
        self.max_force_n=self.scorer.max_force;self.max_torque_nm=self.scorer.max_torque
        report=self.scorer.report()
        if self.scorer.error or self.scorer.safety_failure:
            self.fail(report['reason'],sample)
        elif sample.time_s>=self.trajectory.duration+self.terminal_hold_s-1e-9:
            self.result='success' if report['success'] else 'failed';self.reason=report['reason']
            self.events.append(dict(stage=self.result,sim_time_s=float(sample.time_s),reason=self.reason))
        return self.reference.copy(),self.velocity.copy()

    def fail(self,reason,sample):
        self.result='failed';self.reason=reason
        self.reference=sample.joints.copy();self.velocity=np.zeros(8)
        self.events.append(dict(stage='failed',sim_time_s=float(sample.time_s),reason=reason))

    def metrics(self,sample):
        # Keep raw compatibility telemetry, but evaluation never reads these flags.
        metrics = {}
        metrics.update(task_version=self.version,fixed_trajectory_sha256=self.trajectory_sha256,
                       independent_evaluation=self.scorer.report())
        return metrics

    def scorer_spec(self):
        s=self.scorer
        spec=dict(task=s.task,final_tool_position=s.final.tolist(),hole=None if s.hole is None else s.hole.tolist(),
                  hole_rotation=s.hole_rotation.tolist(),tip=s.tip,flange=s.flange,keyed=s.keyed,moving_hole=s.moving_hole,
                  pickup=None if s.pickup is None else s.pickup.tolist(),
                  pickup_rotation=None if s.pickup_rotation is None else s.pickup_rotation.tolist(),
                  moving_pickup=s.moving_pickup)
        if s.drawer_monitor is not None:spec['drawer_spec']=s.drawer_monitor.specification()
        return deepcopy(spec)
