"""Adapter into the production task runtime; the donor controller never steps."""
from copy import deepcopy
import json
import numpy as np
from .fixed_trajectory import FrozenTrajectory, sha256
from .scoring import PhysicalScorer


def scorer_for(task, trajectory):
    kwargs={}
    if hasattr(task, 'hole'):
        kwargs=dict(hole=task.hole,hole_rotation=task.hole_rotation,
                    tip=task.config.shaft_tip_local_m,flange=task.config.flange_local_m,keyed=task.profile.keyed)
    final=trajectory.document['samples'][-1]['position_m']
    if trajectory.document['task']=='drawer_cycle':
        # Delivered fixed model starts at zero slide displacement. Retreat is a
        # task geometry requirement, independent of where a failed donor stops.
        final=task.grip_origin+task.axis*task.config.clearance_m
    return PhysicalScorer(trajectory.document['task'], final, **kwargs)


def physical_row(sample, task, reference):
    p,_=task.chain.forward(sample.joints[:6]);ref,_=task.chain.forward(reference[:6])
    row=dict(time_s=float(sample.time_s),tool_position_m=p.tolist(),reference_position_m=ref.tolist(),
             joint_position=sample.joints.tolist(),wrench=sample.wrench.tolist(),wrench_valid=bool(sample.wrench_valid))
    if hasattr(sample,'drawer_m'):
        row.update(drawer_m=float(sample.drawer_m),drawer_velocity_m_s=float(sample.drawer_velocity_m_s),
                   buckle_rad=float(sample.buckle_rad),contacts=sample.finger_drawer_contacts)
    else:
        row.update(plug_position_m=sample.plug_position_body_m.tolist(),plug_rotation=sample.plug_rotation_body.tolist(),
                   plug_velocity_m_s=sample.plug_velocity_body_m_s.tolist(),plug_angular_velocity_rad_s=sample.plug_angular_velocity_body_rad_s.tolist(),
                   contacts=sample.finger_plug_contacts)
    return row


class FixedReplayTask:
    version='fixed-replay/1'
    terminal_hold_s=2.5

    def __init__(self, donor, path, model, initial, server_root):
        from dataclasses import asdict
        self.donor=donor
        self.trajectory=FrozenTrajectory(json.loads(path.read_text(encoding='utf-8')))
        self.trajectory.validate_runtime(model,initial,server_root)
        if self.trajectory.document['task']!=donor.name:raise ValueError('Trajectory task mismatch')
        frozen_config=self.trajectory.document.get('donor_config')
        if frozen_config is not None and frozen_config!=json.loads(json.dumps(asdict(donor.config))):
            raise ValueError('Trajectory task configuration mismatch')
        for q in self.trajectory.commands:
            if np.any(q[:6]<donor.lower) or np.any(q[:6]>donor.upper) or not 0<=q[6]<=1.01 or not -1.01<=q[7]<=0:
                raise ValueError('Frozen reference violates actuator limits')
        self.scorer=scorer_for(donor,self.trajectory)
        self.name=donor.name;self.reference=np.asarray(initial).copy();self.velocity=np.zeros(8)
        self.stage='fixed_playback';self.result=None;self.reason='';self.events=[]
        self.max_force_n=0.;self.max_torque_nm=0.;self.last_physical_row=None
        self.trajectory_sha256=sha256(path)

    def __getattr__(self,name):
        return getattr(self.donor,name)

    def step(self,sample):
        if self.result is not None:return self.reference.copy(),np.zeros(8)
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
        metrics=self.donor.metrics(sample)
        metrics.update(task_version=self.version,fixed_trajectory_sha256=self.trajectory_sha256,
                       independent_evaluation=self.scorer.report())
        return metrics

    def scorer_spec(self):
        s=self.scorer
        return deepcopy(dict(task=s.task,final_tool_position=s.final.tolist(),hole=None if s.hole is None else s.hole.tolist(),
                             hole_rotation=s.hole_rotation.tolist(),tip=s.tip,flange=s.flange,keyed=s.keyed))
