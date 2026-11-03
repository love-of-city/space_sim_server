"""Connect upstream native planning to locally validated contact controllers.

Planning changes actuator references only. The source model, retention physics,
raw wrench and scene benchmark overload stop stay unchanged.
"""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from .scene_plug import PlugInsertTask
from .motion_timing import time_joint_path
from .environment_motion import plan
from .operation_clearance import check_operation
from .drawer_initialization import NativePoseProbe
from .plug_clearance import cylindrical_release_clearance,cylindrical_retract_candidates,plan_cylindrical_retract,plug_hold_opening


class EnvironmentPlugTask(PlugInsertTask):
    def __init__(self,model_path,config=None):
        super().__init__(model_path,config)
        self.model_path=Path(model_path)
        self.environment_planning=True
        self.motion_plans=[]
        self._timing=None
        self._initial_checked=False
        self.chain.enable_native_geometry()

    def path_reference(self,progress):
        q=self._timing.path.at(progress)
        opening=float(q[6]) if len(q)==7 else float(self.reference[6])
        return np.r_[q[:6],opening,-opening]

    def _plan(self,goal,rotation):
        # Keep the intended endpoint even when the nominal IK path fails.
        self._goal=np.asarray(goal).copy();self._rotation=np.asarray(rotation).copy()
        super()._plan(goal,rotation)
        old_duration=self._duration
        poses=[self.chain.forward(q)[0] for q in self._path]
        distance=float(np.linalg.norm(np.diff(poses,axis=0),axis=1).sum())
        timing=time_joint_path(self._path,distance,0.,joint_speed=self.config.arm_speed_rad_s,
            linear_speed=self.config.retract_speed_m_s if self.stage=='retract' else self.config.linear_speed_m_s,
            angular_speed=.2,linear_acceleration=.15,angular_acceleration=.4,joint_acceleration=.6,
            linear_jerk=.8,angular_jerk=1.6,joint_jerk=8.)
        if timing.duration<old_duration:
            scale=old_duration/timing.duration
            timing=replace(timing,times=timing.times*scale,velocity=timing.velocity/scale)
        self._timing=timing;self._duration=timing.duration

    def _follow_pickup(self,sample,dt,previous):
        # The checked spline must be the executed spline. Keep the existing
        # measured grasp tracker only during stationary opening/gripping.
        if self._path is None:super()._follow_pickup(sample,dt,previous)

    def _check_opening(self,sample):
        probe=NativePoseProbe(self.model_path,clearance_m=.002)
        try:
            probe.use_measured_qpos(sample.native_qpos,sample.joints)
            for opening in np.linspace(max(sample.joints[6],-sample.joints[7]),1.,33):
                q=sample.joints.copy();q[6:]=[opening,-opening]
                hit=probe.pose_collision(q,'free_motion',arm_penetration_m=0.,clearance_m=.002)
                if hit:raise ValueError('opening_corridor_blocked: '+str(hit))
        finally:probe.close()

    def _enter(self,stage,sample):
        if stage=='retract' and self.profile.prefix=='cylindrical' and sample.native_qpos is not None:
            if not cylindrical_release_clearance(self,sample):return
        self._timing=None
        error=None
        free=stage in ('raise','above_pickup','align','descend','transfer','above_hole','retract')
        try:super()._enter(stage,sample)
        except ValueError as exc:
            if not free:raise
            error=str(exc);self._timeout=max(30.,getattr(self,'_duration',0.)*2.5+15.)
        if sample.native_qpos is None:return
        if stage=='grip':self._closed_reference=plug_hold_opening(self,sample)
        if stage=='open':self._check_opening(sample)
        if stage in ('lift','insert'):
            proxy=SimpleNamespace(**vars(self))
            # The legacy lift is one continuous axial extraction/raise.
            proxy.stage='extract' if stage=='lift' else stage
            proxy.name=self.name;proxy.chain=self.chain;proxy.metrics=self.metrics
            proxy._extract_z=self._transport_coordinate(pickup=True)
            proxy._extract_rotation=self.pickup_rotation @ np.diag([1., -1., -1.])
            proxy._axis_goal=self._axis_goal
            check_operation(proxy,sample)
        if free:
            candidates=None
            if stage=='retract' and self.profile.prefix=='cylindrical':
                candidates=cylindrical_retract_candidates(self)
                self._goal=candidates[0]['position'];self._rotation=candidates[0]['rotation']
                try:plan_cylindrical_retract(self,sample,candidates)
                except ValueError as exc:error=str(exc)
            if self._goal is None:raise ValueError('Missing intended free-motion goal')
            report=plan(self,sample,self._goal,self._rotation,
                -self._rotation[:,2] if stage!='retract' else self._rotation[:,2],
                held=stage in ('transfer','above_hole'),error=error,
                opening=float(self.reference[6]),path_reference=self.path_reference,
                fallback_positions=None if candidates is None else [c['position'] for c in candidates])
            if report['route']=='searched':self._goal,self._rotation=self._goal.copy(),self._rotation.copy()

    def step(self,sample):
        if not self._initial_checked and sample.native_qpos is not None:
            probe=NativePoseProbe(self.model_path)
            try:
                probe.use_measured_qpos(sample.native_qpos,sample.joints)
                hit=probe.pose_collision(sample.joints,'free_motion')
                if hit:self.fail('initial_collision: '+str(hit),sample)
            finally:probe.close()
            self._initial_checked=True
        return super().step(sample)

    def metrics(self,sample):
        plans=getattr(self,'motion_plans',[])
        return dict(super().metrics(sample),motion_plan_count=len(plans),
                    last_motion_route=plans[-1].get('route') if plans else None)
