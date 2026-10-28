"""Unscrew a physically threaded bolt, carry it, and screw it into its target.

Multi-turn motion uses the parallel gripper like a human hand: turn, loosen the
grip, return the wrist and grasp again. Only eight actuator references are sent.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from simulation.serial_chain_kinematics import (
    axis_angle_to_matrix, rotation_matrix_to_vector, matrix_to_quaternion_wxyz,
)
from .carry_path import CarryPosePath
from .plug_insert import fixed_site_pose
from .pose_tracking import PoseTargetFilter
from .threaded_motion import (
    ThreadedMotionConfig, ThreadedSample, ThreadedTaskBase, model_body_pose,
)


@dataclass(frozen=True)
class AviationBoltConfig(ThreadedMotionConfig):
    part_body: str = "bolt"
    thread_pitch_m: float = .016  # Lead per revolution of the delivered two-start thread.
    # The 85-mm tip clears the 5-mm mouth after 58 mm. Finish another
    # 0.5 mm along the helix so handover occurs after actual disengagement.
    source_clearance_m: float = .0585
    shaft_tip_local_m: float = .085
    target_clearance_m: float = .060
    # Align the original C-shaped finger notch with the paired head tabs.
    # The old +2 mm height contacted the outer head before entering the notch.
    head_offset_m: float = -.005
    seat_height_m: float = .027
    regrasp_turn_rad: float = math.pi
    finish_turn_margin_rad: float = .5
    regrasp_open_rad: float = .40
    retreat_distance_m: float = .10
    radial_tolerance_m: float = .001
    seat_tolerance_m: float = .0015
    axis_tolerance_rad: float = .035
    rotation_tolerance_rad: float = .15
    # Allow the real tab/thread backlash while requiring most of each held
    # half-turn to be accompanied by measured axial advance (<=1.27 mm lead).
    thread_follow_lead_rad: float = .5

    def __post_init__(self):
        super().__post_init__()
        if self.part_body != "bolt" or not .005 <= self.thread_pitch_m <= .03:
            raise ValueError("Aviation bolt requires the delivered bolt and thread lead")
        if not .2 <= self.regrasp_turn_rad <= math.pi:
            raise ValueError("Regrasp must remain inside one half-turn")
        if not 0. < self.finish_turn_margin_rad <= .5:
            raise ValueError("Final held turn must stay within the checked wrist margin")
        if not 0. < self.regrasp_open_rad <= 1.:
            raise ValueError("Invalid opening for the released wrist return")
        if not .05 <= self.thread_follow_lead_rad <= .5:
            raise ValueError("Thread following lead must stay inside the checked short arc")


class AviationBoltTask(ThreadedTaskBase):
    name = "aviation_bolt_insert"
    version = "aviation-bolt/4-straight-approach"
    stages = (
        "settle", "approach", "grip", "unscrew", "regrasp_release",
        "regrasp_rewind", "regrasp_grip", "carry", "screw_in",
        "release", "retract", "verify",
    )

    @classmethod
    def recorded_stages(cls, version):
        if version not in (cls.version, "aviation-bolt/3-environment-planning", "aviation-bolt/2-latch-smooth-regrasp"):
            raise ValueError(f"Unsupported aviation bolt version {version}")
        return cls.stages

    @classmethod
    def validate_recorded_stages(cls, stages):
        edges = {
            "settle": {"approach"}, "approach": {"grip"}, "grip": {"unscrew"},
            "unscrew": {"regrasp_release", "carry"},
            "regrasp_release": {"regrasp_rewind"},
            "regrasp_rewind": {"regrasp_grip"},
            "regrasp_grip": {"unscrew", "screw_in"},
            "carry": {"screw_in"}, "screw_in": {"regrasp_release", "release"},
            "release": {"retract"}, "retract": {"verify"}, "verify": set(),
        }
        return bool(stages and stages[0] == "settle" and stages[-1] == "verify"
                    and all(b in edges.get(a,set()) for a,b in zip(stages,stages[1:]))
                    and all(s in stages for s in ("unscrew","carry","screw_in","release")))

    def __init__(self, model_path, config=None):
        super().__init__(model_path,config or AviationBoltConfig())
        self.pickup,self.pickup_rotation = fixed_site_pose(self.root,"bolt_storage_hole")
        self.hole,self.hole_rotation = fixed_site_pose(self.root,"bolt_assembly_hole")
        self.outward = self.pickup_rotation[:,2]
        self.target_outward = self.hole_rotation[:,2]
        self.initial_position,self.initial_rotation = model_body_pose(self.root,"bolt")
        self.seated_position = self.hole+self.target_outward*self.config.seat_height_m
        # Delivered female thread starts 2 mm below the male SDF datum at the
        # seated height: 2/16 revolution = +45 degrees (two-start symmetry pi).
        self.seated_rotation = (self.hole_rotation
                                @axis_angle_to_matrix(np.array([0.,0.,1.]),math.pi/4)
                                @np.diag([1.,-1.,-1.]))
        self.source_rotation_rad = self.target_rotation_rad = 0.
        self.source_unscrewed = self.ever_lifted = False
        self._previous_part_rotation = None
        self._phase = "unscrew"
        self._stroke_start = self._stroke_turn = 0.
        self._stroke_regrasp = False
        self._spin_speed = self._spin_acceleration = 0.
        self._spin_goal_position = self._spin_goal_rotation = None
        self._pickup_filter = None
        self._approach_anchor = None
        self._carry_profile = None
        self._release_opening = 0.
        self._wrist_start = None
        self._last_part_rotation = None
        self._target_started = False
        self.carry_thread_symmetry_rad = 0.

    @property
    def source_required_rotation_rad(self):
        return 2*math.pi*self.config.source_clearance_m/self.config.thread_pitch_m

    @property
    def target_required_rotation_rad(self):
        return 2*math.pi*self.config.target_clearance_m/self.config.thread_pitch_m

    def _stroke_angle(self,remaining):
        """Leave a substantial final held turn instead of releasing at exit."""
        half_turn=self.config.regrasp_turn_rad
        if remaining<=half_turn+self.config.finish_turn_margin_rad:
            return remaining
        if remaining<2*half_turn:
            # A full half-turn here can leave less than a millimetre engaged.
            # Split the last two strokes so the release occurs well inside
            # the bore, before the screw becomes free in microgravity.
            return remaining/2
        return half_turn

    def metrics(self,sample):
        offset=sample.part_position_body_m-self.seated_position
        axis=sample.part_rotation_body[:,2]
        depth=float((sample.part_position_body_m-self.initial_position)@self.outward)
        tip=sample.part_position_body_m+sample.part_rotation_body[:,2]*self.config.shaft_tip_local_m
        tip_clearance=float((tip-self.pickup)@self.outward)
        return dict(
            **super().metrics(sample),source_rotation_rad=self.source_rotation_rad,
            target_rotation_rad=self.target_rotation_rad,
            source_required_rotation_rad=self.source_required_rotation_rad,
            target_required_rotation_rad=self.target_required_rotation_rad,
            source_unscrewed=self.source_unscrewed,ever_lifted=self.ever_lifted,
            source_clearance_m=self.config.source_clearance_m,
            target_clearance_m=self.config.target_clearance_m,
            source_displacement_m=depth,thread_pitch_m=self.config.thread_pitch_m,
            source_tip_clearance_m=tip_clearance,
            radial_error_m=float(np.linalg.norm(offset-self.target_outward*(offset@self.target_outward))),
            seat_gap_m=float(offset@self.target_outward),
            axis_error_rad=float(np.arccos(np.clip(-axis@self.target_outward,-1.,1.))),
            target_position_body_m=self.seated_position.tolist(),target_site="bolt_assembly_hole",
            carry_thread_symmetry_rad=self.carry_thread_symmetry_rad,
            commanded_thread_rotation_rad=self._stroke_start+self._stroke_turn,
        )

    def _enter(self,stage,sample):
        self._set_stage(stage,sample)
        p,r=self.chain.forward(self.reference[:6])
        if stage=="approach":
            goal,rotation=self._grasp_pose(sample,offset_local=(0.,0.,self.config.head_offset_m))
            self._pickup_filter=PoseTargetFilter(goal,rotation)
            self._approach_anchor=(goal.copy(),rotation.copy())
            error = None
            try:
                self._plan_pose(goal,rotation,opening=self.config.approach_open_rad,approach=True)
            except ValueError as exc:
                if not self.environment_planning:
                    raise
                error = str(exc)
            if self.environment_planning:
                from .collision_motion import plan_free_motion
                plan_free_motion(self,sample,goal,rotation,-rotation[:,2],template_error=error)
        elif stage in ("grip","regrasp_grip"):
            gp,gr=self._regrasp_pose(sample)
            self._pickup_filter=PoseTargetFilter(gp,gr)
            self._start_opening(0.,speed=self.config.gripper_speed_rad_s)
        elif stage in ("unscrew","screw_in"):
            self._phase=stage
            if stage=="screw_in":self._target_started=True
            self._capture_grasp(sample)
            self._stroke_start=self.source_rotation_rad if stage=="unscrew" else self.target_rotation_rad
            self._stroke_turn=0.
            required = self.source_required_rotation_rad if stage=="unscrew" else self.target_required_rotation_rad
            remaining = max(0., required-self._stroke_start)
            self._stroke_regrasp=remaining>self.config.regrasp_turn_rad+self.config.finish_turn_margin_rad
            self._stroke_goal=self._stroke_angle(remaining)+(0. if self._stroke_regrasp else .04)
            self._spin_speed=self._spin_acceleration=0.
            self._spin_goal_position=sample.part_position_body_m.copy()
            self._spin_goal_rotation=sample.part_rotation_body.copy()
            axis=self.outward if stage=="unscrew" else self.target_outward
            # A tiny carried tilt can offset an 85-mm tip by more than the
            # thread clearance. Preserve measured thread phase, but align the
            # shaft with its bore before following the helix; rotating the
            # measured tilted frame forever caused the tip to jam at entry.
            current_axis=self._spin_goal_rotation[:,2]
            correction=np.cross(current_axis,-axis)
            sine=float(np.linalg.norm(correction))
            if sine>1e-12:
                angle=math.atan2(sine,float(np.clip(current_axis@(-axis),-1.,1.)))
                self._spin_goal_rotation=axis_angle_to_matrix(correction/sine,angle)@self._spin_goal_rotation
            self._wrist_start=r.copy()
        elif stage=="regrasp_release":
            self._release_opening=float(max(sample.joints[6],-sample.joints[7]))
            self._start_opening(self.config.regrasp_open_rad,sample,unload=True)
            # Smoothly remove accumulated servo preload while fingers open.
            goal,rotation=self.chain.forward(sample.joints[:6])
            # The jaw timer below owns opening. Retiming this arm-only unload
            # by the same jaw travel delayed every wrist return after the
            # physical fingers had already cleared the head.
            self._plan_pose(goal,rotation)
        elif stage=="regrasp_rewind":
            # Returning by exactly the physical half-turn preserves equivalent
            # hex-head flats. No ungrasped part rotation is commanded.
            direction=1. if self._phase=="unscrew" else -1.
            axis=self.outward if self._phase=="unscrew" else self.target_outward
            # Paired physical tabs repeat after pi, including when the last
            # two held strokes were shorter. Returning only the preceding
            # partial angle would leave the fingers misaligned with the tabs.
            rewind_angle=math.pi
            nominal=axis_angle_to_matrix(axis,-direction*rewind_angle)@r
            goal_p,goal_r=self._regrasp_pose(sample,nominal)
            correction=rotation_matrix_to_vector(goal_r@nominal.T)
            correction_angle=float(np.linalg.norm(correction))
            def rewind(u):
                blend=u**3*(10.-15.*u+6.*u*u)
                rr=axis_angle_to_matrix(axis,-direction*rewind_angle*u)@r
                if correction_angle>1e-12:
                    rr=axis_angle_to_matrix(correction/correction_angle,blend*correction_angle)@rr
                return p+blend*(goal_p-p),rr
            self._plan_pose(goal_p,goal_r,opening=self.config.regrasp_open_rad,path_fn=rewind,
                            angle=rewind_angle+correction_angle)
        elif stage=="carry":
            self._plan_carry(sample)
        elif stage=="release":
            self._release_opening=float(max(sample.joints[6],-sample.joints[7]))
            self._start_opening(self.config.release_open_rad,sample,unload=True)
            goal,rotation=self.chain.forward(sample.joints[:6])
            self._plan_pose(goal,rotation)
        elif stage=="retract":
            # Retraction now owns both arm and jaw references. A partially
            # consumed release timer must not overwrite the completed opening
            # when verification starts and the arm path stops advancing it.
            self._gripper_timing=None
            self._plan_retract(sample)

    def _integrate_rotation(self,sample):
        if self._previous_part_rotation is not None:
            increment=rotation_matrix_to_vector(sample.part_rotation_body@self._previous_part_rotation.T)
            if not self.source_unscrewed:
                self.source_rotation_rad+=float(increment@self.outward)
            elif self._target_started:
                self.target_rotation_rad-=float(increment@self.target_outward)
        self._previous_part_rotation=sample.part_rotation_body.copy()

    def _regrasp_pose(self,sample,hint=None):
        p,r=self._grasp_pose(sample,offset_local=(0.,0.,self.config.head_offset_m))
        if hint is None:hint=self.chain.forward(sample.joints[:6])[1]
        if r[:,0]@hint[:,0]<0:r=r@np.diag([-1.,-1.,1.])
        return p,r

    def _spin(self,sample,dt):
        source=self.stage=="unscrew"
        axis=self.outward if source else self.target_outward
        origin=self.initial_position if source else self.seated_position+self.target_outward*self.config.target_clearance_m
        actual=self.source_rotation_rad if source else self.target_rotation_rad
        required=self.source_required_rotation_rad if source else self.target_required_rotation_rad
        direction=1. if source else -1.
        remaining=max(0.,required-actual)
        # Brake before every released half-turn, as well as at the final seat.
        # A phase threshold must not turn a moving wrist into a zero-speed path.
        stroke_remaining=max(0.,self._stroke_goal-(actual-self._stroke_start))
        # At the mouth, rotation alone does not prove thread engagement. Keep
        # angular progress near the measured axial advance, so a stalled tip
        # cannot spin through a half-turn above the hole and then be released.
        # This limits the actuator trajectory, not the screw's physical state.
        advance=direction*float((sample.part_position_body_m-origin)@axis)
        axial_turn=advance*2*math.pi/self.config.thread_pitch_m
        phase_remaining=max(0.,axial_turn+self.config.thread_follow_lead_rad
                            -(self._stroke_start+self._stroke_turn))
        desired=min(self.config.rotation_speed_rad_s,2.*stroke_remaining,2.*phase_remaining)
        acceleration=np.clip(
            8.*(desired-self._spin_speed),
            -self.config.rotation_acceleration_rad_s2,
            self.config.rotation_acceleration_rad_s2,
        )
        self._spin_acceleration+=np.clip(
            acceleration-self._spin_acceleration,
            -self.config.rotation_jerk_rad_s3*dt,
            self.config.rotation_jerk_rad_s3*dt,
        )
        self._spin_speed=max(0.,self._spin_speed+self._spin_acceleration*dt)
        commanded=self._spin_speed*dt
        # Keep commanded thread position near measured rotation. A stalled
        # thread never accumulates metres of hidden reference penetration.
        self._stroke_turn=min(self._stroke_turn+commanded,max(0.,actual-self._stroke_start)+.10)
        turn=self._stroke_start+self._stroke_turn
        base_rotation=self._spin_goal_rotation
        rotation=axis_angle_to_matrix(axis,direction*self._stroke_turn)@base_rotation
        travel=self.config.thread_pitch_m*turn/(2*math.pi)
        position=origin+direction*axis*travel
        radial=sample.part_position_body_m-position
        radial-=axis*(radial@axis)
        alignment=float(np.linalg.norm(np.cross(sample.part_rotation_body[:,2],axis)))
        fraction=float(np.clip(max(np.linalg.norm(radial)/.003,alignment/.08),0.,1.))
        fade=1.-fraction*fraction*(3.-2.*fraction)
        omega=direction*axis*self._spin_speed*fade
        speed=direction*axis*self.config.thread_pitch_m*self._spin_speed*fade/(2*math.pi)
        # A free screw must actually advance along the commanded helix. Servo
        # deflection against the thread mouth cannot be treated as achieved
        # motion: otherwise it spins above the hole and drifts on release.
        # Use measured FK here with the stable finite-response reference
        # generator; open-jaw approach/regrasp still follows command space.
        self._track_part(position,rotation,sample,dt,speed,omega,measured_feedback=True)
        metrics=self.metrics(sample)
        if source:
            complete=(actual>=required-.05
                      and metrics["source_tip_clearance_m"]>=.0001)
        else:
            # Release can elastically unwind the loaded thread. Finish the
            # measured turn before opening and keep the wider final tolerance
            # for the genuinely released, stable part.
            complete=(actual>=required-.02
                      and abs(metrics["seat_gap_m"])<self.config.seat_tolerance_m
                      and metrics["radial_error_m"]<self.config.radial_tolerance_m
                      and metrics["axis_error_rad"]<self.config.axis_tolerance_rad)
        stopped=(self._spin_speed < .012 and self._motion_stopped(sample)
                 and np.linalg.norm(sample.part_angular_velocity_body_rad_s)<.025)
        if complete and stopped:
            if source:
                self.source_unscrewed=self.ever_lifted=True
                self._enter("carry",sample)
            else:
                self._enter("release",sample)
        elif (self._stroke_regrasp and actual-self._stroke_start>=self._stroke_goal-.02 and stopped
              and remaining>self.config.finish_turn_margin_rad):
            # Keep supporting a nearly disengaged screw. Opening just before
            # the last few degrees lets the now-free part drift in microgravity.
            # Preflight also checks the bounded extra wrist travel used here.
            self._stroke_turn=actual-self._stroke_start
            self._enter("regrasp_release",sample)

    def step(self,sample:ThreadedSample):
        dt,old=self._step_start(sample)
        if self.result is not None:
            return self._step_finish(sample,old,dt)
        self._integrate_rotation(sample)
        try:
            if self.stage=="pending":
                self.initial_position=sample.part_position_body_m.copy()
                self.initial_rotation=sample.part_rotation_body.copy()
                if np.linalg.norm((self.initial_position-self.pickup)[:2])>.01:
                    self.fail("bolt_not_at_initial_thread",sample)
                else:
                    self._enter("settle",sample)
            elif self.stage=="settle":
                if sample.time_s-self._entered>=self.config.initial_settle_s:
                    self._enter("approach",sample)
            elif self.stage=="approach":
                done=self._path_step(sample,dt)
                # A timed path owns this reference until it stops. Applying a
                # feedback integrator after resetting it to the next path point
                # re-applied the path velocity and created large oscillations.
                # The open-jaw grip phase follows the measured settled thread
                # pose before the fingers reach the tabs.
                if done:self._enter("grip",sample)
            elif self.stage in ("grip","regrasp_grip"):
                self._advance_opening(dt)
                p,r=self.chain.forward(sample.joints[:6])
                goal,gr=self._regrasp_pose(sample)
                goal,gr=self._pickup_filter.update(goal,gr,dt)
                # Contact count may change on successive physics samples.
                # Skipping the arm update on a bilateral sample set a moving
                # reference to zero for one tick and caused the visible twitch.
                self._follow_pose(goal,gr,sample,dt)
                speed=np.linalg.norm(self.chain.jacobian(sample.joints[:6])[:3]@sample.joint_velocities[:6]-sample.part_velocity_body_m_s)
                heading=abs(r[:,0]@sample.part_rotation_body[:,1])
                held=(self._latch_engaged(sample)
                      and np.linalg.norm(p-goal)<.005 and heading>math.cos(.06) and speed<.015)
                self._stable=self._stable+dt if held else 0.
                if self._stable>=self.config.grip_confirm_s:
                    self._enter("unscrew" if self.stage=="grip" else self._phase,sample)
            elif self.stage in ("unscrew","screw_in"):
                self._spin(sample,dt)
            elif self.stage in ("regrasp_release","release"):
                unloaded=self._path_step(sample,dt)
                self._advance_opening(dt)
                if self.stage=="regrasp_release":
                    # A private native sweep found .3551 rad clears every hex
                    # corner; .40 adds .0449 rad without the full-open sweep.
                    opening_clear=min(sample.joints[6],-sample.joints[7])>=self.config.regrasp_open_rad-.01
                else:
                    opening_clear=max(sample.joints[6],-sample.joints[7])>self._release_opening+.10
                clear=(sample.finger_part_contacts is not None and not any(sample.finger_part_contacts)
                       and opening_clear)
                self._stable=self._stable+dt if clear else 0.
                if self._stable>=.1 and unloaded and self._motion_stopped(sample):
                    self._enter("regrasp_rewind" if self.stage=="regrasp_release" else "retract",sample)
            elif self.stage=="regrasp_rewind":
                if self._path_step(sample,dt):self._enter("regrasp_grip",sample)
            elif self.stage=="carry":
                if self._path_step(sample,dt):self._enter("screw_in",sample)
            elif self.stage=="retract":
                if self._path_step(sample,dt):self._enter("verify",sample)
            elif self.stage=="verify":
                self._advance_opening(dt)
                m=self.metrics(sample)
                seated=(self.source_unscrewed and self.ever_lifted
                        and self.source_rotation_rad>=self.source_required_rotation_rad-self.config.rotation_tolerance_rad
                        and self.target_rotation_rad>=self.target_required_rotation_rad-self.config.rotation_tolerance_rad
                        and abs(m["seat_gap_m"])<self.config.seat_tolerance_m
                        and m["radial_error_m"]<self.config.radial_tolerance_m
                        and m["axis_error_rad"]<self.config.axis_tolerance_rad
                        and np.linalg.norm(sample.part_velocity_body_m_s)<.01
                        and np.linalg.norm(sample.part_angular_velocity_body_rad_s)<.03
                        and sample.finger_part_contacts==(0,0))
                self._stable=self._stable+dt if seated else 0.
                if self._stable>=self.config.verify_hold_s:self._succeed(sample)
        except (ValueError,np.linalg.LinAlgError) as exc:
            self.fail(f"motion_planning_failed: {exc}",sample)
        return self._step_finish(sample,old,dt)

    def _plan_carry_phase(self,sample,symmetry):
        """Plan one physically equivalent thread phase without changing state."""
        position=self.seated_position+self.target_outward*self.config.target_clearance_m
        rotation=(axis_angle_to_matrix(self.target_outward,
                    self.target_required_rotation_rad+symmetry)@self.seated_rotation)
        relp,relr=self._grasp_transform
        self._carry_profile=CarryPosePath(sample.part_position_body_m,sample.part_rotation_body,
                                          position,rotation,self.config.transport_height_m)
        def route(u):
            pp,rr=self._carry_profile.pose(u)
            tr=rr@relr.T
            return pp-tr@relp,tr
        error = None
        try:
            self._plan_pose(*route(1.),path_fn=route,distance=self._carry_profile.length,
                            angle=1.875*self._carry_profile.angle)
        except ValueError as exc:
            if not self.environment_planning:
                raise
            error = str(exc)
        if self.environment_planning:
            from .collision_motion import plan_free_motion
            plan_free_motion(self,sample,*route(1.),self.target_outward,
                             held=True,template_error=error)

    def _record_preflight_path(self,stage,body_pose,held=False,grasp_transform=None):
        from simulation.serial_chain_kinematics import quaternion_wxyz_to_matrix
        from tools.captive_fastener import body_frames

        bus_position,bus_quaternion=body_frames(self.root)["cubesat_bus"]
        bus_rotation=quaternion_wxyz_to_matrix(bus_quaternion)
        if hasattr(self,"_planning_bus_pose"):
            bus_position,bus_rotation=self._planning_bus_pose
        references=np.array([self.planned_path_reference(t) for t in np.linspace(0.,self._duration,241)])
        start_p,start_r=self.chain.forward(self.reference[:6])
        relp=start_r.T@(body_pose[0]-start_p)
        relr=start_r.T@body_pose[1]
        if grasp_transform is not None:
            relp,relr=grasp_transform
        poses=[]
        for ref in references:
            if held:
                pp,rr=self.chain.forward(ref[:6])
                bp,br=pp+rr@relp,rr@relr
            else:
                bp,br=body_pose
            poses.append(np.r_[bus_position+bus_rotation@bp,
                                matrix_to_quaternion_wxyz(bus_rotation@br)])
        self.reference[:]=references[-1]
        if held:
            pp,rr=self.chain.forward(self.reference[:6])
            body_pose=(pp+rr@relp,rr@relr)
        return (stage,references,("bolt",),{"bolt_freejoint":np.asarray(poses)}),body_pose

    def _thread_preflight_paths(self,prefix,axis,total,direction,held_opening,body_pose):
        paths=[]
        progress=0.
        def record(stage,held=False):
            nonlocal body_pose
            planned,body_pose=self._record_preflight_path(stage,body_pose,held)
            paths.append(planned)
        while progress<total-1e-8:
            remaining=total-progress
            turn=self._stroke_angle(remaining)
            def plan_turn(angle,label):
                distance=self.config.thread_pitch_m*angle/(2*math.pi)
                p,r=self.chain.forward(self.reference[:6])
                def helix(u):
                    return (p+direction*axis*distance*u,
                            axis_angle_to_matrix(axis,direction*u*angle)@r)
                self._plan_pose(*helix(1.),path_fn=helix,distance=distance,angle=angle,opening=held_opening)
                record(label,held=True)
            continuation=min(remaining,turn+self.config.finish_turn_margin_rad)
            if continuation>turn+1e-8:
                before_reference=self.reference.copy()
                before_body=tuple(value.copy() for value in body_pose)
                plan_turn(continuation,prefix+"_finish_envelope")
                self.reference[:]=before_reference
                body_pose=before_body
            plan_turn(turn,prefix+"_turn")
            progress+=turn
            if progress<total-1e-8:
                p,r=self.chain.forward(self.reference[:6])
                self._plan_pose(p,r,opening=self.config.regrasp_open_rad)
                record(prefix+"_open")
                rotation=axis_angle_to_matrix(axis,-direction*math.pi)@r
                def rewind(u):return p,axis_angle_to_matrix(axis,-direction*math.pi*u)@r
                self._plan_pose(p,rotation,opening=self.config.regrasp_open_rad,path_fn=rewind,angle=math.pi)
                record(prefix+"_rewind")
                self._plan_pose(p,rotation,opening=held_opening)
                record(prefix+"_regrip")
        return paths,body_pose

    def _carry_candidate_paths(self,sample,symmetry):
        # Only this separate planning object changes its references/mechanism
        # poses. Runtime continues to send actuator references exclusively.
        probe=type(self)(self.model_path,self.config)
        probe.chain=self.chain
        self._candidate_motion_plans=probe.motion_plans
        probe.environment_planning=self.environment_planning
        if hasattr(self,"_planning_native_state"):
            probe._planning_native_state=self._planning_native_state
        probe.reference[:]=self.reference
        # A zero servo preload command does not close the physical jaws through
        # the hex head. Only this geometry probe uses their measured opening.
        probe.reference[6:]=sample.joints[6:]
        probe._capture_grasp(sample)
        probe._plan_carry_phase(sample,symmetry)
        body_pose=(sample.part_position_body_m.copy(),sample.part_rotation_body.copy())
        carry,body_pose=probe._record_preflight_path("carry",body_pose,held=True,
                                                     grasp_transform=probe._grasp_transform)
        carry[1][:,6:]=sample.joints[6:]
        probe.reference[6:]=sample.joints[6:]
        opening=float(max(sample.joints[6],-sample.joints[7]))
        target,body_pose=probe._thread_preflight_paths("target",probe.target_outward,
                    probe.target_required_rotation_rad,-1.,opening,body_pose)
        release=probe._release_retract_preflight_paths(body_pose)
        return [carry,*target,*release]

    def _plan_retract(self,sample):
        """Withdraw upward, checking the complete open-jaw sweep privately."""
        from .drawer_initialization import NativePoseProbe

        p,r=self.chain.forward(self.reference[:6])
        side=r[:,1]-self.target_outward*(r[:,1]@self.target_outward)
        side/=np.linalg.norm(side)
        directions=(self.target_outward,self.target_outward+side,self.target_outward-side)
        geometry=NativePoseProbe(self.model_path)
        selected=None
        self.retract_branch_checks=[]
        try:
            if sample.native_qpos is not None:
                self._planning_native_state=(sample.native_qpos.copy(),sample.joints.copy())
            state=getattr(self,"_planning_native_state",None)
            if state is not None:
                geometry.use_measured_qpos(*state)
            if self.environment_planning:
                geometry.contacts(self.reference)
                self._planning_bus_pose=geometry.body_pose("cubesat_bus")
            for direction in directions:
                direction=direction/np.linalg.norm(direction)
                try:
                    private=type(self)(self.model_path,self.config)
                    private.chain=self.chain
                    if hasattr(self,"_planning_bus_pose"):
                        private._planning_bus_pose=self._planning_bus_pose
                    private.reference[:]=self.reference
                    private.reference[6:]=sample.joints[6:]
                    private._plan_pose(p+self.config.retreat_distance_m*direction,r,
                                       opening=self.config.release_open_rad)
                    body_pose=(sample.part_position_body_m.copy(),sample.part_rotation_body.copy())
                    path,_=private._record_preflight_path("retract",body_pose)
                    if sample.native_qpos is not None:
                        # Its actual world pose is already in the measured
                        # baseline; nominal model bus coordinates differ after
                        # six minutes of real free-flying dynamics.
                        path=(path[0],path[1],path[2],{})
                    result=geometry.check_threaded_paths(self.name,[path])
                except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
                    result=dict(accepted=False,reason="planning_failed",detail=str(exc))
                self.retract_branch_checks.append(dict(direction_body=direction.tolist(),**result))
                if result["accepted"]:
                    selected=direction
                    break
        finally:
            geometry.close()
        if self.events and self.events[-1]["stage"]=="retract":
            self.events[-1]["retract_branch_checks"]=self.retract_branch_checks
        if selected is None and not self.environment_planning:
            raise ValueError("No collision-free upward threaded-part withdrawal")
        if selected is None:
            selected=self.target_outward
            template_error="upward_candidates_blocked"
        else:
            template_error=None
        try:
            self._plan_pose(p+self.config.retreat_distance_m*selected,r,
                            opening=self.config.release_open_rad)
        except ValueError as exc:
            if not self.environment_planning:
                raise
            template_error=str(exc)
        if self.environment_planning:
            from .collision_motion import plan_free_motion
            plan_free_motion(self,sample,p+self.config.retreat_distance_m*selected,r,
                             -self.target_outward,template_error=template_error,
                             opening=self.config.release_open_rad,stage="retract")

    def _release_retract_preflight_paths(self,body_pose):
        # Runtime begins withdrawal as soon as actual fingers clear the head,
        # then continues opening during the same uninterrupted upward motion.
        p,r=self.chain.forward(self.reference[:6])
        opening=min(self.config.release_open_rad,float(self.reference[6])+.11)
        self._plan_pose(p,r,opening=opening)
        release,body_pose=self._record_preflight_path("target_release",body_pose)
        sample=ThreadedSample(0.,self.reference.copy(),np.zeros(8),*body_pose,
                              np.zeros(3),np.zeros(3),np.zeros(6),True,(0,0))
        self._plan_retract(sample)
        retract,_=self._record_preflight_path("retract",body_pose)
        return [release,retract]

    def _plan_carry(self,sample,check_all=False):
        from .drawer_initialization import NativePoseProbe

        self._capture_grasp(sample)
        self.carry_branch_checks=[]
        selected=None
        geometry_probe=NativePoseProbe(self.model_path)
        try:
            if sample.native_qpos is not None:
                geometry_probe.use_measured_qpos(sample.native_qpos,sample.joints)
            for symmetry in (0.,math.pi):
                try:
                    paths=self._carry_candidate_paths(sample,symmetry)
                    result=geometry_probe.check_threaded_paths(self.name,paths)
                except (ValueError,RuntimeError,np.linalg.LinAlgError) as exc:
                    result=dict(accepted=False,reason="planning_failed",detail=str(exc),
                                planning_failure=getattr(exc,"report",None))
                result["motion_plans"]=getattr(self,"_candidate_motion_plans",[])
                self.carry_branch_checks.append(dict(symmetry_rad=symmetry,**result))
                if result["accepted"]:
                    if selected is None:selected=symmetry
                    if not check_all:break
        finally:
            geometry_probe.close()
        if self.events and self.events[-1]["stage"]=="carry":
            self.events[-1]["carry_branch_checks"]=self.carry_branch_checks
        if selected is None:
            failures="; ".join(f"phase {r['symmetry_rad']:.3f}: {r.get('detail',r['reason'])}"
                               for r in self.carry_branch_checks)
            raise ValueError("No collision-free reachable thread transport phase: "+failures)
        self._plan_carry_phase(sample,selected)
        self.carry_thread_symmetry_rad=selected

    def preflight_paths(self,initial_q):
        from .drawer_initialization import NativePoseProbe

        probe=type(self)(self.model_path,self.config)
        self.preflight_motion_plans=probe.motion_plans
        probe.environment_planning=self.environment_planning
        probe.chain=self.chain
        sample=ThreadedSample(0.,np.asarray(initial_q).copy(),np.zeros(8),
                              probe.initial_position,probe.initial_rotation,np.zeros(3),np.zeros(3),np.zeros(6),True)
        probe.reference[:]=initial_q
        probe._enter("approach",sample)
        body_pose=(probe.initial_position.copy(),probe.initial_rotation.copy())
        approach,body_pose=probe._record_preflight_path("approach",body_pose)
        paths=[approach]
        geometry_probe=NativePoseProbe(probe.model_path)
        try:
            held_opening=geometry_probe.grasp_opening(probe.reference,"bolt")
        finally:
            geometry_probe.close()
        p,r=probe.chain.forward(probe.reference[:6])
        probe._plan_pose(p,r,opening=held_opening)
        grip,body_pose=probe._record_preflight_path("source_grip",body_pose)
        paths.append(grip)
        source,body_pose=probe._thread_preflight_paths("source",probe.outward,
                    probe.source_required_rotation_rad,1.,held_opening,body_pose)
        paths.extend(source)
        moved=ThreadedSample(0.,probe.reference.copy(),np.zeros(8),body_pose[0],body_pose[1],
                             np.zeros(3),np.zeros(3),np.zeros(6),True)
        try:
            probe._plan_carry(moved,check_all=True)
        finally:
            self.preflight_carry_branches=probe.carry_branch_checks
            self.preflight_motion_plans=probe.motion_plans
        self.preflight_carry_symmetry_rad=probe.carry_thread_symmetry_rad
        paths.extend(probe._carry_candidate_paths(moved,probe.carry_thread_symmetry_rad))
        return paths
