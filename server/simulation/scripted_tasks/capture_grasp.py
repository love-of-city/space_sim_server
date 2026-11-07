"""State-driven capture expert; real joint servos and contact feedback only.

No image estimator or spacecraft approach guidance is used. Privileged target
poses belong to this expert/validator and are not policy observation features.
"""
from dataclasses import dataclass, asdict
import os
import numpy as np

from simulation.capture_arm_target import CaptureArmTarget
from simulation.serial_chain_kinematics import rotation_matrix_to_vector

TASK = 'capture_target_grasp'
VERSION = 'capture-target-grasp/5-preferred-handoff-branch'
UNBOUNDED_DIAGNOSTIC_ENV = 'SPACE_SIM_UNBOUNDED_DIAGNOSTIC'
JOINT_NAMES = tuple([f'bottom_joint{i}_rad' for i in range(1, 7)] +
                    ['axial_m', 'cam_angle_rad'] + [f'jaw_slide_{i}_m' for i in range(4)])
INTERFACE_CENTER = np.array([.1755, .05475, .0420787001299893])
TARGET_FROM_INTERFACE = np.array([[1., 0, 0], [0, 0, 1], [0, -1, 0]])
PREPARED_ARM = np.array([1.5548788018436606, -2.3516604215016366,
    1.2054740957705201, .7971602513859034, 1.565349712704783, .014948273804433395])


def unbounded_diagnostic_enabled():
    """Enable only for controller diagnosis; keep model physics authoritative."""
    return os.environ.get(UNBOUNDED_DIAGNOSTIC_ENV, '').strip().lower() in {'1', 'true', 'yes'}


@dataclass(frozen=True)
class CaptureGraspConfig:
    clearance_m: float = .060
    seated_gap_m: float = .006
    # Use a faster bounded traverse while the tool is still far from the
    # capture interface.  The final approach keeps the lower limit so contact
    # is still made gently instead of carrying the align-stage crawl speed all
    # the way from the initial gap.
    align_linear_speed_m_s: float = .012
    # Keep the final approach cap equal to the traverse cap.  The P controller
    # already reduces the command as the seated gap closes; dropping the cap
    # at the align/approach handoff creates a visible one-frame hesitation.
    linear_speed_m_s: float = .012
    align_angular_speed_rad_s: float = .08
    angular_speed_rad_s: float = .08
    align_handoff_distance_m: float = .010
    # Keep very small initial offsets on the original exact alignment gate;
    # the early handoff is for the normal multi-centimetre traverse only.
    align_handoff_min_distance_m: float = .002
    position_tolerance_m: float = .0001
    angle_tolerance_rad: float = .003
    required_hold_s: float = 20.
    jaw_force_min_n: float = 4.5
    jaw_force_max_n: float = 5.5
    peak_contact_limit_n: float = 10.
    maximum_sim_seconds: float = 1200.
    maximum_capture_seconds: float = 360.
    handoff_position_tolerance_m: float = .005
    handoff_angle_tolerance_rad: float = .017453292519943295


class CaptureGraspTask:
    name = TASK
    version = VERSION

    def __init__(self, model, config=None, *, initial_joints=None):
        self.model = model
        self.config = config or CaptureGraspConfig()
        self.target = CaptureArmTarget(model)
        initial_joints = np.asarray(PREPARED_ARM if initial_joints is None else initial_joints)
        if (initial_joints.shape != (6,) or not np.isfinite(initial_joints).all()
                or np.any(initial_joints < self.target.lower[:6])
                or np.any(initial_joints > self.target.upper[:6])):
            raise ValueError('capture initial arm posture exceeds model joint limits')
        self.target.position[:6] = initial_joints
        # Open and retracted at reset, without starting in contact.
        self.target.position[6:] = [-.006, 0., 0., 0., 0., 0.]
        self.chain = self.target.kinematics
        self.lower, self.upper = self.target.lower, self.target.upper
        self.reference = self.target.position.copy()
        self.stage, self.result, self.reason = 'settle', None, None
        self.events = []
        self.start = 0.
        self.stable_since = None
        self.hold_s = 0.
        self.previous_t = None
        self.peak_force_n = 0.
        self.ever_self_contact = False
        self.contact_offset = None
        self.tracking = {}
        self.carry_timing = None
        self.carry_plan = None
        self.handoff = {}
        self.grasp_hold_s = 0.
        self.unbounded_diagnostic = unbounded_diagnostic_enabled()

    def transition(self, stage, t):
        self.stage, self.start = stage, t
        self.events.append(dict(stage=stage, elapsed_sim_seconds=t))

    def fail(self, reason, sample=None):
        self.result, self.reason = 'failed', reason
        self.transition('abort', 0. if sample is None else sample['t'])

    def metrics(self):
        return dict(name=self.name, task_version=self.version, stage=self.stage,
                    result=self.result, reason=self.reason, stable_hold_s=self.hold_s,
                    peak_contact_force_n=self.peak_force_n,
                    self_contact=self.ever_self_contact, task_parameters=asdict(self.config),
                    tracking=self.tracking, grasp_hold_s=self.grasp_hold_s,
                    handoff=self.handoff, carry_plan=(None if self.carry_plan is None else
                        {k:v for k,v in self.carry_plan.items() if k != 'path_waypoints'}))

    def step(self, sample):
        t, q = sample['t'], sample['q']
        dt = 0. if self.previous_t is None else t-self.previous_t
        self.previous_t = t
        f = sample['feedback']
        self.peak_force_n = max(self.peak_force_n, f.get('peak_target_contact_force_n', 0.))
        self.ever_self_contact |= bool(sample.get('unplanned_contacts'))
        if self.result is None:
            if self.ever_self_contact:
                self.fail('unplanned_collision', sample)
            elif self.peak_force_n > self.config.peak_contact_limit_n:
                self.fail('target_contact_force_over_10n', sample)
            elif f.get('pad_compression_m', 0.) > .002:
                self.fail('pad_compression_over_2mm', sample)
            elif t > self.config.maximum_sim_seconds:
                self.fail('total_simulation_timeout', sample)
            elif t > self.config.maximum_capture_seconds and self.stage in ('settle','align','approach','close','tighten'):
                self.fail('capture_phase_timeout',sample)
        action = dict(deadman=True, active_arm='capture',
            end_effector_linear_velocity_body_m_s=[0., 0., 0.],
            end_effector_angular_velocity_body_rad_s=[0., 0., 0.],
            capture_radial_velocity_m_s=0., capture_axial_velocity_m_s=0.)
        if self.result is not None:
            return self.reference.copy(), np.zeros(12)
        p, r = self.chain.forward(q[:6])
        pi, ri = sample['interface_position'], sample['interface_rotation']
        offset = ri.T @ (p-pi)
        if self.contact_offset is None and f.get('peak_target_contact_force_n', 0.) > .05:
            self.contact_offset = offset.copy()
        if self.contact_offset is not None and np.linalg.norm(offset-self.contact_offset) > .05:
            self.fail('relative_tool_displacement_after_contact_over_50mm', sample)
            return self.reference.copy(), np.zeros(12)
        omega = rotation_matrix_to_vector(ri @ r.T)
        cfg = self.config
        if self.stage == 'settle':
            if t >= .5:
                self.transition('align', t)
        elif self.stage in ('align', 'approach'):
            gap = cfg.clearance_m if self.stage == 'align' else cfg.seated_gap_m
            error = ri @ (np.array([0., 0., gap])-offset)
            interface_velocity = np.asarray(sample['interface_linear_velocity'])
            interface_omega = np.asarray(sample['interface_angular_velocity'])
            goal_velocity = interface_velocity + np.cross(interface_omega, ri @ [0., 0., gap])
            linear = goal_velocity + .8*error
            if not self.unbounded_diagnostic:
                linear_limit = (cfg.align_linear_speed_m_s if self.stage == 'align'
                                else cfg.linear_speed_m_s)
                linear *= min(1., linear_limit/max(np.linalg.norm(linear), 1e-12))
            angular = interface_omega + .8*omega
            if not self.unbounded_diagnostic:
                angular_limit = (cfg.align_angular_speed_rad_s if self.stage == 'align'
                                 else cfg.angular_speed_rad_s)
                angular *= min(1., angular_limit/max(np.linalg.norm(angular), 1e-12))
            self.tracking = dict(position_error_m=float(np.linalg.norm(error)),
                orientation_error_rad=float(np.linalg.norm(omega)),
                goal_velocity_body_m_s=goal_velocity.tolist(),
                commanded_velocity_body_m_s=linear.tolist())
            action['end_effector_linear_velocity_body_m_s'] = linear.tolist()
            action['end_effector_angular_velocity_body_rad_s'] = angular.tolist()
            aligned = (np.linalg.norm(error) < cfg.position_tolerance_m and
                       np.linalg.norm(omega) < cfg.angle_tolerance_rad)
            # Begin the seated approach before the clearance waypoint is
            # reached.  This changes the goal gap while the tool is still
            # moving, so the old clearance target cannot produce a stop.
            handoff = (self.stage == 'align' and
                       cfg.align_handoff_min_distance_m < np.linalg.norm(error) <
                       cfg.align_handoff_distance_m and
                       np.linalg.norm(omega) < cfg.angle_tolerance_rad)
            if aligned:
                self.transition('approach' if self.stage == 'align' else 'close', t)
            elif handoff:
                self.transition('approach', t)
            elif self.stage == 'approach' and f.get('jaw_support_count', 0) > 0 and offset[2] > .012:
                self.fail('jaw_contact_before_seated', sample)
        elif self.stage == 'close':
            action['capture_radial_velocity_m_s'] = -.002
            if np.max(np.abs(q[8:]+.012)) < .0002:
                self.transition('tighten', t)
            elif t-self.start > 12.:
                self.fail('jaw_close_timeout', sample)
        elif self.stage in ('tighten', 'hold', 'pull', 'handoff_hold'):
            # Same admittance principle as the platform's existing capture tool.
            axial = .0002*(5.-f['pad_force_n'])
            action['capture_axial_velocity_m_s'] = float(
                axial if self.unbounded_diagnostic else np.clip(axial, -.0005, .0005))
            supported = f.get('jaw_support_count', 0) == 4
            stable = supported and cfg.jaw_force_min_n <= f.get('jaw_force_n', 0.) <= cfg.jaw_force_max_n
            if self.stage in ('pull','handoff_hold'):
                from .capture_carry import handoff_error
                pe, ae = handoff_error(sample)
                self.handoff = dict(target_position_error_m=pe,target_angle_error_rad=ae,
                    arm_reference_error_rad=float(np.max(np.abs(q[:6]-self.reference[:6]))))
                if self.stage == 'pull':
                    if not supported:
                        self.fail('capture_support_lost_during_pull',sample)
                    elif t-self.start >= self.carry_timing.duration:
                        self.stable_since = None; self.hold_s = 0.
                        self.transition('handoff_hold',t)
                else:
                    arrived = pe <= cfg.handoff_position_tolerance_m and ae <= cfg.handoff_angle_tolerance_rad
                    if stable and arrived:
                        if self.stable_since is None: self.stable_since = t
                        self.hold_s = t-self.stable_since
                        if self.hold_s >= cfg.required_hold_s:
                            self.result,self.reason = 'success','captured_and_delivered_to_operating_pose'
                    else:
                        self.stable_since = None; self.hold_s = 0.
                    if t-self.start > 60.:
                        self.fail('handoff_pose_or_hold_timeout',sample)
            elif stable:
                if self.stable_since is None:
                    self.stable_since = t
                self.hold_s = t-self.stable_since
                if self.stage != 'hold':
                    self.transition('hold', t)
                if self.hold_s >= cfg.required_hold_s:
                    self.grasp_hold_s = self.hold_s
                    try:
                        from .capture_carry import plan_capture_carry
                        self.carry_timing,self.carry_plan = plan_capture_carry(self.model,self,sample)
                        self.transition('pull',t)
                    except ValueError as error:
                        self.carry_plan = getattr(error,'report',dict(accepted=False,reason=str(error)))
                        self.fail('capture_handoff_planning_failed',sample)
            else:
                self.stable_since, self.hold_s = None, 0.
                if self.stage == 'hold':
                    self.transition('tighten', t)
            if t-self.start > 60. and self.stage == 'tighten':
                self.fail('force_hold_not_established', sample)
        if self.result is not None:
            return self.reference.copy(), np.zeros(12)
        self.reference[:], velocity = self.target.step(dt, action, False, q)
        if self.stage in ('pull','handoff_hold'):
            seconds = t-self.start if self.stage == 'pull' else self.carry_timing.duration
            arm,rate,_,_ = self.carry_timing.joint_state(seconds)
            self.reference[:6] = arm
            self.target.position[:6] = arm
            velocity[:6] = rate
        return self.reference.copy(), velocity
