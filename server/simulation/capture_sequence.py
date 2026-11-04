"""Opt-in contact experiment: actuator commands only, no body constraints."""
import numpy as np
import json
import time
from pathlib import Path
from simulation.serial_chain_kinematics import rotation_matrix_to_vector


class CaptureContactGuard:
    """Observe contact disturbance without treating free-base recoil as impact.

    The displacement is a relative body-origin diagnostic, not target-only
    inertial displacement. Arm the 50 mm limit at the first force-bearing
    cross-spacecraft contact; keep the per-contact 10 N limit active throughout.
    """
    def __init__(self):
        self.first_contact_time = None
        self.contact_baseline = None
        self.last_displacement_m = 0.

    def observe(self, t, relative_body_m, peak_force_n):
        p = np.asarray(relative_body_m, dtype=float)
        if (p.shape != (3,) or not np.all(np.isfinite(p))
                or not np.isfinite(t) or not np.isfinite(peak_force_n)):
            return dict(reason='invalid_capture_guard_feedback')
        force = abs(float(peak_force_n))
        if force > 1e-5 and self.contact_baseline is None:
            self.first_contact_time = float(t)
            self.contact_baseline = p.copy()
        if force > 10.:
            return dict(reason='target_contact_force_over_10n', peak_contact_force_n=force)
        if self.contact_baseline is not None:
            self.last_displacement_m = float(np.linalg.norm(p-self.contact_baseline))
            if self.last_displacement_m > .05:
                return dict(reason='post_contact_relative_displacement_over_50mm',
                            displacement_m=self.last_displacement_m)
        return None


class CaptureSequence:
    def __init__(self, force_control=False):
        self.phase = 'wait'
        self.start = 0.
        self.force_control = force_control
        self.stable_since = None
        self.failure_reason = None

    def command(self, t, q, offset, orientation_error, feedback=None):
        a = dict(deadman=True, active_arm='capture',
                 end_effector_linear_velocity_body_m_s=[0., 0., 0.],
                 end_effector_angular_velocity_body_rad_s=[0., 0., 0.],
                 capture_radial_velocity_m_s=0., capture_axial_velocity_m_s=0.)
        if self.force_control and self.phase in ('tighten', 'hold_force'):
            f = feedback or {}
            if not f:
                self.phase, self.failure_reason = 'abort', 'missing_force_feedback'
                return a, self.phase
            force = float(f['pad_force_n'])
            compression = float(f['pad_compression_m'])
            if compression > .002 or force > 10 or f.get('jaw_force_n', 0) > 20:
                self.phase = 'abort'
                self.failure_reason = 'pad_compression_limit' if compression > .002 else 'contact_force_limit'
                return a, self.phase
            # Admittance outer loop, existing bounded position servo inside.
            # Freeze whole-arm approach: do not drive against the seated interface.
            a['capture_axial_velocity_m_s'] = float(np.clip(.0002*(5.-force), -.0005, .0005))
            supported = f.get('jaw_support_count', 0) >= 2
            if abs(force-5.) <= .5 and supported:
                if self.stable_since is None: self.stable_since = t
                if t-self.stable_since >= 1.: self.phase = 'hold_force'
            else:
                self.stable_since = None
                if self.phase == 'hold_force': self.phase, self.start = 'tighten', t
            if self.phase == 'tighten' and t-self.start > 35:
                self.phase, self.failure_reason = 'abort', 'force_not_established'
                a['capture_axial_velocity_m_s'] = 0.
            return a, self.phase
        if self.phase == 'wait' and t >= 20:
            self.phase, self.start = 'open', 20.
        elapsed = t - self.start
        nxt = None
        if self.phase == 'open':
            if np.max(np.abs(q[8:])) < .00015: nxt = 'extend'
            elif elapsed > 12: nxt = 'abort'
            else: a['capture_radial_velocity_m_s'] = .003
        elif self.phase == 'extend':
            if abs(q[6] + .006) < .00015: nxt = 'approach'
            elif elapsed > 16: nxt = 'abort'
            else: a['capture_axial_velocity_m_s'] = -.003
        elif self.phase in ('approach', 'close', 'tighten'):
            error = np.array([0., 0., .006]) - offset
            angular = rotation_matrix_to_vector(orientation_error)
            linear = .8 * error
            linear *= min(1., .004 / max(np.linalg.norm(linear), 1e-12))
            a['end_effector_linear_velocity_body_m_s'] = linear.tolist()
            a['end_effector_angular_velocity_body_rad_s'] = np.clip(.8*angular, -.04, .04).tolist()
            if self.phase == 'approach':
                if np.linalg.norm(error) < .0004 and np.linalg.norm(angular) < .0087: nxt = 'close'
                # 0.5 m at the existing 4 mm/s limit needs at least 125 s;
                # leave time for free-base motion and final convergence.
                elif elapsed > 240: nxt = 'abort'
            elif self.phase == 'close':
                if np.max(np.abs(q[8:] + .012)) < .0002: nxt = 'tighten'
                elif elapsed > 12: nxt = 'abort'
                else: a['capture_radial_velocity_m_s'] = -.002
            elif self.phase == 'tighten':
                if q[6] >= .0030: nxt = 'hold'
                elif elapsed > 14: nxt = 'hold_load_limited'
                else: a['capture_axial_velocity_m_s'] = .001
        if nxt:
            self.phase, self.start = nxt, t
        return a, self.phase


class VisualCaptureProbe:
    """Opt-in short visual-servo probe. No target truth input or fallback.

    Preparation uses measured tool joints. Arm motion uses only the exported
    image pose and encoder-derived gripper orientation. This probe stops before
    contact; it deliberately cannot claim completion of a full visual grasp.
    """
    def __init__(self, path, session_id, *, goal_gap_m=.48, wall_now_ns=time.time_ns):
        self.path=Path(path);self.session_id=session_id;self.goal_gap_m=float(goal_gap_m)
        self.wall_now_ns=wall_now_ns;self.sequence=CaptureSequence(force_control=True)
        self.last_packet=None;self.input_status='waiting';self.motion_samples=0
        self.last_time=None;self.commanded_path_m=0.

    def _pose(self, t):
        data=json.loads(self.path.read_text('utf-8'))
        self.last_packet=data
        if not data.get('valid') or data.get('status')!='tracking':
            raise ValueError(data.get('status','unavailable'))
        if data.get('session_id')!=self.session_id:raise ValueError('session_mismatch')
        age=float(data['age_s'])+max(0.,(self.wall_now_ns()-int(data['published_wall_time_ns']))/1e9)
        if not np.isfinite(age) or age>1.5:raise ValueError('stale')
        source_t=int(data['sim_time_ns'])/1e9
        if source_t>t+.025:raise ValueError('future_frame')
        a=data['result']['alignment'];p=np.asarray(a['interface_position_grip_m'],float)
        r=np.asarray(a['R_grip_from_interface'],float)
        if not a.get('valid') or p.shape!=(3,) or r.shape!=(3,3) or not np.all(np.isfinite(p)) or not np.all(np.isfinite(r)):
            raise ValueError('invalid_pose')
        if not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
            raise ValueError('invalid_rotation')
        self.input_status='tracking'
        return p,r

    def command(self,t,q,body_from_grip,feedback):
        dt=0. if self.last_time is None else float(np.clip(t-self.last_time,0.,.02))
        self.last_time=t
        neutral=dict(deadman=False,active_arm='capture',
            end_effector_linear_velocity_body_m_s=[0.,0.,0.],
            end_effector_angular_velocity_body_rad_s=[0.,0.,0.],
            capture_radial_velocity_m_s=0.,capture_axial_velocity_m_s=0.)
        if self.sequence.phase in ('abort','probe_complete'):
            return neutral,'visual_'+self.sequence.phase
        if self.sequence.phase in ('open','extend'):
            a,phase=self.sequence.command(t,q,np.zeros(3),np.eye(3),feedback)
            return a,'visual_'+phase
        try:p,r=self._pose(t)
        except (OSError,ValueError,TypeError,KeyError) as e:
            self.input_status=str(e)
            return neutral,'visual_waiting_'+self.input_status
        if self.sequence.phase=='wait':
            if t<2:return neutral,'visual_waiting_start'
            self.sequence.phase,self.sequence.start='open',t
            return neutral,'visual_open'
        error=p-np.array([0.,0.,-self.goal_gap_m])
        angular=rotation_matrix_to_vector(r)
        if np.linalg.norm(error)<.0004 and np.linalg.norm(angular)<.0087:
            self.sequence.phase='probe_complete'
            return neutral,'visual_probe_complete'
        linear=.4*error
        linear*=min(1.,.002/max(np.linalg.norm(linear),1e-12))
        self.commanded_path_m+=float(np.linalg.norm(linear))*dt
        if self.commanded_path_m>.03:
            self.sequence.phase='abort';self.sequence.failure_reason='visual_probe_30mm_motion_budget'
            return neutral,'visual_abort_motion_budget'
        omega=.4*angular
        omega*=min(1.,.01/max(np.linalg.norm(omega),1e-12))
        a=dict(neutral,deadman=True,
            end_effector_linear_velocity_body_m_s=(body_from_grip@linear).tolist(),
            end_effector_angular_velocity_body_rad_s=(body_from_grip@omega).tolist())
        self.motion_samples+=1
        return a,'visual_approach'
