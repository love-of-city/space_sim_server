"""Heartbeat-gated, measured-state fixed-sequence preparation.

Only joint references are produced here. Collision/contact belong exclusively
to the running native MuJoCo scene. No offline model, path search, or target-drift
replanning is used. Preparation moves J2--J5 only; J1/J6 retain their
starting references. Normal Cartesian IK is not restricted by this policy.
"""
from __future__ import annotations
import numpy as np


class SmoothJointSegment:
    """Synchronized straight joint segment with cosine velocity ramps."""
    RAMP_SECONDS = .5

    def __init__(self, start, end, speed_limits):
        self.start = np.asarray(start, dtype=float).copy()
        self.end = np.asarray(end, dtype=float).copy()
        self.delta = self.end - self.start
        self.ramp = self.RAMP_SECONDS
        self.duration = max(2 * self.ramp, self.ramp + float(np.max(np.abs(self.delta) / speed_limits)))
        self.peak = self.delta / (self.duration - self.ramp)

    def sample(self, seconds):
        t = float(np.clip(seconds, 0., self.duration))
        r, total = self.ramp, self.duration
        if t < r:
            distance = .5 * (t - r / np.pi * np.sin(np.pi * t / r))
            speed = .5 * (1. - np.cos(np.pi * t / r))
        elif t <= total - r:
            distance, speed = t - .5 * r, 1.
        else:
            remaining = total - t
            distance = total - r - .5 * (remaining - r / np.pi * np.sin(np.pi * remaining / r))
            speed = .5 * (1. - np.cos(np.pi * remaining / r))
        if t >= total:
            return self.end.copy(), np.zeros(6)
        return self.start + self.peak * distance, self.peak * speed


class ArmPreparation:
    ACTIVE = {'arming', 'moving', 'settling'}
    POSITION_TOLERANCE = np.deg2rad(.8)
    STOP_SPEED = .02
    STABLE_SECONDS = .35
    TRACKING_LIMIT = np.deg2rad(8.)

    def __init__(self, required, goal_rad, lower, upper, speed_limits):
        self.required = bool(required)
        self.default_goal = np.asarray(goal_rad, dtype=float).copy()
        self.default_goal[[0, 5]] = 0.  # zero-start display, including old saved presets
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        # J6's 0.35 Nm drive cannot follow a 1 rad/s reference against the
        # existing damping/friction. Leave torque/model untouched; command .5.
        self.speed_limits = np.minimum(np.asarray(speed_limits, dtype=float), [.7, .7, .7, .9, 1., .5])
        self.reset()

    def reset(self):
        self.status = 'waiting' if self.required else 'legacy'
        self.reason = self.request_id = ''
        self.seen_ids = set()
        self.goal = self.default_goal.copy()
        self.progress = self.elapsed = self.stable = self.phase_elapsed = 0.
        self.max_error_deg = self.max_velocity_rad_s = None
        self.waypoints = []
        self.phase = 0
        self.phase_times = []
        self.segment = None
        self.hold_position = None
        self.resume_phase = None

    @property
    def ready(self):
        return self.status in {'ready', 'legacy'}

    def telemetry(self):
        return dict(required=self.required, status=self.status, ready=self.ready,
                    request_id=self.request_id, reason=self.reason,
                    goal_deg=np.rad2deg(self.goal).tolist(), progress=self.progress,
                    elapsed_s=self.elapsed, phase=self.phase + 1 if self.waypoints else 0,
                    phase_count=len(self.waypoints), phase_times=self.phase_times.copy(),
                    max_error_deg=self.max_error_deg, max_velocity_rad_s=self.max_velocity_rad_s,
                    strategy='fixed-two-stage-v2', held_joints=['joint1', 'joint6'], collision_authority='running_native_MJScene',
                    speed_limits_rad_s=self.speed_limits.tolist())

    def stop(self, reason, failed=False):
        self.status = 'failed' if failed else 'cancelled'
        self.reason = reason

    def _start(self, request, q, reference):
        request_id = request.get('request_id')
        if not isinstance(request_id, str) or not request_id:
            raise ValueError('准备请求缺少有效标识')
        self.request_id = request_id
        self.seen_ids.add(request_id)
        goal = np.deg2rad(np.asarray(request.get('joint_position_deg'), dtype=float))
        if (goal.shape != (6,) or not np.all(np.isfinite(goal))
                or np.any(goal < self.lower) or np.any(goal > self.upper)):
            raise ValueError('操作姿态超出模型限位或不是六个有限角度')
        # Ignore old/custom J1/J6 targets for preparation, including stale browsers.
        goal[[0, 5]] = np.clip(reference, self.lower, self.upper)[[0, 5]]
        # Same-goal retries continue the interrupted stage, not the opening
        # posture again with an already rotated wrist. A new goal starts a new sequence.
        self.resume_phase = (self.phase if self.waypoints and self.phase < len(self.waypoints)
                             and np.array_equal(goal[1:5], self.goal[1:5]) and not self.ready else None)
        self.goal = goal
        self.hold_position = np.clip(q, self.lower, self.upper)
        self.hold_position[[0, 5]] = self.goal[[0, 5]]
        self.elapsed = self.phase_elapsed = self.stable = self.progress = 0.
        self.reason = '等待机械臂稳定停止'
        self.status = 'arming'

    def _begin_sequence(self, q):
        if np.max(np.abs(q - self.goal)) <= self.POSITION_TOLERANCE:
            # Re-clicking after arrival must not unfold J3 again with the wrist
            # already rotated. Confirm/settle locally at the same literal goal.
            self.waypoints = [self.goal.copy()]
            self.phase = 0
            self.phase_times = []
        elif self.resume_phase is None:
            opening = q.copy()
            opening[[1, 3]] = self.goal[[1, 3]]
            opening[2] = np.deg2rad(30.)
            self.waypoints = [opening, self.goal.copy()]
            self.phase = 0
            self.phase_times = []
        else:
            self.phase = self.resume_phase
        for waypoint in self.waypoints:
            waypoint[[0, 5]] = self.goal[[0, 5]]
        if np.any(np.asarray(self.waypoints) < self.lower) or np.any(np.asarray(self.waypoints) > self.upper):
            raise ValueError('固定准备顺序的中间姿态超出模型限位')
        self._begin_phase(q)

    def _begin_phase(self, start):
        start = np.asarray(start).copy()
        start[[0, 5]] = self.goal[[0, 5]]
        self.segment = SmoothJointSegment(start, self.waypoints[self.phase], self.speed_limits)
        self.phase_elapsed = self.stable = 0.
        self.status = 'moving'
        self.reason = ''

    def step(self, dt, action, stale, reference, measured, velocity, snapshot=None):
        # snapshot is accepted only for compatibility; target pose never vetoes
        # motion. No collision calculation is performed by this controller.
        if not self.required:
            return None
        reference = np.asarray(reference, dtype=float)
        q, qd = np.asarray(measured, dtype=float), np.asarray(velocity, dtype=float)
        if (q.shape != (6,) or qd.shape != (6,) or not np.isfinite(q).all()
                or not np.isfinite(qd).all()):
            self.stop('实测关节状态包含无效数值', failed=True)
            return reference.copy(), np.zeros(6)
        self.max_error_deg = float(np.rad2deg(np.max(np.abs(q - self.goal))))
        self.max_velocity_rad_s = float(np.max(np.abs(qd)))
        request = action.get('arm_preparation')
        authorized = isinstance(request, dict) and bool(request) and action.get('deadman') and not stale
        if self.status in self.ACTIVE and (not authorized or request.get('request_id') != self.request_id):
            self.stop('准备授权已撤销，机械臂保持当前位置')
            return np.clip(q, self.lower, self.upper), np.zeros(6)
        try:
            if authorized and request.get('request_id') not in self.seen_ids and self.status not in self.ACTIVE:
                self._start(request, q, reference)
            if self.status in self.ACTIVE:
                if not np.isfinite(dt) or dt <= 0. or dt > .05:
                    raise ValueError('准备控制周期无效')
                if np.any(q < self.lower - .01) or np.any(q > self.upper + .01):
                    raise ValueError('实测关节触及位置限制')
                if np.max(np.abs(reference - q)) > self.TRACKING_LIMIT:
                    raise ValueError('关节跟踪误差超过 8°，已中止（请查看运行中 MuJoCo 接触和关节状态）')
                self.elapsed += dt
                if self.status == 'arming':
                    stopped = self.max_velocity_rad_s <= self.STOP_SPEED
                    self.stable = self.stable + dt if stopped else 0.
                    if self.stable >= self.STABLE_SECONDS:
                        self._begin_sequence(np.clip(q, self.lower, self.upper))
                    elif self.elapsed > 10.:
                        raise ValueError('准备前未能稳定停止，请检查实测关节状态')
                    return self.hold_position.copy(), np.zeros(6)
                self.phase_elapsed += dt
                target, speed = self.segment.sample(self.phase_elapsed)
                self.progress = (self.phase + min(1., self.phase_elapsed / self.segment.duration)) / len(self.waypoints)
                if self.phase_elapsed >= self.segment.duration:
                    self.status = 'settling'
                    arrived = (np.max(np.abs(target - q)) <= self.POSITION_TOLERANCE
                               and self.max_velocity_rad_s <= self.STOP_SPEED)
                    self.stable = self.stable + dt if arrived else 0.
                    if self.stable >= self.STABLE_SECONDS:
                        self.phase_times.append(self.elapsed)
                        if self.phase == len(self.waypoints) - 1:
                            self.status = 'ready'
                            self.reason = '实测角度与速度稳定到位'
                            self.progress = 1.
                            return q.copy(), np.zeros(6)
                        self.phase += 1
                        self._begin_phase(target)
                    elif self.phase_elapsed > self.segment.duration + 12.:
                        raise ValueError('到位超时，未进入可操作状态')
                return target, speed
        except (ValueError, TypeError, KeyError) as error:
            self.stop(str(error), failed=True)
            return np.clip(q, self.lower, self.upper), np.zeros(6)
        if self.ready and not request:
            return None
        return reference.copy(), np.zeros(6)
