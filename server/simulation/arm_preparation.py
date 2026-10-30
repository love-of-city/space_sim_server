"""Heartbeat-gated joint preparation with measured arrival and saved paths."""
from __future__ import annotations
import numpy as np


class SmoothJointSegment:
    """Synchronized straight joint segment with cosine velocity ramps."""
    RAMP_SECONDS = .5

    def __init__(self, start, end, speed_limits, acceleration_limits=None):
        self.start = np.asarray(start, dtype=float).copy()
        self.end = np.asarray(end, dtype=float).copy()
        self.delta = self.end - self.start
        self.ramp = self.RAMP_SECONDS
        if acceleration_limits is not None:
            self.ramp = max(self.ramp, float(np.max(np.pi * speed_limits / acceleration_limits)))
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

    def __init__(self, required, goal_rad, lower, upper, speed_limits, *, plan=None):
        self.required = bool(required)
        self.plan = plan
        self.default_goal = np.asarray(goal_rad, dtype=float).copy()
        if plan is None:
            self.default_goal[[0, 5]] = 0.
        else:
            from simulation.preparation_contract import validate_preparation_plan
            validate_preparation_plan(plan, np.rad2deg(self.default_goal).tolist())
        self.lower = np.asarray(lower, dtype=float)
        self.upper = np.asarray(upper, dtype=float)
        # J6's 0.35 Nm drive cannot follow a 1 rad/s reference against the
        # existing damping/friction. Leave torque/model untouched; command .5.
        self.speed_limits = np.minimum(np.asarray(speed_limits, dtype=float), [.7, .7, .7, .9, 1., .5])
        self.acceleration_limits = np.array([.5, .5, .5, .7, .7, .3])
        if plan is not None:
            self.speed_limits = np.minimum(self.speed_limits, [.5, .5, .5, .65, .6, .25])
            waypoints = np.asarray(plan['waypoints_rad'])
            if np.any(waypoints < self.lower) or np.any(waypoints > self.upper):
                raise ValueError('saved preparation path exceeds joint limits')
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
        self.clock_scale = 1.
        self.blocked_seconds = self.phase_wall_seconds = 0.
        self.brake_position = self.brake_velocity = None
        self.stop_status = 'cancelled'

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
                    strategy='validated-waypoints-v1' if self.plan else 'fixed-two-stage-v2',
                    held_joints=[] if self.plan else ['joint1', 'joint6'], collision_authority='running_native_MJScene',
                    speed_limits_rad_s=self.speed_limits.tolist(), clock_scale=self.clock_scale,
                    acceleration_limits_rad_s2=self.acceleration_limits.tolist() if self.plan else None)

    def stop(self, reason, failed=False):
        self.stop_status = 'failed' if failed else 'cancelled'
        self.status = 'pausing' if self.plan and self.status in self.ACTIVE else self.stop_status
        self.reason = reason

    def _brake(self, dt, measured, velocity):
        if self.brake_position is None:
            self.brake_position = np.clip(measured, self.lower, self.upper)
            self.brake_velocity = np.clip(velocity, -self.speed_limits, self.speed_limits)
        previous = self.brake_velocity.copy()
        self.brake_velocity -= np.clip(self.brake_velocity, -self.acceleration_limits * dt, self.acceleration_limits * dt)
        self.brake_position = np.clip(self.brake_position + (previous + self.brake_velocity) * .5 * dt, self.lower, self.upper)
        if np.max(np.abs(self.brake_velocity)) < 1e-10:
            self.status = self.stop_status
        return self.brake_position.copy(), self.brake_velocity.copy()

    def _advance_clock(self, dt, reference, measured, snapshot):
        effort = np.asarray((snapshot or {}).get('effort_ratio', np.zeros(6)), dtype=float)
        if effort.shape != (6,) or not np.isfinite(effort).all():
            raise ValueError('准备力矩遥测无效')
        lag = np.abs(reference - measured)
        scale = float(np.clip((.08 - np.max(lag)) / .045, 0., 1.))
        saturated = np.any((effort >= .98) & (lag > .025))
        if saturated:
            scale = 0.
        self.blocked_seconds = self.blocked_seconds + dt if scale < .2 else 0.
        if self.blocked_seconds > 3.:
            raise ValueError('准备运动受阻或持续力矩饱和，请复原场景并检查路径')
        rate = float(np.min(self.acceleration_limits / (2 * self.speed_limits)))
        self.clock_scale += float(np.clip(scale - self.clock_scale, -rate * dt, rate * dt))
        self.phase_wall_seconds += dt
        if self.phase_wall_seconds > self.segment.duration * 3 + 12:
            raise ValueError('准备阶段超时，请复原场景重试')
        return dt * self.clock_scale

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
        if self.plan is not None:
            if not np.allclose(goal, self.default_goal, atol=1e-10, rtol=0):
                raise ValueError('目标角度与已验证路径不一致，请重新创建场景')
            if np.max(np.abs(q)) > self.POSITION_TOLERANCE:
                raise ValueError('自动展开必须从零位开始；请先复原场景再重试')
            self.goal = goal
            self.hold_position = np.zeros(6)
            self.elapsed = self.phase_elapsed = self.stable = self.progress = 0.
            self.brake_position = self.brake_velocity = None
            self.status = 'arming'
            self.reason = '等待零位稳定停止'
            return
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
        if self.plan is not None:
            self.waypoints = [np.asarray(point, dtype=float) for point in self.plan['waypoints_rad']]
            self.phase = 0
            self.phase_times = []
            self._begin_phase(np.zeros(6))
            return
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
        if self.plan is None:
            start[[0, 5]] = self.goal[[0, 5]]
        self.segment = SmoothJointSegment(start, self.waypoints[self.phase], self.speed_limits,
                                         self.acceleration_limits if self.plan else None)
        self.phase_elapsed = self.stable = 0.
        self.clock_scale = 1.
        self.blocked_seconds = self.phase_wall_seconds = 0.
        self.status = 'moving'
        self.reason = ''

    def step(self, dt, action, stale, reference, measured, velocity, snapshot=None):
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
        if self.status == 'pausing':
            return self._brake(float(np.clip(dt, 0., .05)), q, qd)
        request = action.get('arm_preparation')
        authorized = isinstance(request, dict) and bool(request) and action.get('deadman') and not stale
        if self.status in self.ACTIVE and (not authorized or request.get('request_id') != self.request_id):
            self.stop('准备授权已撤销，机械臂保持当前位置')
            if self.plan:
                return self._brake(dt, q, qd)
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
                self.phase_elapsed += self._advance_clock(dt, reference, q, snapshot) if self.plan else dt
                target, speed = self.segment.sample(self.phase_elapsed)
                speed *= self.clock_scale
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
            if self.status == 'pausing':
                return self._brake(float(np.clip(dt, 0., .05)), q, qd)
            return np.clip(q, self.lower, self.upper), np.zeros(6)
        if self.ready and not request:
            return None
        return reference.copy(), np.zeros(6)
