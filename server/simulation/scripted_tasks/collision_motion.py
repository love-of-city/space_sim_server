"""Environment-aware free motion using the production native collision model."""
from __future__ import annotations

import math
import time
import numpy as np

from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, rotation_matrix_to_vector
from .motion import solve_pose
from .motion_timing import time_joint_path
from .obstacle_planner import JointRouteSearch, PlanningFailure, endpoint_seed


class NativeMotionQuery:
    def __init__(self, task, sample, *, held=False, clearance_m=.002, goal_part=None):
        from .drawer_initialization import NativePoseProbe
        self.task, self.sample, self.held = task, sample, held
        self.clearance = clearance_m
        self.goal_part = goal_part
        self.probe = NativePoseProbe(task.model_path, clearance_m=clearance_m)
        if sample.native_qpos is not None:
            task._planning_native_state = (sample.native_qpos.copy(), sample.joints.copy())
        state = getattr(task, "_planning_native_state", None)
        if state is not None:
            self.probe.use_measured_qpos(*state)
        self.probe.contacts(sample.joints)
        self.bus_p, self.bus_r = self.probe.body_pose("cubesat_bus")
        self.baseline = self.probe.home.copy()

    def __call__(self, configuration):
        arm = np.asarray(configuration[:6])
        aperture = (float(max(self.sample.joints[6], -self.sample.joints[7]))
                    if self.held else float(configuration[6]))
        q = np.r_[arm, aperture, -aperture]
        bp, br = self.sample.part_position_body_m, self.sample.part_rotation_body
        if self.held:
            p, r = self.task.chain.forward(arm)
            relp, relr = self.task._grasp_transform
            bp, br = p+r@relp, r@relr
        body = getattr(self.task,"planning_part_body","bolt")
        joint = getattr(self.task,"planning_part_joint","bolt_freejoint")
        overrides = dict(getattr(self.task,"planning_scalars",{}))
        pose = np.r_[self.bus_p+self.bus_r@bp, matrix_to_quaternion_wxyz(self.bus_r@br)]
        # A seated, ungrasped bolt is part of the static scene. Its designed
        # close fit in its own holder must not reject an unrelated arm motion.
        part_clearance = self.clearance
        if self.held:
            # The extracted tip starts just beyond the bore, and the transfer
            # ends at its mating bore. Grow a positive gap away from these
            # intentional contact neighborhoods, still rejecting penetration.
            distances = [np.linalg.norm(bp-self.sample.part_position_body_m)]
            if self.goal_part is not None:
                distances.append(np.linalg.norm(bp-self.goal_part))
            part_clearance *= min(1., min(distances)/.01)
        if joint is not None:
            overrides[joint] = pose
        return self.probe.pose_collision(q, self.task.name if self.held else "free_motion", (body,) if self.held else (),
                    overrides, self.baseline,
                    arm_penetration_m=0., clearance_m=self.clearance,
                    mechanism_clearance_m=part_clearance)

    def close(self):
        self.probe.close()


def _configuration(task, u, held):
    # Adapters keep a bound path_reference on the source task. After installing
    # a searched path on their proxy, that method still describes the original
    # candidate. Check the new curve itself, including its actual finger law.
    if held or task._timing.path.coefficients.shape[-1] == 7:
        q = task._timing.path.at(u)
        return q[:6] if held else q[:7]
    q = task.path_reference(u)
    return q[:6] if held else q[:7]


def _check_template(task, search, held):
    # This spacing bound covers dimensions encoded in the joint spline.
    # A separate approach/release opening law is evaluated at the same poses;
    # it is not part of the spline derivative bound or a continuous CCD claim.
    slope = float(task._timing.path.derivative_bounds()[0].max())
    count = max(300, math.ceil(slope/search.options.final_step_rad))
    for u in np.linspace(0., 1., count+1):
        if not search.valid(_configuration(task,u,held)):
            return False
    return True


def _terminal(task, position, rotation, axis, opening, seed, held):
    """Reach the grasp directly; only carried parts retain a mating corridor."""
    if not held:
        q = solve_pose(task.chain,position,rotation,seed,task.lower,task.upper)
        return np.asarray([np.r_[q,opening]])
    clearance = task.config.pregrasp_distance_m
    start = position+axis*clearance
    q = solve_pose(task.chain,start,rotation,seed,task.lower,task.upper)
    points = []
    for u in np.linspace(0.,1.,41):
        q = solve_pose(task.chain,start+(position-start)*u,rotation,q,task.lower,task.upper)
        points.append(q.copy() if held else np.r_[q,opening])
    return np.asarray(points)


def _install(task, knots, *, held):
    points, rotations = zip(*(task.chain.forward(q[:6]) for q in knots))
    distance = float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    angle = sum(float(np.linalg.norm(rotation_matrix_to_vector(b@a.T)))
                for a,b in zip(rotations,rotations[1:]))
    task._path = knots.copy()
    if held:
        # A failed template can leave the previous rewind/opening law behind.
        # A searched transfer always keeps the current grasp command.
        opening = float(task.reference[6])
        task._path_opening = (opening, opening, None)
    task._timing = time_joint_path(knots,distance,angle,
        joint_speed=task.config.arm_speed_rad_s,linear_speed=task.config.linear_speed_m_s,
        angular_speed=task.config.angular_speed_rad_s,linear_acceleration=.15,
        angular_acceleration=.4,joint_acceleration=.6,linear_jerk=.8,
        angular_jerk=1.6,joint_jerk=8.,slow_distance=.02,
        slow_speed=task.config.grasp_approach_speed_m_s)
    if not held:
        task._timing = task._timing.limit_derivatives(
            task._timing.path.derivative_bounds()[:,6],task.config.gripper_speed_rad_s,.4,1.6)
    # Joint arc length need not correspond to TCP arc length. Bound sampled
    # Cartesian derivatives too, so a searched path cannot silently speed up.
    u = np.linspace(0.,1.,501)
    linear, angular = [], []
    for v in u:
        q, dq, _, _ = task._timing.path.state(v)
        twist = task.chain.jacobian(q[:6])@dq[:6]
        linear.append(twist[:3]); angular.append(twist[3:])
    for values, limits in ((linear,(task.config.linear_speed_m_s,.15,.8)),
                           (angular,(task.config.angular_speed_rad_s,.4,1.6))):
        values = np.asarray(values)
        acceleration = np.gradient(values,u,axis=0,edge_order=2)
        jerk = np.gradient(acceleration,u,axis=0,edge_order=2)
        bounds = 1.1*np.array([np.linalg.norm(v,axis=1).max() for v in (values,acceleration,jerk)])
        task._timing = task._timing.limit_derivatives(bounds,*limits)
    task._duration, task._motion_s = task._timing.duration, 0.


def plan_free_motion(task, sample, position, rotation, axis, *, held=False,
                     template_error=None, opening=None, stage=None, fallback_positions=None):
    """Check the caller's straight approach before searching a smooth detour."""
    started = time.monotonic()
    stage = stage or ("carry" if held else "approach")
    approach = not held and stage.endswith("approach")
    start = task.reference[:6].copy() if held else task.reference[:7].copy()
    opening = task.config.approach_open_rad if opening is None else opening
    lower = task.lower if held else np.r_[task.lower,0.]
    upper = task.upper if held else np.r_[task.upper,1.]
    goals = np.asarray([position] if fallback_positions is None else fallback_positions,dtype=float)
    if goals.ndim != 2 or goals.shape[1] != 3 or not 1 <= len(goals) <= 12 or not np.isfinite(goals).all():
        raise ValueError("Expected one to twelve finite retreat/approach positions")
    if held and fallback_positions is not None:
        raise ValueError("Alternative free-motion positions are only supported without a held part")
    goal_part = position+rotation@task._grasp_transform[0] if held else None
    query = NativeMotionQuery(task,sample,held=held,goal_part=goal_part)
    task._planning_bus_pose = (query.bus_p.copy(), query.bus_r.copy())
    search = JointRouteSearch(lower,upper,query,seed=endpoint_seed(start,position))
    try:
        if template_error is None and _check_template(task,search,held):
            report = {**search.report,"accepted":True,"route":"straight" if approach else "template"}
        else:
            hit = search.report.get("first_collision") or {}
            candidate_failure = template_error or (
                "candidate_clearance_violation" if hit.get("separation_m", -1.) >= 0.
                else "candidate_in_collision")
            # One IK branch can be blocked even when another reaches the same
            # target pose. Try bounded, deterministic alternatives; budget
            # exhaustion never means the layout is physically unreachable.
            seeds = [start[:6].copy()]
            if task._path is not None:
                seeds.insert(0, task._path[-1][:6].copy())
            for sign in (-1., 1.):
                seed = start[:6].copy()
                seed[1:3] += sign*np.array([.7,-.7])
                seeds.append(np.clip(seed,task.lower,task.upper))
            attempts = []
            knots = None
            for seed in seeds:
                # All spatial endpoints share the native model, RNG and total
                # query budget. A blocked first goal must not hide a reachable
                # rising retreat on the other side of a neighboring component.
                for goal_index,goal in enumerate(goals):
                    try:
                        terminal = _terminal(task,goal,rotation,axis,opening,seed,held)
                        if not all(search.valid(q) for q in terminal):
                            raise PlanningFailure("axial_operation_corridor_blocked" if held
                                                  else "goal_in_collision",search.report)
                        route = search.search(start,terminal[0],monotone_axis=None if held else 6)
                        knots = search.smooth(route,terminal=terminal if held else None)
                        break
                    except (PlanningFailure, ValueError) as exc:
                        attempts.append(str(exc))
                        if str(exc) in ("collision_query_budget_exhausted","start_in_collision"):
                            raise
                if knots is not None:
                    break
            search.report["candidate_failures"] = attempts
            if knots is None:
                raise PlanningFailure("no_valid_route_within_budget",search.report)
            _install(task,knots,held=held)
            task._goal,task._rotation = goal.copy(),np.asarray(rotation).copy()
            if not _check_template(task,search,held):
                raise PlanningFailure("retimed_route_in_collision",search.report)
            report = {**search.report,"route":"searched",
                      "direct_failure":candidate_failure,
                      "goal_candidate_index":goal_index,
                      "goal_position_body_m":goal.tolist(),
                      "opening_law":"held" if held else "monotone_cubic"}
        report.update(stage=stage,clearance_m=query.clearance,
                      wall_seconds=time.monotonic()-started,duration_s=task._duration)
        task._timeout = max(task._timeout, task._duration*1.5+10.)
        task.motion_plans.append(report)
        if task.events:
            task.events[-1]["motion_plan"] = report
        return report
    except PlanningFailure as exc:
        task.motion_plans.append({**exc.report,"stage":stage,
                                  "wall_seconds":time.monotonic()-started})
        raise
    finally:
        query.close()
