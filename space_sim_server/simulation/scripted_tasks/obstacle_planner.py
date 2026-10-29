"""Deterministic bounded joint-space search and collision-checked C2 smoothing.

The caller supplies collision queries against a private native model. Planning
never changes the live simulation. Edge/spline checks are finite samples, not
a claim of exact continuous collision detection or guaranteed search success.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
import numpy as np

from .joint_path import JointPath


class PlanningFailure(ValueError):
    def __init__(self, reason, report):
        self.report = {**report, "accepted": False, "reason": reason}
        super().__init__(reason)


@dataclass(frozen=True)
class SearchOptions:
    edge_step_rad: float = .02
    final_step_rad: float = .006
    extension_rad: float = .22
    iterations: int = 1600
    query_budget: int = 80000
    shortcut_attempts: int = 100


def resample_polyline(points, count):
    points = np.asarray(points, dtype=float)
    lengths = np.r_[0., np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))]
    keep = np.r_[True, np.diff(lengths) > 1e-10]
    points, lengths = points[keep], lengths[keep]
    if len(points) == 1:
        return np.repeat(points, count, axis=0)
    samples = np.linspace(0., lengths[-1], count)
    return np.column_stack([np.interp(samples, lengths, points[:, j]) for j in range(points.shape[1])])


def rounded_polyline(points, fraction):
    """Quintic corner arcs join straight segments with zero end curvature."""
    points = np.asarray(points)
    result = [points[0]]
    for before, corner, after in zip(points, points[1:], points[2:]):
        a, b = corner-before, after-corner
        la, lb = np.linalg.norm(a), np.linalg.norm(b)
        if min(la, lb) < 1e-10:
            continue
        a, b = a/la, b/lb
        trim = fraction*min(la, lb)
        start, end = corner-trim*a, corner+trim*b
        result.extend(np.linspace(result[-1], start, 8)[1:])
        control = [start, start+.4*trim*a, start+.8*trim*a,
                   end-.8*trim*b, end-.4*trim*b, end]
        for u in np.linspace(0., 1., 25)[1:]:
            result.append(sum(math.comb(5,j)*(1-u)**(5-j)*u**j*p for j,p in enumerate(control)))
    result.extend(np.linspace(result[-1], points[-1], 8)[1:])
    return np.asarray(result)


class JointRouteSearch:
    def __init__(self, lower, upper, collision, *, seed=0, options=None):
        self.lower, self.upper = np.asarray(lower), np.asarray(upper)
        if self.lower.shape != self.upper.shape or np.any(self.lower >= self.upper):
            raise ValueError("Invalid planning limits")
        self.collision = collision
        self.options = options or SearchOptions()
        self.rng = np.random.default_rng(seed)
        self.report = dict(strategy="rrt-connect-c2/1", queries=0, iterations=0,
                           edge_step_rad=self.options.edge_step_rad,
                           final_step_rad=self.options.final_step_rad, first_collision=None)
        self.monotone_axis = None
        self.monotone_direction = 0.

    def valid(self, q):
        if self.report["queries"] >= self.options.query_budget:
            raise PlanningFailure("collision_query_budget_exhausted", self.report)
        self.report["queries"] += 1
        if not np.isfinite(q).all() or np.any(q < self.lower-1e-9) or np.any(q > self.upper+1e-9):
            return False
        hit = self.collision(q)
        if hit is not None and self.report["first_collision"] is None:
            self.report["first_collision"] = hit
        return hit is None

    def edge(self, a, b, step=None, *, reverse=False):
        if self.monotone_axis is not None:
            change = (b[self.monotone_axis]-a[self.monotone_axis])*self.monotone_direction
            if (-change if reverse else change) < -1e-10:
                return False
        n = max(1, math.ceil(float(np.max(np.abs(b-a)))/(step or self.options.edge_step_rad)))
        return all(self.valid(a+(b-a)*u) for u in np.linspace(0., 1., n+1)[1:])

    def curve_valid(self, path):
        try:
            path.validate_limits(self.lower, self.upper)
        except ValueError:
            return False
        slope = float(path.derivative_bounds()[0].max())
        n = max(240, math.ceil(slope/self.options.final_step_rad))
        return all(self.valid(path.at(u)) for u in np.linspace(0., 1., n+1))

    def search(self, start, goal, *, monotone_axis=None):
        start, goal = np.asarray(start), np.asarray(goal)
        self.monotone_axis = monotone_axis
        self.monotone_direction = (0. if monotone_axis is None else
                                   float(np.sign(goal[monotone_axis]-start[monotone_axis])))
        if not self.valid(start):
            raise PlanningFailure("start_in_collision", self.report)
        if not self.valid(goal):
            raise PlanningFailure("goal_in_collision", self.report)
        if self.edge(start, goal):
            return np.array([start, goal])
        trees = [([start.copy()], [-1]), ([goal.copy()], [-1])]

        def extend(tree, target, reverse=False):
            nodes, parents = tree
            distances = np.linalg.norm(np.asarray(nodes)-target, axis=1)
            if self.monotone_axis is not None:
                direction = -self.monotone_direction if reverse else self.monotone_direction
                changes = (target[self.monotone_axis]-np.asarray(nodes)[:,self.monotone_axis])*direction
                distances[changes < -1e-10] = np.inf
            nearest = int(np.argmin(distances))
            if not np.isfinite(distances[nearest]):
                return None, False
            delta = target-nodes[nearest]
            length = float(np.linalg.norm(delta))
            if length < 1e-10:
                return nearest, True
            candidate = nodes[nearest]+delta*min(1., self.options.extension_rad/length)
            if not self.edge(nodes[nearest], candidate, reverse=reverse):
                return None, False
            nodes.append(candidate); parents.append(nearest)
            return len(nodes)-1, length <= self.options.extension_rad

        def ancestry(tree, index):
            values = []
            while index >= 0:
                values.append(tree[0][index]); index = tree[1][index]
            return values[::-1]

        for iteration in range(self.options.iterations):
            self.report["iterations"] = iteration+1
            active = iteration % 2
            # Prefer the neighborhood of this task, but retain full bounded
            # configuration-space coverage for routes around large obstacles.
            if self.rng.random() < .18:
                target = trees[1-active][0][0]
            elif self.rng.random() < .75:
                target = self.rng.uniform(np.maximum(self.lower,np.minimum(start,goal)-1.1),
                                          np.minimum(self.upper,np.maximum(start,goal)+1.1))
            else:
                target = self.rng.uniform(self.lower,self.upper)
            if self.monotone_axis is not None:
                i = self.monotone_axis
                target = target.copy()
                target[i] = np.clip(target[i],min(start[i],goal[i]),max(start[i],goal[i]))
            index, _ = extend(trees[active], target, reverse=active == 1)
            if index is None:
                continue
            target = trees[active][0][index]
            while True:
                other, reached = extend(trees[1-active], target, reverse=active == 0)
                if other is None:
                    break
                if reached:
                    left, right = ancestry(trees[active],index), ancestry(trees[1-active],other)
                    path = left+right[-2::-1]
                    return np.asarray(path if active == 0 else path[::-1])
        raise PlanningFailure("search_budget_exhausted", self.report)

    def smooth(self, route, *, terminal=None):
        points = list(np.asarray(route))
        for _ in range(self.options.shortcut_attempts):
            if len(points) < 3:
                break
            i,j = sorted(self.rng.choice(len(points),2,replace=False))
            if j > i+1 and self.edge(points[i],points[j]):
                points[i+1:j] = []
        for fraction in (.3,.15,.06):
            rounded = rounded_polyline(points,fraction)
            if terminal is not None:
                rounded = np.vstack([rounded,np.asarray(terminal)[1:]])
            for count in (81,161):
                knots = resample_polyline(rounded,count)
                if self.monotone_axis is not None:
                    # Use one coordinated opening law, rather than replaying
                    # arbitrary jaw samples from the joint-space search. This
                    # global cubic is reproduced exactly by the C2 interpolant
                    # and cannot reverse or overshoot. Its actual finger poses
                    # are checked below after replacing the search aperture.
                    i = self.monotone_axis
                    u = np.linspace(0.,1.,count)
                    knots[:,i] = points[0][i]+(points[-1][i]-points[0][i])*u*u*(3.-2.*u)
                path = JointPath.from_waypoints(knots)
                if self.curve_valid(path):
                    self.report.update(accepted=True,shortcut_waypoints=len(points),spline_waypoints=count)
                    return knots
        raise PlanningFailure("smoothed_route_in_collision", self.report)


def endpoint_seed(start, goal):
    data = np.round(np.r_[start,goal],5).astype('<f8').tobytes()
    return int.from_bytes(hashlib.sha256(data).digest()[:8],'little')
