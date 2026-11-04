"""Offline, multi-start pose IK with a geometric SARM elbow-height preference.

Never called in the teleoperation update loop. Returned endpoints are NOT paths:
callers must plan/validate a transition before using one on a moving arm.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Iterable

import numpy as np

from simulation.serial_chain_kinematics import (
    SerialChainKinematics, matrix_to_quaternion_wxyz,
)


@dataclass(frozen=True)
class ElbowPreference:
    # Relative to shoulder in the spacecraft BODY frame, not inertial/world Z.
    up_axis: tuple[float, float, float] = (0.0, 0.0, 1.0)
    shoulder_joint: str = "joint2"
    elbow_joint: str = "joint3"
    preferred_height_m: float = 0.30
    length_scale_m: float = 0.54
    elbow_weight: float = 8.0
    motion_weight: float = 0.05
    limit_weight: float = 0.02
    singularity_weight: float = 0.05
    singular_value_reference: float = 0.02
    position_tolerance_m: float = 1.0e-4
    orientation_tolerance_rad: float = 1.0e-3
    max_iterations: int = 120
    random_seed_count: int = 12

    def __post_init__(self) -> None:
        axis = np.asarray(self.up_axis, dtype=float)
        if axis.shape != (3,) or not np.all(np.isfinite(axis)) or np.linalg.norm(axis) < 1e-12:
            raise ValueError("up_axis must be a finite nonzero 3-vector")
        for name in ("length_scale_m", "singular_value_reference", "position_tolerance_m",
                     "orientation_tolerance_rad"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive and finite")
        for name in ("elbow_weight", "motion_weight", "limit_weight", "singularity_weight"):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative and finite")
        if not math.isfinite(self.preferred_height_m):
            raise ValueError("preferred_height_m must be finite")
        if not isinstance(self.max_iterations, int) or self.max_iterations < 1:
            raise ValueError("max_iterations must be a positive integer")
        if not isinstance(self.random_seed_count, int) or self.random_seed_count < 0:
            raise ValueError("random_seed_count must be a nonnegative integer")


@dataclass(frozen=True)
class PoseIkCandidate:
    joint_position_rad: np.ndarray
    elbow_height_m: float
    position_error_m: float
    orientation_error_rad: float
    minimum_singular_value: float
    cost: float


@dataclass(frozen=True)
class PoseIkSelection:
    candidate: PoseIkCandidate | None
    candidates: tuple[PoseIkCandidate, ...]
    attempted_seeds: int
    rejected_collision: int


def elbow_height(chain: SerialChainKinematics, q: np.ndarray,
                 preference: ElbowPreference = ElbowPreference()) -> float:
    origins = chain.joint_origins(q)
    shoulder = chain.joint_names.index(preference.shoulder_joint)
    elbow = chain.joint_names.index(preference.elbow_joint)
    axis = np.asarray(preference.up_axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    return float(axis @ (origins[elbow] - origins[shoulder]))


def _rotation_error(rotation: np.ndarray) -> np.ndarray:
    # Quaternion log is well-defined at pi, unlike angle/sin(angle).
    quat = matrix_to_quaternion_wxyz(rotation)
    sine = float(np.linalg.norm(quat[1:]))
    if sine < 1e-12:
        return 2.0 * quat[1:]
    return quat[1:] * (2.0 * math.atan2(sine, max(0.0, float(quat[0]))) / sine)


def _error(chain, q, position, rotation):
    p, r = chain.forward(q)
    return np.r_[position - p, _rotation_error(rotation @ r.T)]


def _nearby_equivalent(q, reference, lower, upper):
    """Nearest *legal* 2pi branch, never blindly wrap limited joint angles."""
    first = np.ceil((lower - q) / (2 * np.pi))
    last = np.floor((upper - q) / (2 * np.pi))
    turns = np.clip(np.round((reference - q) / (2 * np.pi)), first, last)
    return q + 2 * np.pi * turns


def _iterate(chain, seed, position, rotation, lower, upper, pref):
    q = np.clip(seed, lower, upper).copy()
    for _ in range(pref.max_iterations):
        error = _error(chain, q, position, rotation)
        if (np.linalg.norm(error[:3]) <= pref.position_tolerance_m
                and np.linalg.norm(error[3:]) <= pref.orientation_tolerance_rad):
            return q
        # Metres/radians are normalized with the arm's characteristic length.
        weights = np.array([1.0] * 3 + [pref.length_scale_m] * 3)
        e = weights * error
        j = weights[:, None] * chain.jacobian(q)
        step = j.T @ np.linalg.solve(j @ j.T + 0.003**2 * np.eye(6), e)
        step *= min(1.0, 0.35 / max(float(np.max(np.abs(step))), 1e-12))
        improved = False
        for factor in (1.0, 0.5, 0.25, 0.125, 0.0625):
            trial = np.clip(q + factor * step, lower, upper)
            if np.linalg.norm(weights * _error(chain, trial, position, rotation)) < np.linalg.norm(e) - 1e-12:
                q = trial
                improved = True
                break
        if not improved:
            return None
    # Final step is subject to the same strict acceptance check as every seed.
    error = _error(chain, q, position, rotation)
    return q if (np.linalg.norm(error[:3]) <= pref.position_tolerance_m
                 and np.linalg.norm(error[3:]) <= pref.orientation_tolerance_rad) else None


def select_elbow_preferred_pose(
    chain: SerialChainKinematics,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    current_joints: np.ndarray,
    *,
    joint_position_min: np.ndarray,
    joint_position_max: np.ndarray,
    configuration_is_valid: Callable[[np.ndarray], bool],
    preference: ElbowPreference = ElbowPreference(),
    seeds: Iterable[np.ndarray] | None = None,
) -> PoseIkSelection:
    """Choose an accurate, collision-validated endpoint; None means no valid IK.

    The callback is required so production callers cannot silently skip contact
    validation. It should validate the *selected scene*, including target/fingers.
    No minimum elbow height is imposed: lower configurations remain eligible.
    """
    n = len(chain.joint_names)
    q = np.asarray(current_joints, dtype=float)
    lower = np.asarray(joint_position_min, dtype=float)
    upper = np.asarray(joint_position_max, dtype=float)
    p, r = np.asarray(target_position, dtype=float), np.asarray(target_rotation, dtype=float)
    if any(a.shape != (n,) or not np.all(np.isfinite(a)) for a in (q, lower, upper)):
        raise ValueError("joint vectors must be finite and match the chain")
    if np.any(lower >= upper) or np.any(q < lower) or np.any(q > upper):
        raise ValueError("invalid joint bounds or initial joints outside bounds")
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError("target position must be a finite 3-vector")
    if (r.shape != (3, 3) or not np.all(np.isfinite(r))
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-7)
            or not np.isclose(np.linalg.det(r), 1.0, atol=1e-7)):
        raise ValueError("target rotation must be a proper rotation matrix")
    if not callable(configuration_is_valid):
        raise ValueError("a configuration validator is required")
    # Also validate the configured joint names before spending time solving.
    elbow_height(chain, q, preference)
    seed_list = [q.copy()]
    if seeds is not None:
        seed_list.extend(np.asarray(seed, dtype=float) for seed in seeds)
    else:
        shoulder = chain.joint_names.index(preference.shoulder_joint)
        elbow = chain.joint_names.index(preference.elbow_joint)
        for a, b in ((-1.2, -1.2), (-1.2, 1.2), (1.2, -1.2), (1.2, 1.2),
                     (-2.4, 2.4), (2.4, -2.4)):
            seed = q.copy()
            seed[shoulder] += a
            seed[elbow] += b
            seed_list.append(np.clip(seed, lower, upper))
        # Fixed independent stream: repeatable, no changes to scene randomization.
        rng = np.random.default_rng(20260920)
        seed_list.extend(rng.uniform(lower, upper, size=(preference.random_seed_count, n)))
    if any(seed.shape != (n,) or not np.all(np.isfinite(seed)) for seed in seed_list):
        raise ValueError("seeds must be finite joint vectors")
    candidates: list[PoseIkCandidate] = []
    seen: list[np.ndarray] = []
    rejected_collision = 0
    for seed in seed_list:
        solution = _iterate(chain, seed, p, r, lower, upper, preference)
        if solution is None:
            continue
        solution = _nearby_equivalent(solution, q, lower, upper)
        error = _error(chain, solution, p, r)
        pe, re = float(np.linalg.norm(error[:3])), float(np.linalg.norm(error[3:]))
        if (not np.all(np.isfinite(solution)) or np.any(solution < lower) or np.any(solution > upper)
                or pe > preference.position_tolerance_m or re > preference.orientation_tolerance_rad):
            continue
        if any(np.linalg.norm(solution - other) < 1e-3 for other in seen):
            continue
        seen.append(solution)
        if not configuration_is_valid(solution.copy()):
            rejected_collision += 1
            continue
        height = elbow_height(chain, solution, preference)
        sigma = float(np.linalg.svd(chain.jacobian(solution), compute_uv=False)[-1])
        margin = np.minimum(solution - lower, upper - solution) / (upper - lower)
        cost = (
            preference.elbow_weight * (max(0.0, preference.preferred_height_m - height) / preference.length_scale_m)**2
            + preference.motion_weight * float(np.sum(((solution - q) / np.pi)**2))
            + preference.limit_weight * float(np.sum((np.maximum(0.0, 0.1 - margin) / 0.1)**2))
            + preference.singularity_weight * (max(0.0, preference.singular_value_reference - sigma)
                                               / preference.singular_value_reference)**2
        )
        candidates.append(PoseIkCandidate(solution.copy(), height, pe, re, sigma, cost))
    candidates.sort(key=lambda item: item.cost)
    return PoseIkSelection(candidates[0] if candidates else None, tuple(candidates),
                           len(seed_list), rejected_collision)
