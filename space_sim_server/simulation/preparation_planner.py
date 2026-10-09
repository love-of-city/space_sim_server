"""Offline zero-start planning; run this worker outside the live simulator.

Integration: plan_preparation(Path(model_path), randomization, goal_deg) returns
an arm-preparation-plan/1 dictionary. The CLI accepts those three JSON fields
on stdin and emits one JSON object; status='failed' also exits with code 1.
Only status='planned' contains executable waypoints_rad, without a start prefix
and ending at the literal six-axis goal (no angle wrapping or goal adjustment).
The six arm axes start at zero; randomized fingers and target remain fixed.

Sampling is linear in joint space, at most 0.5 degrees per axis, with at least
301 samples per nonzero segment. sample_count counts the accepted route only,
including both ends of every segment (shared boundaries count twice).
checked_sample_count additionally counts endpoint checks and rejected routes.
model_sha256 hashes the selected MJCF file bytes, not its external assets.
This finite candidate search screens static arm-involving contacts only; it
does not prove continuous/dynamic safety or implement motion execution.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Callable, Iterator

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from simulation.prepare_elbow_posture import ALL_NAMES, ARM_NAMES, SceneConfigurationValidator
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.joint_limits import load_joint_limits

SCHEMA = "arm-preparation-plan/1"
STRATEGY = "validated-waypoints-v1"
MAX_STEP_RAD = math.radians(0.5)
MIN_SEGMENT_INTERVALS = 300


def _report() -> dict:
    return {
        "schema": SCHEMA,
        "strategy": STRATEGY,
        "status": "failed",
        "validation": "sampled-static-contacts",
        "initial_joint_position_rad": [0.0] * 6,
        "max_sample_step_rad": MAX_STEP_RAD,
        "sample_count": 0,
        "checked_sample_count": 0,
        "attempted_candidates": 0,
        "rejected_candidates": [],
    }


def _finite_vector(values, size: int, name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (size,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain exactly {size} finite values")
    return vector.copy()


def _candidate_waypoints(goal: np.ndarray) -> Iterator[tuple[str, list[np.ndarray]]]:
    zero = np.zeros(6)
    if np.array_equal(goal, zero):
        yield "already-at-goal", [goal.copy()]
        return
    opening = np.array([0.0, goal[1], math.radians(30.0), goal[3], 0.0, 0.0])
    middle = goal.copy()
    middle[[0, 5]] = 0.0
    seen = set()
    for name, first_axis in (("safe-unfold-j1-j6", 0), ("safe-unfold-j6-j1", 5)):
        rotation = middle.copy()
        rotation[first_axis] = goal[first_axis]
        waypoints = []
        previous = zero
        for waypoint in (opening, middle, rotation, goal):
            if not np.array_equal(waypoint, previous):
                waypoints.append(waypoint.copy())
                previous = waypoint
        identity = tuple(tuple(waypoint) for waypoint in waypoints)
        if identity not in seen:
            seen.add(identity)
            yield name, waypoints
    yield "direct", [goal.copy()]


def _segment_samples(start: np.ndarray, end: np.ndarray) -> Iterator[np.ndarray]:
    distance = float(np.max(np.abs(end - start)))
    if distance == 0.0:
        yield end.copy()
        return
    intervals = max(MIN_SEGMENT_INTERVALS, math.ceil(distance / MAX_STEP_RAD))
    for index in range(intervals + 1):
        if index == 0:
            yield start.copy()
        elif index == intervals:
            yield end.copy()
        else:
            fraction = index / intervals
            yield (1.0 - fraction) * start + fraction * end


def _plan_with_validator(
    goal_rad: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    configuration_is_valid: Callable[[np.ndarray], bool],
) -> dict:
    """Pure search seam: False means contact; validator exceptions fail closed."""
    report = _report()
    try:
        goal = _finite_vector(goal_rad, 6, "goal_joint_position_rad")
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if (lower.shape != (6,) or upper.shape != (6,)
                or np.any(np.isnan(lower)) or np.any(np.isnan(upper))
                or np.any(lower >= upper)):
            raise ValueError("invalid six-axis joint limits")
        report["goal_joint_position_rad"] = goal.tolist()

        def in_limits(position):
            return bool(np.all(np.isfinite(position))
                        and np.all(position >= lower) and np.all(position <= upper))

        def valid(position):
            report["checked_sample_count"] += 1
            return bool(configuration_is_valid(position.copy()))

        zero = np.zeros(6)
        for name, position in (("initial_zero", zero), ("goal", goal)):
            if not in_limits(position):
                report["reason"] = f"{name}_outside_joint_limits"
                return report
        for name, position in (("initial_zero", zero), ("goal", goal)):
            if not valid(position):
                report["reason"] = f"{name}_contact"
                return report

        for name, waypoints in _candidate_waypoints(goal):
            report["attempted_candidates"] += 1
            rejection = None
            for index, waypoint in enumerate(waypoints):
                if not in_limits(waypoint):
                    rejection = {"candidate": name, "reason": "waypoint_outside_joint_limits",
                                 "waypoint_index": index}
                    break
            sample_count = 0
            start = zero
            if rejection is None:
                for segment_index, end in enumerate(waypoints):
                    for sample_index, position in enumerate(_segment_samples(start, end)):
                        sample_count += 1
                        if not in_limits(position) or not valid(position):
                            rejection = {"candidate": name, "reason": "sample_contact_or_limit",
                                         "segment_index": segment_index, "sample_index": sample_index,
                                         "joint_position_rad": position.tolist()}
                            break
                    if rejection is not None:
                        break
                    start = end
            if rejection is not None:
                report["rejected_candidates"].append(rejection)
                continue
            report.update(status="planned", reason="validated_candidate", candidate=name,
                          waypoints_rad=[waypoint.tolist() for waypoint in waypoints],
                          sample_count=sample_count)
            return report
        report["reason"] = "no_valid_candidate_in_finite_search"
    except Exception as error:
        report["reason"] = f"{type(error).__name__}: {error}"
    return report


def plan_preparation(model_path: Path, randomized: dict, goal_deg: list[float]) -> dict:
    """Plan in an isolated Python process; never import the live simulator."""
    report = _report()
    try:
        goal = np.deg2rad(_finite_vector(goal_deg, 6, "goal_deg"))
        report["goal_joint_position_rad"] = goal.tolist()
        model_path = Path(model_path).resolve(strict=True)
        report["model_sha256"] = hashlib.sha256(model_path.read_bytes()).hexdigest()
        joints = _finite_vector(randomized["arm_joint_position_rad"], 8, "arm_joint_position_rad")
        _finite_vector(randomized["target_position_m"], 3, "target_position_m")
        orientation = _finite_vector(randomized["target_orientation_wxyz"], 4, "target_orientation_wxyz")
        if not np.any(orientation):
            raise ValueError("target_orientation_wxyz must be nonzero")
        if "target_hinge_position_rad" in randomized:
            _finite_vector([randomized["target_hinge_position_rad"]], 1, "target_hinge_position_rad")
        limits = load_joint_limits(model_path, ALL_NAMES)
        lower, upper = np.asarray(limits.lower), np.asarray(limits.upper)
        if np.any(joints[6:] < lower[6:]) or np.any(joints[6:] > upper[6:]):
            raise ValueError("randomized fingers outside joint limits")
        chain = SerialChainKinematics.from_mjcf(
            model_path, base_body="cubesat_bus", joint_names=ARM_NAMES, tool_site="sarm_ee",
        )
        scene = dict(randomized)
        scene["arm_joint_position_rad"] = [0.0] * 6 + joints[6:].tolist()
        validator = SceneConfigurationValidator(model_path, scene, chain)
        report.update(_plan_with_validator(goal, lower[:6], upper[:6], validator))
        report["mujoco_version"] = validator.mj.__version__
    except Exception as error:
        report.update(status="failed", reason=f"{type(error).__name__}: {error}")
        report.pop("waypoints_rad", None)
    return report


def main() -> int:
    try:
        payload = json.loads(sys.stdin.buffer.read().decode("utf-8-sig"))
        result = plan_preparation(Path(payload["model_path"]), payload["randomization"], payload["goal_deg"])
    except Exception as error:
        result = _report()
        result["reason"] = f"{type(error).__name__}: {error}"
    print(json.dumps(result, allow_nan=False))
    return 0 if result["status"] == "planned" else 1


if __name__ == "__main__":
    sys.exit(main())
