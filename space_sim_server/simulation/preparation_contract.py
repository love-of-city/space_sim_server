"""Saved six-axis preparation plans, with no physics imports."""
from __future__ import annotations

import math


def validate_preparation_plan(plan: dict, goal_deg: list[float]) -> None:
    if not isinstance(plan, dict) or plan.get("schema") != "arm-preparation-plan/1":
        raise ValueError("missing or unsupported preparation plan")
    if plan.get("strategy") != "validated-waypoints-v1":
        raise ValueError("unsupported preparation strategy")
    vectors = [plan.get("initial_joint_position_rad"), plan.get("goal_joint_position_rad")]
    waypoints = plan.get("waypoints_rad")
    if not isinstance(waypoints, list) or not 1 <= len(waypoints) <= 20:
        raise ValueError("invalid preparation waypoints")
    vectors.extend(waypoints)
    for vector in vectors:
        if not isinstance(vector, list) or len(vector) != 6 or not all(
            isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)
            for value in vector
        ):
            raise ValueError("preparation plan requires six finite joint angles")
    if any(abs(value) > 1e-12 for value in vectors[0]):
        raise ValueError("preparation plan must start at zero")
    expected = [math.radians(value) for value in goal_deg]
    if len(expected) != 6 or not all(math.isfinite(value) for value in expected):
        raise ValueError("invalid preparation goal")
    if any(abs(actual - goal) > 1e-10 for vector in [vectors[1], waypoints[-1]]
           for actual, goal in zip(vector, expected, strict=True)):
        raise ValueError("preparation goal does not match saved plan")
