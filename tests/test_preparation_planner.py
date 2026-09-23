import json
import hashlib
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from simulation.preparation_planner import MAX_STEP_RAD, _plan_with_validator


def test_injected_validator_selects_safe_unfold_and_checks_small_steps():
    goal = np.deg2rad([0.0, -67.6, -86.6, 143.2, -85.5, 0.0])
    seen = []

    def validator(position):
        seen.append(position.copy())
        return not (position[2] < 0.0 and position[3] < goal[3] - 0.1)

    result = _plan_with_validator(goal, np.full(6, -np.pi), np.full(6, np.pi), validator)

    assert result["status"] == "planned"
    assert result["candidate"] == "safe-unfold-j1-j6"
    assert result["initial_joint_position_rad"] == [0.0] * 6
    assert np.allclose(result["waypoints_rad"][-1], goal)
    assert len(seen) == result["checked_sample_count"]
    assert result["max_sample_step_rad"] <= MAX_STEP_RAD
    assert result["sample_count"] >= 2 * 301


def test_injected_validator_rejects_all_finite_candidates_with_reason():
    goal = np.deg2rad([0.0, -67.6, -86.6, 143.2, -85.5, 0.0])
    result = _plan_with_validator(goal, np.full(6, -np.pi), np.full(6, np.pi), lambda _: False)

    assert result["status"] == "failed"
    assert result["reason"] == "initial_zero_contact"
    assert "waypoints_rad" not in result


def test_direct_contact_is_not_accepted_when_only_safe_route_is_available():
    goal = np.deg2rad([0.0, -67.6, -86.6, 143.2, -85.5, 0.0])

    def validator(position):
        direct_region = position[2] < 0.0 and position[3] < goal[3] - 0.1
        return not direct_region

    result = _plan_with_validator(goal, np.full(6, -np.pi), np.full(6, np.pi), validator)

    assert result["status"] == "planned"
    assert result["candidate"] != "direct"
    assert result["rejected_candidates"] == []


@pytest.mark.skipif(not __import__("importlib").util.find_spec("mujoco"), reason="MuJoCo unavailable")
def test_default_mujoco_scene_plans_without_basilisk():
    from simulation.preparation_planner import plan_preparation

    root = Path(__file__).resolve().parents[1]
    model = root / "model/SARM/platform/sarm_ground_target_self_collision.xml"
    randomized = {
        "arm_joint_position_rad": [0.0] * 6 + [0.01875, 0.01875],
        "target_position_m": [0.94, 0.039086, 0.4],
        "target_orientation_wxyz": [1.0, 0.0, 0.0, 0.0],
        "target_hinge_position_rad": 0.0,
    }
    result = plan_preparation(model, randomized, [0.0, -67.6, -86.6, 143.2, -85.5, 0.0])

    assert result["status"] == "planned", result
    assert result["schema"] == "arm-preparation-plan/1"
    assert result["strategy"] == "validated-waypoints-v1"
    assert result["validation"] == "sampled-static-contacts"
    assert result["sample_count"] >= 602
    assert result["model_sha256"] == hashlib.sha256(model.read_bytes()).hexdigest()
    assert np.allclose(result["waypoints_rad"][-1], np.deg2rad([0.0, -67.6, -86.6, 143.2, -85.5, 0.0]))


def test_cli_failure_is_nonzero_and_has_explicit_reason(tmp_path):
    payload = {"model_path": str(tmp_path / "missing.xml"), "randomization": {}, "goal_deg": [0] * 6}
    process = subprocess.run(
        [sys.executable, "simulation/preparation_planner.py"],
        input=json.dumps(payload), text=True, capture_output=True,
        cwd=Path(__file__).resolve().parents[1],
    )

    assert process.returncode != 0
    result = json.loads(process.stdout)
    assert result["status"] == "failed"
    assert result["reason"]


@pytest.mark.parametrize('goal_deg,reason', [
    ([0, -60, -75, 130, -75, 0], 'goal_contact'),
    ([0, -65, -85, 141, -85, 0], 'no_valid_candidate_in_finite_search'),
])
def test_rejected_custom_targets_do_not_create_scene_files(tmp_path, goal_deg, reason):
    from space_arm_platform.models import SceneInstanceCreate
    from space_arm_platform.scene_runtime import SceneRuntimeManager

    manager = SceneRuntimeManager(None, project_root=Path(__file__).resolve().parents[1])
    manager.scene_root = tmp_path
    with pytest.raises(ValueError, match=reason):
        manager.create_instance(SceneInstanceCreate(
            seed=123, randomization_profile='teleop-zero-prepare-v2',
            operating_arm_joint_position_deg=goal_deg,
        ))
    assert list(tmp_path.iterdir()) == []
