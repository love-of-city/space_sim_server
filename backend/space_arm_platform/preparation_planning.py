"""Run startup trajectory screening outside the authoritative physics process."""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import time


def preparation_ready_for_recording(instance: dict, status: dict) -> bool:
    observation = status.get("latest_observation") or {}
    preparation = observation.get("arm_preparation") or {}
    try:
        age = (time.time_ns() - int(observation["wall_time_ns"])) / 1e9
    except (KeyError, ValueError, TypeError):
        return False
    return bool(status.get("connected") and not status.get("resetting")
                and observation.get("scene_instance_id") == instance.get("instance_id")
                and 0 <= age < 2 and preparation.get("status") == "ready"
                and preparation.get("ready") is True)


def prepare_arm_plan(model_path: Path, randomized: dict, goal_deg: list[float]) -> dict:
    root = Path(__file__).resolve().parents[2]
    python = os.environ.get("SPACE_SIM_POSTURE_PYTHON") or sys.executable
    environment = os.environ.copy()
    environment.pop("MUJOCO_GL", None)
    try:
        completed = subprocess.run(
            [python, str(root / "simulation/preparation_planner.py")],
            input=json.dumps({"model_path": str(model_path.resolve()),
                              "randomization": randomized, "goal_deg": goal_deg}),
            capture_output=True, text=True, encoding="utf-8", timeout=60,
            env=environment,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        report = json.loads(completed.stdout)
        if not isinstance(report, dict):
            raise ValueError("路径规划器未返回对象")
        if completed.returncode or report.get("schema") != "arm-preparation-plan/1":
            raise ValueError(report.get("reason") or "路径规划器返回无效结果")
        from simulation.preparation_contract import validate_preparation_plan
        validate_preparation_plan(report, goal_deg)
        return report
    except (OSError, subprocess.SubprocessError, ValueError, TypeError, KeyError) as error:
        raise ValueError(f"无法从零位展开到操作姿态：{error}") from error
