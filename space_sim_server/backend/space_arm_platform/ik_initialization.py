"""Standard-library bridge to startup-only IK in an isolated Python process."""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import subprocess
import sys


def prepare_initial_posture(model_path: Path, randomized: dict) -> dict:
    mode = os.environ.get("SPACE_SIM_ELBOW_MODE", "prefer")
    if mode not in {"off", "prefer"}:
        raise ValueError("SPACE_SIM_ELBOW_MODE must be off or prefer")
    if mode == "off":
        return {"mode": mode, "status": "skipped", "reason": "disabled"}
    root = Path(__file__).resolve().parents[2]
    try:
        python = os.environ.get("SPACE_SIM_POSTURE_PYTHON")
        if not python:
            config_path = root / "run" / "ik_posture_worker.json"
            config = json.loads(config_path.read_text(encoding="utf-8-sig")) if config_path.is_file() else {}
            if not isinstance(config, dict):
                raise ValueError("posture worker configuration must be an object")
            python = config.get("python") or sys.executable
        if not isinstance(python, str) or not python.strip():
            raise ValueError("posture worker Python must be an executable path")
        completed = subprocess.run(
            [python, str(root / "simulation" / "prepare_elbow_posture.py")],
            input=json.dumps({"model_path": str(model_path.resolve()), "randomization": randomized}),
            capture_output=True, text=True, encoding="utf-8", timeout=45, check=True,
            # Do not create visible helper windows on Windows.
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        report = json.loads(completed.stdout)
        if not isinstance(report, dict) or report.get("status") not in {"selected", "kept", "fallback"}:
            raise ValueError("invalid posture worker status")
        if report["status"] in {"selected", "kept"}:
            q = report.get("joint_position_rad")
            if not isinstance(q, list) or len(q) != 6 or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in q):
                raise ValueError("invalid posture worker joint vector")
        return report
    except (OSError, subprocess.SubprocessError, ValueError, TypeError) as error:
        return {"mode": mode, "status": "fallback", "reason": f"{type(error).__name__}: {error}"}
