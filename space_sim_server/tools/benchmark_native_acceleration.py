"""Sequential, fresh-process compute-only A/B trials of posture/native MJ pools.

Never connects to the live backend/UE, mutates the saved scene, or changes
integrator tolerances/control rates. Stores all commands, logs and observations.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CONFIGS = {
    "python-t0": ("python", 0),
    "native-t0": ("native", 0),
    "native-t1": ("native", 1),
    "native-t2": ("native", 2),
    "native-t4": ("native", 4),
    "native-t8": ("native", 8),
    "python-t2": ("python", 2),
}
STATE_FIELDS = (
    "arm_joint_position_rad", "arm_joint_velocity_rad_s", "target_arm_joint_position_rad",
    "joint_position_rad", "joint_velocity_rad_s", "target_joint_position_rad",
    "end_effector_position_body_m", "end_effector_orientation_body_wxyz",
)
SOURCE_FILES = (
    "native/posture.cpp", "native/mjscene_probe.cpp",
    "simulation/native_posture.py", "simulation/online_elbow_ik.py",
    "simulation/serial_chain_kinematics.py", "simulation/mjscene_threadpool.py",
    "simulation/teleop_grasp_unreal.py", "simulation/native_integration.py",
    "tools/profile_simulation_runtime.py",
)


def assert_platform_idle(root: Path = ROOT) -> None:
    state = root / "run/scene_runtime.json"
    if state.is_file():
        document = json.loads(state.read_text(encoding="utf-8-sig"))
        if document.get("phase") in {"launching", "starting_renderer", "starting_simulation", "running"}:
            raise RuntimeError("A live scene is active; run CPU benchmarks only after it is stopped.")


def compare_observations(reference: Path, candidate: Path) -> dict:
    import numpy as np
    a = [json.loads(line) for line in reference.read_text(encoding="utf-8").splitlines()]
    b = [json.loads(line) for line in candidate.read_text(encoding="utf-8").splitlines()]
    if [r["sim_time_ns"] for r in a] != [r["sim_time_ns"] for r in b]:
        raise RuntimeError("Simulation clocks/observation counts differ between trials")
    errors = {}
    for key in STATE_FIELDS:
        if key not in a[0]:
            continue
        x = np.asarray([r[key] for r in a], dtype=float)
        y = np.asarray([r[key] for r in b], dtype=float)
        if x.shape != y.shape or not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
            raise RuntimeError(f"Invalid state in {key}")
        errors[key] = float(np.max(np.abs(x-y)))
    return {"samples": len(a), "max_absolute_errors": errors}


def run(args) -> dict:
    if args.repeat < 1 or not math.isfinite(args.duration) or args.duration < 1:
        raise ValueError("repeat/duration must be positive")
    out = args.output_root.resolve()
    if out.exists():
        raise ValueError("Use a new output-root; existing measurements are never overwritten")
    out.mkdir(parents=True)
    configs = args.configs.split(",")
    if len(configs) != len(set(configs)) or any(c not in CONFIGS for c in configs):
        raise ValueError("Unknown or duplicate configuration")
    if "python-t0" not in configs:
        raise ValueError("Include python-t0 as the accuracy/throughput baseline")
    assert_platform_idle()
    manifest = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "python": sys.executable, "scene": str(args.scene_instance.resolve()),
        "adapter": str(args.adapter_root.resolve()),
        "duration_sim_s": args.duration, "motion": args.motion, "speed": args.speed,
        "repeats": args.repeat, "logical_cpus": os.cpu_count(),
        "excluded": ["startup", "UE/GPU rendering", "network", "recording/disk writer"],
        "order": "forward then reverse on alternating repeats",
        "trials": [],
        "source_sha256": {name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in SOURCE_FILES},
    }
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
    for repeat in range(args.repeat):
        order = configs if repeat % 2 == 0 else list(reversed(configs))
        for config in order:
            assert_platform_idle()
            backend, workers = CONFIGS[config]
            name = f"r{repeat+1}-{config}"
            result_path = out/f"{name}.json"
            command = [
                sys.executable, "-B", str(ROOT/"tools/profile_simulation_runtime.py"),
                "--adapter-root", str(args.adapter_root.resolve()),
                "--scene-instance", str(args.scene_instance.resolve()),
                "--output", str(result_path),
                "--initial-state", "operating", "--duration", str(args.duration),
                "--motion", args.motion, "--speed", str(args.speed),
                "--posture-backend", backend, "--mj-threads", str(workers),
            ]
            environment = os.environ.copy()
            environment["PYTHONIOENCODING"] = "utf-8"
            environment["PYTHONDONTWRITEBYTECODE"] = "1"
            print(f"START {name}: {args.duration:g}s {args.motion}", flush=True)
            with (out/f"{name}.log").open("wb") as stream:
                process = subprocess.run(command, cwd=ROOT, env=environment,
                                         stdout=stream, stderr=subprocess.STDOUT, timeout=600)
            manifest["trials"].append({"name":name,"config":config,"command":command,
                                       "returncode":process.returncode})
            (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
            if process.returncode:
                raise RuntimeError(f"{name} failed with {process.returncode}; see its log")
            result = json.loads(result_path.read_text(encoding="utf-8"))
            print(f"RESULT {name}: execute={result['execute_wall_s']:.3f}s "
                  f"CPU={result['execute_cpu_s']:.3f}s RTF={result['compute_real_time_factor']:.3f} "
                  f"contacts={result['mjscene_probe']['max_contacts']}", flush=True)
    baseline = out/"r1-python-t0.observations.jsonl"
    summary = {}
    for config in configs:
        samples = [json.loads((out/f"r{r+1}-{config}.json").read_text(encoding="utf-8"))
                   for r in range(args.repeat)]
        comparisons = [
            compare_observations(baseline, out/f"r{r+1}-{config}.observations.jsonl")
            for r in range(args.repeat)
        ]
        summary[config] = {
            "execute_wall_s": [s["execute_wall_s"] for s in samples],
            "median_execute_wall_s": statistics.median(s["execute_wall_s"] for s in samples),
            "compute_rtf": [s["compute_real_time_factor"] for s in samples],
            "median_compute_rtf": statistics.median(s["compute_real_time_factor"] for s in samples),
            "max_contacts": max(s["mjscene_probe"]["max_contacts"] for s in samples),
            "max_constraint_islands": max(s["mjscene_probe"]["max_constraint_islands"] for s in samples),
            "state_comparisons": comparisons,
        }
    base_time = summary["python-t0"]["median_execute_wall_s"]
    for result in summary.values():
        result["speedup_vs_python"] = base_time/result["median_execute_wall_s"]
        result["elapsed_reduction_percent"] = 100*(1-result["median_execute_wall_s"]/base_time)
    document = {"manifest": manifest, "summary": summary}
    (out/"summary.json").write_text(json.dumps(document,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"summary_file":str(out/"summary.json"),"summary":summary},indent=2),flush=True)
    return document


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter-root",type=Path,required=True)
    parser.add_argument("--scene-instance",type=Path,required=True)
    parser.add_argument("--output-root",type=Path,required=True)
    parser.add_argument("--duration",type=float,default=8)
    parser.add_argument("--motion",choices=("hold","linear_x","linear_y","linear_z","roll","pitch","yaw"),default="linear_x")
    parser.add_argument("--speed",type=float,default=.05)
    parser.add_argument("--repeat",type=int,default=2)
    parser.add_argument("--configs",default="python-t0,native-t0,native-t1,native-t2,native-t4,native-t8")
    run(parser.parse_args())
