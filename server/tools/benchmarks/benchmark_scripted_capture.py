"""Run one short recording (8 simulated seconds by default), without a task round.

A deliberate total_simulation_timeout is expected: this is a throughput probe,
not a successful training episode. Reuse the same scene/model/catalog for each
measurement. Startup and finalization are reported separately from recording.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", default="cylindrical_plug_insert")
    parser.add_argument("--seconds", type=float, default=8.)
    parser.add_argument("--physics-only", action="store_true")
    parser.add_argument("--unreal", type=Path)
    parser.add_argument("--port-base", type=int, default=29400)
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1 <= args.seconds <= 60:
        parser.error("seconds must be in [1,60] for a short probe")
    if not args.physics_only and args.catalog is None:
        parser.error("recording requires --catalog")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    entry = "run_scripted_task.py" if args.physics_only else "record_scripted_task.py"
    command = [sys.executable, "-u", str(ROOT / "collection" / entry),
               "--task", args.task, "--scene", str(args.scene.resolve()),
               "--model", str(args.model.resolve()), "--output", str(output / "run"),
               "--max-sim-seconds", str(args.seconds), "--warmup-sim-seconds", "0.08"]
    if args.physics_only:
        command += ["--native-geometry", "--no-save-trace"]
    else:
        command += ["--catalog", str(args.catalog.resolve()), "--offline", "--gpu-index", "0",
                    "--episodes-root", str(output / "episodes"), "--warmup-seconds", "5",
                    "--recording-config", str(ROOT / "config/recording-veryfast.json")]
        for offset, name in enumerate(("control", "capture", "render", "signalling")):
            command += [f"--{name}-port", str(args.port_base + offset)]
        if args.unreal:
            command += ["--unreal", str(args.unreal.resolve())]
    sources = ("collection/run_scripted_task.py", "simulation/components/plug_retention.py",
               "simulation/native_plug_retention.py", "simulation/plug_retention.py",
               "native/local_mujoco_stepper.cpp", "native/sarm_thread_sdf.cpp", "native/posture.cpp",
               "simulation/local_mujoco_stepper.py", "simulation/native_posture.py",
               "simulation/scripted_tasks/motion.py", "simulation/scripted_tasks/plug_insert.py")
    manifest = dict(diagnostic_only=True, command=command, sources_sha256={
        name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources})
    manifest["native_builds"] = {name: json.loads((ROOT / "run/native_acceleration" / f"{name}.json").read_text())
                                  for name in ("local_mujoco_stepper", "sarm_thread", "posture")}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1",
                       MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
    with (output / "run.log").open("w") as log:
        process = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT)
    result = json.loads((output / "run/summary.json").read_text())
    task = result if args.physics_only else result["task"]
    if task.get("reason") != "total_simulation_timeout" or process.returncode not in (0, 1):
        raise RuntimeError(f"Probe did not reach its duration: {task.get('reason')}; see {output / 'run.log'}")
    if args.physics_only:
        report = {key: result[key] for key in ("simulated_seconds", "task_loop_wall_seconds",
                  "execute_simulation_wall_seconds", "initialization_wall_seconds")}
        report["realtime_factor"] = result["simulated_seconds"] / result["task_loop_wall_seconds"]
    else:
        report = {key: result[key] for key in ("recorded_sim_seconds", "recording_wall_seconds",
                  "recording_realtime_factor", "startup_wall_seconds", "finalization_wall_seconds",
                  "end_to_end_wall_seconds", "episode_path")}
        report["frames_per_camera"] = result["episode"]["dataset_frame_count"]
        report["captures"] = result["episode"]["capture_count"]
    report["diagnostic_only"] = True
    (output / "timing.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
