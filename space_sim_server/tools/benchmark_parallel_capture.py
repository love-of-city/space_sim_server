"""Small parallel benchmark of the unchanged production collector (CPU processes).

Results and episodes stay under --output. A short --seconds run is explicitly
reported as a bounded task segment, not as a successful drawer demonstration.
Run GPU experiments separately: simultaneous suites invalidate timings.
"""

from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from datetime import datetime
import psutil

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--mode", choices=["physics", "capture"], default="physics")
    p.add_argument("--workers", type=int, choices=[1, 2, 3, 4], default=2)
    p.add_argument("--jobs", type=int, choices=[1, 2, 3, 4], default=2)
    p.add_argument("--seconds", type=float, default=180)
    p.add_argument("--scene", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--port-base", type=int, default=19760)
    args = p.parse_args()
    if not 0 < args.seconds <= 540:
        p.error("--seconds must be in (0,540]")
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    report = dict(
        mode=args.mode,
        workers=args.workers,
        jobs=args.jobs,
        max_sim_seconds=args.seconds,
        started_at=datetime.now().astimezone().isoformat(),
        commands=[],
        runs=[],
        resources=[],
        complete=False,
        baseline=dict(
            logical_cpus=psutil.cpu_count(),
            memory_total_bytes=psutil.virtual_memory().total,
            memory_available_bytes=psutil.virtual_memory().available,
        ),
    )
    active = []
    pending = list(range(args.jobs))
    started = time.perf_counter()
    last_sample = -10.0
    last_gpu = -10.0
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    try:
        while pending or active:
            while pending and len(active) < args.workers:
                index = pending.pop(0)
                dest = out / f"world-{index}"
                script = (
                    "record_drawer_task.py"
                    if args.mode == "capture"
                    else "run_scripted_task.py"
                )
                command = [
                    sys.executable,
                    str(ROOT / "tools" / script),
                    "--task",
                    "drawer_cycle",
                    "--scene",
                    str(args.scene.resolve()),
                    "--output",
                    str(dest),
                    "--max-sim-seconds",
                    str(args.seconds),
                    "--substeps",
                    "1",
                ]
                if args.mode == "capture":
                    port = args.port_base + index * 10
                    command += [
                        "--episodes-root",
                        str(out / "episodes"),
                        "--timeout-seconds",
                        "3600",
                        "--control-port",
                        str(port),
                        "--capture-port",
                        str(port + 1),
                        "--render-port",
                        str(port + 2),
                        "--signalling-port",
                        str(port + 3),
                    ]
                log = (out / f"world-{index}.log").open("w", encoding="utf-8")
                begin = time.perf_counter()
                proc = subprocess.Popen(
                    command,
                    cwd=ROOT,
                    env=env,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                active.append((index, proc, log, begin, dest))
                report["commands"].append(command)
                print(
                    json.dumps(dict(event="started", world=index, pid=proc.pid)),
                    flush=True,
                )
            now = time.perf_counter()
            if now - started - last_sample >= 1:
                tree = {}
                for _, proc, _, _, _ in active:
                    try:
                        parent = psutil.Process(proc.pid)
                        for child in [parent, *parent.children(recursive=True)]:
                            tree[child.pid] = child
                    except psutil.Error:
                        pass
                rss = cpu = 0
                ue_process_count = 0
                for child in tree.values():
                    try:
                        rss += child.memory_info().rss
                        if child.name().lower().startswith("unrealeditor"):
                            ue_process_count += 1
                        times = child.cpu_times()
                        cpu += times.user + times.system
                    except psutil.Error:
                        pass
                sample = dict(
                    elapsed_s=now - started,
                    rss_bytes=rss,
                    process_count=len(tree),
                    active_workers=len(active),
                    ue_process_count=ue_process_count,
                    system_memory_available_bytes=psutil.virtual_memory().available,
                    process_cpu_seconds=cpu,
                    system_cpu_percent=psutil.cpu_percent(),
                )
                if now - started - last_gpu >= 5:
                    gpu = subprocess.run(
                        [
                            "nvidia-smi",
                            "--query-gpu=utilization.gpu,memory.used",
                            "--format=csv,noheader,nounits",
                        ],
                        capture_output=True,
                        text=True,
                        creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                        timeout=10,
                    )
                    if gpu.returncode == 0:
                        util, mem = gpu.stdout.strip().splitlines()[0].split(",")
                        sample.update(gpu_percent=int(util), gpu_memory_mib=int(mem))
                    last_gpu = now - started
                report["resources"].append(sample)
                last_sample = now - started
            for entry in active[:]:
                index, proc, log, begin, dest = entry
                if proc.poll() is None:
                    continue
                log.close()
                result = dict(
                    world=index,
                    exit_code=proc.returncode,
                    wall_seconds=time.perf_counter() - begin,
                    started_elapsed_s=begin - started,
                    finished_elapsed_s=time.perf_counter() - started,
                )
                path = dest / "summary.json"
                if path.exists():
                    summary = json.loads(path.read_text(encoding="utf-8"))
                    result.update(
                        result=summary.get("result"),
                        reason=summary.get("reason"),
                        summary_path=str(path),
                        simulated_seconds=summary.get("simulated_seconds"),
                        recording_wall_seconds=summary.get("recording_wall_seconds"),
                    )
                    if args.mode == "capture":
                        episode = summary.get("episode", {})
                        result.update(
                            episode_id=summary.get("episode_id"),
                            episode_path=summary.get("episode_path"),
                            task=summary.get("task"),
                            dataset_status=episode.get("dataset_status"),
                            frames=episode.get("dataset_frame_count"),
                            images=episode.get("capture_count"),
                            incomplete=episode.get("incomplete_sample_count"),
                            rejected=episode.get("rejected_capture_count"),
                        )
                        result["valid"] = (
                            episode.get("dataset_status") == "complete"
                            and episode.get("dataset_frame_count", 0) > 0
                            and episode.get("capture_count")
                            == 2 * episode.get("dataset_frame_count", 0)
                            and episode.get("incomplete_sample_count") == 0
                            and episode.get("rejected_capture_count") == 0
                            and not summary.get("error")
                        )
                    else:
                        result["valid"] = summary.get("result") == "success" or (
                            summary.get("reason") == "total_simulation_timeout"
                            and summary.get("simulated_seconds", 0) >= args.seconds
                        )
                else:
                    result["valid"] = False
                report["runs"].append(result)
                print(json.dumps(dict(event="completed", **result)), flush=True)
                active.remove(entry)
            time.sleep(0.1)
        report["complete"] = len(report["runs"]) == args.jobs and all(
            r["valid"] for r in report["runs"]
        )
    finally:
        report["wall_seconds"] = time.perf_counter() - started
        report["finished_at"] = datetime.now().astimezone().isoformat()
        for _, proc, log, _, _ in active:
            try:
                parent = psutil.Process(proc.pid)
                children = parent.children(recursive=True)
                for child in reversed(children):
                    child.terminate()
                parent.terminate()
                psutil.wait_procs([*children, parent], timeout=5)
            except psutil.Error:
                pass
            log.close()
        (out / "benchmark.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
    print(
        json.dumps(
            dict(
                event="benchmark_complete",
                wall_seconds=report["wall_seconds"],
                complete=report["complete"],
                report=str(out / "benchmark.json"),
            )
        ),
        flush=True,
    )
    return 0 if report["complete"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
