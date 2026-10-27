"""Run isolated scripted collection workers and report validated throughput.

Resource settings live in config/collection.json. Each worker owns a renderer,
four ports, a disjoint seed range, logs, and its own dataset directory.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]

import psutil
from pydantic import BaseModel, ConfigDict, Field, model_validator
from space_arm_platform.collection_processes import ManagerLease, WindowsProcessGroup
from space_arm_platform.collection_runtime import check_port_available
from space_arm_platform.recording_config import load_recording_config
from space_arm_platform.unreal_runtime import default_unreal_root
from simulation.scripted_tasks.plug_profiles import PLUG_PROFILES


class WorkerConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    gpu_index: int = Field(default=0, ge=0, le=255)
    cpu_affinity: list[int] = Field(default_factory=list)
    math_threads: int = Field(default=1, ge=1, le=64)

    @model_validator(mode="after")
    def validate_cpus(self):
        if any(cpu < 0 for cpu in self.cpu_affinity) or len(set(self.cpu_affinity)) != len(self.cpu_affinity):
            raise ValueError("CPU affinity requires distinct nonnegative CPU indices")
        return self


class CollectionConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    workers: list[WorkerConfig] = Field(min_length=1, max_length=32)
    episodes_per_worker: int = Field(default=2, ge=1, le=1000)
    max_attempts_per_worker: int = Field(default=4, ge=1, le=1000)
    seed: int = Field(default=2026101900, ge=0)
    range_deg: float = Field(default=10.0, ge=0, le=50, allow_inf_nan=False)
    randomize_orbit_phase: bool = False
    randomize_overview_camera: bool = False
    layout_pool: str | None = None
    port_base: int = Field(default=21000, ge=1024, le=65000)
    offline: bool = True
    reuse_renderer: bool = True
    reuse_dynamics: bool = False
    dynamics_max_episodes: int = Field(default=4, ge=1, le=1000)
    dynamics_max_rss_mb: int = Field(default=8192, ge=1)
    simulation_startup_timeout_seconds: float = Field(default=600.0, gt=0, le=7200)
    warmup_seconds: float = Field(default=5.0, ge=0, le=60)
    reuse_warmup_seconds: float = Field(default=5.0, ge=0, le=60)
    recording_config: str = "recording.json"
    sample_seconds: float = Field(default=3.0, ge=1, le=60)
    worker_timeout_seconds: float = Field(default=7200.0, ge=30, le=86400)

    @model_validator(mode="after")
    def check_budget(self):
        if self.episodes_per_worker > self.max_attempts_per_worker:
            raise ValueError("episode target exceeds attempt budget")
        return self


def worker_plan(config, index):
    base = config.port_base + index * 10
    return dict(worker=index, seed=config.seed + index * config.max_attempts_per_worker,
                ports=dict(control=base, capture=base + 1, render=base + 2, signalling=base + 3))


def atomic_json(path, document):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(document, ensure_ascii=False, indent=2), encoding="utf-8")
    temporary.replace(path)


def validated_throughput(reports, seconds):
    accepted = sum(len(report.get("accepted_episodes", [])) for report in reports)
    return dict(accepted_episodes=accepted,
                accepted_episodes_per_hour=accepted * 3600 / seconds if seconds > 0 else 0)


class ResourceSampler:
    def __init__(self):
        self.last_cpu = {}
        self.last_time = time.monotonic()

    def sample(self, roots):
        now = time.monotonic()
        found = {}
        for pid in roots:
            try:
                root = psutil.Process(pid)
                for process in [root, *root.children(recursive=True)]:
                    found[process.pid] = process
            except psutil.NoSuchProcess:
                pass
        cpu_delta, rss = 0.0, 0
        current = {}
        for pid, process in found.items():
            try:
                times = process.cpu_times()
                key = (pid, process.create_time())
                current[key] = times.user + times.system
                if key in self.last_cpu:
                    cpu_delta += max(0, current[key] - self.last_cpu[key])
                rss += process.memory_info().rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        self.last_cpu = current
        percent = 100 * cpu_delta / max(now - self.last_time, 0.001)
        self.last_time = now
        result = dict(time_ns=str(time.time_ns()), process_count=len(current),
                      worker_cpu_percent_one_core=percent, worker_rss_bytes=rss,
                      host_cpu_percent=psutil.cpu_percent(), host_memory_percent=psutil.virtual_memory().percent)
        try:
            query = subprocess.run(["nvidia-smi", "--query-gpu=index,uuid,utilization.gpu,memory.used,utilization.encoder",
                                    "--format=csv,noheader,nounits"], capture_output=True, text=True,
                                   timeout=5, creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            result["gpu_system_csv"] = query.stdout.strip() if query.returncode == 0 else query.stderr.strip()
        except (OSError, subprocess.TimeoutExpired) as error:
            result["gpu_query_error"] = str(error)
        return result


def resource_summary(samples):
    if not samples:
        return {}
    # GPU utilization includes any unrelated GPU users; RSS can count shared pages
    # more than once. CPU is the sum of worker trees (100% equals one logical core).
    cpu = [sample["worker_cpu_percent_one_core"] for sample in samples[1:]]
    return dict(samples=len(samples),
                worker_cpu_percent_one_core_mean=sum(cpu) / len(cpu) if cpu else 0,
                worker_cpu_percent_one_core_peak=max(cpu, default=0),
                worker_rss_bytes_peak=max(sample["worker_rss_bytes"] for sample in samples),
                host_cpu_percent_peak=max(sample["host_cpu_percent"] for sample in samples))


def run_worker(plan_path):
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if plan["cpu_affinity"]:
        psutil.Process().cpu_affinity(plan["cpu_affinity"])
    # Apply limits before importing any numerical libraries in the collector.
    for name in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[name] = str(plan["math_threads"])
    import runpy
    sys.argv = plan["command"][1:]
    runpy.run_path(str(ROOT / "tools/collect_scripted_tasks.py"), run_name="__main__")


def run(args):
    config = CollectionConfig.model_validate_json(args.config.read_text(encoding="utf-8-sig"))
    if args.workers is not None:
        if not 1 <= args.workers <= len(config.workers):
            raise ValueError("--workers must select 1..configured worker count")
        config.workers = config.workers[:args.workers]
    if args.episodes_per_worker is not None:
        config.episodes_per_worker = args.episodes_per_worker
    config = CollectionConfig.model_validate(config.model_dump())
    recording_path = (args.config.parent / config.recording_config).resolve()
    if not recording_path.is_file():
        raise FileNotFoundError(recording_path)
    recording = load_recording_config(recording_path)
    layout_pool = None
    if config.layout_pool:
        from space_arm_platform.collection_layouts import LayoutPool
        layout_pool = LayoutPool((args.config.parent / config.layout_pool).resolve())
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    pool_snapshot = None
    if layout_pool is not None:
        pool_snapshot = output / "layout-pool.json"
        atomic_json(pool_snapshot, layout_pool.document)
    episodes = args.episodes_root.resolve() if args.episodes_root else output / "episodes"
    entries, leases, handles, processes, groups = [], [], [], [], []
    started = time.monotonic()
    report = dict(schema="parallel-collection/1", status="starting", config=config.model_dump(),
                  requested_episodes=len(config.workers) * config.episodes_per_worker,
                  source_scene=str(args.scene.resolve()), workers=entries)
    stop = False

    def request_stop(*_):
        nonlocal stop
        stop = True

    previous = {sig: signal.signal(sig, request_stop) for sig in (signal.SIGINT, signal.SIGTERM)}
    sampler = ResourceSampler()
    measurements = []
    try:
        # Shared port leases cover the bind/start race between CLI coordinators.
        lock_dir = Path(tempfile.gettempdir()) / "spacesim-collection-ports"
        lock_dir.mkdir(exist_ok=True)
        for i in range(len(config.workers)):
            for port in worker_plan(config, i)["ports"].values():
                leases.append(ManagerLease(lock_dir / f"{port}.lock"))
                check_port_available(port)
        for i, resource in enumerate(config.workers):
            entry = worker_plan(config, i)
            directory = output / f"worker-{i:02d}"
            directory.mkdir()
            cancel = directory / "cancel.request"
            per_worker = recording.model_copy(deep=True)
            per_worker.video = type(per_worker.video).model_validate(
                {**per_worker.video.model_dump(), "gpu_index": resource.gpu_index})
            atomic_json(directory / "recording.json", per_worker.model_dump(mode="json"))
            command = [sys.executable, str(ROOT / "tools/collect_scripted_tasks.py"),
                       "--scene", str(args.scene.resolve()), "--task", args.task,
                       "--output", str(directory / "batch"), "--episodes-root", str(episodes / f"worker-{i:02d}"),
                       "--cancel-file", str(cancel), "--recording-config", str(directory / "recording.json"),
                       "--adapter", str(args.adapter.resolve()), "--unreal", str(args.unreal.resolve()),
                       "--count", str(config.episodes_per_worker), "--max-attempts", str(config.max_attempts_per_worker),
                       "--seed", str(entry["seed"]), "--range-deg", str(config.range_deg),
                       "--gpu-index", str(resource.gpu_index), "--warmup-seconds", str(config.warmup_seconds),
                       "--reuse-warmup-seconds", str(config.reuse_warmup_seconds),
                       "--simulation-startup-timeout-seconds", str(config.simulation_startup_timeout_seconds)]
            if layout_pool is not None:
                command += ["--layout-pool", str(pool_snapshot)]
            for name, port in entry["ports"].items():
                command += [f"--{name}-port", str(port)]
            command.append("--randomize-orbit-phase" if config.randomize_orbit_phase
                           else "--no-randomize-orbit-phase")
            command.append("--randomize-overview-camera" if config.randomize_overview_camera
                           else "--no-randomize-overview-camera")
            if config.offline:
                command.append("--offline")
            if config.reuse_renderer:
                command.append("--reuse-renderer")
            if config.reuse_dynamics:
                command += ["--reuse-dynamics", "--dynamics-max-episodes", str(config.dynamics_max_episodes),
                            "--dynamics-max-rss-mb", str(config.dynamics_max_rss_mb)]
            plan = dict(command=command, **resource.model_dump())
            atomic_json(directory / "plan.json", plan)
            log = (directory / "worker.log").open("w", encoding="utf-8")
            handles.append(log)
            env = dict(os.environ, PYTHONUNBUFFERED="1", PYTHONIOENCODING="utf-8")
            process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()), "--worker-plan", str(directory / "plan.json")],
                                       env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=os.name != "nt",
                                       creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            processes.append(process)
            groups.append(WindowsProcessGroup(process))
            entry.update(pid=process.pid, directory=str(directory), resources=resource.model_dump())
            entries.append(entry)
        report["status"] = "running"
        atomic_json(output / "summary.json", report)
        with (output / "resources.jsonl").open("w", encoding="utf-8") as samples:
            while any(process.poll() is None for process in processes):
                if stop:
                    report["status"] = "cancelled"
                    break
                if time.monotonic() - started > config.worker_timeout_seconds:
                    raise TimeoutError("Parallel collection exceeded worker_timeout_seconds")
                if any(process.poll() not in (None, 0) for process in processes):
                    raise RuntimeError("A worker failed; stopping remaining workers; inspect worker.log")
                measurement = sampler.sample([p.pid for p in processes if p.poll() is None])
                measurements.append(measurement)
                samples.write(json.dumps(measurement) + "\n")
                samples.flush()
                time.sleep(config.sample_seconds)
        if not stop:
            report["status"] = "complete" if all(p.returncode == 0 for p in processes) else "failed"
    except BaseException as error:
        report["status"] = "failed"
        report["error"] = f"{type(error).__name__}: {error}"
    finally:
        for entry in entries:
            (Path(entry["directory"]) / "cancel.request").write_text("coordinator stopped", encoding="utf-8")
        deadline = time.monotonic() + 150
        while any(p.poll() is None for p in processes) and time.monotonic() < deadline:
            time.sleep(0.2)
        for process in processes:
            # Terminate only this worker's session, including orphan descendants.
            if os.name != "nt":
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
            elif process.poll() is None:
                process.kill()
            process.wait(timeout=10)
        for group in groups:
            group.close()
        for handle in handles:
            handle.close()
        reports = []
        for entry, process in zip(entries, processes):
            entry["returncode"] = process.returncode
            path = Path(entry["directory"]) / "batch/summary.json"
            if path.exists():
                batch = json.loads(path.read_text(encoding="utf-8"))
                reports.append(batch)
                entry["batch"] = batch
        report["wall_seconds"] = time.monotonic() - started
        report.update(validated_throughput(reports, report["wall_seconds"]))
        report["resource_summary"] = resource_summary(measurements)
        if report["status"] == "complete" and report["accepted_episodes"] != report["requested_episodes"]:
            report["status"] = "failed"
            report["error"] = "Validated episode count does not match requested count"
        atomic_json(output / "summary.json", report)
        for lease in leases:
            lease.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    print(json.dumps({k: report[k] for k in ("status", "wall_seconds", "accepted_episodes", "accepted_episodes_per_hour")}), flush=True)
    return 0 if report["status"] == "complete" else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker-plan", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--config", type=Path, default=ROOT / "config/collection.json")
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--episodes-root", type=Path)
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter")
    parser.add_argument("--unreal", type=Path, default=default_unreal_root())
    parser.add_argument("--task", default="drawer_cycle", choices=["drawer_cycle", *PLUG_PROFILES])
    parser.add_argument("--workers", type=int)
    parser.add_argument("--episodes-per-worker", type=int)
    args = parser.parse_args()
    if args.worker_plan:
        return run_worker(args.worker_plan)
    if args.scene is None or args.output is None:
        parser.error("--scene and --output are required")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
