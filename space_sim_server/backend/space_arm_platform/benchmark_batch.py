"""Frozen, sequential process-isolated trials; resume never retries a result."""
from __future__ import annotations

from collections import Counter
from contextlib import contextmanager
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

import psutil

from .benchmark_execution import rescore_episode, sha256, source_identity, write_json
from .benchmark_external import ExternalMethodDefinition
from .benchmark_registry import Task, method_ids
from .benchmark_paths import evidence_path
from .benchmark_runtime import plan_runtime_identity, runtime_identity
from .collection_processes import launch_collector

TERMINAL_OUTCOMES = {"success", "safety_violation", "time_limit", "method_error",
                     "infrastructure_error", "cancelled", "invalid_evidence", "invalid_initialization", "task_failure"}


def summarize(trials: list[dict]) -> dict:
    """All planned trials remain in the primary denominator, including errors."""
    outcomes = Counter(trial["outcome"] for trial in trials)
    successes, total = outcomes["success"], len(trials)
    finished = total - outcomes["pending"] - outcomes["running"]
    completed = finished == total
    interval = None
    if completed and total:
        z = 1.959963984540054
        p = successes / total
        denominator = 1 + z * z / total
        center = (p + z * z / (2 * total)) / denominator
        radius = z * math.sqrt(p * (1-p) / total + z*z / (4*total*total)) / denominator
        interval = [max(0., center-radius), min(1., center+radius)]
    return {"schema_version": "spacesim-autonomy-batch-summary/1", "planned": total,
            "finished": finished, "complete": completed, "outcomes": dict(sorted(outcomes.items())),
            "successes": successes, "success_rate_all_planned": successes / total if total else None,
            "wilson_95_interval": interval,
            "denominator_policy": "all frozen planned trials, including method, evidence and infrastructure failures",
            "trials": trials}


@contextmanager
def batch_lock(directory: Path):
    path = directory / "batch.lock"
    owner = {"host": platform.node(), "pid": os.getpid(), "created": psutil.Process().create_time()}
    if path.exists():
        previous = json.loads(path.read_text(encoding="utf-8"))
        if previous.get("host") != platform.node():
            raise ValueError("batch is locked on another host")
        try:
            alive = psutil.Process(previous["pid"]).create_time() == previous["created"]
        except psutil.NoSuchProcess:
            alive = False
        if alive:
            raise ValueError("another process owns this batch")
        path.unlink()
    with path.open("x", encoding="utf-8") as stream:
        json.dump(owner, stream)
    try:
        yield
    finally:
        if path.exists() and json.loads(path.read_text(encoding="utf-8")) == owner:
            path.unlink()


def _completed_trial(directory: Path, planned: dict, task: Task, method: str, external_identity=None, *, source_sha256: dict,
                     runtime_identity: dict | None = None) -> dict:
    status = json.loads((directory / "status.json").read_text(encoding="utf-8"))
    result_path = directory / "run/result.json"
    if status.get("outcome") == "running" and "child_pid" in status:
        try:
            if psutil.Process(status["child_pid"]).create_time() == status["child_created"]:
                raise ValueError("previous trial process is still alive; wait for it before resuming")
        except psutil.NoSuchProcess:
            pass
    if result_path.is_file():
        result = json.loads(result_path.read_text(encoding="utf-8"))
        manifest = json.loads((directory / "run/manifest.json").read_text(encoding="utf-8"))
        if (result["seed"] != planned["seed"] or result["method"] != method
                or manifest["task_fingerprint"] != task.fingerprint()):
            raise ValueError("trial identity differs from frozen plan")
        if manifest.get("external_method") != external_identity:
            raise ValueError("trial method implementation differs from frozen plan")
        if manifest.get("source", {}).get("source_sha256") != source_sha256:
            raise ValueError("trial source implementation differs from frozen plan")
        if runtime_identity is not None and manifest.get('runtime_identity') != runtime_identity:
            raise ValueError('trial runtime dependencies differ from frozen plan')
        if runtime_identity is not None and result['outcome'] not in {'infrastructure_error', 'cancelled'} and result.get('runtime_identity_verified') is not True:
            raise ValueError('trial did not verify unchanged runtime dependencies')
        if status.get("result_sha256") and sha256(result_path) != status["result_sha256"]:
            raise ValueError("completed trial result has changed")
        if result["outcome"] not in TERMINAL_OUTCOMES:
            raise ValueError("trial returned a nonterminal outcome")
        if result["outcome"] not in {"infrastructure_error", "cancelled"}:
            replay = rescore_episode(directory / "run")
            if not replay["matches_recorded_physics_score"] or replay["outcome"] != result["outcome"]:
                raise ValueError("independent replay differs from recorded trial")
        return {**planned, "outcome": result["outcome"], "result_sha256": sha256(result_path),
                "completion_time_s": (result.get("physics_score") or {}).get("completion_time_s")}
    # A previously started process without a result is an observed failure, not
    # a pending trial. Resume must not replace it with a luckier repeated seed.
    return {**planned, "outcome": status.get("outcome", "infrastructure_error")
            if status.get("outcome") in TERMINAL_OUTCOMES else "infrastructure_error",
            "detail": "previous attempt has no complete result; it was not retried"}


def run_batch(task: Task, method: str | None, split: str, output: Path, *, resume=False,
              method_spec: Path | None = None, stop_file: Path | None = None, seed: int | None = None) -> dict:
    task = type(task).model_validate(task)
    definition = ExternalMethodDefinition(method_spec, task) if method_spec is not None else None
    if definition is not None:
        if method is not None and method != definition.spec.method_id:
            raise ValueError("method ID differs from external method specification")
        method = definition.spec.method_id
    else:
        method = method or method_ids(task)[0]
        if method not in method_ids(task):
            raise ValueError("method is not registered for this task family")
    if split not in {"train", "validation", "test"}:
        raise ValueError("unknown frozen split")
    seeds = getattr(task.seeds, split)
    if seed is not None:
        if type(seed) is not int or seed not in seeds:
            raise ValueError("single trial seed must belong to the selected frozen split")
        seeds = (seed,)
    output = evidence_path(output)
    if not resume:
        output.mkdir(parents=True, exist_ok=False)
    elif not (output / "plan.json").is_file():
        raise ValueError("resume requires an existing frozen batch plan")
    root = Path(__file__).resolve().parents[2]
    source = source_identity(root)
    planned = [{"trial_id": f"{index:04d}-seed-{seed}", "seed": seed} for index, seed in enumerate(seeds)]
    runtime = runtime_identity(root)
    plan = {"schema_version": "spacesim-autonomy-batch-plan/2", "configuration_id": task.configuration_id,
            "task_fingerprint": task.fingerprint(), "method": method, "split": split,
            "source_sha256": source["source_sha256"], "runtime_identity": runtime, "wall_time_limit_s": 900, "trials": planned}
    if definition is not None:
        plan["external_method"] = definition.identity
    if seed is not None:
        plan["trial_selection"] = {"kind": "single", "seed": seed}
    with batch_lock(output):
        if resume:
            previous = json.loads((output / "plan.json").read_text(encoding="utf-8"))
            if previous != plan:
                raise ValueError("task, method, split or source differs from frozen batch plan")
        else:
            write_json(output / "plan.json", plan)
        results = []
        cancelled = False
        for trial in planned:
            directory = output / trial["trial_id"]
            if (directory / "status.json").is_file():
                result = _completed_trial(directory, trial, task, method, plan.get("external_method"), source_sha256=plan["source_sha256"], runtime_identity=runtime)
                prior = json.loads((directory / "status.json").read_text(encoding="utf-8"))
                write_json(directory / "status.json", {**prior, **result})
                results.append(result)
                continue
            if cancelled or (stop_file is not None and stop_file.exists()):
                cancelled = True
                results.append({**trial, "outcome": "pending"})
                continue
            # Catch source edits during a long-running batch before launching a
            # different method/backend under the same frozen trial matrix.
            if source_identity(root)["source_sha256"] != plan["source_sha256"]:
                raise ValueError("source changed during batch execution; start a new batch")
            if runtime_identity(root) != runtime:
                raise ValueError('runtime dependencies changed during batch execution; start a new batch')
            if definition is not None:
                definition.verify_unchanged()
            directory.mkdir(exist_ok=False)
            write_json(directory / "status.json", {"outcome": "running"})
            command = [sys.executable, str(root / "tools/autonomy_benchmark.py"), "run", task.configuration_id,
                       "--seed", str(trial["seed"]), "--output", str(directory / "run")]
            command += ["--method-spec", str(definition.path)] if definition is not None else ["--method", method]
            if stop_file is not None:
                command += ["--stop-file", str(stop_file.resolve())]
            with (directory / "stdout.log").open("w", encoding="utf-8") as stdout, \
                    (directory / "stderr.log").open("w", encoding="utf-8") as stderr:
                child, owner = launch_collector(command, cwd=root, stdout=stdout, stderr=stderr,
                                               creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
                write_json(directory / "status.json", {"outcome": "running", "child_pid": child.pid,
                                                        "child_created": owner.created})
                write_json(output / "summary.json", summarize(results + [{**trial, "outcome": "running"}]
                    + [{**pending, "outcome": "pending"} for pending in planned[len(results)+1:]]))
                started, stop_started = time.monotonic(), None
                try:
                    while child.poll() is None:
                        if stop_file is not None and stop_file.exists():
                            cancelled = True
                            stop_started = stop_started or time.monotonic()
                            if time.monotonic() - stop_started > 10:
                                break
                        if time.monotonic() - started > plan["wall_time_limit_s"]:
                            write_json(directory / "status.json", {"outcome": "infrastructure_error", "detail": "trial wall-time limit"})
                            break
                        try:
                            child.wait(timeout=.2)
                        except subprocess.TimeoutExpired:
                            pass
                except KeyboardInterrupt:
                    cancelled = True
                finally:
                    # Kernel job / subreaper ownership also handles descendants
                    # left by a crashed method or a killed batch coordinator.
                    owner.close()
                    child.wait(timeout=5)
                returncode = child.returncode
            if cancelled and not (directory / "run/result.json").exists():
                write_json(directory / "status.json", {"outcome": "cancelled", "returncode": returncode})
            result = _completed_trial(directory, trial, task, method, plan.get("external_method"), source_sha256=plan["source_sha256"], runtime_identity=runtime)
            write_json(directory / "status.json", {**result, "returncode": returncode})
            results.append(result)
            current = summarize(results + [{**pending, "outcome": "pending"} for pending in planned[len(results):]])
            write_json(output / "summary.json", current)
            print(json.dumps({"trial": trial["trial_id"], "outcome": result["outcome"],
                              "finished": len(results), "planned": len(planned)}), flush=True)
        summary = summarize(results)
        write_json(output / "summary.json", summary)
        return summary


def reconcile_batch(task: Task, method: str, output: Path, *, interrupted_outcome="infrastructure_error") -> dict:
    """After owned processes have exited, recover durable attempts without running any."""
    output = evidence_path(output)
    plan = json.loads((output / "plan.json").read_text(encoding="utf-8"))
    runtime = plan_runtime_identity(plan)
    if plan["task_fingerprint"] != task.fingerprint() or plan["method"] != method:
        raise ValueError("batch identity does not match its frozen task")
    with batch_lock(output):
        results = []
        for trial in plan["trials"]:
            directory = output / trial["trial_id"]
            path = directory / "status.json"
            if not path.is_file():
                results.append({**trial, "outcome": "pending"})
                continue
            status = json.loads(path.read_text(encoding="utf-8"))
            # _completed_trial checks the recorded child identity before any
            # recovery write. A live worker is never treated as interrupted.
            result = _completed_trial(directory, trial, task, method, plan.get("external_method"), source_sha256=plan["source_sha256"], runtime_identity=runtime)
            if status["outcome"] == "running" and not (directory / "run/result.json").is_file():
                result["outcome"] = interrupted_outcome
            write_json(path, {**status, **result})
            results.append(result)
        summary = summarize(results)
        write_json(output / "summary.json", summary)
        return summary
