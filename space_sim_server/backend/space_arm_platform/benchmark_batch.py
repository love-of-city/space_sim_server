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

import psutil

from .benchmark_execution import rescore_episode, sha256, source_identity, write_json
from .benchmark_external import ExternalMethodDefinition
from .benchmark_registry import Task, method_ids

TERMINAL_OUTCOMES = {"success", "safety_violation", "time_limit", "method_error",
                     "infrastructure_error", "cancelled", "invalid_evidence"}


def summarize(trials: list[dict]) -> dict:
    """All planned trials remain in the primary denominator, including errors."""
    outcomes = Counter(trial["outcome"] for trial in trials)
    successes, total = outcomes["success"], len(trials)
    finished = total - outcomes["pending"]
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


def _completed_trial(directory: Path, planned: dict, task: Task, method: str, external_identity=None) -> dict:
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


def run_batch(task: Task, method: str | None, split: str, output: Path, *, resume=False, method_spec: Path | None = None) -> dict:
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
    output = output.resolve()
    if not resume:
        output.mkdir(parents=True, exist_ok=False)
    elif not (output / "plan.json").is_file():
        raise ValueError("resume requires an existing frozen batch plan")
    root = Path(__file__).resolve().parents[2]
    source = source_identity(root)
    planned = [{"trial_id": f"{index:04d}-seed-{seed}", "seed": seed} for index, seed in enumerate(seeds)]
    plan = {"schema_version": "spacesim-autonomy-batch-plan/1", "configuration_id": task.configuration_id,
            "task_fingerprint": task.fingerprint(), "method": method, "split": split,
            "source_sha256": source["source_sha256"], "wall_time_limit_s": 900, "trials": planned}
    if definition is not None:
        plan["external_method"] = definition.identity
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
                result = _completed_trial(directory, trial, task, method, plan.get("external_method"))
                prior = json.loads((directory / "status.json").read_text(encoding="utf-8"))
                write_json(directory / "status.json", {**prior, **result})
                results.append(result)
                continue
            if cancelled:
                results.append({**trial, "outcome": "pending"})
                continue
            # Catch source edits during a long-running batch before launching a
            # different method/backend under the same frozen trial matrix.
            if source_identity(root)["source_sha256"] != plan["source_sha256"]:
                raise ValueError("source changed during batch execution; start a new batch")
            if definition is not None:
                definition.verify_unchanged()
            directory.mkdir(exist_ok=False)
            write_json(directory / "status.json", {"outcome": "running"})
            command = [sys.executable, str(root / "tools/autonomy_benchmark.py"), "run", task.configuration_id,
                       "--seed", str(trial["seed"]), "--output", str(directory / "run")]
            command += ["--method-spec", str(definition.path)] if definition is not None else ["--method", method]
            with (directory / "stdout.log").open("w", encoding="utf-8") as stdout, \
                    (directory / "stderr.log").open("w", encoding="utf-8") as stderr:
                child = subprocess.Popen(command, cwd=root, stdout=stdout, stderr=stderr)
                write_json(directory / "status.json", {"outcome": "running", "child_pid": child.pid,
                                                        "child_created": psutil.Process(child.pid).create_time()})
                try:
                    returncode = child.wait(timeout=plan["wall_time_limit_s"])
                except KeyboardInterrupt:
                    # Only the direct child created for this trial is stopped.
                    child.terminate()
                    child.wait(timeout=30)
                    cancelled = True
                    returncode = child.returncode
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=30)
                    returncode = child.returncode
                    write_json(directory / "status.json", {"outcome": "infrastructure_error", "detail": "trial wall-time limit"})
            if cancelled and not (directory / "run/result.json").exists():
                write_json(directory / "status.json", {"outcome": "cancelled", "returncode": returncode})
            result = _completed_trial(directory, trial, task, method, plan.get("external_method"))
            write_json(directory / "status.json", {**result, "returncode": returncode})
            results.append(result)
            current = summarize(results + [{**pending, "outcome": "pending"} for pending in planned[len(results):]])
            write_json(output / "summary.json", current)
            print(json.dumps({"trial": trial["trial_id"], "outcome": result["outcome"],
                              "finished": len(results), "planned": len(planned)}), flush=True)
        summary = summarize(results)
        write_json(output / "summary.json", summary)
        return summary
