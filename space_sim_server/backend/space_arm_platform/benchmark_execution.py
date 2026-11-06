"""Single-episode execution and tamper-evident offline scoring artifacts.

Native imports are deferred until execution so catalogue/spec/replay tooling
works without Basilisk. Each CLI run owns a new directory and process.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

from .benchmark_contract import load_task
from .benchmark_formation import FormationTask
from .atomic_json import write_json as atomic_write_json
from .benchmark_external import ExternalMethod, ExternalMethodDefinition
from .benchmark_registry import Task, make_scorer, method_classes, method_ids, native_environment, parse_action
from .benchmark_paths import evidence_path


class MethodFailure(RuntimeError):
    pass


def write_json(path: Path, value: dict) -> None:
    atomic_write_json(path, value, allow_nan=False, trailing_newline=True)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_identity(server_root: Path) -> dict:
    paths = set((server_root / "simulation").rglob("*.py"))
    paths.update((server_root / "backend/space_arm_platform").glob("*.py"))
    paths.add(server_root / "native/local_mujoco_stepper.cpp")
    paths.add(server_root / "tools/autonomy_benchmark.py")
    hashes = {path.relative_to(server_root).as_posix(): sha256(path) for path in sorted(paths)}
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=server_root, text=True, capture_output=True, check=False)
    return {"git_revision": revision.stdout.strip() if revision.returncode == 0 else None,
            "python": sys.version, "platform": platform.platform(), "source_sha256": hashes}


def run_episode(task: Task, method_id: str | None, seed: int, output: Path, *, method_spec: Path | None = None,
                stop_file: Path | None = None) -> dict:
    """Record failures too. Output must be new; a retry cannot overwrite a trial."""
    task = type(task).model_validate(task)
    split = task.seeds.split_for(seed)
    definition = ExternalMethodDefinition(method_spec, task) if method_spec is not None else None
    methods = method_classes(task) if definition is None else {}
    if definition is not None:
        if method_id is not None and method_id != definition.spec.method_id:
            raise ValueError("method ID differs from external method specification")
        method_id = definition.spec.method_id
    else:
        method_id = method_id or method_ids(task)[0]
        if method_id not in methods:
            raise ValueError(f"unknown benchmark method: {method_id}")
    output = evidence_path(output)
    output.mkdir(parents=True, exist_ok=False)
    write_json(output / "task.json", task.model_dump(mode="json"))
    root = Path(__file__).resolve().parents[2]
    manifest = {"schema_version": "spacesim-autonomy-run/1", "configuration_id": task.configuration_id,
                "task_fingerprint": task.fingerprint(), "seed": seed, "split": split, "method": method_id,
                "source": source_identity(root)}
    if definition is not None:
        manifest["external_method"] = definition.identity
    write_json(output / "manifest.json", manifest)
    started = time.monotonic()
    environment = None
    method = None
    provenance = None
    outcome = "infrastructure_error"
    detail = None
    physics_score = None
    try:
        environment = native_environment(task, output)
        observation = environment.reset(seed)
        manifest.update({"episode_id": environment.episode_id,
                         "physics_path": (environment.episode_directory / "physics.jsonl").relative_to(output).as_posix()})
        if isinstance(task, FormationTask):
            manifest["spacecraft_masses_kg"] = list(environment.context.spacecraft_masses_kg)
        else:
            manifest["chaser_mass_kg"] = environment.context.chaser_mass_kg
        write_json(output / "manifest.json", manifest)
        try:
            method = ExternalMethod(definition, output) if definition is not None else methods[method_id]()
            method.reset(environment.context)
        except Exception as error:
            raise MethodFailure(f"{type(error).__name__}: {error}") from error
        with (output / "observations.jsonl").open("w", encoding="utf-8", newline="\n") as observations, \
                (output / "actions.jsonl").open("w", encoding="utf-8", newline="\n") as actions:
            observations.write(observation.model_dump_json() + "\n")
            while environment.evaluation()["outcome"] == "incomplete":
                if stop_file is not None and stop_file.exists():
                    outcome, detail = "cancelled", "operator requested episode stop"
                    break
                try:
                    raw_action = method.act(observation)
                    action = parse_action(task, raw_action, observation)
                except Exception as error:
                    outcome, detail = "method_error", f"{type(error).__name__}: {error}"
                    break
                # The request is persisted before execution; physics evidence
                # contains the actual native motor readback on every tick.
                actions.write(action.model_dump_json() + "\n")
                observation = environment.step(action)
                observations.write(observation.model_dump_json() + "\n")
            else:
                outcome = environment.evaluation()["outcome"]
    except MethodFailure as error:
        outcome, detail = "method_error", str(error)
    except KeyboardInterrupt:
        outcome, detail = "cancelled", "operator interrupted the episode"
    except Exception as error:
        outcome, detail = "infrastructure_error", f"{type(error).__name__}: {error}"
    finally:
        if environment is not None:
            try:
                if hasattr(environment, "_scorer"):
                    physics_score = environment.evaluation()
                if hasattr(environment, "initial_states_j2000"):
                    provenance = environment.provenance()
            except Exception as error:
                outcome = "infrastructure_error"
                detail = (detail + "; " if detail else "") + f"evidence finalization: {type(error).__name__}: {error}"
            finally:
                environment.close()
        if method is not None:
            try:
                method.close()
            except Exception as error:
                if outcome not in {"infrastructure_error", "cancelled"}:
                    outcome = "method_error"
                detail = (detail + "; " if detail else "") + f"method close: {type(error).__name__}: {error}"
    if provenance is not None:
        write_json(output / "provenance.json", provenance)
    evidence = {"task.json": sha256(output / "task.json"), "manifest.json": sha256(output / "manifest.json")}
    for relative in (manifest.get("physics_path"), "observations.jsonl", "actions.jsonl", "provenance.json",
                     "method_stderr.log", "method_calls.jsonl"):
        if relative and (output / relative).is_file():
            evidence[relative] = sha256(output / relative)
    result = {"schema_version": "spacesim-autonomy-result/1", "configuration_id": task.configuration_id,
              "seed": seed, "split": split, "method": method_id, "outcome": outcome,
              "success": outcome == "success", "detail": detail, "wall_time_s": time.monotonic() - started,
              "physics_score": physics_score, "evidence_sha256": evidence}
    if isinstance(method, ExternalMethod):
        result["method_telemetry"] = method.telemetry()
    write_json(output / "result.json", result)
    return result


def verify_episode_evidence(output: Path) -> tuple[dict, dict]:
    """Check recorded file hashes, including incomplete infrastructure attempts."""
    output = evidence_path(output)
    result = json.loads((output / "result.json").read_text(encoding="utf-8"))
    if not {"task.json", "manifest.json"}.issubset(result["evidence_sha256"]):
        raise ValueError("run has no hashed task and manifest")
    for relative, expected in result["evidence_sha256"].items():
        path = (output / relative).resolve()
        if not path.is_relative_to(output) or sha256(path) != expected:
            raise ValueError(f"changed or invalid evidence artifact: {relative}")
    manifest = json.loads((output / "manifest.json").read_text(encoding="utf-8"))
    return result, manifest


def rescore_episode(output: Path) -> dict:
    """Verify artifact hashes, replay truth and preserve non-physical failures."""
    output = evidence_path(output)
    result, manifest = verify_episode_evidence(output)
    task = load_task(output / "task.json")
    if task.fingerprint() != manifest["task_fingerprint"]:
        raise ValueError("task does not match run fingerprint")
    physics_path = manifest.get("physics_path")
    if not physics_path or physics_path not in result["evidence_sha256"]:
        raise ValueError("run has no complete hashed physical evidence")
    scorer = make_scorer(task, manifest["spacecraft_masses_kg"] if isinstance(task, FormationTask) else manifest["chaser_mass_kg"])
    with (output / physics_path).open(encoding="utf-8") as stream:
        for line in stream:
            scorer.add(json.loads(line))
    score = scorer.result()
    preserved = result["outcome"] in {"infrastructure_error", "method_error", "cancelled"}
    outcome = result["outcome"] if preserved else score["outcome"]
    return {"schema_version": "spacesim-autonomy-rescore/1", "outcome": outcome, "success": outcome == "success",
            "physics_score": score, "matches_recorded_physics_score": score == result.get("physics_score"),
            "original_outcome": result["outcome"]}


# Compatibility for the original waypoint Python entrypoints and archived
# notebooks. New code uses family-neutral names; wire schemas remain versioned.
run_waypoint = run_episode
rescore_waypoint = rescore_episode
