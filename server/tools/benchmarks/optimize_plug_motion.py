"""Bounded offline plug-motion parameter search; never changes production files.

Example (from server):
  python tools/benchmarks/optimize_plug_motion.py --cases /tmp/cases.json \
      --candidates /tmp/candidates.json --output /tmp/plug-motion-study

Cases: [{"name":"round-layout-1", "task":"cylindrical_plug_insert",
         "scene":"/absolute/scene.json", "model":"/absolute/model.xml"}].
Candidates: [{"name":"creep", "parameters":{"insert_creep_delay_s":0.1}}].
An empty-override baseline always runs first. All cases are required. The first
candidate passing success, measured continuity and baseline load gates wins.
The baseline must succeed. Both whole-task and separate extract/insert peak
loads must remain within the selected multiplier of their baseline values;
missing or invalid contact-stage sensor samples cannot pass.
The iteration budget includes that baseline. This is a physics diagnostic,
not a production collection or autonomous source-code rewriting service.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from tools.benchmarks.plug_motion_metrics import analyse_run, rejection_reasons


def _sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")


def _named_list(path, kind):
    values = json.loads(path.read_text(encoding="utf-8-sig"))
    if not isinstance(values, list) or not values:
        raise ValueError(f"{kind} must be a nonempty JSON list")
    names = set()
    for item in values:
        name = item.get("name") if isinstance(item, dict) else None
        if not isinstance(name, str) or not re.fullmatch(r"[a-zA-Z0-9][a-zA-Z0-9_-]{0,79}", name):
            raise ValueError(f"Invalid {kind} name: {name!r}")
        if name in names:
            raise ValueError(f"Duplicate {kind} name: {name}")
        names.add(name)
    return values


def prepare_inputs(cases_path, candidates_path):
    from simulation.scripted_tasks.plug_profiles import is_plug_task

    cases = _named_list(cases_path, "case")
    candidates = _named_list(candidates_path, "candidate")
    baseline = {"name": "baseline", "parameters": {}}
    if any(c["name"] == "baseline" for c in candidates):
        if candidates[0] != baseline:
            raise ValueError("baseline must be the first candidate with empty parameters")
    else:
        candidates.insert(0, baseline)
    for case in cases:
        if (set(case) != {"name", "task", "scene", "model"}
                or not isinstance(case["task"], str) or not is_plug_task(case["task"])):
            raise ValueError("Cases require exactly name, plug task, scene and model")
        for field in ("scene", "model"):
            path = Path(case[field]).expanduser()
            case[field] = str((cases_path.parent / path).resolve() if not path.is_absolute() else path.resolve())
            if not Path(case[field]).is_file():
                raise ValueError(f"Missing {field}: {case[field]}")
        scene = json.loads(Path(case["scene"]).read_text(encoding="utf-8-sig"))
        case["scene_data"] = scene
        case["input_sha256"] = {field: _sha(Path(case[field])) for field in ("scene", "model")}
        for candidate in candidates:
            if set(candidate) != {"name", "parameters"} or not isinstance(candidate["parameters"], dict):
                raise ValueError("Candidates require exactly name and parameters")
            candidate_scene(case["task"], scene, candidate["parameters"])
            if any(isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                   for value in candidate["parameters"].values()):
                raise ValueError("Motion candidate parameters must be finite numbers")
    return cases, candidates


def candidate_scene(task_name, source_scene, parameters):
    """Derive a new frozen experiment while retaining its source config identity."""
    from dataclasses import asdict
    from simulation.scripted_tasks.plug_motion_config import (
        MOTION_PARAMETERS, config_snapshot, plug_motion_config,
    )

    unknown = set(parameters) - MOTION_PARAMETERS
    if unknown:
        raise ValueError(f"Unsupported plug motion parameters: {sorted(unknown)}")
    source = plug_motion_config(task_name, source_scene)
    source_snapshot = config_snapshot(source, task_name)
    options = asdict(source)
    options.pop("plug_body")
    options.update(parameters)
    candidate = plug_motion_config(task_name, **options)
    scene = json.loads(json.dumps(source_scene))
    collection = scene.setdefault("collection_config", {})
    collection.setdefault("plug_motion_parameters", {}).update(parameters)
    collection["plug_config_snapshot"] = config_snapshot(candidate, task_name)
    collection["plug_config_derivation"] = {
        "source_resolved_config_sha256": source_snapshot["sha256"],
        "source_had_snapshot": "plug_config_snapshot" in source_scene.get("collection_config", {}),
        "parameter_changes": dict(parameters),
    }
    return scene


def _new_output(path):
    output = path.resolve()
    for parent in output.parents:
        if (parent / "cancel.request").exists() or ((parent / "plan.json").exists() and (parent / "progress.json").exists()):
            raise ValueError("Diagnostics cannot write inside a collection batch")
    output.mkdir(parents=True, exist_ok=False)
    return output


def run_search(args):
    cases, candidates = prepare_inputs(args.cases.resolve(), args.candidates.resolve())
    output = _new_output(args.output)
    sources = [ROOT / "collection/run_scripted_task.py", Path(__file__),
               Path(__file__).with_name("plug_motion_metrics.py")]
    sources.extend(sorted((ROOT / "simulation/scripted_tasks").glob("*.py")))
    manifest = dict(schema="plug-motion-search/1", diagnostic_only=True,
                    created_at=datetime.now(timezone.utc).isoformat(),
                    cases=[{k: v for k, v in c.items() if k != "scene_data"} for c in cases],
                    candidates=candidates, source_sha256={str(p.relative_to(ROOT)): _sha(p) for p in sources},
                    input_sha256={"cases": _sha(args.cases), "candidates": _sha(args.candidates)},
                    budgets={key: getattr(args, key) for key in ("max_iterations", "max_sim_seconds", "timeout_seconds", "max_stagnant_seconds", "speed_threshold_m_s", "load_multiplier")})
    _write(output / "manifest.json", manifest)
    for source in sources:
        snapshot = output / "sources" / source.relative_to(ROOT)
        snapshot.parent.mkdir(parents=True, exist_ok=True)
        snapshot.write_bytes(source.read_bytes())
    (output / "cases.input.json").write_bytes(args.cases.read_bytes())
    (output / "candidates.input.json").write_bytes(args.candidates.read_bytes())
    inputs = output / "inputs"
    inputs.mkdir()
    for case in cases:
        (inputs / (case["name"] + ".scene.json")).write_bytes(Path(case["scene"]).read_bytes())
    baseline_metrics, results = {}, []
    selected = None
    environment = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1")
    for candidate in candidates[:args.max_iterations]:
        candidate_dir = output / candidate["name"]
        candidate_dir.mkdir()
        candidate_result = dict(name=candidate["name"], parameters=candidate["parameters"], cases=[], accepted=False)
        for case in cases:
            scene = candidate_scene(case["task"], case["scene_data"], candidate["parameters"])
            scene_path = candidate_dir / (case["name"] + ".scene.json")
            _write(scene_path, scene)
            run_dir = candidate_dir / case["name"]
            command = [sys.executable, "-u", str(ROOT / "collection/run_scripted_task.py"), "--task", case["task"],
                       "--scene", str(scene_path), "--model", case["model"], "--output", str(run_dir),
                       "--native-geometry", "--save-trace", "--max-sim-seconds", str(args.max_sim_seconds)]
            record = dict(name=case["name"], command=command, scene_sha256=_sha(scene_path), output=str(run_dir))
            _write(candidate_dir / (case["name"] + ".command.json"), record)
            start = time.monotonic()
            try:
                # A changed source/model invalidates comparisons; never silently mix revisions.
                if any(_sha(ROOT / name) != digest for name, digest in manifest["source_sha256"].items()):
                    raise RuntimeError("source_changed_during_search")
                if _sha(Path(case["model"])) != case["input_sha256"]["model"]:
                    raise RuntimeError("model_changed_during_search")
                with (candidate_dir / (case["name"] + ".log")).open("w") as log:
                    process = subprocess.run(command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT,
                                             timeout=args.timeout_seconds, check=False)
                record["returncode"] = process.returncode
                if any(_sha(ROOT / name) != digest for name, digest in manifest["source_sha256"].items()):
                    raise RuntimeError("source_changed_during_run")
                if _sha(Path(case["model"])) != case["input_sha256"]["model"]:
                    raise RuntimeError("model_changed_during_run")
                metrics = analyse_run(run_dir, speed_threshold_m_s=args.speed_threshold_m_s)
                record["metrics"] = metrics
                if candidate["name"] == "baseline":
                    baseline_metrics[case["name"]] = metrics
                baseline = baseline_metrics.get(case["name"]) if candidate["name"] != "baseline" else None
                record["rejections"] = rejection_reasons(metrics, baseline, max_stagnant_seconds=args.max_stagnant_seconds,
                                                         load_multiplier=args.load_multiplier)
                if candidate["name"] != "baseline" and baseline is None:
                    record["rejections"].append("baseline_unavailable")
                if process.returncode != 0:
                    record["rejections"].append("process_failed")
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired) as error:
                record["rejections"] = [f"{type(error).__name__}: {error}"]
            record["wall_seconds"] = time.monotonic() - start
            candidate_result["cases"].append(record)
            _write(candidate_dir / (case["name"] + ".assessment.json"), record)
            print(json.dumps({"candidate": candidate["name"], "case": case["name"], "rejections": record["rejections"]}), flush=True)
        candidate_result["accepted"] = all(not case["rejections"] for case in candidate_result["cases"])
        results.append(candidate_result)
        _write(output / "results.json", results)
        if candidate_result["accepted"]:
            selected = candidate_result
            break
    recommendation = dict(schema="plug-motion-recommendation/1", status="validated" if selected else "no_candidate_passed",
                          selected_candidate=selected["name"] if selected else None,
                          parameters=selected["parameters"] if selected else None,
                          tested_candidates=len(results), required_cases=[c["name"] for c in cases],
                          production_applied=False, scope="native physics; camera collection and unlisted layouts unverified")
    _write(output / "recommendation.json", recommendation)
    report = ["Plug motion parameter search", "", f"Status: {recommendation['status']}",
              f"Maximum permitted contact stagnation: {args.max_stagnant_seconds:g} s at |v_z| < {args.speed_threshold_m_s:g} m/s.",
              "No production source, scene, collection batch, or dataset was modified.", ""]
    for candidate in results:
        for case in candidate["cases"]:
            metrics = case.get("metrics", {})
            report.append(f"- {candidate['name']} / {case['name']}: result={metrics.get('result')}; max stagnant={metrics.get('max_stagnant_seconds')} s; rejected={case['rejections']}")
    report += ["", recommendation["scope"]]
    (output / "report.md").write_text("\n".join(report) + "\n", encoding="utf-8")
    print(json.dumps(recommendation, indent=2))
    return 0 if selected else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cases", type=Path)
    parser.add_argument("--candidates", type=Path)
    parser.add_argument("--output", type=Path, required=True, help="New diagnostic directory; existing paths refused")
    parser.add_argument("--analyse-run", type=Path, nargs="+", help="Read existing diagnostic runs; no simulation")
    parser.add_argument("--max-iterations", type=int, default=8, help="Includes empty-override baseline")
    parser.add_argument("--max-sim-seconds", type=float, default=240.0)
    parser.add_argument("--timeout-seconds", type=float, default=900.0, help="Per case wall-clock limit")
    parser.add_argument("--max-stagnant-seconds", type=float, default=0.5)
    parser.add_argument("--speed-threshold-m-s", type=float, default=0.001)
    parser.add_argument("--load-multiplier", type=float, default=1.1)
    args = parser.parse_args(argv)
    for key in ("max_iterations", "max_sim_seconds", "timeout_seconds", "max_stagnant_seconds", "speed_threshold_m_s", "load_multiplier"):
        value = getattr(args, key)
        if not math.isfinite(value) or value <= 0:
            parser.error(key + " must be finite and positive")
    try:
        if args.analyse_run:
            output = _new_output(args.output)
            results = []
            for run in args.analyse_run:
                metrics = analyse_run(run.resolve(), speed_threshold_m_s=args.speed_threshold_m_s)
                results.append(dict(run=str(run.resolve()), metrics=metrics,
                                    rejections=rejection_reasons(metrics, max_stagnant_seconds=args.max_stagnant_seconds)))
            _write(output / "analysis.json", results)
            print(json.dumps(results, indent=2))
            return 0
        if not args.cases or not args.candidates:
            parser.error("--cases and --candidates are required unless --analyse-run is used")
        return run_search(args)
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
