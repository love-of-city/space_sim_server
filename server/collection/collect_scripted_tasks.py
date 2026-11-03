"""Collect independent manipulation episodes using the live production recorder.

Failures and rejected initializations remain in the batch audit. Optional noise
and domain randomization are configured per episode; runtime state is never
teleported to make a task succeed.
"""

from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from copy import deepcopy
from dataclasses import dataclass, field

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from simulation.scripted_tasks.motion_options import (
    add_motion_options,
    motion_arguments,
)
from simulation.scripted_tasks.task_initialization import (
    randomized_scene,
)


from space_arm_platform.task_catalog import TASK_CHOICES, DRAWER_TASKS, THREADED_TASKS


from collection import recording_validation
from space_arm_platform.unreal_runtime import default_unreal_root
from space_arm_platform.collection_runtime import PersistentRenderer, check_port_available
from space_arm_platform.dynamics_worker import PersistentDynamics
from space_arm_platform.collection_layouts import LayoutPool, attach_layout, validate_episode_layout
from space_arm_platform.overview_camera_records import validate_episode_overview_camera


@dataclass
class CollectionSession:
    """Own workers/renderers across tasks; compatible graphs reset episode state."""
    output: Path
    renderer: object = None
    dynamics: object = None
    recorded: int = 0
    renderer_scene_key: tuple | None = None
    pending_validations: list = field(default_factory=list)

    def audit_pending(self):
        results = []
        while self.pending_validations:
            summary_path, directory, episode, task = self.pending_validations.pop(0)
            report = json.loads(summary_path.read_text())
            entry = next(e for e in report['attempts'] if e['directory'] == str(directory))
            started = time.monotonic()
            try:
                validation = recording_validation.validate_recording(episode, task)
                if 'overview_camera' in entry:
                    validation['overview_camera'] = validate_episode_overview_camera(episode, entry['overview_camera'])
                if 'layout_randomization' in entry:
                    validation['layout_randomization'] = validate_episode_layout(episode, entry['layout_randomization'])
                (directory/'validation.json').write_text(json.dumps(validation, indent=2))
                result = dict(episode=str(episode), passed=True)
            except Exception as exc:
                entry.update(status='validation_failed', error=f'{type(exc).__name__}: {exc}')
                report['accepted_episodes'] = [p for p in report['accepted_episodes'] if p != str(episode)]
                report['status'] = 'attempt_limit'
                result = dict(episode=str(episode), passed=False, error=entry['error'])
                (directory/'validation.json').write_text(json.dumps(result, indent=2))
            entry['full_validation_pending'] = False
            entry['deferred_validation_wall_seconds'] = time.monotonic()-started
            result['wall_seconds'] = entry['deferred_validation_wall_seconds']
            report['deferred_validation_wall_seconds'] = sum(e.get('deferred_validation_wall_seconds', 0.) for e in report['attempts'])
            temporary = summary_path.with_suffix('.audit.tmp')
            temporary.write_text(json.dumps(report, indent=2))
            temporary.replace(summary_path)
            results.append(result)
        return results

    def close(self):
        try:
            if self.renderer is not None:
                self.renderer.close()
        finally:
            try:
                if self.dynamics is not None:
                    self.dynamics.close()
            finally:
                self.renderer = self.dynamics = None


def main(default_task="drawer_cycle", argv=None, *, session=None, prepared_scene=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task", choices=TASK_CHOICES, default=default_task
    )
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--layout-pool", type=Path,
                        help="Validated layout pool; select a reproducible layout per attempt")
    parser.add_argument("--model", type=Path, help="Fixed validated layout model for this batch")
    parser.add_argument("--catalog", type=Path, help="Catalog paired with --model")
    parser.add_argument("--cancel-file", type=Path)
    parser.add_argument(
        "--adapter", type=Path, default=ROOT.parent / "ue-adapter"
    )
    parser.add_argument(
        "--unreal", type=Path, default=default_unreal_root()
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--episodes-root", type=Path, default=ROOT / "data/episodes")
    parser.add_argument("--recording-config", type=Path)
    parser.add_argument('--validation-mode', choices=('full', 'deferred'), default='deferred')
    parser.add_argument("--reuse-renderer", action="store_true")
    parser.add_argument("--reuse-dynamics", action="store_true",
                        help="Reuse imports/native libraries; each episode builds fresh physics state")
    parser.add_argument("--dynamics-max-episodes", type=int, default=4)
    parser.add_argument("--dynamics-max-rss-mb", type=int, default=8192)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--gpu-index", type=int)
    parser.add_argument("--warmup-seconds", type=float, default=5.0)
    parser.add_argument("--reuse-warmup-seconds", type=float, default=0.0)
    parser.add_argument("--simulation-startup-timeout-seconds", type=float, default=600.0)
    parser.add_argument("--task-wall-timeout-seconds", type=float, default=None)
    parser.add_argument("--count", type=int, default=5)
    parser.add_argument("--max-attempts", type=int, default=15)
    parser.add_argument("--seed", type=int, default=2026101501)
    parser.add_argument("--range-deg", type=float, default=30.0)
    parser.add_argument("--randomize-orbit-phase", action=argparse.BooleanOptionalAction,
                        default=None, help="Sample a new orbital phase per attempt; otherwise inherit scene setting")
    parser.add_argument("--randomize-overview-camera", action=argparse.BooleanOptionalAction,
                        default=None, help="Choose one of five body-fixed overview camera presets per attempt")
    parser.add_argument("--overview-camera-sampling", choices=("presets",), default="presets")
    parser.add_argument("--control-port", type=int, default=19766)
    parser.add_argument("--capture-port", type=int, default=19767)
    parser.add_argument("--render-port", type=int, default=16558)
    parser.add_argument("--signalling-port", type=int, default=19888)
    add_motion_options(parser)
    args = parser.parse_args(argv)
    if prepared_scene is not None and (args.count != 1 or args.max_attempts != 1):
        parser.error("a prepared scene requires exactly one attempt")
    for field in ("scene", "output", "episodes_root", "adapter", "unreal", "cancel_file",
                  "recording_config", "layout_pool", "model", "catalog"):
        value = getattr(args, field)
        if value is not None:
            setattr(args, field, value.expanduser().resolve())
    try:
        motion_args = motion_arguments(args.task, args.linear_speed, args.threaded_speed_profile)
    except ValueError as error:
        parser.error(str(error))
    if not 1 <= args.count <= args.max_attempts <= 1000 or args.seed < 0:
        parser.error("require 1 <= count <= max-attempts <= 1000 and non-negative seed")
    if not math.isfinite(args.range_deg) or not 0 <= args.range_deg <= 50:
        parser.error("range-deg must be finite and in [0,50]")
    if args.dynamics_max_episodes < 1 or args.dynamics_max_rss_mb < 1:
        parser.error("dynamics recycling limits must be positive")
    if not math.isfinite(args.simulation_startup_timeout_seconds) or not 0 < args.simulation_startup_timeout_seconds <= 7200:
        parser.error("simulation-startup-timeout-seconds must be finite and in (0,7200]")
    if args.task_wall_timeout_seconds is not None and (
            not math.isfinite(args.task_wall_timeout_seconds) or not 0 < args.task_wall_timeout_seconds <= 7200):
        parser.error("task-wall-timeout-seconds must be finite and in (0,7200]")
    if bool(args.model) != bool(args.catalog) or (args.layout_pool and args.model):
        parser.error("provide --model and --catalog together, or --layout-pool")
    source = json.loads(args.scene.read_text(encoding="utf-8-sig"))
    selected_layout = source.get("collection_config", {}).get("layout_randomization", {})
    if (source.get("collection_layout_randomization") or source.get("task_box_layout")
            or selected_layout.get("enabled")) and not (args.layout_pool or args.model):
        parser.error("--layout-pool is required for saved layouts, or explicit --model and --catalog")
    if any(not 0 <= value <= 60 for value in (args.warmup_seconds, args.reuse_warmup_seconds)):
        parser.error("warmup must be between 0 and 60 seconds")
    if args.gpu_index is not None and not 0 <= args.gpu_index <= 255:
        parser.error("gpu-index must be between 0 and 255")
    ports = (args.control_port, args.capture_port, args.render_port, args.signalling_port)
    if len(set(ports)) != 4 or any(not 1024 <= port <= 65535 for port in ports):
        parser.error("four distinct ports in 1024..65535 are required")
    for port in ports:
        if session is not None and session.renderer is not None and port == args.render_port:
            session.renderer.check()
            continue
        check_port_available(port)
    from collection.record_scripted_task import recording_settings
    recording_settings(args.recording_config)
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    if args.cancel_file is None:
        args.cancel_file = args.output / "cancel.request"
    if args.randomize_orbit_phase is not None:
        source["randomize_orbit_phase"] = args.randomize_orbit_phase
    if args.randomize_overview_camera is not None:
        source["randomize_overview_camera"] = args.randomize_overview_camera
    source["overview_camera_sampling"] = args.overview_camera_sampling
    pool = LayoutPool(args.layout_pool) if args.layout_pool else None
    if source.get("collection_layout_randomization") and pool is None and not args.model:
        raise ValueError("Layout randomization requires --layout-pool; refusing fixed-layout fallback")
    model = args.model or ROOT / "model/SARM/platform/sarm_20261012.xml"
    fixed_placement = None
    if args.model and prepared_scene is None:
        from simulation.capture_layout import validated_capture_layout
        fixed_placement = validated_capture_layout(model, args.catalog)
    started = time.monotonic()
    report = dict(
        schema="scripted-batch/2",
        task=args.task,
        linear_speed_m_s=(args.linear_speed if args.linear_speed is not None else 0.1)
        if args.task in DRAWER_TASKS
        else None,
        threaded_speed_profile=(args.threaded_speed_profile
                                if args.task in THREADED_TASKS else None),
        started_at_ns=str(time.time_ns()),
        status="running",
        requested_successes=args.count,
        base_seed=args.seed,
        attempts=[],
        accepted_episodes=[],
        source_scene=str(args.scene.resolve()),
        reuse_renderer=args.reuse_renderer,
        reuse_dynamics=args.reuse_dynamics,
        timing_scope="attempt start through quick completion check; full audit measured separately when deferred",
    )

    def save():
        report["wall_seconds"] = time.monotonic() - started
        temporary = args.output / "summary.tmp"
        temporary.write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
        )

        temporary.replace(args.output / "summary.json")

    owned_session = session is None
    session = session or CollectionSession(args.output)
    renderer = session.renderer
    dynamics = session.dynamics
    child = None
    recorded = session.recorded
    try:
        save()
        if pool is not None:
            pool.write_snapshot(args.output / "layout-pool.snapshot.json")
            report["layout_pool_sha256"] = pool.sha256
        for index in range(args.max_attempts):
            if args.cancel_file and args.cancel_file.exists():
                report["status"] = "cancelled"
                return 1
            if len(report["accepted_episodes"]) >= args.count:
                break
            seed = args.seed + index
            attempt_started = time.monotonic()
            attempt_started_at_ns = time.time_ns()
            scene = (deepcopy(prepared_scene) if prepared_scene is not None
                     else randomized_scene(source, seed, args.range_deg, args.task))
            if prepared_scene is not None and (
                    scene.get("seed") != seed or scene["collection_config"]["task"] != args.task):
                raise ValueError("Prepared scene does not match the requested task and seed")
            # Freeze the episode colour assignment before recording starts.
            from simulation.scripted_tasks.randomization import prepare_episode
            prepare_episode(scene)
            if args.task in THREADED_TASKS:
                scene["collection_config"]["threaded_speed_profile"] = args.threaded_speed_profile
            catalog = args.catalog
            if pool is not None:
                model, catalog = attach_layout(scene, pool, seed)
            elif fixed_placement is not None:
                scene["task_box_layout"] = fixed_placement
                scene["collection_layout_randomization"] = False
            directory = args.output / f"attempt-{index + 1:03d}-seed-{seed}"
            directory.mkdir()
            from simulation.scripted_tasks.randomization import write_model_variant

            scene["scripted_task_name"] = args.task
            # Resolve colour bindings before freezing the scene; no trajectory
            # rehearsal or collision-model construction occurs in this process.
            write_model_variant(model, directory / "prepared-model.xml", scene)
            scene_path = directory / "scene.json"
            scene_path.write_text(
                json.dumps(scene, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            entry = dict(
                attempt=index + 1,
                seed=seed,
                started_at_ns=str(attempt_started_at_ns),
                initial_state=scene["collection_config"]["initialization"],
                directory=str(directory),
                status="preflight",
            )
            color_config = scene.get("collection_config", {}).get(
                "color_randomization",
                scene.get("collection_config", {}).get("episode_color_assignment"),
            )
            if color_config is not None:
                entry["color_randomization"] = {
                    "schema": color_config.get("schema"),
                    "assignment_sha256": color_config.get("assignment_sha256"),
                    "hole_colors": color_config.get("hole_colors"),
                    "source_colors": color_config.get("source_colors"),
                    "bolt_storage_color": color_config.get("bolt_storage_color"),
                }
            report["attempts"].append(entry)
            if scene.get("randomize_overview_camera"):
                entry["overview_camera"] = scene["overview_camera"]
            if scene.get("collection_config", {}).get("layout_randomization", {}).get("enabled"):
                entry["layout_randomization"] = scene["collection_config"]["layout_randomization"]
            if catalog is not None:
                entry["model"], entry["catalog"] = str(model), str(catalog)
            save()
            entry["scene_preparation_wall_seconds"] = time.monotonic() - attempt_started
            entry["preflight"] = dict(scope="initial_state_in_worker", deferred=True)
            entry["preflight_wall_seconds"] = 0.0
            command = [
                sys.executable,
                "-X",
                "utf8",
                str(ROOT / "collection/record_scripted_task.py"),
                "--task",
                args.task,
                "--scene",
                str(scene_path),
                "--output",
                str(directory / "recording"),
                "--episodes-root",
                str(args.episodes_root.resolve()),
                "--substeps",
                "1",
                "--max-sim-seconds",
                str(min(900 if args.task in THREADED_TASKS else 450,
                        args.task_wall_timeout_seconds or 900)),
                "--timeout-seconds",
                str(args.task_wall_timeout_seconds or (5400 if args.task in THREADED_TASKS else 1800)),
                "--simulation-startup-timeout-seconds",
                str(args.simulation_startup_timeout_seconds),
            ]
            command += motion_args
            if catalog is not None:
                command += ["--model", str(model), "--catalog", str(catalog)]
            scene_key = (str(model.resolve()), str(catalog),
                         json.dumps(scene.get('overview_camera', {}), sort_keys=True),
                         json.dumps(scene['collection_config'].get('episode_color_assignment', {}), sort_keys=True))
            same_scene = renderer is not None and recorded and session.renderer_scene_key == scene_key
            command += ["--warmup-seconds", str(args.reuse_warmup_seconds if same_scene else args.warmup_seconds)]
            session.renderer_scene_key = scene_key
            if args.offline:
                command.append("--offline")
            if args.gpu_index is not None:
                command += ["--gpu-index", str(args.gpu_index)]
            if args.reuse_renderer:
                if renderer is None:
                    renderer = PersistentRenderer(args, session.output)
                report["renderer_pid"] = renderer.process.pid
                renderer.check()
                command.append("--external-renderer")
                command += ["--renderer-runtime-report", str(renderer.runtime_report_path),
                            "--renderer-pid", str(renderer.process.pid)]
            if args.reuse_dynamics:
                if dynamics is not None and not dynamics.reusable():
                    dynamics.close()
                    dynamics = None
                if dynamics is None:
                    dynamics = PersistentDynamics(ROOT, session.output,
                        max_episodes=args.dynamics_max_episodes,
                        max_rss_mb=args.dynamics_max_rss_mb)
                command += ["--dynamics-worker", str(dynamics.directory)]
                entry["dynamics_worker_pid"] = dynamics.pid
                entry["dynamics_worker_directory"] = str(dynamics.directory)
            command += ["--adapter", str(args.adapter), "--unreal", str(args.unreal)]
            if args.recording_config:
                command += ["--recording-config", str(args.recording_config.resolve())]
            if args.cancel_file:
                command += ["--cancel-file", str(args.cancel_file)]
            for name in (
                "control_port",
                "capture_port",
                "render_port",
                "signalling_port",
            ):
                command += ["--" + name.replace("_", "-"), str(getattr(args, name))]
            print(
                json.dumps(
                    dict(stage="attempt_start", seed=seed, directory=str(directory))
                ),
                flush=True,
            )
            with (directory / "recording.log").open("w", encoding="utf-8") as log:
                child = subprocess.Popen(
                    command,
                    cwd=ROOT,
                    env={**os.environ, "PYTHONPATH": os.pathsep.join(
                        [str(ROOT), str(ROOT / "backend"), str(args.adapter / "Adapters")])},
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                )
                while child.poll() is None:
                    if (args.task_wall_timeout_seconds is not None and
                            time.monotonic()-attempt_started >= args.task_wall_timeout_seconds):
                        raise TimeoutError('Task exceeded configured wall-clock limit')
                    if renderer is not None:
                        renderer.check()
                    time.sleep(0.2)
            recorded += 1
            entry["returncode"] = child.returncode
            entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
            recording_summary = directory / "recording/summary.json"
            if recording_summary.is_file():
                result_info = json.loads(recording_summary.read_text(encoding="utf-8"))
                entry["episode_id"] = result_info.get("episode_id")
                entry["task_result"] = result_info.get("task")
                entry["recording_wall_seconds"] = result_info.get(
                    "recording_wall_seconds"
                )
                entry["error"] = result_info.get("error")
                entry["failure_kind"] = result_info.get("failure_kind")
                entry["recorded_sim_seconds"] = result_info.get("recorded_sim_seconds")
                entry["recording_process_wall_seconds"] = result_info.get(
                    "end_to_end_wall_seconds"
                )
                for metric in ("startup_to_recording_end_wall_seconds", "startup_to_finalized_wall_seconds",
                               "recording_ended_at_ns", "recording_finalized_at_ns"):
                    entry[metric] = result_info.get(metric)
                entry["dynamics_worker"] = result_info.get("dynamics_worker")
            if args.cancel_file and args.cancel_file.exists():
                entry["status"] = "cancelled"
                report["status"] = "cancelled"
                return 1
            if child.returncode != 0:
                entry["status"] = "recording_or_task_failed"
                save()
                print(
                    json.dumps(
                        dict(stage="attempt_failed", attempt=index + 1, detail=entry)
                    ),
                    flush=True,
                )
                # Infrastructure failures must be investigated, not retried forever.
                skippable_error = entry.get("failure_kind") in {"task_timeout", "recording_validation", "initialization_rejected"}
                if (child.returncode not in (0, 1)
                        or (entry.get("error") and not skippable_error)
                        or (not entry.get("task_result") and not skippable_error)):
                    raise RuntimeError(
                        f"Recording infrastructure failed; see {directory}"
                    )
                continue
            episode = args.episodes_root.resolve() / entry["episode_id"]
            entry["status"] = "validating"
            save()
            try:
                validation_started = time.monotonic()
                deferred = args.validation_mode == 'deferred'
                validation = (recording_validation.validate_recording_fast(episode, args.task) if deferred
                              else recording_validation.validate_recording(episode, args.task))
                if not deferred and scene.get("randomize_overview_camera"):
                    validation["overview_camera"] = validate_episode_overview_camera(
                        episode, entry["overview_camera"])
                if not deferred and "layout_randomization" in entry:
                    validation["layout_randomization"] = validate_episode_layout(
                        episode, entry["layout_randomization"])
                entry["validation_wall_seconds"] = time.monotonic() - validation_started
            except Exception as exc:
                entry["status"] = "validation_failed"
                entry["error"] = f"{type(exc).__name__}: {exc}"
                entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
                entry["finished_at_ns"] = str(time.time_ns())
                save()
                print(
                    json.dumps(
                        dict(stage="attempt_failed", attempt=index + 1, detail=entry)
                    ),
                    flush=True,
                )
                # A bad episode is retained for audit, but must not stop a
                # long batch whose target is a number of valid episodes.
                continue
            (directory / "validation.json").write_text(
                json.dumps(validation, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            entry["status"] = "accepted"
            entry['full_validation_pending'] = deferred
            if deferred:
                session.pending_validations.append((args.output/'summary.json', directory, episode, args.task))
            entry["end_to_end_wall_seconds"] = time.monotonic() - attempt_started
            entry["finished_at_ns"] = str(time.time_ns())
            report["accepted_episodes"].append(str(episode))
            save()
            print(
                json.dumps(
                    dict(
                        stage="accepted",
                        successes=len(report["accepted_episodes"]),
                        episode=str(episode),
                    )
                ),
                flush=True,
            )
        report["status"] = (
            "complete"
            if len(report["accepted_episodes"]) == args.count
            else "attempt_limit"
        )
        save()
        if owned_session and session.pending_validations:
            session.audit_pending()
            report = json.loads((args.output / "summary.json").read_text())
        return 0 if report["status"] == "complete" else 1
    except BaseException as exc:
        report["status"] = (
            "interrupted" if isinstance(exc, KeyboardInterrupt) else "failed"
        )
        report["error"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if child is not None and child.poll() is None:
            # Stop this attempt only: writing the batch cancel file here used
            # to abort every following task after a recoverable failure.
            (directory / 'recording' / 'stop.request').parent.mkdir(parents=True, exist_ok=True)
            (directory / 'recording' / 'stop.request').write_text('attempt stopping', encoding='utf-8')
            try:
                child.wait(5)
            except subprocess.TimeoutExpired:
                import psutil
                try:
                    descendants = psutil.Process(child.pid).children(recursive=True)
                except psutil.NoSuchProcess:
                    descendants = []
                for process in reversed(descendants):
                    try:
                        process.kill()
                    except psutil.NoSuchProcess:
                        pass
                child.kill()
                child.wait(5)
            # A persistent worker is a sibling of the recorder, not a child.
            if dynamics is not None:
                dynamics.close()
                dynamics = None
        session.renderer, session.dynamics = renderer, dynamics
        session.recorded = recorded
        if owned_session:
            session.close()
        save()


if __name__ == "__main__":
    raise SystemExit(main())
