"""Validate a scripted task with the platform's authoritative BSK/MuJoCo dynamics.

No UI or running scene required. Output is a diagnostic trace, not a camera
training episode. Original contacts, latch, free base, servos and SDFs are kept.
"""

from __future__ import annotations
import argparse
from contextlib import nullcontext
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT),
    str(ROOT / "backend"),
    str(ROOT.parent / "ue-adapter/Adapters"),
    str(ROOT.parent / "ue-adapter/Unreal/BskUnrealRenderer/examples"),
]

from simulation.scripted_tasks.motion_options import (
    add_motion_options,
    motion_arguments,
)
from simulation.scripted_tasks.plug_profiles import (
    is_plug_task,
    plug_profile,
)
from simulation.scripted_tasks.drawer_tasks import DRAWER_TASKS
from simulation.scripted_tasks.threaded_tasks import create_threaded_task
from space_arm_platform.task_catalog import RUNNER_TASK_CHOICES, THREADED_TASKS, task_default_duration_s


def create_task(args, model, scene):
    """Bind the selected model and effective options to one current controller."""
    options = dict(lift_m=args.lift_m, grip_offset_m=tuple(args.grip_offset_m),
                   force_limit_n=80.0 if args.force_limit is None else args.force_limit)
    if args.arm_speed is not None:
        options["arm_speed_rad_s"] = args.arm_speed
    if args.linear_speed is not None:
        options["linear_speed_m_s"] = args.linear_speed
    if args.task in THREADED_TASKS:
        threaded = {name: value for name, value in (
            ("arm_speed_rad_s", args.arm_speed), ("linear_speed_m_s", args.linear_speed)
        ) if value is not None}
        return create_threaded_task(args.task, model, speed_profile=args.threaded_speed_profile, **threaded)
    if is_plug_task(args.task):
        from simulation.scripted_tasks.plug_insert import PlugInsertTask
        from simulation.scripted_tasks.plug_motion_config import plug_motion_config
        config = plug_motion_config(
            args.task, scene, target_site=args.target_site, arm_speed_rad_s=args.arm_speed,
            grasp_offset_m=tuple(args.plug_grasp_offset) if args.plug_grasp_offset is not None else None,
            force_limit_n=min(args.force_limit, 40.0) if args.force_limit is not None else None,
        )
        task = PlugInsertTask(model, config)
        scene.setdefault("collection_config", {})["plug_config_snapshot"] = task.configuration_snapshot
        return task
    if args.task in DRAWER_TASKS:
        from simulation.scripted_tasks.drawer_tasks import create_drawer_task
        return create_drawer_task(args.task, model, **options)
    from simulation.scripted_tasks.drawer_lift import DrawerLiftConfig, DrawerLiftTask
    return DrawerLiftTask(model, DrawerLiftConfig(**options))


def close_task_runtime(sim, stepper, orbit, capture, graph_cache):
    """Release episode transport and runtime resources not retained by the cache."""
    if capture is not None:
        capture.close()
    if graph_cache is None or not graph_cache.holds(sim):
        if stepper is not None:
            stepper.close()
        if orbit is not None and orbit.gravity_factory is not None:
            orbit.gravity_factory.unloadSpiceKernels()


def default_target_state(model):
    """Use the selected model, not the legacy grasp demo's nearby target pose."""
    target = ET.parse(model).getroot().find('./worldbody/body[@name="capture_target"]')
    return dict(
        target_position_m=np.fromstring(target.get("pos", "0 0 0"), sep=" ").tolist(),
        target_orientation_wxyz=np.fromstring(
            target.get("quat", "1 0 0 0"), sep=" "
        ).tolist(),
        target_linear_velocity_m_s=[0.01, -0.004, 0.002],
        target_angular_velocity_rad_s=[0.0, 0.0, 0.0],
    )


def run(args, *, graph_cache=None):
    initialization_started = time.perf_counter()
    initialization_seconds = {}
    # Establish the production DLL load order. Never import Python mujoco here.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(ROOT / "model/SARM/platform")
    from simulation.compiled_model_cache import configure_asset_cache
    configure_asset_cache()
    initialization_seconds["native_module_load"] = time.perf_counter() - initialization_started
    phase_started = time.perf_counter()
    from simulation.assembly import AssemblyContext, SessionConfig
    from simulation.components.attitude import AttitudeComponent
    from simulation.components.clock import ClockComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.arm_reference import JointReferenceComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from simulation.sarm_model_profile import load_model_profile
    from simulation.teleop_scene import (
        DEFAULT_ORBIT,
        DEFAULT_EPHEMERIS_EPOCH_UTC,
        apply_orbital_initial_state,
    )
    from simulation.wrench_sensors import WRIST_SENSOR
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
    from space_arm_platform.sampling import (
        tick_time_ns,
        DYNAMICS_HZ,
        SCRIPTED_TASK_HZ,
        RENDER_HZ,
        WRENCH_HZ,
        capture_preroll_ns,
    )
    from simulation.orbital_configuration import resolve_orbital_physics, environment_metadata

    model = (args.model or ROOT / "model/SARM/platform/sarm_20261012.xml").resolve()
    from simulation.scripted_tasks.randomization import prepare_episode, write_model_variant
    from simulation.scripted_tasks.disturbances import EpisodeDisturbances
    source_model = model
    scene_instance = json.loads(args.scene.read_text(encoding="utf-8-sig")) if args.scene else {}
    scene_instance["scripted_task_name"] = args.task
    scene_instance.setdefault("collection_config", {})["task"] = args.task
    if args.task == "drawer_close":
        scene_instance["collection_config"]["drawer_initial_opening_m"] = args.lift_m
    if args.task in THREADED_TASKS:
        scene_instance["collection_config"]["threaded_speed_profile"] = args.threaded_speed_profile
    prepare_episode(scene_instance)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    model = write_model_variant(source_model, output / "model.xml", scene_instance)
    disturbances = EpisodeDisturbances(scene_instance)
    native.MODEL_PATH = model
    profile = load_model_profile(model)
    native.JOINTS, native.ACTUATORS = profile.joints, profile.arm_actuators
    native.KP, native.KD, native.TORQUE_LIMITS = (
        np.full(6, 500.0),
        np.full(6, 50.0),
        np.full(6, 100.0),
    )
    native.TIME_STEP = 1 / DYNAMICS_HZ
    orbital_physics = resolve_orbital_physics(scene_instance)
    if args.scene and scene_instance.get("template_id") != "sarm-task-box-20261012":
        raise ValueError("The scripted task requires the SARM 2026-10-12 scene")
    initial = np.r_[
        np.deg2rad(args.initial_joints_deg or DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0
    ]
    if args.scene and not args.initial_joints_deg:
        initial = np.asarray(
            scene_instance["randomization"]["arm_joint_position_rad"], dtype=float
        )
    task = create_task(args, model, scene_instance)
    config = task.config
    if getattr(args, "native_geometry", False):
        task.chain.enable_native_geometry()
    from simulation.scripted_tasks.native_state import STATE_READERS

    if (
        initial.shape != (8,)
        or not np.isfinite(initial).all()
        or np.any(initial[:6] < task.lower)
        or np.any(initial[:6] > task.upper)
    ):
        raise ValueError(
            "Initial arm angles must be finite and inside model joint limits"
        )
    command, velocity = initial.copy(), np.zeros(8)
    if args.task in THREADED_TASKS or is_plug_task(args.task) or args.task in DRAWER_TASKS:
        scene_instance.setdefault("collection_config", {})["environment_planning"] = {
            "schema": "native-environment-motion/2",
            "enabled": True,
            "free_motion": "straight coordinated approach first; checked free motion, then deterministic RRT-Connect and C2 smoothing",
            "approach_opening": "synchronized monotone opening; checked execution owns the approach reference",
            "contact_motion": "constrained corridor checks; original physical controllers",
            "collision_check": "finite joint-space samples; not exact continuous collision detection",
            "free_motion_clearance_m": .002,
        }
    publisher = HeldJointReferencePublisher(lambda t: (command, velocity), 8)
    sim = stepper = orbit = capture = None
    report = dict(
        task_name=args.task,
        result="failed",
        reason="initialization_incomplete",
        model=str(model),
        model_sha256=hashlib.sha256(model.read_bytes()).hexdigest(),
        config=asdict(config),
        collection_config=scene_instance.get("collection_config"),
        initial_arm_joint_deg=np.rad2deg(initial[:6]).tolist(),
        physics_hz=DYNAMICS_HZ,
        task_hz=SCRIPTED_TASK_HZ,
        native_geometry=getattr(args, "native_geometry", False),
        capture_hz=RENDER_HZ,
        wrench_hz=WRENCH_HZ,
        environment="BSK Earth/Sun + differential gravity",
        cameras=bool(args.capture_control_port),
        lerobot_recording="external_production_recorder"
        if args.capture_control_port
        else False,
        wrench_max_scope="maximum of 200 Hz task samples; not all physics substeps",
    )
    started = time.perf_counter()
    initialization_seconds["imports_model_and_task"] = started - phase_started
    report["initialization_seconds"] = initialization_seconds
    report["runtime_provenance"] = {
        "schema": "scripted-runtime-provenance/1",
        "collection_runtime_version": "optimized-collection/20261031",
        "physics_graph_reuse_enabled": graph_cache is not None,
        "collision_filter_modes": {name: os.environ.get(name, '0') for name in
                                   ('CYLINDER_FILTER_MODE', 'MESH_FILTER_MODE', 'THREAD_FILTER_MODE')},
        "python": sys.executable,
        "sources_sha256": {
            str(path.resolve().relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (Path(__file__), Path(native.__file__),
                         ROOT / "simulation/publication_scene.py",
                         ROOT / "simulation/local_mujoco_stepper.py",
                         ROOT / "simulation/local_mujoco_bindings.py",
                         ROOT / "simulation/teleop_scene.py",
                         ROOT / "simulation/scripted_tasks/task_initialization.py",
                         ROOT / "simulation/scripted_tasks/native_geometry_probe.py",
                         ROOT / "backend/space_arm_platform/task_catalog.py",
                         ROOT / "simulation/scripted_tasks/physics_graph.py")
        },
    }
    if is_plug_task(args.task):
        report["config_schema"] = task.config_schema
        report["configuration_snapshot"] = task.configuration_snapshot
        for filename in ("plug_insert.py", "plug_contact_feedback.py", "plug_clearance.py",
                         "plug_motion_config.py", "plug_profiles.py"):
            path = ROOT / "simulation/scripted_tasks" / filename
            report["runtime_provenance"]["sources_sha256"][str(path.relative_to(ROOT))] = (
                hashlib.sha256(path.read_bytes()).hexdigest())
    sample = None
    simulation_wall_seconds = 0.0
    loop_started = None
    try:
        if args.task in THREADED_TASKS or is_plug_task(args.task) or args.task in DRAWER_TASKS:
            phase_started = time.perf_counter()
            from copy import deepcopy
            from simulation.scripted_tasks.environment_motion import enable
            enable(task)
            checked_scene = deepcopy(scene_instance)
            checked_scene.setdefault("randomization", {}).update(arm_joint_position_rad=initial.tolist())
            print(json.dumps({"type":"task_preflight","output":str(output)}),flush=True)
            probe = task._planning_context.acquire()
            try:
                report["preflight"] = probe.validate_initial(checked_scene, task)
            finally:
                probe.close()
                initialization_seconds["native_preflight"] = time.perf_counter() - phase_started
            if not report["preflight"]["accepted"]:
                # The private query has released its lease; no live graph has
                # been restored or changed yet. Treat this as a sampled-pose
                # rejection, preserving the healthy worker and its old graph.
                report.update(reason="initialization_rejected: " + json.dumps(report["preflight"]),
                              failure_kind="initialization_rejected", events=task.events,
                              physics_graph_retained=graph_cache is not None and graph_cache.state is not None)
                return 1
        phase_started = time.perf_counter()
        reuse_graph = graph_cache is not None and graph_cache.compatible(locals())
        initialization_seconds["graph_compatibility"] = time.perf_counter() - phase_started
        if reuse_graph:
            sim, scene, models, recorders, ctx, orbit, plug_retention, capture, stepper, state_reader = graph_cache.restore(locals())
        else:
            print(json.dumps({"type": "loading_task", "output": str(output)}), flush=True)
            phase_started = time.perf_counter()
            sim, scene, models, recorders = native._build_simulation(
                external_reference=publisher,
                dynamics_backend="local",
                local_substeps=args.substeps,
                record_history=False,
                defer_attitude_control=True,
            )
            initialization_seconds["build_physics"] = time.perf_counter() - phase_started
            report["publication_scene"] = getattr(scene, "publication_preparation", None)
            report["model_compilation"] = sim.local_stepper.model_compilation
            phase_started = time.perf_counter()
            stepper = sim.local_stepper
            disturbances.install_external_force(stepper)
            ctx = AssemblyContext(
                sim,
                scene,
                config=SessionConfig(
                    model_path=model,
                    dynamics_backend="local",
                    publish_stride=stepper.publish_every,
                ),
            )
            orbit = OrbitComponent(
                epoch_utc=scene_instance.get("environment", {}).get(
                    "ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC
                ),
                physics=orbital_physics,
            )
            ctx.install(
                [
                    ClockComponent(),
                    orbit,
                    DynamicsCoreComponent(model),
                    OrbitalGravityComponent(mode="bsk"),
                    JointReferenceComponent(
                        publisher,
                        joints=profile.joint_names[:6],
                        actuators=profile.arm_actuators,
                        kp=native.KP,
                        kd=native.KD,
                        limits=native.TORQUE_LIMITS,
                        ik_rate_hz=SCRIPTED_TASK_HZ,
                        wire_servos=False,
                    ),
                    AttitudeComponent(model),
                ]
            )
            from simulation.components.plug_retention import install_configured_plug_retention
            plug_retention = install_configured_plug_retention(ctx)
            report["plug_retention"] = (plug_retention.configuration() if plug_retention else {"enabled": False})
            if orbital_physics.drag is not None:
                from simulation.components.drag import configured_drag_components
                ctx.install(configured_drag_components(
                    orbital_physics.drag, planet_radius_m=float(orbit.earth.radEquator)))
            report["orbital_physics"] = environment_metadata(
                orbital_physics, orbit.earth, epoch_utc=orbit.epoch_utc)
            state_reader = STATE_READERS[args.task](task, scene, stepper)
            initialization_seconds["assemble_components"] = time.perf_counter() - phase_started
            phase_started = time.perf_counter()
            if args.capture_control_port:
                from simulation.scripted_tasks.capture_transport import (
                    ScriptedCaptureTransport,
                )

                # The bridge reads material metadata from this MJCF. Using the
                # source layout here would silently restore its old colours even
                # though physics and episode metadata use the appearance variant.
                # Variants retain geom order and resolve the same mesh asset paths,
                # so the layout catalog remains valid for this realized model.
                report["render_model_path"] = str(model)
                report["render_model_sha256"] = hashlib.sha256(model.read_bytes()).hexdigest()
                capture = ScriptedCaptureTransport(
                    ctx,
                    task,
                    profile,
                    command,
                    report["model_sha256"],
                    scene_instance,
                    control_port=args.capture_control_port,
                    render_port=args.capture_render_port,
                    catalog=args.catalog,
                    state_reader=state_reader,
                    disturbances=disturbances,
                    render_model_path=model,
                )
            initialization_seconds["capture_transport"] = time.perf_counter() - phase_started
            phase_started = time.perf_counter()
            sim.InitializeSimulation()
            initialization_seconds["initialize_simulation"] = time.perf_counter() - phase_started
            report["physics_graph_reused"] = False
            if graph_cache is not None:
                graph_cache.remember(locals())
        if is_plug_task(args.task):
            report["contact_feedback"] = task.configure_contact_feedback(
                native.KP, plug_retention.parameters if plug_retention else None)
        report["asset_compilation_cache"] = configure_asset_cache()
        phase_started = time.perf_counter()
        report["orbit"] = apply_orbital_initial_state(
            scene,
            native,
            orbit.earth,
            scene_instance.get("environment", {}).get("orbit", DEFAULT_ORBIT),
            scene_instance.get("randomization") or default_target_state(model),
            initial,
            orbit.origin,
        )
        joints = [scene.getBody(b).getScalarJoint(j) for b, j in profile.joints]
        qadr, dadr = [j.getQposAdr() for j in joints], [j.getQvelAdr() for j in joints]
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        initialization_seconds["apply_initial_state"] = time.perf_counter() - phase_started
        report["initialization_wall_seconds"] = time.perf_counter() - initialization_started
        report["initial_state"] = dict(qpos=stepper.qpos.tolist(), qvel=stepper.qvel.tolist())
        print(json.dumps(dict(type="initialization_complete",
                             seconds=report["initialization_wall_seconds"],
                             phases=initialization_seconds,
                             publication=report["publication_scene"])), flush=True)
        loop_started = time.perf_counter()
        event_count = 0
        preroll_ns = capture_preroll_ns(args.warmup_sim_seconds)
        task_epoch_ns = None if capture else int(round(args.warmup_sim_seconds * 1e9))
        frame = 0
        # Disjoint timings inside the active recording loop. Exclude startup,
        # pre-roll and video finalization; no per-tick logging or profiling.
        recording_work = dict(transport_s=0., backpressure_wait_s=0., state_sample_s=0.,
                              task_control_s=0., simulation_s=0., loop_other_s=0.)
        trace_context = (output / "trace.jsonl").open("w", encoding="utf-8") if getattr(args, "save_trace", True) else nullcontext(None)
        with trace_context as trace:
            while True:
                iteration_started = time.perf_counter()
                timing_active = bool(capture and capture.enabled and task.result is None)
                if capture:
                    capture.flush()
                    if args.capture_stop_file and args.capture_stop_file.exists():
                        break
                    if (
                        task.result is not None
                        and not capture.enabled
                        and not capture.render.bridge.last_capture_state.get(
                            "capture_episode_id"
                        )
                    ):
                        # OFF has been published; keep transport alive while videos finalize.
                        time.sleep(0.01)
                        continue
                    if not capture.can_advance():
                        wait_started = time.perf_counter()
                        time.sleep(getattr(args, "capture_backpressure_poll_s", 0.002))
                        if timing_active:
                            recording_work["transport_s"] += wait_started - iteration_started
                            recording_work["backpressure_wait_s"] += time.perf_counter() - wait_started
                        continue
                transport_finished = time.perf_counter()
                stamp = tick_time_ns(frame, SCRIPTED_TASK_HZ)
                if (
                    capture
                    and task_epoch_ns is None
                    and stamp >= preroll_ns
                    and not capture.enabled
                ):
                    # Deterministic pre-roll: wall-clock UE startup must not change the initial physical state.
                    time.sleep(0.01)
                    continue
                if capture and capture.enabled and task_epoch_ns is None:
                    task_epoch_ns = stamp
                    capture.task_start_ns = stamp
                elapsed = (
                    0.0 if task_epoch_ns is None else (stamp - task_epoch_ns) * 1e-9
                )
                sample_started = time.perf_counter()
                wrench = WRIST_SENSOR.observation(stepper, stamp)
                if wrench is None:
                    raise RuntimeError(
                        "The task requires the wrist force/torque sensor"
                    )
                sample = state_reader.sample(
                    elapsed,
                    stepper.qpos[qadr].copy(),
                    stepper.qvel[dadr].copy(),
                    wrench,
                )
                # Metrics are serialized only for trace/progress output. The
                # controller consumes the original sample on every 200 Hz tick.
                if trace is not None or frame % 150 == 0:
                    metrics = state_reader.metrics(sample)
                control_started = time.perf_counter()
                if task_epoch_ns is not None and elapsed >= 0:
                    if task.result is None:
                        if capture and not capture.enabled:
                            task.fail("capture_stopped_before_task_completed", sample)
                        elif elapsed >= args.max_sim_seconds:
                            task.fail("total_simulation_timeout", sample)
                    from simulation.scripted_tasks.obstacle_planner import PlanningFailure
                    try:
                        nominal_command, nominal_velocity = task.step(sample)
                    except PlanningFailure as error:
                        # Publish the failed task normally so both camera queues
                        # can finish and the dataset records a rejected episode.
                        task.fail(f"planning_failed: {error}", sample)
                        nominal_command, nominal_velocity = task.reference.copy(), np.zeros(8)
                    if task.result is None:
                        command[:], velocity[:] = disturbances.action(nominal_command, nominal_velocity, elapsed, task.lower, task.upper)
                    else:
                        command[:], velocity[:] = nominal_command, nominal_velocity
                    if capture:
                        capture.sequence += 1
                        capture.task_elapsed_s = elapsed
                control_finished = time.perf_counter()
                applied_wrench = disturbances.write_external_force(elapsed, active=task_epoch_ns is not None and elapsed >= 0 and task.result is None)
                measured_q, measured_v, measured_stamp = disturbances.observe(sample.joints, sample.joint_velocities, stamp)
                if trace is not None:
                    pose, rotation = task.chain.forward(sample.joints[:6])
                    row = dict(
                        sim_time_ns=str(stamp),
                        stage=task.stage,
                        result=task.result,
                        joint_position=sample.joints.tolist(),
                        joint_velocity=sample.joint_velocities.tolist(),
                        joint_reference=command.tolist(),
                        joint_reference_velocity=velocity.tolist(),
                        tool_position_body_m=pose.tolist(),
                        tool_rotation_body=rotation.tolist(),
                        **metrics,
                        wrist_wrench=wrench,
                        measured_wrist_wrench=disturbances.wrench(wrench),
                        measured_joint_position=measured_q.tolist(),
                        measured_joint_velocity=measured_v.tolist(),
                        joint_measurement_time_ns=str(measured_stamp),
                        nominal_joint_reference=task.reference.tolist(),
                        applied_external_wrench=applied_wrench.tolist(),
                        plug_retention=plug_retention.telemetry() if plug_retention else {},
                        local_qpos=stepper.qpos.tolist(),
                    )
                    trace.write(json.dumps(row) + "\n")
                if len(task.events) > event_count:
                    for event in task.events[event_count:]:
                        print(json.dumps({"type": "task_stage", **event}), flush=True)
                    event_count = len(task.events)
                    if trace is not None:
                        trace.flush()
                if frame % 150 == 0:
                    if trace is None:
                        pose, _ = task.chain.forward(sample.joints[:6])
                    print(
                        json.dumps(
                            {
                                "type": "task_progress",
                                "sim_seconds": sample.time_s,
                                "stage": task.stage,
                                "task_state": metrics,
                                "tool_position_body_m": pose.tolist(),
                                "force_n": float(np.linalg.norm(sample.wrench[:3])),
                            }
                        ),
                        flush=True,
                    )
                if not capture and task.result is not None:
                    break
                sim.ConfigureStopTime(tick_time_ns(frame + 1, SCRIPTED_TASK_HZ))
                step_started = time.perf_counter()
                sim.ExecuteSimulation()
                step_finished = time.perf_counter()
                simulation_wall_seconds += step_finished - step_started
                if timing_active:
                    recording_work["transport_s"] += transport_finished - iteration_started
                    recording_work["state_sample_s"] += control_started - sample_started
                    recording_work["task_control_s"] += control_finished - control_started
                    recording_work["simulation_s"] += step_finished - step_started
                    recording_work["loop_other_s"] += (sample_started - transport_finished
                                                        + step_started - control_finished)
                frame += 1
            if task.result is None:
                task.fail("total_simulation_timeout", sample)
        report.update(
            recording_work_wall_seconds={**recording_work, "total_s": sum(recording_work.values())},
            result=task.result,
            reason=task.reason,
            simulated_seconds=sample.time_s,
            task_seconds=next(
                (
                    event["sim_time_s"]
                    for event in task.events
                    if event["stage"] in ("success", "failed")
                ),
                sample.time_s,
            ),
            **state_reader.metrics(sample),
            max_wrist_force_n=task.max_force_n,
            max_wrist_torque_nm=task.max_torque_nm,
            events=task.events,
            substeps=stepper.substeps,
            dynamics=stepper.telemetry(),
            schedule=ctx.summary(),
            plug_retention_final=plug_retention.telemetry() if plug_retention else {},
        )
        _ = models, recorders, ctx
    except (Exception, KeyboardInterrupt) as exc:
        report.update(
            result="failed", reason=f"{type(exc).__name__}: {exc}", events=task.events
        )
        if graph_cache is not None:
            graph_cache.close()
            if sim is not None and getattr(sim, "local_stepper", None) is stepper:
                # The cache may have owned and just closed these resources.
                if getattr(stepper, "_handle", None) is None:
                    stepper = orbit = capture = None
        raise
    finally:
        unwinding = sys.exc_info()[0] is not None
        provenance_error = None
        try:
            from collection.runtime_provenance import loaded_native_provenance
            report["runtime_provenance"]["loaded_native"] = loaded_native_provenance(ROOT)
        except Exception as error:
            provenance_error = error
            report["runtime_provenance_error"] = f"{type(error).__name__}: {error}"
            if not unwinding:
                report.update(result="failed", reason="native_runtime_identity_mismatch")
        finished = time.perf_counter()
        report["wall_seconds"] = finished - started
        report["task_loop_wall_seconds"] = (
            None if loop_started is None else finished - loop_started
        )
        report["execute_simulation_wall_seconds"] = simulation_wall_seconds
        report["motion_plans"] = getattr(task,"motion_plans",[])
        if hasattr(task, "_planning_context"):
            report["planning_context"] = task._planning_context.stats
        (output / "summary.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        close_task_runtime(sim, stepper, orbit, capture, graph_cache)
        print(
            json.dumps(
                {
                    "type": "task_result",
                    "result": report["result"],
                    "reason": report["reason"],
                    "report": str(output / "summary.json"),
                }
            ),
            flush=True,
        )
        if provenance_error is not None:
            if graph_cache is not None:
                graph_cache.close()
            if not unwinding:
                raise provenance_error
    return 0 if task.result == "success" else 1


def main(default_task="drawer_lift", argv=None, *, graph_cache=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task",
        choices=RUNNER_TASK_CHOICES,
        default=default_task,
    )
    parser.add_argument(
        "--model",
        type=Path,
        help="Diagnostic model override; recording requires a validated layout manifest and matching catalog",
    )
    parser.add_argument("--target-site", default=None)
    parser.add_argument(
        "--plug-grasp-offset", type=float, nargs=3, default=None
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--max-sim-seconds", type=float, default=None)
    parser.add_argument("--substeps", type=int, default=None)
    parser.add_argument("--save-trace", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--native-geometry", action="store_true", help="Use the installed exact C++ FK/Jacobian helper")
    parser.add_argument("--capture-backpressure-poll-s", type=float, default=0.002)
    parser.add_argument("--warmup-sim-seconds", type=float, default=0.0)
    parser.add_argument("--lift-m", type=float, default=0.08)
    parser.add_argument("--arm-speed", type=float, default=None)
    add_motion_options(parser)
    parser.add_argument("--force-limit", type=float, default=None,
                        help="Explicit force limit; otherwise keep the task's resolved configuration")
    parser.add_argument(
        "--grip-offset-m", type=float, nargs=3, default=[0.0, 0.0, -0.005]
    )
    parser.add_argument("--initial-joints-deg", type=float, nargs=6)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--capture-control-port", type=int)
    parser.add_argument("--capture-render-port", type=int)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--capture-stop-file", type=Path)
    args = parser.parse_args(argv)
    if args.max_sim_seconds is None:
        args.max_sim_seconds = task_default_duration_s(args.task)
    try:
        motion_arguments(args.task, args.linear_speed, args.threaded_speed_profile)
    except ValueError as error:
        parser.error(str(error))
    if (
        args.model is not None
        and not args.model.with_name("attitude_control.json").is_file()
    ):
        parser.error(
            "--model needs its companion attitude_control.json in the same directory"
        )
    if args.output is None:
        folder = args.task.replace("_", "-")
        args.output = ROOT / "run" / folder / time.strftime("%Y%m%d-%H%M%S")
    if args.capture_control_port and not (
        args.capture_render_port
        and args.catalog
        and args.scene
        and args.capture_stop_file
    ):
        parser.error("capture mode requires render port, catalog, scene, and stop file")
    if args.model is not None and args.capture_control_port:
        from simulation.capture_layout import validated_capture_layout
        try:
            layout = validated_capture_layout(args.model, args.catalog)
            scene = json.loads(args.scene.read_text(encoding="utf-8-sig"))
            if scene.get("task_box_layout") != layout:
                raise ValueError("Capture scene and selected layout manifest differ")
        except (ValueError, KeyError, OSError) as error:
            parser.error(f"Generated layout capture is not configured: {error}")
    if (
        not np.isfinite(args.warmup_sim_seconds)
        or not 0 <= args.warmup_sim_seconds <= 60
    ):
        parser.error("--warmup-sim-seconds must be finite and in [0,60]")
    if not np.isfinite(args.max_sim_seconds) or args.max_sim_seconds <= 0:
        parser.error("--max-sim-seconds must be finite and positive")
    if not 0.001 <= args.capture_backpressure_poll_s <= 0.1:
        parser.error("--capture-backpressure-poll-s must be in [0.001, 0.1]")
    return run(args, graph_cache=graph_cache)


if __name__ == "__main__":
    sys.exit(main())
