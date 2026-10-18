"""Validate a scripted task with the platform's authoritative BSK/MuJoCo dynamics.

No UI or running scene required. Output is a diagnostic trace, not a camera
training episode. Original contacts, latch, free base, servos and SDFs are kept.
"""

from __future__ import annotations
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [
    str(ROOT),
    str(ROOT / "backend"),
    str(ROOT.parent / "space_sim_UE_Adapter/Adapters"),
    str(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples"),
]

from simulation.scripted_tasks.plug_profiles import (
    PLUG_PROFILES,
    is_plug_task,
    plug_profile,
)


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


def run(args):
    # Establish the production DLL load order. Never import Python mujoco here.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(ROOT / "model/SARM/platform")
    from simulation.assembly import AssemblyContext, SessionConfig
    from simulation.components.attitude import AttitudeComponent
    from simulation.components.clock import ClockComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.arm_reference import JointReferenceComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from simulation.sarm_model_profile import load_model_profile
    from simulation.scripted_tasks.drawer_lift import (
        DrawerLiftConfig,
        DrawerLiftTask,
    )
    from simulation.teleop_grasp_unreal import (
        DEFAULT_ORBIT,
        DEFAULT_EPHEMERIS_EPOCH_UTC,
        _apply_orbital_initial_state,
    )
    from simulation.wrench_sensors import WRIST_SENSOR
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
    from space_arm_platform.sampling import (
        tick_time_ns,
        DYNAMICS_HZ,
        SCRIPTED_TASK_HZ,
        RENDER_HZ,
        WRENCH_HZ,
    )

    model = (args.model or ROOT / "model/SARM/platform/sarm_20261012.xml").resolve()
    native.MODEL_PATH = model
    profile = load_model_profile(model)
    native.JOINTS, native.ACTUATORS = profile.joints, profile.arm_actuators
    native.KP, native.KD, native.TORQUE_LIMITS = (
        np.full(6, 500.0),
        np.full(6, 50.0),
        np.full(6, 100.0),
    )
    native.TIME_STEP = 1 / DYNAMICS_HZ
    scene_instance = (
        json.loads(args.scene.read_text(encoding="utf-8-sig")) if args.scene else {}
    )
    if any(
        scene_instance.get("collection_config", {}).get("disturbances", {}).values()
    ):
        raise ValueError("This baseline collector supports disabled disturbances only")
    if scene_instance and scene_instance.get("template_id") != "sarm-task-box-20261012":
        raise ValueError("The scripted task requires the SARM 2026-10-12 scene")
    initial = np.r_[
        np.deg2rad(args.initial_joints_deg or DEFAULT_OPERATING_JOINT_DEG), 0.0, 0.0
    ]
    if args.scene and not args.initial_joints_deg:
        initial = np.asarray(
            scene_instance["randomization"]["arm_joint_position_rad"], dtype=float
        )
    config = DrawerLiftConfig(
        lift_m=args.lift_m,
        grip_offset_m=tuple(args.grip_offset_m),
        arm_speed_rad_s=args.arm_speed,
        force_limit_n=args.force_limit,
    )
    if is_plug_task(args.task):
        from simulation.scripted_tasks.plug_insert import (
            PlugInsertTask,
            PlugInsertConfig,
        )

        config = PlugInsertConfig(
            plug_body=plug_profile(args.task).body,
            target_site=args.target_site,
            arm_speed_rad_s=args.arm_speed,
            grasp_offset_m=tuple(args.plug_grasp_offset),
            force_limit_n=min(args.force_limit, 40.0),
        )
        task = PlugInsertTask(model, config)
    elif args.task == "drawer_cycle":
        from simulation.scripted_tasks.drawer_cycle import (
            DrawerCycleTask,
            DrawerCycleConfig,
        )

        config = DrawerCycleConfig(**asdict(config))
        task = DrawerCycleTask(model, config)
    else:
        task = DrawerLiftTask(model, config)
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
    publisher = HeldJointReferencePublisher(lambda t: (command, velocity), 8)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
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
    sample = None
    simulation_wall_seconds = 0.0
    loop_started = None
    try:
        print(json.dumps({"type": "loading_task", "output": str(output)}), flush=True)
        sim, scene, models, recorders = native._build_simulation(
            external_reference=publisher,
            dynamics_backend="local",
            local_substeps=args.substeps,
            record_history=False,
            defer_attitude_control=True,
        )
        stepper = sim.local_stepper
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
            )
        )
        ctx.install(
            [
                ClockComponent(),
                orbit,
                DynamicsCoreComponent(model),
                OrbitalGravityComponent(),
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
        state_reader = STATE_READERS[args.task](task, scene, stepper)
        if args.capture_control_port:
            from simulation.scripted_tasks.capture_transport import (
                ScriptedCaptureTransport,
            )

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
            )
        sim.InitializeSimulation()
        report["orbit"] = _apply_orbital_initial_state(
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
        loop_started = time.perf_counter()
        event_count = 0
        task_epoch_ns = None if capture else int(round(args.warmup_sim_seconds * 1e9))
        frame = 0
        with (output / "trace.jsonl").open("w", encoding="utf-8") as trace:
            while True:
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
                        time.sleep(0.01)
                        continue
                stamp = tick_time_ns(frame, SCRIPTED_TASK_HZ)
                if (
                    capture
                    and task_epoch_ns is None
                    and stamp >= int(round(args.warmup_sim_seconds * 1e9))
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
                metrics = state_reader.metrics(sample)
                if task_epoch_ns is not None and elapsed >= 0:
                    if task.result is None:
                        if capture and not capture.enabled:
                            task.fail("capture_stopped_before_task_completed", sample)
                        elif elapsed >= args.max_sim_seconds:
                            task.fail("total_simulation_timeout", sample)
                    command[:], velocity[:] = task.step(sample)
                    if capture:
                        capture.sequence += 1
                        capture.task_elapsed_s = elapsed
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
                    local_qpos=stepper.qpos.tolist(),
                )
                trace.write(json.dumps(row) + "\n")
                if len(task.events) > event_count:
                    for event in task.events[event_count:]:
                        print(json.dumps({"type": "task_stage", **event}), flush=True)
                    event_count = len(task.events)
                    trace.flush()
                if frame % 150 == 0:
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
                simulation_wall_seconds += time.perf_counter() - step_started
                frame += 1
            if task.result is None:
                task.fail("total_simulation_timeout", sample)
        report.update(
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
        )
        _ = models, recorders, ctx
    except (Exception, KeyboardInterrupt) as exc:
        report.update(
            result="failed", reason=f"{type(exc).__name__}: {exc}", events=task.events
        )
        raise
    finally:
        finished = time.perf_counter()
        report["wall_seconds"] = finished - started
        report["task_loop_wall_seconds"] = None if loop_started is None else finished - loop_started
        report["execute_simulation_wall_seconds"] = simulation_wall_seconds
        (output / "summary.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
        if capture is not None:
            capture.close()
        if stepper is not None:
            stepper.close()
        if orbit is not None and orbit.gravity_factory is not None:
            orbit.gravity_factory.unloadSpiceKernels()
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
    return 0 if task.result == "success" else 1


def main(default_task="drawer_lift"):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--task",
        choices=["drawer_lift", "drawer_cycle", "plug_insert", *PLUG_PROFILES],
        default=default_task,
    )
    parser.add_argument("--model", type=Path, help="Offline diagnostic model override (no camera recording)")
    parser.add_argument("--target-site", default=None)
    parser.add_argument(
        "--plug-grasp-offset", type=float, nargs=3, default=[0.0, 0.0, -0.003]
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument("--max-sim-seconds", type=float, default=360.0)
    parser.add_argument("--substeps", type=int, default=None)
    parser.add_argument("--warmup-sim-seconds", type=float, default=0.0)
    parser.add_argument("--lift-m", type=float, default=0.08)
    parser.add_argument("--arm-speed", type=float, default=0.08)
    parser.add_argument("--force-limit", type=float, default=80.0)
    parser.add_argument(
        "--grip-offset-m", type=float, nargs=3, default=[0.0, 0.0, -0.005]
    )
    parser.add_argument("--initial-joints-deg", type=float, nargs=6)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--capture-control-port", type=int)
    parser.add_argument("--capture-render-port", type=int)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--capture-stop-file", type=Path)
    args = parser.parse_args()
    if args.model is not None and not args.model.with_name("attitude_control.json").is_file():
        parser.error("--model needs its companion attitude_control.json in the same directory")
    if args.model is not None and args.capture_control_port:
        parser.error("--model is for offline diagnostics; camera capture requires the scene catalog model")
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
    if (
        not np.isfinite(args.warmup_sim_seconds)
        or not 0 <= args.warmup_sim_seconds <= 60
    ):
        parser.error("--warmup-sim-seconds must be finite and in [0,60]")
    if not np.isfinite(args.max_sim_seconds) or args.max_sim_seconds <= 0:
        parser.error("--max-sim-seconds must be finite and positive")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
