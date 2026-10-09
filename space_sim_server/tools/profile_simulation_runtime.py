"""Profile the CURRENT SARM teleop path without connecting to a live platform.

Uses the production selectable dynamics backend, 240/120/30 Hz scheduling, contact, gravity, wheel safety,
IK, render serialization and authoritative observations. No UE/GPU, sockets,
LeRobot writer or disk I/O are included in timed simulation intervals. Run each
case in a fresh Basilisk-enabled Python process (never preload Python MuJoCo).
"""
from __future__ import annotations

import argparse
from contextlib import ExitStack
import cProfile
import json
import math
from pathlib import Path
import pstats
import sys
import time
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
MOTIONS = ("hold", "linear_x", "linear_y", "linear_z", "roll", "pitch", "yaw", "grasp_script", "sweep_script")
# Contact-free six-axis teleop sweep for trajectory-accuracy comparisons.
SWEEP_SCRIPT = (
    (0.25, 1.25, (0.05, 0.0, 0.0), (0.0, 0.0, 0.0), 0.0),
    (1.25, 2.25, (0.0, -0.05, 0.0), (0.0, 0.0, 0.0), 0.0),
    (2.25, 3.25, (0.0, 0.0, 0.05), (0.0, 0.0, 0.0), 0.0),
    (3.25, 4.25, (0.0, 0.0, 0.0), (0.3, 0.0, 0.0), 0.0),
    (4.25, 5.25, (0.0, 0.0, 0.0), (0.0, 0.3, 0.0), 0.0),
    (5.25, 6.25, (0.0, 0.0, 0.0), (0.0, 0.0, 0.3), 0.005),
)
# Fixed, reproducible contact replay for the parked-plug diagnostic scene:
# (start_s, stop_s, linear_body_m_s, angular_body_rad_s, gripper_m_s).
GRASP_SCRIPT = (
    (0.25, 2.25, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), -0.01),  # close and squeeze the plug
    (2.25, 3.75, (0.0, 0.0, 0.02), (0.0, 0.0, 0.0), 0.0),   # drag it along body +Z
    (3.75, 5.25, (0.0, 0.0, 0.0), (0.0, 0.0, 0.1), 0.0),    # yaw while holding it
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--adapter-root", type=Path, required=True)
    result.add_argument("--scene-instance", type=Path, required=True)
    result.add_argument("--model-path", type=Path, help="Optional MJCF override for controlled collision-cost comparison")
    result.add_argument("--output", type=Path, required=True, help="New JSON summary path; sidecars use the same stem")
    result.add_argument("--duration", type=float, default=6.0)
    result.add_argument("--motion", choices=MOTIONS, default="hold")
    result.add_argument("--speed", type=float, help="m/s for translation, rad/s for rotation (defaults .02 / .1)")
    result.add_argument("--initial-state", choices=("saved", "operating"), default="saved")
    result.add_argument("--ik-mode", choices=("ik_pose", "strict"), default="ik_pose")
    result.add_argument("--elbow-mode", choices=("default", "off"), default="default")
    result.add_argument("--wrist-mode", choices=("default", "off"), default="default")
    result.add_argument("--joint3-mode", choices=("default", "off"), default="default")
    result.add_argument("--disable-attitude", action="store_true")
    result.add_argument("--enable-extra-eom-call", action="store_true")
    result.add_argument("--disable-extra-eom-call", action="store_true", help=argparse.SUPPRESS)
    result.add_argument("--rkf-relative-tol", type=float, default=1.0e-4)
    result.add_argument("--rkf-absolute-tol", type=float, default=1.0e-4)
    result.add_argument("--rkf-state-abs-tol", type=float, default=None,
                        help="Absolute tolerance applied to the bulk mujocoQpos/mujocoQvel states only")
    result.add_argument("--profile", action="store_true", help="Profile Python callback bodies; adds overhead")
    result.add_argument("--native-timers", action="store_true", help="Bracket SPICE/gravity/MJScene; adds callbacks/overhead")
    result.add_argument("--eom-cost-probe", action="store_true",
                        help="Time one MJScene::equationsOfMotion call (one RKF45 stage); read-only")
    result.add_argument("--posture-backend", choices=("python", "native"), default=None,
                        help="Defaults to SPACE_SIM_POSTURE_BACKEND, else native")
    result.add_argument("--mj-threads", type=int, choices=range(9), default=None,
                        help="Experimental MuJoCo worker threads + caller; 0 probes the serial baseline. Never imports Python MuJoCo.")
    result.add_argument("--dynamics-backend", choices=("basilisk", "local"), default=None,
                        help="Defaults to SPACE_SIM_DYNAMICS_BACKEND, else local. "
                             "basilisk: MJScene + RKF45 (reference); local: fixed-step MuJoCo")
    result.add_argument("--orbital-mode", choices=("bsk", "linear_tidal"), default=None,
                        help="Native BSK orbital components or legacy tidal baseline (local only)")
    result.add_argument("--local-substeps", type=int, default=None, help="MuJoCo substeps per 240 Hz step (local)")
    result.add_argument("--local-publish-stride", type=int, default=None,
                        help="Body/site kinematics publication stride (local; SPACE_SIM_LOCAL_PUBLISH_STRIDE)")
    result.add_argument("--atmospheric-drag", action="store_true",
                        help="Add the exponential-atmosphere drag component (local backend only)")
    return result


def benchmark(args) -> dict:
    import numpy as np

    if not math.isfinite(args.duration) or args.duration < 1:
        raise ValueError("duration must be finite and at least one simulated second")
    if args.speed is not None and (not math.isfinite(args.speed) or args.speed <= 0):
        raise ValueError("speed must be positive and finite")
    if args.dynamics_backend == "local":
        # These probe MJScene's own RKF45 integration, which does not run.
        incompatible = [flag for flag, used in (
            ("--native-timers", args.native_timers), ("--eom-cost-probe", args.eom_cost_probe),
            ("--rkf-state-abs-tol", args.rkf_state_abs_tol is not None), ("--mj-threads", args.mj_threads is not None),
            ("--enable-extra-eom-call", args.enable_extra_eom_call)) if used]
        if incompatible:
            raise ValueError(f"{', '.join(incompatible)} apply only to the basilisk backend")
    elif args.local_substeps is not None or args.local_publish_stride is not None:
        raise ValueError("--local-substeps/--local-publish-stride apply only to the local backend")
    adapter = args.adapter_root.resolve()
    examples = adapter / "Unreal/BskUnrealRenderer/examples"
    if not (examples / "scenario_spacecraft_arm_grasp_unreal.py").is_file():
        raise ValueError("adapter-root must be the repository containing Adapters/ and Unreal/")
    out = args.output.resolve()
    suffixes = (".json", ".scene.json", ".observations.jsonl", ".frames.jsonl", ".pstats", ".profile.txt")
    if out.suffix != ".json" or any(out.with_suffix(s).exists() for s in suffixes):
        raise ValueError("output must be a NEW .json path with unused sidecar names")
    # Copy, never modify, the saved scene; pacing alone is accelerated. The
    # optional operating pose is explicit, since old scenes required preparation.
    document = json.loads(args.scene_instance.read_text(encoding="utf-8-sig"))
    document["runtime"]["simulation_rate"] = 100.0
    if args.initial_state == "operating":
        from_angles = document.get("operating_arm_joint_position_deg")
        if from_angles is None:
            raise ValueError("scene has no saved operating joint angles")
        document["randomization"]["arm_joint_position_rad"][:6] = np.deg2rad(from_angles).tolist()
        document["arm_preparation_required"] = False
        # This copied diagnostic scene starts directly at the accepted operating
        # pose. Never disable the preparation gate in the user's saved instance.
        if document.get("randomization_profile") == "teleop-zero-prepare-v2":
            document["randomization_profile"] = "teleop-zero-prepare-v1"
            document.pop("arm_preparation_plan", None)
    out.parent.mkdir(parents=True, exist_ok=True)
    scene_path = out.with_suffix(".scene.json")
    scene_path.write_text(json.dumps(document, indent=2), encoding="utf-8")
    sys.path[:0] = [str(ROOT), str(ROOT / "backend"), str(adapter / "Adapters"), str(examples)]
    import os
    os.environ["SPACE_SIM_POSTURE_BACKEND"] = args.posture_backend
    os.environ["SPACE_SIM_DYNAMICS_BACKEND"] = args.dynamics_backend
    if args.local_publish_stride is not None:
        os.environ["SPACE_SIM_LOCAL_PUBLISH_STRIDE"] = str(args.local_publish_stride)
    if args.enable_extra_eom_call and args.disable_extra_eom_call:
        raise ValueError("enable/disable extra EOM flags are mutually exclusive")
    if args.enable_extra_eom_call:
        os.environ["SPACE_SIM_EXTRA_EOM_CALL"] = "1"
    elif args.disable_extra_eom_call:
        os.environ["SPACE_SIM_EXTRA_EOM_CALL"] = "0"
    if args.elbow_mode == "off": os.environ["SPACE_SIM_ONLINE_ELBOW_MODE"] = "off"
    if args.wrist_mode == "off": os.environ["SPACE_SIM_ONLINE_WRIST_MODE"] = "off"
    if args.joint3_mode == "off": os.environ["SPACE_SIM_ONLINE_JOINT3_MODE"] = "off"
    import scenario_spacecraft_arm_grasp_unreal as loader
    native = loader.load_native_grasp_module(ROOT / "model/SARM/platform")
    if args.model_path is not None:
        model_path = args.model_path.resolve()
        if not model_path.is_file():
            raise ValueError(f"model-path does not exist: {model_path}")
        native.MODEL_PATH = model_path
    import bsk_render_adapter
    from bsk_render_adapter.protocol import RecordingOnlyPublisher, encode_packet
    from simulation import teleop_grasp_unreal as teleop
    from simulation import attitude_control
    from simulation import native_integration
    if not math.isfinite(args.rkf_relative_tol) or args.rkf_relative_tol <= 0:
        raise ValueError("rkf-relative-tol must be positive and finite")
    if not math.isfinite(args.rkf_absolute_tol) or args.rkf_absolute_tol <= 0:
        raise ValueError("rkf-absolute-tol must be positive and finite")
    native_integration.RELATIVE_TOLERANCE = args.rkf_relative_tol
    native_integration.ABSOLUTE_TOLERANCE = args.rkf_absolute_tol
    from simulation.physics_clock import RationalPhysicsClock
    from simulation.observation_capture import AuthoritativeObservationModel
    from simulation.joint_reference_publisher import HeldJointReferencePublisher

    profile = cProfile.Profile()
    execute_wall, execute_cpu, observations, frames = [], [], [], []
    timing = {}
    scene_probes, probe_samples = [], []
    counters = {"render_frames": 0, "render_encoded_bytes": 0}
    clock = {"seconds": 0.0}

    class Sink(RecordingOnlyPublisher):
        def publish_frame(self, message):
            counters["render_frames"] += 1
            counters["render_encoded_bytes"] += len(encode_packet(message))
            # Inertial origin + local positions for backend comparisons; the
            # same small cost is paid by both backends.
            frames.append({"sim_time_ns": message["sim_time_ns"], "origin_N_m": message["origin_N_m"],
                           "objects": {o["object_id"]: o["position_m"] for o in message["objects"]}})

    class OfflineBridge(bsk_render_adapter.BasiliskRenderBridge):
        def __init__(self, **kwargs):
            super().__init__(**kwargs, publisher=Sink())

        def add_mj_scene(self, scene, **kwargs):
            kwargs["mesh_asset_catalog"] = None  # No UE assets needed; not in timed region.
            return super().add_mj_scene(scene, **kwargs)

    class TimedTarget(teleop.CartesianTeleopTarget):
        def update(self, seconds):
            clock["seconds"] = seconds
            return super().update(seconds)

    class OfflineClient(teleop.SimulationControlClient):
        def start(self):
            pass

        def wait_until_ready(self, timeout=10.0):
            pass  # Deliberately no live backend in this compute-only benchmark.

        def close(self):
            pass

        def latest_action(self):
            if args.motion in ("grasp_script", "sweep_script"):
                now = clock["seconds"]
                script = GRASP_SCRIPT if args.motion == "grasp_script" else SWEEP_SCRIPT
                for start, stop, linear, angular, gripper in script:
                    if start <= now < stop:
                        return dict(deadman=True, server_sequence="1", allow_reference_recovery=False,
                                    end_effector_linear_velocity_body_m_s=list(linear),
                                    end_effector_angular_velocity_body_rad_s=list(angular),
                                    gripper_velocity_m_s=gripper), False
                return dict(deadman=False, server_sequence="1", allow_reference_recovery=True,
                            end_effector_linear_velocity_body_m_s=[0.0] * 3,
                            end_effector_angular_velocity_body_rad_s=[0.0] * 3,
                            gripper_velocity_m_s=0.0), False
            active = args.motion != "hold" and .25 <= clock["seconds"] < args.duration - .25
            linear, angular = [0.0] * 3, [0.0] * 3
            if active:
                if args.motion.startswith("linear_"):
                    linear[("linear_x", "linear_y", "linear_z").index(args.motion)] = args.speed or .02
                else:
                    angular[("roll", "pitch", "yaw").index(args.motion)] = args.speed or .1
            return dict(deadman=active, server_sequence="1", allow_reference_recovery=not active,
                        end_effector_linear_velocity_body_m_s=linear,
                        end_effector_angular_velocity_body_rad_s=angular,
                        gripper_velocity_m_s=0.0), False

        def send_observation(self, message):
            observations.append(message)
            return True

    class Timer(native.sysModel.SysModel):
        def __init__(self, key, before):
            super().__init__()
            self.key, self.before = key, before
            self.ModelTag = f"diagnostic_{key}_{before}"
            timing.setdefault(key, {"calls": 0, "elapsed_s": 0.0, "start": 0.0})

        def UpdateState(self, nanos):
            now, sample = time.perf_counter(), timing[self.key]
            if self.before:
                sample["start"] = now
            else:
                sample["elapsed_s"] += now - sample["start"]
                sample["calls"] += 1

    original_build = native._build_simulation

    sessions = []

    def build(**kwargs):
        sim, scene, models, recorders = original_build(**kwargs)
        sessions.append(sim)
        if args.native_timers:
            original_add = scene.AddModelToDynamicsTask

            def register(model, priority):
                if model.ModelTag == "earthSunGravity" or "spice" in model.ModelTag.lower():
                    before, after = Timer(model.ModelTag, True), Timer(model.ModelTag, False)
                    original_add(before, priority + 1)
                    original_add(after, priority - 1)
                    models.extend((before, after))
                return original_add(model, priority)

            scene.AddModelToDynamicsTask = register
            before, after = Timer("MJScene", True), Timer("MJScene", False)
            sim.AddModelToTask("graspTask", before, 1)
            sim.AddModelToTask("graspTask", after, -1)
            models.extend((before, after))
        original_execute = sim.ExecuteSimulation

        def execute(*positional, **keywords):
            wall, cpu = time.perf_counter(), time.process_time()
            try:
                return original_execute(*positional, **keywords)
            finally:
                execute_wall.append(time.perf_counter() - wall)
                execute_cpu.append(time.process_time() - cpu)
                # Sampling is outside the timed execute region and identical
                # for 0/1/2/4/8-worker trials.
                if scene_probes:
                    probe_samples.append(scene_probes[-1].snapshot())

        sim.ExecuteSimulation = execute
        return sim, scene, models, recorders

    original_initialize = native._initialize_state

    def initialize(sim, scene):
        original_initialize(sim, scene)
        if args.rkf_state_abs_tol is not None:
            # MJScene registers whole-robot qpos/qvel as two bulk states, so this
            # relaxes arm joints and free bodies together. Diagnostic only.
            integrator = scene._teleop_integrator
            for name in ("mujocoQpos", "mujocoQvel"):
                integrator.setAbsoluteTolerance(name, float(args.rkf_state_abs_tol))
            print(json.dumps({"type": "state_tolerance_probe",
                              "states": ["mujocoQpos", "mujocoQvel"],
                              "absolute_tolerance": args.rkf_state_abs_tol}), flush=True)
        if args.eom_cost_probe:
            # One equationsOfMotion call is exactly one full MuJoCo forward
            # dynamics pass, i.e. one RKF45 stage. Dividing the measured step
            # cost by this reveals how many adaptive stages a step really took.
            import statistics
            samples = []
            for index in range(20):
                start = time.perf_counter()
                scene.equationsOfMotion(index * 1.0e-6, 1.0 / 240.0)
                samples.append(time.perf_counter() - start)
            print(json.dumps({"type": "eom_cost_probe", "samples": len(samples),
                              "median_ms": statistics.median(samples) * 1e3,
                              "min_ms": min(samples) * 1e3, "max_ms": max(samples) * 1e3}), flush=True)
        if args.mj_threads is not None:
            from simulation.mjscene_threadpool import MJSceneThreadPoolProbe
            probe = MJSceneThreadPoolProbe(scene, args.mj_threads)
            scene_probes.append(probe)
            print(json.dumps({"type": "experimental_mjscene_threadpool",
                              "worker_threads": args.mj_threads, "bound_to_live_mjscene": bool(args.mj_threads),
                              **probe.initial}), flush=True)

    runtime = SimpleNamespace(
        adapter_root=adapter, model_root=ROOT / "model/SARM/platform", scene_instance=scene_path,
        model_path=args.model_path,
        catalog=out.with_suffix(".unused"), control_host="127.0.0.1", control_port=0,
        render_host="127.0.0.1", render_port=0, duration=args.duration,
        simulation_rate=100.0, capture_rate=30.0, ik_rate=120.0, ik_mode=args.ik_mode,
        disable_attitude_control=args.disable_attitude,
        dynamics_backend=args.dynamics_backend, local_substeps=args.local_substeps,
        atmospheric_drag=args.atmospheric_drag, orbital_mode=args.orbital_mode,
    )
    with ExitStack() as stack:
        for obj, name, value in (
            (loader, "load_native_grasp_module", lambda root: native),
            (native, "_build_simulation", build),
            (native, "_initialize_state", initialize),
            (bsk_render_adapter, "BasiliskRenderBridge", OfflineBridge),
            (teleop, "SimulationControlClient", OfflineClient),
            (teleop, "CartesianTeleopTarget", TimedTarget),
        ):
            stack.enter_context(patch.object(obj, name, value))
        if args.profile:
            # Profiling ExecuteSimulation on the caller thread does NOT capture
            # SWIG/native-thread callbacks. Profile inside each Python callback.
            # Do not also wrap WheelDrive when wrapping its containing group.
            wheel_model = getattr(attitude_control, "WheelDriveGroup", attitude_control.WheelDrive)
            from simulation.local_mujoco_stepper import LocalMujocoStepper
            for cls in (wheel_model, attitude_control.InitialReference, RationalPhysicsClock,
                        AuthoritativeObservationModel, HeldJointReferencePublisher,
                        teleop.CartesianIkControlModel, OfflineBridge, LocalMujocoStepper):
                original_update = cls.UpdateState

                def profiled(self, nanos, original=original_update):
                    return profile.runcall(original, self, nanos)

                stack.enter_context(patch.object(cls, "UpdateState", profiled))
        try:
            teleop.run(runtime)
        finally:
            for probe in scene_probes:
                probe.close()
    if args.profile:
        profile.dump_stats(str(out.with_suffix(".pstats")))
        with out.with_suffix(".profile.txt").open("w", encoding="utf-8") as stream:
            pstats.Stats(profile, stream=stream).strip_dirs().sort_stats("cumulative").print_stats(60)
    for sample in timing.values():
        sample.pop("start")
    result = {
        "schema": "space-arm-runtime-benchmark/1",
        "scene_instance": str(args.scene_instance.resolve()),
        "model_path": str(native.MODEL_PATH.resolve()), "initial_state": args.initial_state,
        "duration_sim_s": args.duration, "motion": args.motion, "speed": args.speed,
        "ik_mode": args.ik_mode, "elbow_mode": args.elbow_mode,
        "wrist_mode": args.wrist_mode, "joint3_mode": args.joint3_mode,
        "disable_attitude": args.disable_attitude,
        "extra_eom_call": args.enable_extra_eom_call and not args.disable_extra_eom_call,
        "dynamics_backend": args.dynamics_backend,
        "local_dynamics": (sessions[-1].local_stepper.telemetry()
                           if sessions and getattr(sessions[-1], "local_stepper", None) else None),
        "rkf_relative_tolerance": args.rkf_relative_tol if args.dynamics_backend == "basilisk" else None,
        "rkf_absolute_tolerance": args.rkf_absolute_tol if args.dynamics_backend == "basilisk" else None,
        "profiled": args.profile, "native_timers_enabled": args.native_timers,
        "posture_backend": args.posture_backend, "mj_worker_threads": args.mj_threads,
        "mjscene_probe": {
            "samples": len(probe_samples),
            "initial": scene_probes[0].initial if scene_probes else None,
            "max_contacts": max((s["ncon"] for s in probe_samples), default=None),
            "max_constraint_islands": max((s["nisland"] for s in probe_samples), default=None),
            "max_constraint_rows": max((s["nefc"] for s in probe_samples), default=None),
        },
        "execute_wall_s": sum(execute_wall), "execute_cpu_s": sum(execute_cpu),
        "compute_real_time_factor": args.duration / sum(execute_wall),
        "outer_frame_ms_p50": float(np.percentile(execute_wall, 50)) * 1000,
        "outer_frame_ms_p95": float(np.percentile(execute_wall, 95)) * 1000,
        "outer_frame_ms_max": max(execute_wall) * 1000,
        "timers": timing, **counters,
        "excluded": ["UE rendering", "network waiting", "LeRobot/disk recording", "startup"],
        "note": "Run uninstrumented repeats for throughput; profiling/timers add overhead. Not end-to-end RTF.",
    }
    out.write_text(json.dumps(result, indent=2), encoding="utf-8")
    with out.with_suffix(".observations.jsonl").open("w", encoding="utf-8") as stream:
        for observation in observations:
            stream.write(json.dumps(observation) + "\n")
    with out.with_suffix(".frames.jsonl").open("w", encoding="utf-8") as stream:
        for frame in frames:
            stream.write(json.dumps(frame) + "\n")
    return result


def main():
    options = parser().parse_args()
    # Running this file directly puts tools/ on sys.path, not the repository root.
    sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
    # Resolve the shared defaults here so the benchmark reports the backend and
    # posture implementation it actually ran, not "None".
    from simulation.local_mujoco_stepper import dynamics_backend
    from simulation.online_elbow_ik import posture_backend
    options.dynamics_backend = dynamics_backend(options.dynamics_backend)
    options.posture_backend = posture_backend() if options.posture_backend is None else options.posture_backend
    print(json.dumps(benchmark(options), sort_keys=True))


if __name__ == "__main__":
    main()
