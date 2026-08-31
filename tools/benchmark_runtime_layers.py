"""Benchmark incremental runtime costs in the BSK/MJScene/UE pipeline.

The benchmark deliberately runs faster than wall time: no real-time pacing is
applied.  Every layer uses the same native joint trajectory so that contact
work remains comparable.  The ``ik`` and later layers execute the live
teleoperation IK workload but do not feed its result back into the trajectory.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time
from typing import Any

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
HEADER = struct.Struct("!I")


class PacketSink:
    """Small TCP receiver used to isolate adapter/network cost from UE cost."""

    def __init__(self) -> None:
        self.bytes_received = 0
        self.packets_received = 0
        self.frame_packets = 0
        self._stop = threading.Event()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(1)
        self._listener.settimeout(0.25)
        self.port = int(self._listener.getsockname()[1])
        self._thread = threading.Thread(target=self._run, name="benchmark-packet-sink", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        try:
            self._listener.close()
        except OSError:
            pass
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        connection: socket.socket | None = None
        buffer = bytearray()
        try:
            while not self._stop.is_set() and connection is None:
                try:
                    connection, _ = self._listener.accept()
                    connection.settimeout(0.25)
                except socket.timeout:
                    continue
                except OSError:
                    return
            while not self._stop.is_set() and connection is not None:
                try:
                    chunk = connection.recv(1024 * 1024)
                except socket.timeout:
                    continue
                except OSError:
                    break
                if not chunk:
                    break
                self.bytes_received += len(chunk)
                buffer.extend(chunk)
                while len(buffer) >= HEADER.size:
                    (length,) = HEADER.unpack_from(buffer)
                    packet_end = HEADER.size + length
                    if len(buffer) < packet_end:
                        break
                    payload = bytes(buffer[HEADER.size:packet_end])
                    del buffer[:packet_end]
                    self.packets_received += 1
                    if b'"type":"frame"' in payload:
                        self.frame_packets += 1
        finally:
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass


class IkWorkload:
    """Execute the production IK math while preserving the native trajectory."""

    def __init__(self, native: Any, kinematics: Any) -> None:
        self.native = native
        self.kinematics = kinematics
        self.calls = 0
        self.last_rank = 0
        self.twist = np.array([0.025, -0.018, 0.012, 0.15, -0.10, 0.08])
        self.velocity_limits = np.array([0.70, 0.70, 0.70, 0.90, 1.00])

    def reference(self, seconds: float) -> tuple[np.ndarray, np.ndarray]:
        position, velocity = self.original_reference(seconds)
        result = self.kinematics.inverse_velocity(
            position[:5], self.twist, joint_velocity_limits=self.velocity_limits
        )
        self.calls += 1
        self.last_rank = result.jacobian_rank
        return position, velocity


def build_simulation(native: Any, *, control: bool, record: bool) -> tuple[Any, Any, list[Any], list[Any]]:
    """Build the native scene with optional controller and recorder layers."""

    simulation = native.SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("benchmarkProcess")
    process.addTask(
        simulation.CreateNewTask("benchmarkTask", native.macros.sec2nano(native.TIME_STEP))
    )
    scene = native._load_scene()
    scene.ModelTag = "cubesatSO101BenchmarkScene"
    simulation.AddModelToTask("benchmarkTask", scene)
    dynamics_models: list[Any] = []
    recorders: list[Any] = []

    if control:
        trajectory = native.JointTrajectoryPublisher()
        scene.AddModelToDynamicsTask(trajectory, 9000)
        dynamics_models.append(trajectory)
        for index, ((body_name, joint_name), actuator_name) in enumerate(
            zip(native.JOINTS, native.ACTUATORS, strict=True)
        ):
            joint = scene.getBody(body_name).getScalarJoint(joint_name)
            controller = native.MJJointPIDController.JointPIDController()
            controller.ModelTag = f"{joint_name}BenchmarkPID"
            controller.setProportionalGain(float(native.KP[index]))
            controller.setDerivativeGain(float(native.KD[index]))
            controller.setIntegralGain(0.0)
            controller.desiredPosInMsg.subscribeTo(trajectory.positionOutMsgs[index])
            controller.desiredVelInMsg.subscribeTo(trajectory.velocityOutMsgs[index])
            controller.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
            controller.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)
            scene.AddModelToDynamicsTask(controller, 8000 - index)
            dynamics_models.append(controller)

            limiter = native.saturationSingleActuator.SaturationSingleActuator()
            limiter.ModelTag = f"{joint_name}BenchmarkTorqueLimiter"
            limiter.setMinInput(float(-native.TORQUE_LIMITS[index]))
            limiter.setMaxInput(float(native.TORQUE_LIMITS[index]))
            limiter.actuatorInMsg.subscribeTo(controller.outputOutMsg)
            scene.AddModelToDynamicsTask(limiter, 7000 - index)
            dynamics_models.append(limiter)
            scene.getSingleActuator(actuator_name).actuatorInMsg.subscribeTo(
                limiter.actuatorOutMsg
            )
            if record:
                recorder = limiter.actuatorOutMsg.recorder()
                simulation.AddModelToTask("benchmarkTask", recorder)
                recorders.append(recorder)

    if record:
        state_recorder = scene.stateOutMsg.recorder()
        simulation.AddModelToTask("benchmarkTask", state_recorder)
        recorders.insert(0, state_recorder)
    return simulation, scene, dynamics_models, recorders


def add_bridge(args: argparse.Namespace, native: Any, simulation: Any, scene: Any, port: int) -> Any:
    adapter_root = args.adapter_root.resolve()
    sys.path.insert(0, str(adapter_root / "Adapters"))
    from bsk_render_adapter import BasiliskRenderBridge, CameraVisual, SceneSettings

    bridge = BasiliskRenderBridge(
        host=args.render_host,
        port=port,
        origin_object="benchmark/cubesat_bus",
        frame_period_ns=native.macros.sec2nano(1.0 / 30.0),
    )
    body_ids = bridge.add_mj_scene(
        scene,
        namespace="benchmark",
        source_path=native.MODEL_PATH,
        mesh_asset_catalog=args.catalog.resolve(),
        semantic_label="spacecraft_robot_link",
        camera_picture_in_picture=True,
        camera_capture_rate_hz=args.capture_rate,
        camera_capture_products=("rgb", "depth", "segmentation"),
        camera_pip_resolution=(640, 360),
        camera_picture_in_picture_start_slot=2,
        camera_display_names={"so101_wrist_cam": "SO-101 Wrist Camera"},
    )
    bridge.add_camera(
        CameraVisual(
            camera_id="benchmark/camera/spacecraft_overview",
            display_name="Spacecraft Overview",
            parent_id=body_ids["cubesat_bus"],
            position_body_m=(0.0, -0.22, 0.34),
            orientation_body_from_camera_wxyz=(
                0.9602216126462713,
                0.0191150104507704,
                -0.0679365250286891,
                0.2701734619637677,
            ),
            field_of_view_rad=math.radians(70.0),
            resolution=(640, 360),
            semantic_label="spacecraft_overview_camera",
            picture_in_picture=True,
            capture_rate_hz=args.capture_rate,
            capture_products=("rgb", "depth", "segmentation"),
            picture_in_picture_slot=1,
        )
    )
    bridge.set_scene_settings(
        SceneSettings(
            origin_object_id="benchmark/cubesat_bus",
            default_camera_target="benchmark/so101_gripper",
            default_camera_distance_m=1.25,
            orbit_lines=False,
            trajectory_history=False,
            interpolation_delay_ms=15.0,
            max_extrapolation_ms=50.0,
        )
    )
    simulation.AddModelToTask("benchmarkTask", bridge, -10_000)
    return bridge


def process_rss_bytes() -> int | None:
    try:
        import psutil

        return int(psutil.Process(os.getpid()).memory_info().rss)
    except (ImportError, OSError):
        return None


def benchmark_ik_math(model_root: Path, repeats: int, calls: int) -> dict[str, Any]:
    """Measure the individual NumPy operations used by production IK."""

    sys.path.insert(0, str(PROJECT_ROOT))
    from simulation.serial_chain_kinematics import SerialChainKinematics

    model_path = (
        model_root.resolve()
        / "assets"
        / "cubesat_so101_grasp"
        / "cubesat_so101_grasp.xml"
    )
    kinematics = SerialChainKinematics.from_mjcf(
        model_path,
        base_body="cubesat_bus",
        joint_names=(
            "so101_shoulder_pan",
            "so101_shoulder_lift",
            "so101_elbow_flex",
            "so101_wrist_flex",
            "so101_wrist_roll",
        ),
        tool_site="so101_gripperframe",
    )
    q = np.array([0.1, -0.18, 0.22, -0.04, 0.08])
    twist = np.array([0.025, -0.018, 0.012, 0.15, -0.10, 0.08])
    limits = np.array([0.70, 0.70, 0.70, 0.90, 1.00])
    jacobian = kinematics.jacobian(q)
    weights = np.diag([1.0, 1.0, 1.0, 0.30, 0.30, 0.30])
    weighted_jacobian = weights @ jacobian
    lhs = weighted_jacobian.T @ weighted_jacobian + 0.06**2 * np.eye(5)
    rhs = weighted_jacobian.T @ (weights @ twist)
    operations = {
        "forward": lambda: kinematics.forward(q),
        "jacobian": lambda: kinematics.jacobian(q),
        "matrix_rank": lambda: np.linalg.matrix_rank(jacobian, tol=1e-5),
        "solve_5x5": lambda: np.linalg.solve(lhs, rhs),
        "inverse_velocity_full": lambda: kinematics.inverse_velocity(
            q, twist, joint_velocity_limits=limits
        ),
    }
    measurements: dict[str, Any] = {}
    for name, operation in operations.items():
        samples = []
        for _ in range(repeats):
            start = time.perf_counter()
            for _ in range(calls):
                operation()
            samples.append((time.perf_counter() - start) / calls)
        ordered = sorted(samples)
        measurements[name] = {
            "median_us_per_call": ordered[len(ordered) // 2] * 1e6,
            "best_us_per_call": ordered[0] * 1e6,
        }
    return {"repeats": repeats, "calls_per_repeat": calls, "operations": measurements}


def run(args: argparse.Namespace) -> dict[str, Any]:
    adapter_root = args.adapter_root.resolve()
    examples = adapter_root / "Unreal" / "BskUnrealRenderer" / "examples"
    sys.path[:0] = [str(adapter_root / "Adapters"), str(examples), str(PROJECT_ROOT)]
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    from simulation.serial_chain_kinematics import SerialChainKinematics

    native = load_native_grasp_module(args.model_root.resolve())
    native.TIME_STEP = args.time_step
    control = args.layer != "mjscene"
    use_legacy_ik = args.layer == "ik_legacy"
    use_scheduled_ik = args.layer in {"ik", "record", "adapter", "ue"}
    use_ik = use_legacy_ik or use_scheduled_ik
    record = args.layer in {"record", "adapter", "ue"}
    use_bridge = args.layer in {"adapter", "ue"}

    original_reference = native.JointTrajectoryPublisher.__dict__["reference"]
    ik_workload: IkWorkload | None = None
    sink: PacketSink | None = None
    bridge: Any | None = None
    setup_start = time.perf_counter()
    try:
        if use_ik:
            kinematics = SerialChainKinematics.from_mjcf(
                native.MODEL_PATH,
                base_body="cubesat_bus",
                joint_names=(
                    "so101_shoulder_pan",
                    "so101_shoulder_lift",
                    "so101_elbow_flex",
                    "so101_wrist_flex",
                    "so101_wrist_roll",
                ),
                tool_site="so101_gripperframe",
            )
            ik_workload = IkWorkload(native, kinematics)
            ik_workload.original_reference = native.JointTrajectoryPublisher.reference
            if use_legacy_ik:
                native.JointTrajectoryPublisher.reference = classmethod(
                    lambda _cls, seconds: ik_workload.reference(seconds)
                )

        simulation, scene, dynamics_models, recorders = build_simulation(
            native, control=control, record=record
        )
        if use_scheduled_ik:
            class ScheduledIkWorkloadModel(native.sysModel.SysModel):
                def __init__(self) -> None:
                    super().__init__()
                    self.ModelTag = "benchmarkScheduledIk"

                def UpdateState(self, CurrentSimNanos: int) -> None:
                    assert ik_workload is not None
                    ik_workload.reference(CurrentSimNanos * 1.0e-9)

            ik_task_name = "benchmarkIkTask"
            ik_task = simulation.CreateNewTask(
                ik_task_name, native.macros.sec2nano(1.0 / args.ik_rate)
            )
            benchmark_process = next(
                process for process in simulation.procList if process.Name == "benchmarkProcess"
            )
            benchmark_process.addTask(ik_task, 100)
            scheduled_ik_model = ScheduledIkWorkloadModel()
            simulation.AddModelToTask(ik_task_name, scheduled_ik_model)
            dynamics_models.append(scheduled_ik_model)
        if args.layer == "adapter":
            sink = PacketSink()
            sink.start()
            render_port = sink.port
        else:
            render_port = args.render_port
        if use_bridge:
            bridge = add_bridge(args, native, simulation, scene, render_port)
        native._initialize_state(simulation, scene)
        setup_seconds = time.perf_counter() - setup_start

        wall_start = time.perf_counter()
        cpu_start = time.process_time()
        frame_count = int(math.ceil(args.duration * 30.0))
        for frame in range(1, frame_count + 1):
            sim_seconds = min(frame / 30.0, args.duration)
            simulation.ConfigureStopTime(native.macros.sec2nano(sim_seconds))
            simulation.ExecuteSimulation()
        cpu_seconds = time.process_time() - cpu_start
        wall_seconds = time.perf_counter() - wall_start

        if bridge is not None:
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline:
                stats = getattr(bridge.publisher, "stats", None)
                if stats is None or stats.frames_sent >= bridge.last_published_frame_id + 1:
                    break
                time.sleep(0.01)

        publisher_stats = None
        if bridge is not None and hasattr(bridge.publisher, "stats"):
            publisher_stats = vars(bridge.publisher.stats).copy()
        sink_stats = None
        if sink is not None:
            sink_stats = {
                "bytes_received": sink.bytes_received,
                "packets_received": sink.packets_received,
                "frame_packets": sink.frame_packets,
            }
        sample_count = 0
        if recorders:
            try:
                sample_count = len(recorders[0].times())
            except (AttributeError, TypeError):
                sample_count = 0
        return {
            "layer": args.layer,
            "duration_sim_s": args.duration,
            "time_step_s": args.time_step,
            "ik_rate_hz": args.ik_rate if use_scheduled_ik else None,
            "steps": int(round(args.duration / args.time_step)),
            "setup_wall_s": setup_seconds,
            "run_wall_s": wall_seconds,
            "process_cpu_s": cpu_seconds,
            "real_time_factor": args.duration / wall_seconds,
            "wall_ms_per_step": 1000.0 * wall_seconds / (args.duration / args.time_step),
            "average_process_cpu_cores": cpu_seconds / wall_seconds,
            "rss_bytes": process_rss_bytes(),
            "ik_calls": 0 if ik_workload is None else ik_workload.calls,
            "ik_last_rank": 0 if ik_workload is None else ik_workload.last_rank,
            "recorder_samples": sample_count,
            "render_frame_id": -1 if bridge is None else bridge.last_published_frame_id,
            "publisher_stats": publisher_stats,
            "sink_stats": sink_stats,
            "kept_objects": len(dynamics_models) + len(recorders),
        }
    finally:
        native.JointTrajectoryPublisher.reference = original_reference
        if bridge is not None:
            bridge.close()
        if sink is not None:
            sink.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--layer",
        choices=("mjscene", "control", "ik_legacy", "ik", "record", "adapter", "ue"),
    )
    parser.add_argument("--ik-rate", type=float, default=100.0)
    parser.add_argument("--ik-microbenchmark", action="store_true")
    parser.add_argument("--micro-repeats", type=int, default=5)
    parser.add_argument("--micro-calls", type=int, default=1000)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--time-step", type=float, default=0.002)
    parser.add_argument("--capture-rate", type=float, default=10.0)
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=5558)
    parser.add_argument(
        "--adapter-root", type=Path, default=WORKSPACE_ROOT / "space_sim_UE_adapter"
    )
    parser.add_argument(
        "--model-root",
        type=Path,
        default=WORKSPACE_ROOT / "test" / "model" / "spacecraft_and_arm",
    )
    parser.add_argument(
        "--catalog",
        type=Path,
        default=WORKSPACE_ROOT
        / "space_sim_UE_adapter"
        / "Unreal"
        / "BskUnrealRenderer"
        / "Saved"
        / "AssetImport"
        / "cubesat_so101.catalog.json",
    )
    args = parser.parse_args()
    if args.ik_microbenchmark:
        if args.micro_repeats <= 0 or args.micro_calls <= 0:
            parser.error("micro-repeats and micro-calls must be positive")
        print(
            json.dumps(
                benchmark_ik_math(args.model_root, args.micro_repeats, args.micro_calls),
                ensure_ascii=False,
                sort_keys=True,
            )
        )
        return 0
    if args.layer is None:
        parser.error("--layer is required unless --ik-microbenchmark is used")
    if (
        args.duration <= 0.0
        or args.time_step <= 0.0
        or args.capture_rate <= 0.0
        or args.ik_rate <= 0.0
        or args.ik_rate > 500.0
    ):
        parser.error(
            "duration, time-step and capture-rate must be positive; "
            "ik-rate must be in (0, 500]"
        )
    result = run(args)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
