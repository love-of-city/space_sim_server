"""Offline native control-chain experiments; never connect to a live platform."""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


ROOT = Path(__file__).resolve().parents[1]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adapter", type=Path, default=ROOT.parent / "space_sim_UE_Adapter")
    parser.add_argument("--instance", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--motion", choices=["hold", "linear_x", "linear_y", "linear_z", "roll", "pitch", "yaw"], required=True)
    parser.add_argument("--speed", type=float, default=0.0)
    parser.add_argument("--start", type=float, default=1.0)
    parser.add_argument("--stop", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--timestep", type=float, default=0.001)
    parser.add_argument("--reference-mode", choices=["protected", "unprotected"], default="protected")
    parser.add_argument("--reverse-at", type=float)
    parser.add_argument("--pause-from", type=float)
    parser.add_argument("--pause-until", type=float)
    parser.add_argument("--disable-reference-recovery", action="store_true")
    parser.add_argument("--probe-rate", type=float, help="Optional sparse probe Hz for timing, not peak-motion validation")
    parser.add_argument("--wrist-kd-scale", type=float, default=1.0)
    parser.add_argument("--wrist-kp-scale", type=float, default=1.0)
    parser.add_argument("--arm-kp-scale", type=float, default=1.0)
    parser.add_argument("--torque-scale", type=float, default=1.0)
    parser.add_argument("--angular-acceleration", type=float, default=2.0)
    parser.add_argument("--initial", choices=["instance", "home", "sampled"], default="instance")
    parser.add_argument("--sampled-state", type=Path)
    parser.add_argument("--replay-reference", action="store_true", help="Restore sampled arm reference as well as position; not a full dynamic-state replay")
    options = parser.parse_args()
    if options.replay_reference and options.initial != "sampled":
        parser.error("replay-reference requires initial=sampled")
    if not 0 < options.start < options.stop < options.duration:
        parser.error("Require 0 < start < stop < duration to sample all three phases")
    if options.reverse_at is not None and not options.start < options.reverse_at < options.stop:
        parser.error("reverse-at must lie within the moving phase")
    if (options.pause_from is None) != (options.pause_until is None):
        parser.error("pause-from and pause-until must be supplied together")
    if options.pause_from is not None and not options.start < options.pause_from < options.pause_until < options.stop:
        parser.error("pause interval must lie within the moving phase")
    if not np.isfinite(options.timestep) or not 0 < options.timestep <= 0.002:
        parser.error("Require a finite timestep in (0, 0.002]")
    if options.probe_rate is not None and (not np.isfinite(options.probe_rate) or options.probe_rate <= 0):
        parser.error("probe-rate must be finite and positive")
    sample_interval = max(options.timestep, 1.0 / options.probe_rate) if options.probe_rate else options.timestep
    if min(options.start, options.stop - options.start, options.duration - options.stop) < 2 * sample_interval:
        parser.error("Each phase must contain at least two dynamics intervals")
    options.output.mkdir(parents=True, exist_ok=True)
    for entry in [ROOT, ROOT / "backend", options.adapter / "Adapters", options.adapter / "Unreal/BskUnrealRenderer/examples"]:
        sys.path.insert(0, str(entry))
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(ROOT / "model/SARM/platform")
    import scenario_spacecraft_arm_grasp_unreal as loader
    from Basilisk.architecture import sysModel
    import bsk_render_adapter
    from bsk_render_adapter.protocol import RecordingOnlyPublisher
    from simulation import teleop_grasp_unreal as teleop
    from simulation.reference_governor import GovernedReference
    from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME

    loader.load_native_grasp_module = lambda _model: native
    teleop.MAX_ANGULAR_COMMAND_ACCELERATION_RAD_S2 = options.angular_acceleration
    instance = json.loads(options.instance.read_text(encoding="utf-8-sig"))
    if options.initial == "home":
        instance["randomization"]["arm_joint_position_rad"] = list(BALANCED_TELEOP_HOME)
    elif options.initial == "sampled":
        if options.sampled_state is None:
            parser.error("--sampled-state is required with --initial sampled")
        samples = json.loads(options.sampled_state.read_text(encoding="utf-8-sig"))
        instance["randomization"]["arm_joint_position_rad"] = samples[0]["observation"]["joint_position_rad"]
    instance_path = options.output / "instance.json"
    instance_path.write_text(json.dumps(instance, indent=2), encoding="utf-8")
    sampled_reference = None
    if options.replay_reference:
        sampled_reference = np.asarray(samples[0]["observation"]["target_joint_position_rad"], dtype=float)
        if sampled_reference.shape != (8,) or not np.all(np.isfinite(sampled_reference)):
            parser.error("sampled reference must contain eight finite joint positions")
    observations = []
    snapshots = []
    targets = []
    graphs = []
    clock = {"seconds": 0.0}
    original_target = teleop.CartesianTeleopTarget

    class UnprotectedGovernor:
        def reset(self):
            pass

        def apply(self, _reference, _measured, velocity, _dt, *, enabled, **_kwargs):
            return GovernedReference(velocity if enabled else np.zeros(6), float(enabled), "unprotected", ())

    class TracedTarget(original_target):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            if options.reference_mode == "unprotected":
                self.governor = UnprotectedGovernor()
            targets.append(self)

        def update(self, seconds):
            clock["seconds"] = seconds
            return super().update(seconds)

        def reset(self, seconds):
            super().reset(seconds)
            if sampled_reference is not None:
                self.position = sampled_reference.copy()
                self.target_tool_position, self.target_tool_rotation = self.kinematics.forward(self.position[:6])

    class OfflineClient(teleop.SimulationControlClient):
        def start(self):
            pass

        def close(self):
            pass

        def latest_action(self):
            active = options.motion != "hold" and options.start <= clock["seconds"] < options.stop
            if options.pause_from is not None and options.pause_from <= clock["seconds"] < options.pause_until:
                active = False
            linear = [0.0, 0.0, 0.0]
            angular = [0.0, 0.0, 0.0]
            speed = -options.speed if options.reverse_at is not None and clock["seconds"] >= options.reverse_at else options.speed
            if options.motion.startswith("linear_"):
                linear[["linear_x", "linear_y", "linear_z"].index(options.motion)] = speed if active else 0.0
            elif options.motion != "hold":
                angular[["roll", "pitch", "yaw"].index(options.motion)] = speed if active else 0.0
            return dict(deadman=active, server_sequence="1",
                        allow_reference_recovery=not active and not options.disable_reference_recovery,
                        end_effector_linear_velocity_body_m_s=linear,
                        end_effector_angular_velocity_body_rad_s=angular, gripper_velocity_m_s=0.0), False

        def send_observation(self, message):
            observations.append(copy.deepcopy(message))
            return True

    class OfflineBridge(bsk_render_adapter.BasiliskRenderBridge):
        def __init__(self, **kwargs):
            super().__init__(**kwargs, publisher=RecordingOnlyPublisher())

        def add_mj_scene(self, scene, **kwargs):
            kwargs["mesh_asset_catalog"] = None
            return super().add_mj_scene(scene, **kwargs)

    original_build = native._build_simulation

    class DynamicsProbe(sysModel.SysModel):
        def __init__(self, scene, models):
            super().__init__()
            self.ModelTag = "offlineControlDiagnostics"
            self.scene = scene
            self.next_sample_ns = 0
            self.joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]
            self.controllers = [model for model in models if model.ModelTag.endswith("PID")]
            self.limiters = [model for model in models if model.ModelTag.endswith("TorqueLimiter")]

        def UpdateState(self, nanos):
            if nanos < self.next_sample_ns:
                return
            self.next_sample_ns = nanos + round(sample_interval * 1e9)
            state = self.scene.stateOutMsg.read()
            target = targets[-1]
            snapshots.append(dict(time=nanos * 1e-9,
                                  actual=[joint.stateOutMsg.read().state for joint in self.joints],
                                  velocity=[joint.stateDotOutMsg.read().state for joint in self.joints],
                                  target=target.position.copy(), target_velocity=target.velocity.copy(),
                                  raw_torque=[controller.outputOutMsg.read().input for controller in self.controllers],
                                  torque=[limiter.actuatorOutMsg.read().input for limiter in self.limiters],
                                  qpos=list(state.qpos), qvel=list(state.qvel),
                                  tracking_scale=target.tracking_scale, governor_state=target.governor_state,
                                  velocity_scale=target.velocity_scale, minimum_singular_value=target.minimum_singular_value))

    def build(**kwargs):
        native.TIME_STEP = options.timestep
        native.KP[:6] *= options.arm_kp_scale
        native.KD[3:6] *= options.wrist_kd_scale
        native.KP[3:6] *= options.wrist_kp_scale
        native.TORQUE_LIMITS[:6] *= options.torque_scale
        graph = original_build(**kwargs)
        simulation, scene, models, recorders = graph
        probe = DynamicsProbe(scene, models)
        simulation.AddModelToTask("graspTask", probe, -100)
        models.append(probe)
        graphs.append(graph)
        return graph

    native._build_simulation = build
    teleop.CartesianTeleopTarget = TracedTarget
    teleop.SimulationControlClient = OfflineClient
    bsk_render_adapter.BasiliskRenderBridge = OfflineBridge
    teleop.time.sleep = lambda _duration: None
    args = SimpleNamespace(adapter_root=options.adapter, model_root=ROOT / "model/SARM/platform",
                           scene_instance=instance_path, catalog=options.output / "unused.json",
                           control_host="127.0.0.1", control_port=0, render_host="127.0.0.1", render_port=0,
                           duration=options.duration, simulation_rate=1.0, capture_rate=10.0, ik_rate=100.0,
                           disable_attitude_control=False, ik_mode=teleop.IK_MODE_IK_POSE)
    teleop.run(args)
    arrays = {key: np.asarray([sample[key] for sample in snapshots]) for key in snapshots[0]}
    np.savez_compressed(options.output / "dynamics.npz", **arrays)
    (options.output / "observations.json").write_text(json.dumps(observations, indent=2), encoding="utf-8")
    times = arrays["time"]
    summary = {"motion": options.motion, "speed": options.speed, "timestep": options.timestep,
               "samples": len(times), "kp": native.KP.tolist(), "kd": native.KD.tolist(),
               "torque_limits": native.TORQUE_LIMITS.tolist(), "initial": options.initial,
               "reference_mode": options.reference_mode, "reverse_at": options.reverse_at,
               "pause_from": options.pause_from, "pause_until": options.pause_until,
               "reference_recovery_enabled": not options.disable_reference_recovery,
               "sampled_reference_restored": options.replay_reference,
               "probe_rate_hz": options.probe_rate,
               "scope": "Native production dynamics, orbital gravity and IK; offline input, no network or UE; initial-state replica"}
    for phase, mask in [("hold", times < options.start), ("moving", (times >= options.start) & (times < options.stop)),
                        ("released", times >= options.stop)]:
        actual = arrays["actual"][mask]
        velocity = arrays["velocity"][mask]
        target = arrays["target"][mask]
        target_velocity = arrays["target_velocity"][mask]
        torque = arrays["torque"][mask]
        error = target - actual
        summary[phase] = {
            "actual_velocity_peak": np.max(np.abs(velocity), axis=0).tolist(),
            "reference_velocity_peak": np.max(np.abs(target_velocity), axis=0).tolist(),
            "joint_error_peak": np.max(np.abs(error), axis=0).tolist(),
            "saturation_fraction": np.mean(np.abs(torque) >= native.TORQUE_LIMITS * .999, axis=0).tolist(),
            "raw_torque_peak": np.max(np.abs(arrays["raw_torque"][mask]), axis=0).tolist(),
            "target_velocity_jump_peak": np.max(np.abs(np.diff(target_velocity, axis=0)), axis=0).tolist(),
            "actual_velocity_jump_peak": np.max(np.abs(np.diff(velocity, axis=0)), axis=0).tolist(),
            "joint_travel": (np.max(actual, axis=0) - np.min(actual, axis=0)).tolist(),
            "zero_scale_fraction": float(np.mean(arrays["velocity_scale"][mask] < 1e-6)),
            "tracking_limited_fraction": float(np.mean(arrays["tracking_scale"][mask] < 1.0 - 1e-6)),
            "minimum_singular_value": float(np.min(arrays["minimum_singular_value"][mask])),
        }
    summary["position_error_peak_m"] = max(sample["position_tracking_error_m"] for sample in observations)
    summary["orientation_error_peak_rad"] = max(sample["orientation_tracking_error_rad"] for sample in observations)
    summary["final_position_error_m"] = observations[-1]["position_tracking_error_m"]
    summary["final_orientation_error_rad"] = observations[-1]["orientation_tracking_error_rad"]
    (options.output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == "__main__":
    main()
