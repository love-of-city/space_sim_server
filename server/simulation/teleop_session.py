"""Authoritative teleoperation startup, runtime assembly and session lifecycle."""
from __future__ import annotations
import argparse
from dataclasses import asdict
import gc
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any
import numpy as np
from simulation.serial_chain_kinematics import SerialChainKinematics, matrix_to_quaternion_wxyz
from space_arm_platform.protocol import CONTROL_PROTOCOL
from simulation.native_integration import configure_scene_integrator
from space_arm_platform.sampling import DYNAMICS_HZ, RENDER_HZ, LIVE_CAPTURE_FPS, ik_step_stride, tick_time_ns
from space_arm_platform.lighting import DEFAULT_SUNLIGHT_INTENSITY_SCALE
from space_arm_platform.scene_targets import DEFAULT_TEMPLATE, capture_target
from space_arm_platform.orbital_physics import OrbitalPhysics
from simulation.orbital_configuration import resolve_orbital_physics, environment_metadata
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME, DEFAULT_OPERATING_JOINT_DEG
from space_arm_platform.joint_limits import load_joint_limits
from simulation.online_elbow_ik import posture_backend
from space_arm_platform.control_defaults import DEFAULT_DYNAMICS_STEP_S
from simulation.reference_governor import JointReferenceGovernor
from simulation.reference_recovery import ReferenceRecoverySettings
from simulation.runtime_progress import runtime_stage
from simulation.fastener_demo_schedule import demo_command
from simulation.runtime_performance import RuntimePerformanceMonitor, render_transport_status
from simulation.joint_reference_publisher import HeldJointReferencePublisher
from simulation.local_mujoco_stepper import BACKEND_BASILISK, BACKEND_LOCAL, dynamics_backend, local_substeps
from simulation.control_client import SimulationControlClient
from simulation.policy_control import DrawerPolicyState, JointPolicyController, POLICY_PERIOD_NS
from simulation.teleop_control import (
    ARM_JOINT_NAMES, CartesianTeleopTarget, TELEOP_ARM_KD, TELEOP_ARM_KP, TELEOP_ARM_TORQUE_LIMIT
)
from simulation.teleop_scene import (
    CELESTIAL_FIXED_FRAMES, DEFAULT_EPHEMERIS_EPOCH_UTC, DEFAULT_ORBIT, SUPPORTED_EPHEMERIS_CENTER, SUPPORTED_EPHEMERIS_FRAME, apply_orbital_initial_state, load_scene_instance, validate_dynamics_step
)



def run(args: argparse.Namespace) -> None:
    scene_instance = load_scene_instance(args.scene_instance, args.model_root)
    if scene_instance:
        runtime = scene_instance["runtime"]
        args.simulation_rate = float(runtime["simulation_rate"])
        args.capture_rate = float(runtime["capture_rate_hz"])
        args.ik_rate = float(runtime["ik_rate_hz"])
    requested_step = getattr(args, "dynamics_step", None)
    if requested_step is None:
        requested_step = scene_instance["runtime"].get("dynamics_step_s", DEFAULT_DYNAMICS_STEP_S) if scene_instance else DEFAULT_DYNAMICS_STEP_S
    args.dynamics_step = validate_dynamics_step(requested_step)
    if (scene_instance is None or scene_instance.get("runtime", {}).get("dataset_capture", True)) and args.capture_rate not in LIVE_CAPTURE_FPS:
        raise ValueError("LeRobot capture rate must align with the 400 Hz dynamics / 50 Hz render grid: 1, 2, 5, 10, 25, 50 Hz")
    adapter_root = args.adapter_root.resolve()
    ue_examples = adapter_root / "Unreal" / "BskUnrealRenderer" / "examples"
    sys.path.insert(0, str(adapter_root / "Adapters"))
    sys.path.insert(0, str(ue_examples))

    # This wrapper establishes the Windows-safe Basilisk/MuJoCo DLL load order.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    native = load_native_grasp_module(args.model_root.resolve())
    template_id = scene_instance["template_id"] if scene_instance else DEFAULT_TEMPLATE
    target_spec = capture_target(template_id)
    model_override = getattr(args, "model_path", None)
    native.MODEL_PATH = (
        Path(model_override).resolve()
        if model_override is not None
        else target_spec.resolve_model(args.model_root)
    )
    if not native.MODEL_PATH.is_file():
        raise FileNotFoundError(f"Selected target scene is missing: {native.MODEL_PATH}")
    print(json.dumps({"type": "capture_target_configuration", "template_id": template_id,
                      "runtime_model": target_spec.runtime_model, "collision_model": target_spec.collision_model,
                      "runtime_warning": target_spec.runtime_warning,
                      "model_path": str(native.MODEL_PATH),
                      "model_override": model_override is not None}, ensure_ascii=True), flush=True)
    native.TARGET_POS = np.asarray(target_spec.position_m, dtype=float)
    native.TARGET_QUAT = np.asarray(target_spec.orientation_wxyz, dtype=float)
    ik_step_stride(args.ik_rate)  # Reject stale 100 Hz scene configurations.
    native.TIME_STEP = 1.0 / DYNAMICS_HZ  # Initial period; RationalPhysicsClock drives every actual step.
    native.KP = np.asarray(native.KP, dtype=float).copy()
    native.KD = np.asarray(native.KD, dtype=float).copy()
    native.TORQUE_LIMITS = np.asarray(native.TORQUE_LIMITS, dtype=float).copy()
    native.KP[:6] = TELEOP_ARM_KP
    native.KD[:6] = TELEOP_ARM_KD
    native.TORQUE_LIMITS[:6] = TELEOP_ARM_TORQUE_LIMIT
    from simulation.sarm_model_profile import load_model_profile
    native.model_profile = load_model_profile(native.MODEL_PATH)
    native.model_sha256 = hashlib.sha256(native.MODEL_PATH.read_bytes()).hexdigest()
    if native.model_profile.linkage:
        native.JOINTS = native.model_profile.joints
        native.ACTUATORS = native.model_profile.arm_actuators
        native.KP = np.full(6, 500.)
        native.KD = np.full(6, 50.)
        native.TORQUE_LIMITS = np.full(6, 100.)
    selected_posture_backend = posture_backend()
    if selected_posture_backend == "native":
        from simulation.native_posture import available
        if not available():
            raise RuntimeError("Native posture backend was requested but is not built; run tools/assets/build_native_acceleration.py.")
    args.dynamics_backend = dynamics_backend(getattr(args, "dynamics_backend", None))
    if native.model_profile.linkage and args.dynamics_backend != BACKEND_LOCAL:
        raise ValueError('SARM 2026-10-12 requires the local implicitfast backend for its tendon and constraints')
    from simulation.orbital_environment import orbital_mode
    args.orbital_mode = orbital_mode(getattr(args, "orbital_mode", None))
    orbital_physics = resolve_orbital_physics(
        scene_instance, override=getattr(args, "orbital_config", None),
        backend=args.dynamics_backend, mode=args.orbital_mode,
        require_drag=getattr(args, "atmospheric_drag", False))
    args.local_substeps = (local_substeps(getattr(args, "local_substeps", None))
                           if args.dynamics_backend == BACKEND_LOCAL else None)
    print(json.dumps({"type": "teleop_control_configuration", "dynamics_step_s": native.TIME_STEP,
                      "dynamics_backend": args.dynamics_backend,
                      "integrator": ("RKF45" if args.dynamics_backend == BACKEND_BASILISK
                                     else "mujoco_implicitfast_fixed_step"),
                      "local_substeps": args.local_substeps, "ik_rate_hz": args.ik_rate,
                      "orbital_mode": args.orbital_mode if args.dynamics_backend == BACKEND_LOCAL else "basilisk",
                      "online_posture_backend": selected_posture_backend,
                      "arm_torque_limits_nm": native.TORQUE_LIMITS[:6].tolist(),
                      "reference_governor": "bounded_reference_recovery_v2",
                      "reference_governor_settings": asdict(JointReferenceGovernor().settings),
                      "reference_recovery_settings": asdict(ReferenceRecoverySettings())}), flush=True)

    client = SimulationControlClient(args.control_host, args.control_port, "sarm-teleop", policy_runtime=True)
    kinematics = SerialChainKinematics.from_mjcf(
        native.MODEL_PATH,
        base_body="cubesat_bus",
        joint_names=ARM_JOINT_NAMES,
        tool_site="sarm_ee",
    )
    if selected_posture_backend == "native":
        kinematics.enable_native_geometry()
    initial_joints = (
        np.asarray(scene_instance["randomization"]["arm_joint_position_rad"], dtype=float)
        if scene_instance
        else np.asarray(BALANCED_TELEOP_HOME, dtype=float)
    )
    if scene_instance is None:
        initial_joints = np.asarray([
            *np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0.01875, 0.01875
        ], dtype=float)
        posture_report = {"status": "selected", "reason": "direct_operating_pose_startup"}
    else:
        posture_report = scene_instance.get("ik_initialization", {
            "status": "skipped", "reason": "saved_initial_state",
        })
    if native.model_profile.linkage:
        initial_joints[6:] = 0.
    print(json.dumps({"type": "ik_initialization", **posture_report}, ensure_ascii=True), flush=True)
    client.start()
    try:
        with runtime_stage("check_backend_protocol"):
            client.wait_until_ready()
        while True:
            reset_request = _run_session(
                args, native, scene_instance, target_spec, initial_joints, kinematics, client,
                orbital_physics=orbital_physics,
            )
            if reset_request is None:
                break
            # Dispose of the old native graph before creating a fresh one.
            # UE/Pixel Streaming and the backend control connection stay alive.
            with runtime_stage("release_previous_session"):
                gc.collect()
    finally:
        client.close()

def _run_session(
    args: argparse.Namespace, native: Any, scene_instance: dict[str, Any] | None,
    target_spec: Any, initial_joints: np.ndarray, kinematics: SerialChainKinematics,
    client: SimulationControlClient,
    *, orbital_physics: OrbitalPhysics,
) -> str | None:
    """Build one clean physics/controller/ephemeris/render session from the saved initial conditions."""
    template_id = scene_instance["template_id"] if scene_instance else DEFAULT_TEMPLATE
    targets = CartesianTeleopTarget(
        initial_joints, client, kinematics, ik_mode=args.ik_mode,
        joint_limits=load_joint_limits(native.MODEL_PATH, native.model_profile.joint_names),
        preparation_required=scene_instance.get("arm_preparation_required", False) if scene_instance else False,
        operating_joint_deg=scene_instance.get("operating_arm_joint_position_deg", DEFAULT_OPERATING_JOINT_DEG) if scene_instance else DEFAULT_OPERATING_JOINT_DEG,
        preparation_plan=scene_instance.get("arm_preparation_plan") if scene_instance else None,
    )
    # Display-only metadata, prepared once from the exact limits used by this session.
    arm_joint_limits_rad = [
        [float(lo), float(hi)] if np.isfinite(lo) and np.isfinite(hi) else [None, None]
        for lo, hi in zip(targets.joint_min[:6], targets.joint_max[:6])
    ]
    reference_publisher = HeldJointReferencePublisher(targets.cached_reference, len(native.JOINTS))

    from Basilisk.utilities import macros, simIncludeGravBody

    from simulation.assembly import AssemblyContext, SessionConfig
    from simulation.components.arm_reference import JointReferenceComponent
    from simulation.components.attitude import AttitudeComponent
    from simulation.components.clock import ClockComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.observation import ObservationComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.components.render import RenderComponent
    from simulation.components.teleop_ik import TeleopIkComponent
    from simulation.physics_ports import PhysicsPorts

    bridge: Any = None  # BasiliskRenderBridge, created by RenderComponent
    simulation = None
    gravity_factory = None
    orbit_origin = None
    assembly: AssemblyContext | None = None
    policy: JointPolicyController | None = None
    try:
        with runtime_stage("build_physics"):
            simulation, scene, dynamics_models, recorders = native._build_simulation(
                external_reference=reference_publisher,
                record_history=False,
                dynamics_backend=args.dynamics_backend,
                local_substeps=args.local_substeps,
                # The AttitudeComponent installs the chain exactly once, below.
                # Letting the builder create it too would schedule the whole
                # 100 Hz chain twice, with one copy spinning without effect.
                defer_attitude_control=True,
            )
        local_backend = args.dynamics_backend == BACKEND_LOCAL
        if local_backend:
            integration = {"backend": BACKEND_LOCAL, "integrator": "mujoco_implicitfast_fixed_step",
                           "substeps": simulation.local_stepper.substeps,
                           "note": "MJScene publishes only; MuJoCo servos replace the PID/limiter chain"}
        else:
            integration = configure_scene_integrator(scene)
        print(json.dumps({"type": "native_integration_configuration", **integration,
                          "dynamics_rate_hz": DYNAMICS_HZ}, sort_keys=True), flush=True)
        ephemeris_environment = scene_instance.get("environment", {}) if scene_instance else {}
        ephemeris_epoch = str(
            ephemeris_environment.get("ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC)
        )
        dataset_capture = bool(scene_instance is None or scene_instance.get("runtime", {}).get("dataset_capture", True))
        if not local_backend:
            # MJScene already performs the final forward-kinematics pass after an
            # adaptive integration step. A second full equations-of-motion pass
            # repeats gravity, controllers and contact work; forces are recomputed
            # at the next integration stage. Keep it opt-in for diagnostics.
            extra_eom_call = os.environ.get("SPACE_SIM_EXTRA_EOM_CALL", "0").strip().lower() not in {"0", "false", "off", "no"}
            scene.extraEoMCall = extra_eom_call
            print(json.dumps({"type": "native_extra_eom_configuration", "enabled": extra_eom_call},
                             sort_keys=True), flush=True)

        # One assembly pass describes the whole session: components create modules,
        # wire them through the physics ports, and place them in a slot. Nothing
        # below picks a task priority or reaches into the scene by hand.
        ports = PhysicsPorts(scene, stepper=getattr(simulation, "local_stepper", None))
        assembly = AssemblyContext(
            simulation, scene, config=SessionConfig(
                model_path=native.MODEL_PATH, dynamics_backend=args.dynamics_backend,
                local_substeps=args.local_substeps,
                publish_stride=(simulation.local_stepper.publish_every if local_backend
                                else getattr(scene, "publishStride", 1)),
                ik_rate_hz=args.ik_rate, render_rate_hz=RENDER_HZ),
            physics_ports=ports)
        assembly.extra.update({
            "targets": targets, "kinematics": kinematics, "client": client, "args": args,
            "native": native, "scene_instance": scene_instance, "target_spec": target_spec,
            "template_id": template_id, "already_installed": [reference_publisher],
        })
        orbit_component = OrbitComponent(epoch_utc=ephemeris_epoch, physics=orbital_physics)
        render = RenderComponent(
            model_path=native.MODEL_PATH,
            catalog=args.catalog.resolve(),
            host=args.render_host,
            port=args.render_port,
            render_rate_hz=RENDER_HZ,
            capture_rate_hz=args.capture_rate,
            capture_state_provider=(client.capture_state if dataset_capture else
                                    lambda: {"capture_episode_id": "", "capture_request_id": ""}),
            sunlight_intensity_scale=ephemeris_environment.get("lighting", {}).get(
                "sunlight_intensity_scale", DEFAULT_SUNLIGHT_INTENSITY_SCALE),
            wrist_camera_mode=(scene_instance or {}).get("wrist_camera_mode", "rgb"),
            overview_camera=(scene_instance or {}).get("overview_camera"),
            camera_display_names={
                "spacecraft_overview": "Spacecraft Overview",
                "sarm_wrist_cam": "SARM Wrist Camera",
            })
        assembly.install([
            ClockComponent(),
            orbit_component,
            DynamicsCoreComponent(native.MODEL_PATH, substeps=args.local_substeps),
            OrbitalGravityComponent(mode=getattr(args, "orbital_mode", None)),
            TeleopIkComponent(targets, ik_rate_hz=args.ik_rate),
            # The scenario builder already created the reference publisher and wired
            # the local servos from it, so this component only schedules it (and, on
            # the basilisk path, declares the PID/limiter chain's consumer).
            JointReferenceComponent(
                reference_publisher, joints=tuple(j for _, j in native.JOINTS),
                actuators=tuple(native.ACTUATORS), kp=native.KP, kd=native.KD,
                limits=native.TORQUE_LIMITS, ik_rate_hz=args.ik_rate,
                wire_servos=not local_backend),
            AttitudeComponent(
                native.MODEL_PATH,
                enabled=False if getattr(args, "disable_attitude_control", False) else None),
            render,
        ])
        from simulation.components.plug_retention import install_configured_plug_retention
        plug_retention = install_configured_plug_retention(assembly)
        retention_configuration = (plug_retention.configuration() if plug_retention else {"enabled": False})
        if template_id == 'sarm-task-box-20261012-demo':
            from simulation.components.fastener_demo import FastenerDemoComponent
            assembly.install([FastenerDemoComponent()])
        drag_components = []
        if orbital_physics.drag is not None:
            from simulation.components.drag import configured_drag_components

            drag_components = configured_drag_components(
                orbital_physics.drag, planet_radius_m=float(orbit_component.earth.radEquator))
            assembly.install(drag_components)
        gravity_factory, earth, sun = orbit_component.gravity_factory, orbit_component.earth, orbit_component.sun
        orbital_metadata = environment_metadata(orbital_physics, earth, epoch_utc=ephemeris_epoch)
        print(json.dumps({"type": "orbital_physics_configuration", **orbital_metadata}, sort_keys=True), flush=True)
        bridge = render.bridge
        ephemeris = orbit_component.ephemeris
        orbit_origin = orbit_component.origin
        gravity_target_names = orbit_component.gravity_targets
        keep_alive = tuple(assembly._kept) + (dynamics_models, recorders)
        print(json.dumps({"type": "ephemeris_configuration", "epoch_utc": ephemeris_epoch,
                          "center": SUPPORTED_EPHEMERIS_CENTER, "frame": SUPPORTED_EPHEMERIS_FRAME,
                          "planet_fixed_frames": dict(zip(("earth", "sun"), CELESTIAL_FIXED_FRAMES)),
                          "gravity_sources": ["earth", "sun"],
                          "gravity_targets": gravity_target_names}, sort_keys=True), flush=True)
        print(json.dumps({"type": "lighting_configuration",
                          "sunlight_intensity_scale": render.sunlight_intensity_scale,
                          "scope": "rendering_only"}, sort_keys=True), flush=True)
        _ = simIncludeGravBody
        joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]

        # Cache message handles, never payloads: every read still samples the
        # authoritative native state at its original scheduled timestamp.
        position_readers = [joint.stateOutMsg.addSubscriber() for joint in joints]
        velocity_readers = [joint.stateDotOutMsg.addSubscriber() for joint in joints]
        hinge_reader = (
            scene.getBody(target_spec.hinge_body).getScalarJoint(target_spec.hinge_joint)
            .stateOutMsg.addSubscriber() if target_spec.hinge_joint else None
        )

        from simulation.wrench_sensors import WRIST_SENSOR

        from simulation.wrench_sampling import install_wrench_sampler
        from space_arm_platform.sampling import WRENCH_HZ
        wrench_sampler = (install_wrench_sampler(assembly)
                          if local_backend and native.model_profile.linkage else None)

        drawer_policy_state = None
        if local_backend and native.model_profile.linkage:
            try:
                drawer_policy_state = DrawerPolicyState(scene, simulation.local_stepper, model_path=native.MODEL_PATH)
            except (ValueError, RuntimeError, KeyError):
                pass  # Scenes without the physical drawer remain usable for joint policy.

        def publish_policy_reference(now_ns: int) -> None:
            # A policy interval starts at a render boundary, which need not also
            # be an IK tick. Write the held message before any of its substeps.
            stride = reference_publisher.stride
            try:
                reference_publisher.stride = 1
                reference_publisher.UpdateState(now_ns)
            finally:
                reference_publisher.stride = stride

        policy = JointPolicyController(
            client.policy, targets,
            supported=(local_backend and native.model_profile.linkage
                       and 'fastener_demo' not in assembly.extra
                       and not getattr(args, "scripted_task", None)),
            discard_teleop=client.discard_teleop, capture_state=client.capture_state,
            publish_reference=publish_policy_reference,
            available_tasks=("drawer_cycle",) if drawer_policy_state is not None else (),
            reset_task=drawer_policy_state.reset if drawer_policy_state is not None else None,
        )

        def snapshot_observation(render_frame_id: int, render_sim_time_ns: int) -> dict[str, Any]:
            joint_position = [float(read().state) for read in position_readers]
            joint_velocity = [float(read().state) for read in velocity_readers]
            end_effector_position, end_effector_rotation = kinematics.forward(
                np.asarray(joint_position[:6])
            )
            end_effector_twist = kinematics.jacobian(
                np.asarray(joint_position[:6])
            ) @ np.asarray(joint_velocity[:6])
            return (
                {
                    "protocol": CONTROL_PROTOCOL,
                    "type": "observation",
                    "simulation_id": "sarm-teleop",
                    "reset_generation": client.reset_generation,
                    "policy_execution": policy.execution_metadata(),
                    "policy_task_state": (drawer_policy_state.snapshot(
                        joint_position, end_effector_position.tolist(), sim_time_ns=render_sim_time_ns,
                        joint_velocities=joint_velocity, ee_rotation=end_effector_rotation)
                                          if drawer_policy_state is not None else {"task_name": "", "success_available": False, "success": False}),
                    "render_session_id": bridge.session_id,
                    **bridge.last_capture_state,
                    "scene_instance_id": scene_instance.get("instance_id") if scene_instance else None,
                    "scene_seed": scene_instance.get("seed") if scene_instance else None,
                    # The stepper's view includes the actual force cadence it applied.
                    "orbital_dynamics": (
                        simulation.local_stepper.telemetry()["orbital_environment"]
                        if local_backend else {"mode": "basilisk"}),
                    "orbital_physics": orbital_metadata,
                    "plug_retention": {
                        **retention_configuration,
                        "bodies": plug_retention.telemetry() if plug_retention else {},
                    },
                    "atmospheric_drag": [component.telemetry() for component in drag_components],
                    "wrist_wrench": (
                        WRIST_SENSOR.observation(simulation.local_stepper, render_sim_time_ns)
                        if local_backend and native.model_profile.linkage else None),
                    "wrist_wrench_rate_hz": WRENCH_HZ if wrench_sampler else None,
                    "wrist_wrench_samples": (wrench_sampler.drain(render_sim_time_ns, args.capture_rate)
                                             if wrench_sampler else []),
                    "capture_target": {
                        "model_file": target_spec.model_file,
                        "runtime_model": target_spec.runtime_model,
                        "collision_model": target_spec.collision_model,
                        "runtime_warning": target_spec.runtime_warning,
                        "synthetic_mass_kg": target_spec.synthetic_mass_kg,
                        "hinge_position_rad": float(hinge_reader().state) if hinge_reader is not None else None,
                    },
                    "scene_template_id": template_id,
                    "step_id": str(render_frame_id + 1),
                    "observation_source": "authoritative_render_snapshot",
                    "render_frame_id": str(render_frame_id),
                    # 使用权威渲染帧的离散时间，保证状态和相机产品可逐帧严格配对。
                    "sim_time_ns": str(render_sim_time_ns),
                    "wall_time_ns": str(time.time_ns()),
                    "applied_action_sequence": targets.applied_sequence,
                    # Legacy aliases retain 8 mixed-unit entries for older clients.
                    # Explicit SI fields are authoritative for new consumers.
                    "arm_joint_position_rad": joint_position[:6],
                    "arm_joint_velocity_rad_s": joint_velocity[:6],
                    "target_arm_joint_position_rad": targets.position[:6].tolist(),
                    "arm_preparation": targets.preparation.telemetry(),
                    **({"model_revision": "2026-10-12", "gripper_type": "four_bar_tendon",
                        "model_sha256": native.model_sha256, "mujoco_runtime_version": 3007000,
                        "gripper_drive_position_rad": joint_position[6],
                        "gripper_drive_velocity_rad_s": joint_velocity[6],
                        "target_gripper_drive_position_rad": float(targets.position[6])}
                       if native.model_profile.linkage else {
                        "gripper_position_m": joint_position[6:],
                        "gripper_velocity_m_s": joint_velocity[6:],
                        "target_gripper_position_m": targets.position[6:].tolist()}),
                    **simulation.attitude_control.telemetry(),
                    "joint_position_rad": joint_position,
                    "joint_velocity_rad_s": joint_velocity,
                    "target_joint_position_rad": targets.position.tolist(),
                    "end_effector_position_body_m": end_effector_position.tolist(),
                    "end_effector_orientation_body_wxyz": matrix_to_quaternion_wxyz(
                        end_effector_rotation
                    ).tolist(),
                    "end_effector_twist_body": end_effector_twist.tolist(),
                    "cartesian_command_residual": targets.residual_twist.tolist(),
                    "jacobian_rank": targets.jacobian_rank,
                    "ik_mode": targets.ik_mode,
                    "ik_elbow_preference": {
                        **vars(targets.elbow_diagnostics),
                        "reference_height_m": float(kinematics.relative_joint_height(
                            targets.position[:6], up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.shoulder_joint,
                            elbow_joint=targets.elbow_preference.elbow_joint)[0]),
                        "measured_height_m": float(kinematics.relative_joint_height(
                            np.asarray(joint_position[:6]), up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.shoulder_joint,
                            elbow_joint=targets.elbow_preference.elbow_joint)[0]),
                        "preferred_height_m": targets.elbow_preference.preferred_height_m,
                        "reference_wrist_drop_m": float(kinematics.relative_joint_height(
                            targets.position[:6], up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.wrist_lower_joint,
                            elbow_joint=targets.elbow_preference.wrist_upper_joint)[0]),
                        "measured_wrist_drop_m": float(kinematics.relative_joint_height(
                            np.asarray(joint_position[:6]), up_axis=targets.elbow_preference.up_axis,
                            shoulder_joint=targets.elbow_preference.wrist_lower_joint,
                            elbow_joint=targets.elbow_preference.wrist_upper_joint)[0]),
                        "preferred_wrist_drop_m": targets.elbow_preference.preferred_wrist_drop_m,
                        "reference_joint3_rad": float(targets.position[ARM_JOINT_NAMES.index("joint3")]),
                        "measured_joint3_rad": float(joint_position[ARM_JOINT_NAMES.index("joint3")]),
                        "preferred_joint3_upper_rad": -targets.elbow_preference.joint3_negative_margin_rad,
                        "joint3_angle_scale_rad": targets.elbow_preference.joint3_angle_scale_rad,
                        "joint3_length_scale_m": targets.elbow_preference.joint3_length_scale_m,
                        "up_frame": "cubesat_bus",
                        "up_axis": list(targets.elbow_preference.up_axis),
                    },
                    'ik_status': targets.solver_status,
                    'ik_reasons': targets.solver_reasons,
                    'ik_solve_time_ms': targets.solve_time_ms,
                    'operator_twist_body': targets.raw_operator_twist.tolist(),
                    'expected_twist_body': targets.desired_twist.tolist(),
                    'predicted_twist_body': targets.achieved_twist.tolist(),
                    'arm_joint_limits_rad': arm_joint_limits_rad,
                    'joint_limit_margin_rad': [
                        float(min(q-lo, hi-q)) if np.isfinite(lo) or np.isfinite(hi) else None
                        for q, lo, hi in zip(joint_position[:6], targets.joint_min[:6], targets.joint_max[:6])],
                    'fastener_demo': (dict(
                        phase=demo_command(int(render_sim_time_ns)*1e-9)[0],
                        travel_mm=float(assembly.extra['fastener_demo'].travel().state)*1000,
                        angle_deg=math.degrees(float(assembly.extra['fastener_demo'].angle().state)))
                        if 'fastener_demo' in assembly.extra else None),
                    'motion_speed_diagnostics': targets.speed_monitor.latest,
                    'arm_actuator_diagnostics': targets.actuator_diagnostics,
                    "ik_damping": float(targets.ik_damping),
                    "ik_velocity_scale": float(targets.velocity_scale),
                    "ik_minimum_singular_value": float(targets.minimum_singular_value),
                    "ik_condition_number": float(min(targets.condition_number, 1.0e9)),
                    "ik_nullspace_correction_norm": float(targets.nullspace_correction_norm),
                    "command_stale": targets.command_stale,
                    "ik_control_rate_hz": args.ik_rate,
                    "dynamics_rate_hz": DYNAMICS_HZ,
                    "ik_update_count": str(targets.update_count),
                    "ik_solve_count": str(targets.ik_solve_count),
                    "tracking_scale": float(targets.tracking_scale),
                    "reference_governor_state": targets.governor_state,
                    "reference_controller_version": "bounded_reference_recovery_v2",
                    "reference_joint_error_rad": (targets.position[:6] - np.asarray(joint_position[:6])).tolist(),
                    "reference_effort_ratio": targets.arm_effort_ratio.tolist(),
                    "reference_blocked_joints": (np.flatnonzero(getattr(targets.governor, "blocked", np.zeros(6))) + 1).tolist(),
                    "reference_saturation_seconds": getattr(targets.governor, "saturation_seconds", np.zeros(6)).tolist(),
                    "reference_limited_joints": list(targets.governor_limited_joints),
                    "dynamics_step_s": native.TIME_STEP,
                    "position_tracking_error_m": float(
                        np.linalg.norm(targets.position_error)
                    ),
                    "orientation_tracking_error_rad": float(
                        np.linalg.norm(targets.orientation_error)
                    ),
                }
            )

        observation = ObservationComponent(bridge, snapshot_observation)
        assembly.install([observation])
        observation_snapshots = observation.model
        # The attitude chain is created by its component during assembly, so this
        # report has to come after it (the builder defers the chain).
        attitude = simulation.attitude_control
        print(json.dumps({
            "type": "attitude_control_configuration",
            "settings": attitude.settings,
            "enabled": attitude.enabled,
            "hardware_source": str(native.MODEL_PATH),
            "settings_source": str(native.MODEL_PATH.with_name("attitude_control.json")),
            "wheel_axes_body": [list(w.axis) for w in attitude.wheels],
            "spin_inertia_kg_m2": [w.spin_inertia for w in attitude.wheels],
            "max_torque_nm": [w.max_torque for w in attitude.wheels],
            "max_speed_rad_s": [w.max_speed for w in attitude.wheels],
        }, sort_keys=True), flush=True)
        print(json.dumps(assembly.summary(), sort_keys=True), flush=True)
        with runtime_stage("initialize_state"):
            native._initialize_state(simulation, scene)
        environment = scene_instance.get("environment", {}) if scene_instance else {}
        orbit = environment.get("orbit", DEFAULT_ORBIT)
        randomized = scene_instance["randomization"] if scene_instance else {
            "target_position_m": list(native.TARGET_POS),
            "target_orientation_wxyz": list(native.TARGET_QUAT),
            "target_linear_velocity_m_s": list(native.COMMON_VELOCITY),
            "target_angular_velocity_rad_s": list(native.TARGET_SPIN),
        }
        if target_spec.hinge_joint:
            randomized.setdefault("target_hinge_position_rad", 0.0)
        orbit_state = apply_orbital_initial_state(
            scene, native, earth, orbit, randomized, initial_joints, orbit_origin
        )
        print(
            json.dumps(
                {
                    "type": "gravity_configuration",
                    "sources": ([s["name"] for s in simulation.local_stepper.orbital_environment.telemetry().get("sources", [])]
                                if local_backend and simulation.local_stepper.orbital_environment is not None
                                else ["earth", "sun"]),
                    "central_body": "earth",
                    "gravity_targets": gravity_target_names,
                    "local_tidal_source": "earth" if local_backend and args.orbital_mode == "linear_tidal" else None,
                    "orbital_mode": args.orbital_mode if local_backend else "basilisk",
                    "orbit": orbit,
                    "randomize_orbit_phase": bool(scene_instance and scene_instance["randomize_orbit_phase"]),
                    **orbit_state,
                },
                sort_keys=True,
            ),
            flush=True,
        )

        # Enable the physical Earth/Sun fields after the explicit initial
        # MJScene state has been assigned.  NBodyGravity now reads the native
        # MJScene messages at each integration/substep. The local backend's
        # origin O is a separate point that was assigned its orbit above.
        if not local_backend:
            orbit_component.enable_gravity()
        print(
            json.dumps(
                {
                    "type": "gravity_initialization",
                    "native_targets_bound": len(gravity_target_names),
                    "gravity_enabled": True,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        joints = [scene.getBody(body).getScalarJoint(joint) for body, joint in native.JOINTS]
        targets.bind_joint_state_provider(lambda: np.asarray(
            [float(read().state) for read in position_readers[:6]]))
        targets.bind_joint_velocity_provider(lambda: np.asarray(
            [float(read().state) for read in velocity_readers[:6]]))
        if local_backend:
            # The MuJoCo servos publish the same requested/applied efforts.
            stepper = simulation.local_stepper
            requested_torque_readers = [msg.addSubscriber() for msg in stepper.requestedOutMsgs[:6]]
            applied_torque_readers = [msg.addSubscriber() for msg in stepper.appliedOutMsgs[:6]]
        else:
            models_by_tag = {model.ModelTag: model for model in dynamics_models}
            pid_models = [models_by_tag[f"{name}PID"] for name in ARM_JOINT_NAMES]
            limiter_models = [models_by_tag[f"{name}TorqueLimiter"] for name in ARM_JOINT_NAMES]
            requested_torque_readers = [model.outputOutMsg.addSubscriber() for model in pid_models]
            applied_torque_readers = [model.actuatorOutMsg.addSubscriber() for model in limiter_models]
        targets.bind_actuator_state_provider(lambda: {
            "requested_torque_nm": [float(read().input) for read in requested_torque_readers],
            "applied_torque_nm": [float(read().input) for read in applied_torque_readers],
            "torque_limits_nm": native.TORQUE_LIMITS[:6].tolist(),
        })
        arm_actuators = [scene.getSingleActuator(name) for name in native.ACTUATORS[:6]]
        targets.bind_effort_provider(
            lambda: np.asarray([abs(float(actuator.actuatorInMsg().input)) for actuator in arm_actuators]) / native.TORQUE_LIMITS[:6]
        )

        wall_start = time.monotonic()
        performance = RuntimePerformanceMonitor(args.simulation_rate, wall_start=wall_start)
        processing_seconds = 0.0
        frame_count = int(math.ceil(args.duration * RENDER_HZ)) if args.duration > 0.0 else None
        if client.reset_generation:
            bridge.publish_event("scene_reset", {"request_id": client.reset_generation, "sim_time_ns": "0"})
        frame = 0
        next_advance_wall = wall_start
        next_backpressure_log = 0.0
        while frame_count is None or frame < frame_count:
            policy.service()
            reset_request = client.take_reset()
            if reset_request is not None:
                policy.reset_task(None)
                print(json.dumps({"type": "reset_accepted", "request_id": reset_request,
                                  "wall_time_ns": str(time.time_ns())}), flush=True)
                return reset_request
            if policy.paused and not policy.stepping:
                next_advance_wall = time.monotonic()
                time.sleep(.01)
                continue
            # Flow control belongs outside ExecuteSimulation/UpdateState. Both
            # network workers remain alive, so STOP/reset/control stay responsive.
            # At most two render snapshots are emitted by one outer advance.
            # Check even during preview: START can arrive on the network worker
            # after preflight but before the render callback samples its mode.
            if (not client.has_observation_capacity()
                    or not bridge.publisher.has_frame_capacity()):
                now = time.monotonic()
                if now >= next_backpressure_log:
                    print(json.dumps({"type": "capture_backpressure", "sim_frame": frame,
                                      **render_transport_status(bridge.publisher),
                                      **client.observation_transport_status()}), flush=True)
                    next_backpressure_log = now + 2.0
                next_advance_wall = time.monotonic()
                time.sleep(.01)
                continue
            # Bound waits so reset/STOP and monotonic policy deadlines are serviced
            # even with capture backpressure or a very slow simulation rate.
            deadline = next_advance_wall + POLICY_PERIOD_NS * 1e-9 / args.simulation_rate
            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(.01, remaining))
                continue
            policy_stop_ns = policy.prepare_step() if policy.stepping else None
            if policy.paused and policy_stop_ns is None:
                continue
            frame += 1
            stop_ns = policy_stop_ns if policy_stop_ns is not None else tick_time_ns(frame, RENDER_HZ)
            sim_seconds = stop_ns * 1.0e-9
            if frame_count is not None:
                if policy_stop_ns is not None and stop_ns > macros.sec2nano(args.duration):
                    client.policy.invalidate("policy interval would exceed finite simulation duration")
                    policy.service()
                    break
                stop_ns = min(stop_ns, macros.sec2nano(args.duration))
                sim_seconds = stop_ns * 1.0e-9
            processing_start = time.monotonic()
            next_advance_wall = max(deadline, processing_start)
            simulation.ConfigureStopTime(stop_ns)
            simulation.ExecuteSimulation()
            execute_end = time.monotonic()
            if frame == 1:
                client.complete_reset()
            snapshots = observation_snapshots.drain()
            for observation_snapshot in snapshots:
                client.send_observation(observation_snapshot)
            if policy_stop_ns is not None:
                policy.complete_step(snapshots)
            else:
                for observation_snapshot in snapshots:
                    policy.observe(observation_snapshot)
            processing_end = time.monotonic()
            processing_seconds += processing_end - processing_start
            performance_event = performance.record(
                stop_ns, execute_end - processing_start, processing_end - execute_end,
                now=processing_end,
            )
            if performance_event is not None:
                performance_event.update(render_transport_status(bridge.publisher))
                performance_event.update(client.observation_transport_status())
                print(json.dumps(performance_event, sort_keys=True), flush=True)
        if args.duration > 0.0:
            wall_seconds = time.monotonic() - wall_start
            print(
                json.dumps(
                    {
                        "type": "performance_summary",
                        "simulated_seconds": args.duration,
                        "wall_seconds": wall_seconds,
                        "processing_seconds": processing_seconds,
                        "wall_real_time_factor": args.duration / wall_seconds,
                        "processing_real_time_factor": args.duration / processing_seconds,
                        "ik_control_rate_hz": args.ik_rate,
                        "dynamics_rate_hz": DYNAMICS_HZ,
                        "ik_update_count": targets.update_count,
                        "ik_solve_count": targets.ik_solve_count,
                        "dynamics_backend": args.dynamics_backend,
                        "local_dynamics": simulation.local_stepper.telemetry() if local_backend else None,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        _ = keep_alive
    finally:
        if policy is not None:
            client.policy.invalidate("simulation session ended")
            policy.service()
        stepper = getattr(simulation, "local_stepper", None)
        if stepper is not None:
            stepper.close()
        if bridge:
            with runtime_stage("close_render_bridge"):
                bridge.close()
        if gravity_factory is not None:
            with runtime_stage("unload_ephemeris"):
                gravity_factory.unloadSpiceKernels()
