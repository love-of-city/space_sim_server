"""Authoritative native MJScene flight using the default SpaceSim assets."""

from __future__ import annotations

import argparse
import hashlib
import json
import socket
import struct
import sys
import time
import uuid
from dataclasses import asdict
from pathlib import Path

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros

from simulation.attitude_control import AttitudeControl
from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.orbital_configuration import environment_metadata
from simulation.navigation_gravity import BskNavigationGravity, PointMassNavigationGravity
from simulation.local_mujoco_stepper import LocalMujocoStepper
from simulation.physics_ports import PhysicsPorts
from space_arm_platform.rendezvous_config import FlightOptions
from space_arm_platform.gnss_config import GNSSConfig
from simulation.differential_gnss import DifferentialGNSS, JointOrbitFilter
from simulation.gnss_transport import DelayedGNSSNavigation

from .assets import bind_team_catalog
from .model import prepare_model
from .navigation import OrbitFilter
from .reference import EARTH_RADIUS, FlightProfile
from .rendering import flight_scene_settings


class FlightControl(sysModel.SysModel):
    def __init__(self, scene, profile, mass, limit, seed, options=None, ports=None, *, gravity_model=None):
        super().__init__()
        self.ModelTag = "teamOrbitalGNSSControl"
        self.scene, self.profile, self.mass, self.limit = scene, profile, mass, limit
        self.readers = [
            scene.getSite(n).stateOutMsg.addSubscriber()
            for n in ("target_flight_center", "orbital_thrust_center")
        ]
        self.options = options or FlightOptions(seed=seed)
        self.gravity = gravity_model or PointMassNavigationGravity()
        self.filters = [OrbitFilter(self.options.gnss_position_sigma_m, self.options.gnss_velocity_sigma_m_s,
                                    gravity_model=self.gravity) for _ in range(2)]
        self.rng = np.random.default_rng(np.random.SeedSequence([seed, 0]))
        self.paired_receiver = (DifferentialGNSS(GNSSConfig.from_dict(self.options.differential_gnss), seed)
                                if self.options.differential_gnss is not None else None)
        self.joint_filter = JointOrbitFilter(gravity_model=self.gravity) if self.paired_receiver else None
        self.delayed_navigation = None
        if self.paired_receiver and self.paired_receiver.config.delay_s > 0:
            self.delayed_navigation = DelayedGNSSNavigation(self.paired_receiver.config, seed, gravity_model=self.gravity)
            self.paired_receiver = self.delayed_navigation.receiver
            self.joint_filter = self.delayed_navigation.filter
            self.origin_readers = [scene.getBody(name).getOrigin().stateOutMsg.addSubscriber()
                                   for name in ("capture_target", "cubesat_bus")]
        self.messages = [messaging.SingleActuatorMsg() for _ in range(3)]
        for i, message in enumerate(self.messages):
            message.write(messaging.SingleActuatorMsgPayload(input=0.0))
            (ports or PhysicsPorts(scene)).drive_actuator(f"orbital_force_{i}", message, owner="rendezvous_translation")
        self.force_n = np.zeros(3)
        self.rows = []
        self.previous_time = None
        self.aborted = False
        self.navigation_loss_s = None

    def UpdateState(self, nanos):
        t = nanos * 1e-9
        dt = 0.0 if self.previous_time is None else t - self.previous_time
        self.previous_time = t
        payloads = [read() for read in self.readers]
        truth = np.array([np.r_[p.r_BN_N, p.v_BN_N] for p in payloads])
        packet = self.paired_receiver.measure(truth, t) if self.paired_receiver and not self.delayed_navigation else None
        observations = (None if self.delayed_navigation else
                        packet["observations"] if packet else truth + self.rng.normal(size=(2, 6)) * np.array(
                            [self.options.gnss_position_sigma_m] * 3 + [self.options.gnss_velocity_sigma_m_s] * 3))
        navigation_valid = self.navigation_loss_s is None or t < self.navigation_loss_s
        if navigation_valid and self.delayed_navigation:
            origins = [read() for read in self.origin_readers]
            origin_states = np.array([np.r_[p.r_BN_N, p.v_BN_N] for p in origins])
            estimate = self.delayed_navigation.step(
                truth, t, np.array([np.zeros(3), self.force_n / self.mass]),
                origin_offsets=truth-origin_states,
                cooperative={"rotation": rbk.MRP2C(origins[0].sigma_BN).T,
                             # The distant model removes articulated joints at
                             # authored zero. The receiving model restores the
                             # hinge and starts reporting its measured motion.
                             "hinge": 0.0})
            navigation_valid = estimate is not None
            packet = self.delayed_navigation.last_packet
            observations = packet["observations"] if packet else None
            if navigation_valid:
                for i, f in enumerate(self.filters):
                    f.state = estimate[i].copy()
                    f.covariance = self.delayed_navigation.covariance[i*6:(i+1)*6, i*6:(i+1)*6].copy()
            else:
                # Until the first delivery there is no navigation solution.
                # Keep translation off; never bootstrap it from current truth.
                estimate = np.zeros((2, 6))
        elif navigation_valid and self.joint_filter:
            estimate = self.joint_filter.update(packet, np.array([np.zeros(3), self.force_n / self.mass]))
            for i, f in enumerate(self.filters):
                f.state = estimate[i].copy()
                f.covariance = self.joint_filter.covariance[i*6:(i+1)*6, i*6:(i+1)*6].copy()
        elif navigation_valid:
            estimate = np.array(
                [
                    f.update(z, dt, self.force_n / self.mass if i else np.zeros(3), time_s=t)
                    for i, (f, z) in enumerate(zip(self.filters, observations))
                ]
            )
        else:
            self.aborted = True
            estimate = np.array(
                [f.state if f.state is not None else np.zeros(6) for f in self.filters]
            )
        desired_r, desired_v, desired_a = self.profile.chaser(t)
        nominal_r = desired_r.copy()
        target_r, target_v, target_a = self.profile.target(t)
        # Only used for evaluation: compare against a reference following the
        # propagated passive target, not an undisturbed target known in advance.
        evaluation_r = nominal_r + truth[0, :3] - target_r
        acquiring = self.delayed_navigation is not None and self.delayed_navigation.estimate is None
        if acquiring:
            requested = np.zeros(3)
        else:
            desired_r = desired_r + estimate[0, :3] - target_r
            desired_v = desired_v + estimate[0, 3:] - target_v
            desired_a = desired_a + self.gravity.acceleration(estimate[0, :3], t) - target_a
            # No truth-state feedback: commands use GNSS-filter estimates and an
            # independently constructed reference. Native dynamics owns all motion.
            acceleration = (
                desired_a
                - self.gravity.acceleration(estimate[1, :3], t)
                + 0.0004 * (desired_r - estimate[1, :3])
                + 0.04 * (desired_v - estimate[1, 3:])
            )
            requested = self.mass * acceleration
        norm = np.linalg.norm(requested)
        applied = requested * min(1.0, self.limit / max(norm, 1e-15))
        if not np.isfinite(applied).all() or not np.isfinite(truth).all():
            raise RuntimeError("Non-finite native flight state")
        if self.aborted:
            applied[:] = 0.0
        self.force_n = applied
        body_force = rbk.MRP2C(payloads[1].sigma_BN) @ applied
        for message, value in zip(self.messages, body_force):
            message.write(messaging.SingleActuatorMsgPayload(input=float(value)), nanos)
        relative_r = truth[1, :3] - truth[0, :3]
        relative_v = truth[1, 3:] - truth[0, 3:]
        normal = np.cross(truth[0, :3], truth[0, 3:])
        rate = normal / np.linalg.norm(truth[0, :3]) ** 2
        relative_rotating_v = relative_v - np.cross(rate, relative_r)
        self.rows.append(
            {
                "t": t,
                "phase": "Abort" if self.aborted else "NavigationAcquire" if acquiring else self.profile.phase(t),
                "truth": truth.tolist(),
                "gnss": observations.tolist() if navigation_valid else None,
                "navigation_valid": navigation_valid,
                "gnss_model": "paired correlated solutions" if self.paired_receiver else "independent receiver solutions",
                "relative_navigation_covariance": (
                    self.delayed_navigation.relative_covariance.tolist() if self.delayed_navigation and navigation_valid else
                    np.full((6, 6), np.nan).tolist() if self.delayed_navigation else
                    self.joint_filter.relative_covariance.tolist() if self.joint_filter and self.joint_filter.covariance is not None else None),
                "estimate": estimate.tolist(),
                "force_n": applied.tolist(),
                "requested_force_n": requested.tolist(),
                "range_m": float(np.linalg.norm(relative_r)),
                "relative_speed_hill_m_s": float(np.linalg.norm(relative_rotating_v)),
                "tracking_error_m": float(np.linalg.norm(truth[1, :3] - evaluation_r)),
                "target_prediction_error_m": float(np.linalg.norm(truth[0, :3] - estimate[0, :3])),
                "altitude_m": float(np.linalg.norm(truth[1, :3]) - EARTH_RADIUS),
                **(self.delayed_navigation.telemetry(t) if self.delayed_navigation else {}),
            }
        )


def state_packet(scene, controller, row, run_id):
    bodies = []
    for i, (name, role) in enumerate(
        (("capture_target", "target"), ("cubesat_bus", "chaser"))
    ):
        p = scene.getBody(name).getOrigin().stateOutMsg.read()
        bodies.append(
            {
                "body_id": f"rendezvous/{role}",
                "position_m": list(p.r_BN_N),
                "velocity_m_s": list(p.v_BN_N),
                "attitude_wxyz": list(rbk.MRP2EP(p.sigma_BN)),
                "angular_velocity_rad_s": list(p.omega_BN_B),
            }
        )
    return {
        "protocol": "space-sim-state/1",
        "type": "state",
        "sim_time_s": row["t"],
        "bodies": bodies,
        "metadata": {
            "run_id": run_id,
            "phase": row["phase"],
            "capture_time_s": None,
            "model": "SpaceSim SARM + ground validation satellite",
            "reference_frame": "Earth J2000",
            "simulation_authority": "SpaceSim LocalMujocoStepper + Basilisk components",
            "scenario": controller.profile.scenario,
            "position_reference": "body origin",
            "gnss_reference": "assembled center of mass",
            "handoff_distance_m": 100.0,
            "gnss": row["gnss"],
            "navigation": row["estimate"],
            "navigation_valid": row["navigation_valid"],
            "center_distance_m": row["range_m"],
            "mission_options": controller.options.to_dict(),
            **(controller.delayed_navigation.telemetry(row["t"]) if controller.delayed_navigation else {}),
        },
    }


def run(args):
    options = FlightOptions.from_dict(json.loads(args.config.read_text(encoding="utf-8"))) if getattr(args, "config", None) else FlightOptions(seed=args.seed)
    source_hashes = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in Path(__file__).parent.glob("*.py")
    }
    repository = Path(__file__).resolve().parents[2]
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    (output / "mission_options.json").write_text(json.dumps(options.to_dict(), indent=2), encoding="utf-8")
    integration_sources = ["simulation/components/flight_environment.py", "simulation/components/drag.py",
                           "simulation/components/orbit.py", "simulation/components/orbital_gravity.py",
                           "simulation/local_mujoco_stepper.py", "simulation/orbital_environment.py",
                           "simulation/attitude_control.py", "simulation/assembly.py", "simulation/physics_ports.py",
                           "simulation/differential_gnss.py", "simulation/gnss_transport.py", "backend/space_arm_platform/gnss_config.py",
                           "simulation/navigation_gravity.py", "simulation/orbital_configuration.py", "backend/space_arm_platform/orbital_physics.py",
                           "backend/space_arm_platform/rendezvous_config.py", "native/local_mujoco_stepper.cpp"]
    integration_hashes = {name: hashlib.sha256((repository / name).read_bytes()).hexdigest() for name in integration_sources}
    model, mesh_files, manifest = prepare_model(
        repository, output, force_limit_n=args.force_limit
    )
    catalog = bind_team_catalog(args.catalog, repository, manifest, output) if args.catalog else None
    profile = FlightProfile(scenario=args.scenario, handoff_distance_m=options.handoff_distance_m)
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("teamOrbitalFlight")
    navigation_period = (1 / GNSSConfig.from_dict(options.differential_gnss).sample_rate_hz
                         if options.differential_gnss is not None else 0.5)
    if navigation_period < args.step or not np.isclose(navigation_period / args.step, round(navigation_period / args.step)):
        raise ValueError("GNSS period must be an integer multiple of the outer physics step")
    control_period = (args.step if options.differential_gnss is not None
                      and options.differential_gnss["delay_s"] > 0 else navigation_period)
    process.addTask(
        simulation.CreateNewTask("flightDynamics", macros.sec2nano(args.step)), 30
    )
    process.addTask(
        simulation.CreateNewTask("flightNavigation", macros.sec2nano(control_period)), 10
    )
    scene = mujoco.MJScene.fromFile(str(model), files=mesh_files)
    scene.ModelTag = "teamFlightMJScene"
    scene.extraEoMCall = True
    # MJScene is registered for initialization only. The shared platform core
    # owns propagation and publishes its states through the existing messages.
    process.addTask(simulation.CreateNewTask("flightPublisherInit", macros.sec2nano(args.step)))
    simulation.AddModelToTask("flightPublisherInit", scene)
    ctx = AssemblyContext(simulation, scene, config=SessionConfig(
        model_path=model, dynamics_backend="local", publish_stride=1,
        clock_rate_hz=1 / args.step, physics_task="flightDynamics"))
    from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC, _register_celestial_bodies
    orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=options.physics)
    ctx.install([orbit])
    stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin,
                                substeps=max(1, round(args.step * 240)), publish_every=1)
    # Flight uses a 20 Hz outer task with 240 Hz internal integration, unlike
    # teleop's 240 Hz outer task. Keep gravity fresh at every internal substep;
    # inheriting teleop's eight-step hold would reduce it to 2.5 Hz in flight.
    ctx.ports.set_stepper(stepper)
    ctx.add(stepper, Slot.PHYSICS, component="dynamics_core")
    ctx.install([OrbitalGravityComponent(force_every=1)])
    environment = FlightEnvironmentComponent(options, [
        ("capture_target", "target_flight_center", manifest["bodies"]["capture_target"]["mass_kg"], options.target_area_m2),
        ("cubesat_bus", "orbital_thrust_center", manifest["bodies"]["cubesat_bus"]["mass_kg"], options.chaser_area_m2),
    ])
    ctx.install([environment])
    attitude = AttitudeControl(simulation, process, scene, model, drive_task=("flightDynamics", int(Slot.ACTUATOR)))
    controller = FlightControl(
        scene,
        profile,
        manifest["bodies"]["cubesat_bus"]["mass_kg"],
        args.force_limit,
        options.seed, options, ctx.ports, gravity_model=BskNavigationGravity(ctx.ports.orbital_environment()),
    )
    controller.navigation_loss_s = args.navigation_loss
    simulation.AddModelToTask("flightNavigation", controller)
    bridge = None
    render_epoch = DEFAULT_EPHEMERIS_EPOCH_UTC
    if args.catalog:
        from bsk_render_adapter import BasiliskRenderBridge
        from bsk_render_adapter.protocol import RecordingOnlyPublisher

        bridge = BasiliskRenderBridge(
            host=args.render_host,
            port=args.render_port,
            origin_object="team/cubesat_bus",
            frame_period_ns=macros.sec2nano(0.5),
            publisher=None if args.live else RecordingOnlyPublisher(),
            recording_path=output / "team_rendezvous.bskrec",
        )
        bridge.add_mj_scene(
            scene,
            namespace="team",
            source_path=model,
            mesh_asset_catalog=catalog,
            camera_picture_in_picture=False,
        )
        _register_celestial_bodies(bridge, orbit.earth, orbit.sun)
        bridge.set_scene_settings(flight_scene_settings(args.sunlight_intensity_scale, manifest["bodies"]))
        simulation.AddModelToTask("flightDynamics", bridge, -10000)
    simulation.InitializeSimulation()
    simulation.disableTask("flightPublisherInit")
    initial_rng = np.random.default_rng(np.random.SeedSequence([options.seed, 1]))
    initial_offsets = initial_rng.normal(size=(2, 6)) * np.array([options.initial_position_sigma_m] * 3 + [options.initial_velocity_sigma_m_s] * 3)
    initial_states = []
    for index, (body_name, reference) in enumerate((
        ("capture_target", profile.target),
        ("cubesat_bus", profile.chaser),
    )):
        r, v, _ = reference(0.0)
        r, v = r + initial_offsets[index, :3], v + initial_offsets[index, 3:]
        initial_states.append(np.r_[r, v].tolist())
        body = scene.getBody(body_name)
        center = np.array(manifest["bodies"][body_name]["center_body_m"])
        # The only state setters in this module: physical initial condition.
        body.setPosition((r - center).tolist())
        body.setVelocity(v.tolist())
        body.setAttitude([0.0, 0.0, 0.0])
        body.setAttitudeRate([0.0, 0.0, 0.0])
    orbit.origin.set_state(np.asarray(initial_states[0][:3]), np.asarray(initial_states[0][3:]))
    channel = None
    if args.state_port:
        channel = socket.create_connection(("127.0.0.1", args.state_port), timeout=2)
    run_id = uuid.uuid4().hex
    previous_interval = sys.getswitchinterval()
    sys.setswitchinterval(0.0001)
    started = time.monotonic()
    last_phase = None
    duration = (
        min(profile.duration_s, args.duration) if args.duration else profile.duration_s
    )
    wheel_peak = 0.0
    attitude_peak = 0.0
    actuator_rows = []
    environment_rows = []
    sampled_rows = []
    try:
        for tick in range(int(duration * 2) + 1):
            simulation.ConfigureStopTime(macros.sec2nano(tick * 0.5))
            simulation.ExecuteSimulation()
            row = controller.rows[-1]
            sampled_rows.append(row)
            telemetry = attitude.telemetry()
            actuator_rows.append(telemetry)
            environment_rows.append(environment.telemetry())
            wheel_peak = max(
                wheel_peak,
                max(abs(v) for v in telemetry["reaction_wheels"]["speed_rad_s"]),
            )
            attitude_peak = max(
                attitude_peak, telemetry["attitude_control"]["attitude_error_angle_rad"]
            )
            if channel is not None:
                data = json.dumps(
                    state_packet(scene, controller, row, run_id), allow_nan=False
                ).encode()
                try:
                    channel.sendall(struct.pack("!I", len(data)) + data)
                except OSError:
                    channel.close()
                    channel = None
            if row["phase"] != last_phase or tick % 600 == 0:
                print(
                    json.dumps(
                        {
                            "t": row["t"],
                            "phase": row["phase"],
                            "range_m": row["range_m"],
                            "error_m": row["tracking_error_m"],
                            "wall_s": time.monotonic() - started,
                        }
                    ),
                    flush=True,
                )
                last_phase = row["phase"]
            if controller.aborted and row["t"] >= args.navigation_loss + 60:
                break
    finally:
        sys.setswitchinterval(previous_interval)
        if channel:
            channel.close()
        if bridge:
            bridge.close()
    # Navigation may run faster than the 2 Hz evidence/actuator grid. Keep every
    # exported array on the same physical sampling instants.
    rows = sampled_rows
    holds = [r for r in rows if r["phase"] == "100 m handoff hold"]
    hold_duration = holds[-1]["t"] - holds[0]["t"] if holds else 0.0
    checks = {
        "team_assets_loaded": manifest["mesh_count"] == 113
        and manifest["body_count"] == 15,
        "finite_states": bool(np.isfinite(np.array([r["truth"] for r in rows])).all()),
        "force_within_limit": max(np.linalg.norm(r["force_n"]) for r in rows)
        <= args.force_limit + 1e-9,
        "tracking_error_after_acquisition": bool(holds) and max(r["tracking_error_m"] for r in rows if r["t"] >= 600) < 5.0,
        "handoff_dwell_120s": hold_duration >= profile.hold_duration_s,
        "handoff_range_100m": bool(holds)
        and max(abs(r["range_m"] - profile.handoff_distance_m) for r in holds) < 5.0,
        "handoff_relative_speed": bool(holds)
        and max(r["relative_speed_hill_m_s"] for r in holds) < 0.2,
        "minimum_separation": min(r["range_m"] for r in rows) >= 100.0,
        "target_orbit_500km": abs(rows[-1]["altitude_m"] - (np.linalg.norm(rows[-1]["truth"][0][:3]) - EARTH_RADIUS)) < 10.0,
        "single_multibody_integrator": stepper.external_writes == 0,
        "gravity_at_every_substep": stepper.orbital_force_every == 1,
        "wheel_speed_limits": wheel_peak < min(w.max_speed for w in attitude.wheels),
        "wheel_torque_limits": all(
            abs(value) <= wheel.max_torque + 1e-12
            for sample in actuator_rows
            for value, wheel in zip(
                sample["reaction_wheels"]["applied_motor_torque_nm"], attitude.wheels
            )
        ),
        "attitude_hold": attitude_peak < np.deg2rad(0.5),
        "not_aborted": not controller.aborted,
    }
    if args.scenario == "orbit-raise":
        checks["started_at_300km"] = abs(rows[0]["altitude_m"] - (np.linalg.norm(initial_states[1][:3]) - EARTH_RADIUS)) < 0.01
    checks = {key: bool(value) for key, value in checks.items()}
    passed = all(checks.values())
    final_packet = state_packet(scene, controller, rows[-1], run_id)
    final_distance = np.linalg.norm(
        np.array(final_packet["bodies"][0]["position_m"])
        - final_packet["bodies"][1]["position_m"]
    )
    report = {
        "schema": "spacesim-team-flight/1",
        "scenario": args.scenario,
        "profile": asdict(profile),
        "model_manifest": manifest,
        "basic_nominal_passed": passed,
        "checks": checks,
        "outcome": "100 m handoff ready"
        if passed
        else "Aborted"
        if controller.aborted
        else "Incomplete or failed",
        "final_time_s": rows[-1]["t"],
        "final_body_origin_distance_m": float(final_distance),
        "final_center_distance_m": rows[-1]["range_m"],
        "hold_duration_s": hold_duration,
        "wall_time_s": time.monotonic() - started,
        "peak_wheel_speed_rad_s": wheel_peak,
        "peak_attitude_error_rad": attitude_peak,
        "max_tracking_error_m": max(r["tracking_error_m"] for r in rows),
        "dynamics_step_s": args.step,
        "navigation_period_s": navigation_period,
        "control_period_s": control_period,
        **({"gnss_transport": controller.delayed_navigation.snapshot()} if controller.delayed_navigation else {}),
        "environment": "Shared BSK Earth/Sun gravity; exponential co-rotating atmosphere; eclipsed cannonball SRP; seeded Gauss-Markov acceleration",
        "mission_options": options.to_dict(),
        "orbital_physics": environment_metadata(options.physics, orbit.earth, epoch_utc=render_epoch),
        "environment_state": [model.snapshot() for model in environment.models],
        "initial_states_eci": initial_states,
        "initial_offsets_eci": initial_offsets.tolist(),
        "local_dynamics": stepper.telemetry(),
        "module_schedule": ctx.summary(),
        "acceptance_limits": {"minimum_range_m": 100., "hold_error_m": 5., "hold_speed_m_s": 0.2, "acquisition_s": 600., "tracking_after_acquisition_m": 5.},
        "navigation": ("paired correlated GNSS solutions and joint 12-state filter; no raw carrier processing"
                       if controller.paired_receiver else
                       "independent noisy GNSS position/velocity receivers and six-state filters; attitude uses platform truth-navigation hold"),
        "capture": {"latch_time_s": None},
        "source_hashes": source_hashes,
        "integration_source_hashes": integration_hashes,
        "rendering": {
            "ephemeris_epoch_utc": render_epoch,
            "celestial_bodies": ["earth", "sun"] if bridge else [],
            "sunlight_intensity_scale": args.sunlight_intensity_scale,
            "geometry_and_materials": "unchanged SpaceSim repository assets",
        },
    }
    (output / "mission_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    if controller.delayed_navigation:
        audit = controller.delayed_navigation.measurement_audit
        np.savez_compressed(output / "gnss_measurements.npz",
                            **{key: [sample[key] for sample in audit]
                               for key in ("stamp", "due", "sequence", "truth", "observations", "received")})
    np.savez_compressed(
        output / "team_telemetry.npz",
        time_s=[r["t"] for r in rows],
        truth=[r["truth"] for r in rows],
        gnss=[
            r["gnss"] if r["gnss"] is not None else np.full((2, 6), np.nan)
            for r in rows
        ],
        navigation_valid=[r["navigation_valid"] for r in rows],
        estimate=[r["estimate"] for r in rows],
        force_n=[r["force_n"] for r in rows],
        range_m=[r["range_m"] for r in rows],
        altitude_m=[r["altitude_m"] for r in rows],
        tracking_error_m=[r["tracking_error_m"] for r in rows],
        relative_speed_hill_m_s=[r["relative_speed_hill_m_s"] for r in rows],
        wheel_speed_rad_s=[r["reaction_wheels"]["speed_rad_s"] for r in actuator_rows],
        wheel_torque_nm=[r["reaction_wheels"]["applied_motor_torque_nm"] for r in actuator_rows],
        attitude_error_rad=[r["attitude_control"]["attitude_error_angle_rad"] for r in actuator_rows],
        angular_velocity_rad_s=[r["attitude_control"]["angular_velocity_body_rad_s"] for r in actuator_rows],
        environment_force_n=[[[*b["drag_n"], *b["srp_n"], *b["disturbance_n"]] for b in sample] for sample in environment_rows],
        density_kg_m3=[[b["density_kg_m3"] for b in sample] for sample in environment_rows],
        illumination=[[b["illumination"] for b in sample] for sample in environment_rows],
        **({"relative_navigation_covariance": [r["relative_navigation_covariance"] for r in rows]}
           if controller.joint_filter else {}),
        **({key: [np.nan if r[key] is None else r[key] for r in rows] for key in (
            "gnss_sample_time_s", "gnss_receive_time_s", "gnss_measurement_age_s", "gnss_delivered_sequence",
            "gnss_acquired_sequence", "gnss_pending_count", "gnss_delivered_count", "gnss_requested_delay_s")}
           if controller.delayed_navigation else {}),
    )
    handoff = {
        "schema": "spacesim-near-field-handoff/1",
        "accepted": passed,
        "guidance_frame": "orbital-vbar",
        "state": final_packet,
        "source_model": manifest["source_model"],
        "source_sha256": manifest["source_sha256"],
        "ephemeris_epoch_utc": render_epoch,
        "model_configuration": "restore authored scalar joints at zero with zero relative rates",
        "stowed_joints": manifest["stowed_joints"],
        "body_properties": manifest["bodies"],
        "reaction_wheels": [
            {
                "body": w.body,
                "joint": w.joint,
                "position_rad": float(j.stateOutMsg.read().state),
                "speed_rad_s": float(j.stateDotOutMsg.read().state),
            }
            for w, j in zip(attitude.wheels, attitude.joints)
        ],
        "gnss_estimates_eci": rows[-1]["estimate"],
        "covariances": [f.covariance.tolist() for f in controller.filters],
        "gnss_cross_covariance_eci": (controller.delayed_navigation.covariance[:6, 6:].tolist()
                                     if controller.delayed_navigation and controller.delayed_navigation.covariance is not None else
                                     controller.joint_filter.covariance[:6, 6:].tolist()
                                     if controller.joint_filter and controller.joint_filter.covariance is not None else None),
        "gnss_receiver_state": controller.paired_receiver.snapshot() if controller.paired_receiver else None,
        **({"gnss_transport": controller.delayed_navigation.snapshot()} if controller.delayed_navigation else {}),
        "mission_options": options.to_dict(),
        "environment_state": [model.snapshot() for model in environment.models],
    }
    (output / "handoff.json").write_text(
        json.dumps(handoff, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(
        json.dumps(
            {"passed": passed, "checks": checks, "output": str(output)},
            ensure_ascii=False,
        ),
        flush=True,
    )
    stepper.close()
    return 0 if passed else 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario", choices=("in-orbit", "orbit-raise"), default="in-orbit"
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--step", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--config", type=Path, help="validated mission options JSON")
    parser.add_argument("--force-limit", type=float, default=25.0)
    parser.add_argument("--navigation-loss", type=float)
    parser.add_argument("--catalog", type=Path)
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--render-host", default="127.0.0.1")
    parser.add_argument("--render-port", type=int, default=5558)
    parser.add_argument("--sunlight-intensity-scale", type=float, default=1.0)
    parser.add_argument("--state-port", type=int, default=0)
    args = parser.parse_args()
    if not np.isfinite(args.sunlight_intensity_scale) or not 0 <= args.sunlight_intensity_scale <= 20000:
        parser.error("--sunlight-intensity-scale must be finite and in [0, 20000]")
    if args.step not in (0.05, 0.025, 0.01):
        parser.error(
            "--step must resolve the 20 Hz attitude clock (0.05, 0.025 or 0.01)"
        )
    if args.live and not args.catalog:
        parser.error("--live requires the team mesh catalog")
    if not np.isfinite(args.force_limit) or args.force_limit <= 0:
        parser.error("--force-limit must be finite and positive")
    if args.navigation_loss is not None and (
        not np.isfinite(args.navigation_loss) or args.navigation_loss <= 0
    ):
        parser.error("--navigation-loss must be finite and positive")
    if args.seed < 0:
        parser.error("--seed must be nonnegative")
    if args.duration is not None and (
        not np.isfinite(args.duration) or args.duration <= 0
    ):
        parser.error("--duration must be finite and positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
