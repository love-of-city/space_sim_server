"""Headless force acceptance on a selected model and production orbital graph.

Test drives apply forces through physics ports. They do not move bodies during
integration. Insertion starts at an assembly seat as initial-condition setup;
this does not test robot grasping, approach planning, or UE rendering.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "backend"),
               str(ROOT.parent / "space_sim_UE_Adapter/Adapters"),
               str(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples")]


def validate_case(case, model_path=None):
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    native = load_native_grasp_module(ROOT / "model/SARM/platform")
    from Basilisk.architecture import messaging
    from simulation.assembly import AssemblyContext, SessionConfig
    from simulation.components.clock import ClockComponent
    from simulation.components.dynamics_core import DynamicsCoreComponent
    from simulation.components.arm_reference import JointReferenceComponent
    from simulation.components.orbit import OrbitComponent
    from simulation.components.orbital_gravity import OrbitalGravityComponent
    from simulation.components.attitude import AttitudeComponent
    from simulation.components.plug_retention import install_configured_plug_retention
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from simulation.sarm_model_profile import load_model_profile
    from simulation.orbital_configuration import resolve_orbital_physics, environment_metadata
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix
    from simulation.teleop_grasp_unreal import DEFAULT_ORBIT, DEFAULT_EPHEMERIS_EPOCH_UTC, _apply_orbital_initial_state
    from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG
    from space_arm_platform.sampling import DYNAMICS_HZ, tick_time_ns
    from collection.run_scripted_task import default_target_state

    started = time.perf_counter()
    model = (Path(model_path).resolve() if model_path is not None
             else ROOT / "model/SARM/platform/sarm_20261012.xml")
    original_hash = hashlib.sha256(model.read_bytes()).hexdigest()
    native.MODEL_PATH = model
    profile = load_model_profile(model)
    native.JOINTS, native.ACTUATORS = profile.joints, profile.arm_actuators
    native.KP, native.KD, native.TORQUE_LIMITS = np.full(6, 500.), np.full(6, 50.), np.full(6, 100.)
    native.TIME_STEP = 1 / DYNAMICS_HZ
    initial = np.r_[np.deg2rad(DEFAULT_OPERATING_JOINT_DEG), 0., 0.]
    publisher = HeldJointReferencePublisher(lambda t: (initial, np.zeros(8)), 8)
    sim, scene, models, recorders = native._build_simulation(
        external_reference=publisher, dynamics_backend="local", record_history=False, defer_attitude_control=True)
    stepper = sim.local_stepper
    try:
        ctx = AssemblyContext(sim, scene, config=SessionConfig(model_path=model, publish_stride=stepper.publish_every))
        physics = resolve_orbital_physics(None)
        orbit = OrbitComponent(epoch_utc=DEFAULT_EPHEMERIS_EPOCH_UTC, physics=physics)
        ctx.install([ClockComponent(), orbit, DynamicsCoreComponent(model), OrbitalGravityComponent(mode="bsk"),
                     JointReferenceComponent(publisher, joints=profile.joint_names[:6], actuators=profile.arm_actuators,
                                             kp=native.KP, kd=native.KD, limits=native.TORQUE_LIMITS,
                                             ik_rate_hz=100, wire_servos=False),
                     AttitudeComponent(model)])
        retention = install_configured_plug_retention(ctx)
        if retention is None:
            raise RuntimeError("Enable plug retention before running acceptance")
        drives = {}
        for name in retention.models:
            drive = messaging.CmdForceInertialMsg()
            ctx.ports.add_body_wrench(name, source=f"acceptance_{name}", force_inertial=drive)
            drives[name] = drive
        sim.InitializeSimulation()
        orbital = _apply_orbital_initial_state(scene, native, orbit.earth, DEFAULT_ORBIT,
                                               default_target_state(model), initial, orbit.origin)
        seats = {}
        for name, module in retention.models.items():
            seat = module.detent.sockets[0 if case == "extraction" else -1]
            seats[name] = seat
            if case != "extraction":
                plug = scene.getBody(name)
                plug.setPosition(np.asarray(orbital["initial_position_m"]) + seat.position_body_m
                                 + seat.rotation_body[:, 2] * (.004 if case.startswith("partial") else .0079))
                plug.setAttitude(native._quaternion_to_mrp(matrix_to_quaternion_wxyz(seat.plug_rotation_body)))
                plug.setVelocity(np.asarray(orbital["initial_velocity_m_s"]) - seat.rotation_body[:, 2]
                                 * (0. if case.startswith("partial") else .0001))
        state = ctx.ports.scene_state().addSubscriber()
        bus_layout = ctx.ports.free_body_layout("cubesat_bus")

        def snapshot():
            q = np.asarray(state().qpos).reshape(-1)
            qa = bus_layout[0]
            br = quaternion_wxyz_to_matrix(q[qa+3:qa+7])
            values = {}
            for name, module in retention.models.items():
                pa = module.plug_layout[0]
                relative = br.T @ (q[pa:pa+3]-q[qa:qa+3])
                delta = seats[name].rotation_body.T @ (relative-seats[name].position_body_m)
                values[name] = dict(**module.telemetry(), relative_displacement_m=delta.tolist())
            return values

        # Keep the drive above the configured breakaway threshold, with the
        # same sliding-force margin when the configured friction changes.
        pull_force = max(retention.parameters.static_friction_n + 1.,
                         retention.parameters.sliding_friction_n + 4.)
        push_force = max(retention.parameters.static_friction_n + 1.,
                         retention.parameters.sliding_friction_n + 2.)
        phases = ((.5, 1.), (.8, pull_force), (1., 0.)) if case == "extraction" else ((.5, -1.), (.8, -push_force), (2., 0.))
        if case.startswith("partial"):
            sign = -1 if case == "partial_insertion" else 1
            phases = ((.5, sign), (.51, pull_force*sign), (.8, 0.), (1., 0.))
        # Let initial collision penetration settle before measuring load-induced
        # motion. The friction rule does not remove or rewrite real contacts.
        phases = ((.2, 0.), *phases)
        samples, trace, frame = [], [], 0
        for end, force in phases:
            for name, drive in drives.items():
                drive.write(messaging.CmdForceInertialMsgPayload(
                    forceRequestInertial=(seats[name].rotation_body[:, 2] * force).tolist()))
            while frame <= int(round(end * DYNAMICS_HZ)):
                sim.ConfigureStopTime(tick_time_ns(frame, DYNAMICS_HZ))
                sim.ExecuteSimulation()
                trace.append(dict(time_s=frame/DYNAMICS_HZ, applied_axial_force_n=force, bodies=snapshot()))
                frame += 1
            samples.append(dict(time_s=end, applied_axial_force_n=force, bodies=snapshot()))
            print(json.dumps(dict(case=case, time_s=end, axial_positions_m={
                name: row["relative_displacement_m"][2] for name, row in samples[-1]["bodies"].items()})), flush=True)
            if not np.isfinite(np.asarray(state().qpos)).all():
                raise RuntimeError("Non-finite production state")
        checks = {}
        for name in retention.models:
            baseline, a, b, c = [entry["bodies"][name] for entry in samples[:4]]
            z = lambda row: row["relative_displacement_m"][2]
            checks[name] = (dict(small_load_holds=abs(z(a)-z(baseline)) < .00005 and a["status"] != "free",
                                 large_load_releases=z(b) > .008 and b["status"] == "free",
                                 released_keeps_moving=z(c) > z(b) and c["status"] == "free")
                             if case == "extraction" else
                             dict(small_push_resisted=z(a) > .004,
                                  large_push_crosses=z(b) < .004,
                                  seated_after_release=abs(z(c)) < retention.parameters.seated_tolerance_m
                                                       and c["status"] == "retained"))
            if case.startswith("partial"):
                d = samples[4]["bodies"][name]
                checks[name] = dict(small_load_holds=abs(z(a)-z(baseline)) < .00005 and a["status"] != "free",
                                    stronger_force_moves=sign*(z(b)-z(a)) > .00001,
                                    stops_inside_stroke=0 < z(c) < .008,
                                    no_axial_return=abs(z(d)-z(c)) < .00005,
                                    remains_engaged=d["socket"] == seats[name].name)
            rows = [row["bodies"][name] for row in trace]
            sliding = [row for row in rows if row["friction_mode"] == "sliding"]
            checks[name]["friction_bounded"] = all(abs(row["axial_friction_force_n"]) <=
                                                    retention.parameters.static_friction_n + 1e-6 for row in rows)
            checks[name]["sliding_force_constant"] = bool(sliding) and all(
                abs(abs(row["axial_friction_force_n"])-retention.parameters.sliding_friction_n) < 1e-6
                for row in sliding)
        unchanged = original_hash == hashlib.sha256(model.read_bytes()).hexdigest()
        return dict(case=case, model=str(model), model_sha256=original_hash, model_unchanged=unchanged,
                    configuration=retention.configuration(), physics_hz=DYNAMICS_HZ, substeps=stepper.substeps,
                    orbital_physics=environment_metadata(physics, orbit.earth, epoch_utc=orbit.epoch_utc),
                    orbital_environment=stepper.telemetry()["orbital_environment"],
                    dynamics=stepper.telemetry(),
                    elapsed_wall_s=time.perf_counter()-started, samples=samples, trace=trace, checks=checks,
                    passed=unchanged and all(value for item in checks.values() for value in item.values()))
    finally:
        stepper.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", type=Path, help="Optional generated layout model to validate")
    parser.add_argument("--case", choices=("extraction", "insertion", "partial_insertion", "partial_extraction"),
                        action="append", help="Repeat to select cases; default runs all four")
    args = parser.parse_args()
    report = dict(scope="Production geometry, free base, orbit and contacts; direct force drives, no robot or UE test",
                  cases=[validate_case(case, args.model) for case in (args.case or ("extraction", "insertion", "partial_insertion", "partial_extraction"))])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(dict(report=str(args.output), passed=all(case["passed"] for case in report["cases"]))))
    return 0 if all(case["passed"] for case in report["cases"]) else 1


if __name__ == "__main__":
    raise SystemExit(main())
