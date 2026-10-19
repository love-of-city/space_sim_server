"""Native articulated closed-loop demo. Run with --fixture or --handoff PATH."""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import socket
import struct
import sys
import time
import uuid
import xml.etree.ElementTree as ET
from dataclasses import asdict
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np
from Basilisk.simulation import mujoco
from Basilisk.utilities import SimulationBaseClass, macros, RigidBodyKinematics as rbk

from space_arm_platform.rendezvous_config import FlightOptions
from space_arm_platform.near_field_config import NearFieldOptions
from simulation.assembly import AssemblyContext, SessionConfig, Slot
from simulation.attitude_control import AttitudeControl
from simulation.components.orbit import OrbitComponent
from simulation.components.orbital_gravity import OrbitalGravityComponent
from simulation.components.flight_environment import FlightEnvironmentComponent
from simulation.joint_reference_publisher import HeldJointReferencePublisher
from simulation.local_mujoco_stepper import LocalMujocoStepper, servo_specs
from simulation.orbital_rendezvous.assets import bind_team_catalog
from simulation.orbital_rendezvous.handoff import apply_handoff, load_handoff
from simulation.orbital_rendezvous.model import DEFAULT_MODEL
from simulation.orbital_rendezvous.reference import FlightProfile
from simulation.orbital_rendezvous.rendering import flight_scene_settings
from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC, _register_celestial_bodies

from .controller import NearController
from .model import prepare, JOINTS, ACTUATORS, GRASP_TARGET, NOMINAL_TCP
from .navigation import SensorConfig, admit_handoff
from .orbital_guidance import guidance_frame


def fixture(repository, manifest):
    """Clearly labeled 110 m initial-condition fixture; no far-flight acceptance claim."""
    r, v, _ = FlightProfile().target(0.)
    target = np.r_[r, v]
    delta = np.array([110., 0.32242, 0.3029213])
    chaser = np.r_[r - delta, v]
    root = ET.parse(repository / DEFAULT_MODEL).getroot()
    return {"schema": "spacesim-near-field-handoff/1", "accepted": True,
            "source_kind": "synthetic 110 m fixture; not evidence of phases 1-3",
            "guidance_frame": "inertial-fixture",
            "source_sha256": manifest["source_sha256"], "ephemeris_epoch_utc": DEFAULT_EPHEMERIS_EPOCH_UTC,
            "state": {"sim_time_s": 0., "bodies": [
                {"body_id": f"rendezvous/{role}", "position_m": state[:3].tolist(), "velocity_m_s": state[3:].tolist(),
                 "attitude_wxyz": [1, 0, 0, 0], "angular_velocity_rad_s": [0, 0, 0]}
                for role, state in (("target", target), ("chaser", chaser))]},
            "gnss_estimates_eci": [target.tolist(), chaser.tolist()],
            "covariances": [np.diag([0.2**2] * 3 + [0.01**2] * 3).tolist()] * 2,
            "stowed_joints": [{"body": b.get("name"), "joint": dict(j.attrib), "position": 0.}
                              for b in root.iter("body") for j in b.findall("joint")
                              if j.get("type") != "free" and not j.get("name").startswith("rw_")],
            "reaction_wheels": [], "environment_state": [], "mission_options": FlightOptions().to_dict()}


def packet(scene, row, run_id):
    bodies = []
    for name, role in (("capture_target", "target"), ("cubesat_bus", "chaser")):
        p = scene.getBody(name).getOrigin().stateOutMsg.read()
        bodies.append({"body_id": f"rendezvous/{role}", "position_m": list(p.r_BN_N), "velocity_m_s": list(p.v_BN_N),
                       "attitude_wxyz": list(rbk.MRP2EP(p.sigma_BN)), "angular_velocity_rad_s": list(p.omega_BN_B)})
    return {"protocol": "space-sim-state/1", "type": "state", "sim_time_s": row["t"], "bodies": bodies,
            "metadata": {"run_id": run_id, **{k: v for k, v in row.items() if k not in {"truth", "navigation_truth", "covariance", "joints", "joint_reference"}},
                         "navigation_valid": row["communication_valid"], "vision_mode": "parameterized; no left/right images processed",
                         "capture_mode": "logical proxy; no physical connection or three-finger mechanics validation"}}


def run(args):
    repository = Path(__file__).resolve().parents[2]
    output = args.output_dir.resolve()
    model, files, manifest, chain = prepare(repository, output)
    incoming = load_handoff(args.handoff, repository / DEFAULT_MODEL) if args.handoff else fixture(repository, manifest)
    admit_handoff(incoming)
    from space_arm_platform.gnss_config import GNSSConfig
    shared_gnss = incoming.get("mission_options", {}).get("differential_gnss")
    config = SensorConfig(seed=args.seed, loss_after_phase=args.vision_loss_phase or "", communication_loss_s=args.communication_loss,
                          receiver_config=GNSSConfig.from_dict(shared_gnss) if shared_gnss is not None else None)
    (output / "input_handoff.json").write_text(json.dumps(incoming, indent=2))
    (output / "mission_options.json").write_text(json.dumps(NearFieldOptions(args.seed, args.vision_loss_phase or "", args.communication_loss).to_dict(), indent=2))
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("nearField")
    process.addTask(sim.CreateNewTask("nearDynamics", macros.sec2nano(0.05)), 30)
    process.addTask(sim.CreateNewTask("nearInit", macros.sec2nano(0.05)))
    scene = mujoco.MJScene.fromFile(str(model), files=files)
    scene.extraEoMCall = True
    sim.AddModelToTask("nearInit", scene)
    ctx = AssemblyContext(sim, scene, config=SessionConfig(model_path=model, dynamics_backend="local", publish_stride=1,
                                                         clock_rate_hz=20., physics_task="nearDynamics"))
    epoch = datetime.strptime(incoming["ephemeris_epoch_utc"], "%Y %B %d %H:%M:%S.%f") + timedelta(seconds=incoming["state"]["sim_time_s"])
    epoch_text = epoch.strftime("%Y %B %d %H:%M:%S.%f")
    orbit = OrbitComponent(epoch_utc=epoch_text)
    ctx.install([orbit])
    stepper = LocalMujocoStepper(scene, model, [], origin=orbit.origin, substeps=12, publish_every=1)
    ctx.ports.set_stepper(stepper)
    ctx.add(stepper, Slot.PHYSICS, component="dynamics_core")
    # A 20 Hz outer task advances twelve 240 Hz substeps. The teleop default
    # holds gravity for eight outer steps and is not appropriate for this clock.
    ctx.install([OrbitalGravityComponent(force_every=1)])
    env_options = FlightOptions.from_dict(incoming.get("mission_options", {}))
    environment = FlightEnvironmentComponent(env_options, [
        ("capture_target", "target_flight_center", manifest["bodies"]["capture_target"]["mass_kg"], env_options.target_area_m2),
        ("cubesat_bus", "orbital_thrust_center", manifest["bodies"]["cubesat_bus"]["mass_kg"], env_options.chaser_area_m2)])
    ctx.install([environment])
    attitude = AttitudeControl(sim, process, scene, model, drive_task=("nearDynamics", int(Slot.ACTUATOR)),
                              reference_provider=(lambda t: control.attitude_reference(t))
                              if guidance_frame(incoming) == "orbital-vbar" else None)
    control = NearController(scene, ctx.ports, chain, manifest["bodies"]["cubesat_bus"]["mass_kg"], incoming, config, model)
    refs = HeldJointReferencePublisher(control.reference, 8)
    specs = servo_specs(ACTUATORS, JOINTS, [32, 32, 32, 30, 30, 15, 100, 100],
                         [2, 2, 2, .7, .5, .25, 1, 1], [100] * 6 + [.5, .5], refs)
    for spec in specs:
        stepper.connect_servo(spec)
    ctx.add(control, Slot.COMMAND, component="near_control")
    ctx.add(refs, Slot.COMMAND, component="near_joint_reference")
    bridge = None
    if args.catalog:
        from bsk_render_adapter import BasiliskRenderBridge
        from bsk_render_adapter.protocol import RecordingOnlyPublisher
        from bsk_render_adapter.near_field import publish_near_field_status
        catalog = bind_team_catalog(args.catalog, repository, manifest, output)
        bridge = BasiliskRenderBridge(host="127.0.0.1", port=args.render_port, origin_object="team/cubesat_bus",
                    frame_period_ns=macros.sec2nano(0.1), publisher=None if args.live else RecordingOnlyPublisher(),
                    recording_path=output / "near_field.bskrec")
        bridge.add_mj_scene(scene, namespace="team", source_path=model, mesh_asset_catalog=catalog, camera_picture_in_picture=False)
        _register_celestial_bodies(bridge, orbit.earth, orbit.sun)
        settings = flight_scene_settings(bodies=manifest["bodies"])
        # Presentation-only light makes the unchanged meshes visible in eclipse.
        # It does not illuminate simulated sensor images (none are processed),
        # alter SRP, or claim physical visual-navigation availability in darkness.
        settings.fill_light_intensity_lux = 100000.
        settings.headlight_enabled = True
        settings.headlight_diffuse_rgb = (10000., 10000., 10000.)  # display gain at the project's fixed exposure
        bridge.set_scene_settings(settings)
        sim.AddModelToTask("nearDynamics", bridge, -10000)
    sim.InitializeSimulation()
    sim.disableTask("nearInit")
    apply_handoff(scene, incoming)  # The only physical state restore, before propagation.
    by_id = {b["body_id"]: b for b in incoming["state"]["bodies"]}
    target = by_id["rendezvous/target"]
    orbit.origin.set_state(np.array(target["position_m"]), np.array(target["velocity_m_s"]))
    # Publish the restored physical initial condition before the first sensor
    # acquisition. Otherwise subscribers still contain the pre-handoff state.
    scene.postIntegration(0)
    scene.UpdateState(0)  # Publish restored joint/wheel measurements before the first control tick.
    for module, saved in zip(environment.models, incoming.get("environment_state", [])):
        module.restore(saved)
    for i, reader in enumerate(control.joint_readers):
        control.qref[i] = float(reader().state)
    channel = socket.create_connection(("127.0.0.1", args.state_port), timeout=2) if args.state_port else None
    run_id = uuid.uuid4().hex
    started = time.monotonic()
    previous_phase = None
    switch = sys.getswitchinterval()
    sys.setswitchinterval(0.0001)
    status_file = (output / "state_packets.jsonl").open("w", encoding="utf-8")
    continuity = None
    continuity_details = {}
    safe_hold_settled_since = None
    try:
        for tick in range(round(args.duration * 20) + 1):
            sim.ConfigureStopTime(macros.sec2nano(tick * 0.05))
            sim.ExecuteSimulation()
            row = control.rows[-1]
            # Evaluation only, after physics reaches the navigation prediction
            # epoch. Pre-physics sensor/encoder truth remains separately tagged.
            current = [reader() for reader in control.readers]
            current_relative = np.r_[np.array(current[0].r_BN_N)-current[1].r_BN_N,
                                     np.array(current[0].v_BN_N)-current[1].v_BN_N]
            row["navigation_truth"] = current_relative.tolist()
            row["navigation_error_m"] = float(np.linalg.norm(np.array(row["estimate"][:3])-current_relative[:3]))
            if continuity is None:
                for name, role in (("capture_target", "target"), ("cubesat_bus", "chaser")):
                    restored = scene.getBody(name).getOrigin().stateOutMsg.read()
                    saved = by_id[f"rendezvous/{role}"]
                    ep = np.array(rbk.MRP2EP(restored.sigma_BN))
                    saved_ep = np.array(saved["attitude_wxyz"])
                    continuity_details[role] = {
                        "position_m": float(np.linalg.norm(np.array(restored.r_BN_N) - saved["position_m"])),
                        "velocity_m_s": float(np.linalg.norm(np.array(restored.v_BN_N) - saved["velocity_m_s"])),
                        "quaternion": float(min(np.linalg.norm(ep-saved_ep), np.linalg.norm(ep+saved_ep))),
                        "angular_velocity_rad_s": float(np.linalg.norm(np.array(restored.omega_BN_B)-saved["angular_velocity_rad_s"]))}
                continuity = max(item["position_m"] for item in continuity_details.values())
                continuity_details["navigation_state"] = float(np.max(np.abs(np.array(row["estimate"])-control.initial_navigation["state"])))
                continuity_details["covariance"] = float(np.max(np.abs(np.array(row["covariance"])-control.initial_navigation["covariance"])))
                continuity_details["time_s"] = abs(row["t"]-incoming["state"]["sim_time_s"])
                continuity_details["wheel_speed_rad_s"] = max((abs(scene.getBody(w["body"]).getScalarJoint(w["joint"]).stateDotOutMsg.read().state-w["speed_rad_s"]) for w in incoming["reaction_wheels"]), default=0.)
            if tick % 2 == 0:
                state = packet(scene, row, run_id)
                status_file.write(json.dumps(state, allow_nan=False) + "\n")
                if channel:
                    wire = json.dumps(state, allow_nan=False).encode()
                    try:
                        channel.sendall(struct.pack("!I", len(wire)) + wire)
                    except OSError:
                        channel.close()
                        channel = None
            if bridge and (tick % 20 == 0 or row["phase"] != previous_phase):
                publish_near_field_status(bridge, row)
            if row["phase"] != previous_phase or tick % 2000 == 0:
                print(json.dumps({k: row[k] for k in ("elapsed_s", "phase", "platform_to_grasp_m", "tcp_gap_m", "position_sigma_m", "alignment_error_deg")}), flush=True)
                previous_phase = row["phase"]
            if control.phase == "CaptureHold" and row["elapsed_s"] - control.phase_start >= 5:
                break
            # A fixed 20 s timeout may stop while the hold controller is still
            # returning to its setpoint after braking an approach. Require a
            # sustained navigation-speed margin before ending a fault run;
            # independent physical-speed acceptance remains 0.003 m/s.
            if control.phase == "SafeHold" and row["relative_speed_m_s"] < .001:
                if safe_hold_settled_since is None:
                    safe_hold_settled_since = row["elapsed_s"]
            else:
                safe_hold_settled_since = None
            if (control.phase == "SafeHold" and row["elapsed_s"] - control.phase_start >= 20
                    and safe_hold_settled_since is not None
                    and row["elapsed_s"] - safe_hold_settled_since >= 5):
                break
    finally:
        sys.setswitchinterval(switch)
        status_file.close()
        if channel:
            channel.close()
        if bridge:
            bridge.close()
    rows = control.rows
    final = rows[-1]
    phases = [x["phase"] for x in control.transitions]
    normal = not args.vision_loss_phase and args.communication_loss is None
    checks = {"finite_states": bool(np.isfinite([r["truth"] for r in rows]).all()),
              "no_external_state_writes": stepper.external_writes == 0,
              "gravity_at_every_substep": stepper.orbital_force_every == 1,
              "position_continuity_m": continuity < 1e-8,
              "initial_continuity": all(value < 1e-8 for item in continuity_details.values() for value in (item.values() if isinstance(item, dict) else [item])),
              "force_limit": max(np.linalg.norm(r["force_n"]) for r in rows) <= 25.000001,
              "required_terminal": final["phase"] == ("CaptureHold" if normal else "SafeHold"),
              "safe_hold_no_latch": normal or (not control.latch and control.close_inhibited),
              "terminal_speed": final["relative_speed_m_s"] < 0.003}
    if normal:
        checks.update(full_phase_sequence=all(p in phases for p in ("GNSSApproach", "VisualFusion", "VisualApproach", "PlatformHold", "ArmAlign", "FinalApproach", "Grasp", "CaptureConfirm", "CaptureHold")),
                      final_true_lateral=final["truth_lateral_error_m"] < 0.006,
                      final_true_gap=0.01 < final["truth_tcp_gap_m"] < 0.026)
    checks = {key: bool(value) for key, value in checks.items()}
    report = {"schema": "spacesim-near-field/1", "scenario": "near-field", "basic_nominal_passed": all(checks.values()),
              "sensor_clock_contract": "physical-message-epoch/v1",
              "outcome": final["phase"], "checks": checks, "run_id": run_id, "transitions": control.transitions,
              "guidance_frame": guidance_frame(incoming),
              "final_time_s": final["t"], "wall_time_s": time.monotonic() - started,
              "final_center_distance_m": final["center_distance_m"], "final": final,
              "model_manifest": manifest, "mission_options": NearFieldOptions(args.seed, args.vision_loss_phase or "", args.communication_loss).to_dict(), "sensor_config": asdict(config), "local_dynamics": stepper.telemetry(),
              "capture": {"latch_time_s": control.phase_start + control.handoff_time if control.latch else None,
                          "mode": "logical proxy; no physical connection, load transfer or three-finger validation"},
              "vision": "parameterized noisy delayed stereo; no images, detection or PnP executed",
              "handoff_source": str(args.handoff) if args.handoff else incoming["source_kind"],
              "initial_navigation": control.initial_navigation, "initial_physical_position_error_m": continuity,
              "continuity": continuity_details,
              "navigation_updates": control.nav.accepted, "navigation_rejections": control.nav.rejected,
              **({"initial_gnss_transport": incoming["gnss_transport"],
                  "final_gnss_transport": control.sensors.transport_snapshot()}
                 if incoming.get("gnss_transport") is not None else {}),
              "source_hashes": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path(__file__).parent.glob("*.py")},
              "nominal_platform_origin_to_grasp_m": float(np.linalg.norm(NOMINAL_TCP)),
              "nominal_origins_delta_m": (NOMINAL_TCP - GRASP_TARGET).tolist()}
    (output / "mission_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    # Preserve the actual articulated terminal state for the existing operator
    # scene. A logical capture remains labelled as such; no weld is introduced.
    handoff = copy.deepcopy(incoming)
    handoff.update(accepted=normal and all(checks.values()), state=packet(scene, final, run_id),
                   model_configuration="restore recorded articulated joint positions and velocities",
                   source_kind="near-field terminal; capture remains a logical proxy",
                   navigation_reference="body_origin",
                   navigation_sample_time_s=final["t"],
                   environment_state=[module.snapshot() for module in environment.models])
    for item in handoff["stowed_joints"]:
        joint = scene.getBody(item["body"]).getScalarJoint(item["joint"]["name"])
        item["position"] = float(joint.stateOutMsg.read().state)
        item["velocity"] = float(joint.stateDotOutMsg.read().state)
    handoff["reaction_wheels"] = [{"body": wheel.body, "joint": wheel.joint,
        "position_rad": float(joint.stateOutMsg.read().state), "speed_rad_s": float(joint.stateDotOutMsg.read().state)}
        for wheel, joint in zip(attitude.wheels, attitude.joints)]
    # Only relative navigation is updated by this phase. Preserve that estimate
    # exactly; absolute target propagation is explicitly identified, not truth.
    handoff["gnss_estimates_eci"] = [control.target_orbit.tolist(), (control.target_orbit-control.nav.x).tolist()]
    target_P = control.target_covariance
    handoff["covariances"] = [target_P.tolist(), (target_P+control.nav.P).tolist()]
    handoff["gnss_cross_covariance_eci"] = target_P.tolist()
    handoff["navigation_absolute_source"] = "propagated target estimate with conservative uncertainty; relative EKF updated"
    handoff["gnss_receiver_state"] = control.sensors.receiver.snapshot() if control.sensors.receiver else None
    if incoming.get("gnss_transport") is not None:
        handoff["gnss_transport"] = control.sensors.transport_snapshot()
    (output / "handoff.json").write_text(json.dumps(handoff, ensure_ascii=False, indent=2), encoding="utf-8")
    exported = {k: [r[k] for r in rows] for k in (
        "t", "elapsed_s", "phase", "truth", "estimate", "covariance", "force_n", "joints", "joint_reference",
        "tcp_gap_m", "truth_tcp_gap_m", "truth_lateral_error_m", "relative_speed_m_s", "position_sigma_m",
        "vision_valid", "close_inhibited", "navigation_error_m", "center_distance_m", "target_hinge_angle_rad",
        "truth_relative_speed_m_s", "truth_alignment_error_deg", "physical_observation_time_s",
        "navigation_prediction_time_s", "navigation_truth")}
    if incoming.get("gnss_transport") is not None:
        exported.update({key: [np.nan if row[key] is None else row[key] for row in rows] for key in (
            "gnss_sample_time_s", "gnss_receive_time_s", "gnss_measurement_age_s", "gnss_delivered_sequence",
            "gnss_acquired_sequence", "gnss_pending_count", "gnss_delivered_count", "gnss_requested_delay_s")})
    np.savez_compressed(output / "team_telemetry.npz", **exported)
    print(json.dumps({"checks": checks, "output": str(output)}), flush=True)
    stepper.close()
    return 0 if all(checks.values()) else 2


def main():
    p = argparse.ArgumentParser(description=__doc__)
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--fixture", action="store_true")
    source.add_argument("--handoff", type=Path)
    p.add_argument("--output-dir", type=Path, required=True)
    p.add_argument("--duration", type=float, default=2500.)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--config", type=Path)
    p.add_argument("--vision-loss-phase", choices=("VisualApproach", "FinalApproach", "Grasp"))
    p.add_argument("--communication-loss", type=float)
    p.add_argument("--catalog", type=Path)
    p.add_argument("--live", action="store_true")
    p.add_argument("--render-port", type=int, default=5558)
    p.add_argument("--state-port", type=int, default=0)
    args = p.parse_args()
    if args.config:
        options = NearFieldOptions.from_dict(json.loads(args.config.read_text(encoding="utf-8")))
        args.seed, args.vision_loss_phase, args.communication_loss = options.seed, options.vision_loss_phase, options.communication_loss_s
    if args.live and not args.catalog:
        p.error("--live requires --catalog")
    if not np.isfinite(args.duration) or not 0 < args.duration <= 10000 or not 0 <= args.seed < 2**32:
        p.error("duration must be in (0,10000], seed in [0,2**32)")
    if args.communication_loss is not None and (not np.isfinite(args.communication_loss) or args.communication_loss <= 0):
        p.error("communication loss must be finite and positive")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
