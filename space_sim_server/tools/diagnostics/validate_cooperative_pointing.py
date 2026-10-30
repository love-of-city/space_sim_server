"""Offline torque-driven pre-alignment fixture; no UE or mission release.

Uses the authored target assembly's zero-pose mass/COM/inertia as one rigid
body. Gravity, joints, contact and physical AOCS actuators are excluded. GNSS
has the shared receiver-solution noise model; attitude telemetry is ideal.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SERVER), str(SERVER / "backend")]

from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros

from space_arm_platform.gnss_config import GNSSConfig
from simulation.cooperative_pointing import (
    bounded_pointing_torque, camera_facing_reference, pointing_status,
)
from simulation.differential_gnss import DifferentialGNSS
from simulation.native_integration import configure_scene_integrator
from simulation.near_field.navigation import RelativeEKF
from simulation.orbital_rendezvous.model import assembly_properties

DT = .05
PHYSICS_PERIOD_NS = macros.sec2nano(1 / 240)
CONTROL_PERIOD_NS = 12 * PHYSICS_PERIOD_NS
# Illustrative frozen-arm installation, NOT imported from the live camera model.
CAMERA = np.array([.4, -.15, .35])
MARKER = np.array([.1755, .05475, .0420787001299893])
RELATIVE = np.array([17.4, .3, -.2])


def reference(relative):
    return camera_facing_reference(relative_position_n_m=relative,
        service_rotation_n_from_body=np.eye(3), camera_midpoint_service_m=CAMERA,
        marker_center_target_m=MARKER, up_n=[0, 0, 1])


def fixture(path, properties):
    root = ET.Element("mujoco", model="cooperative_pointing_rigid_fixture")
    ET.SubElement(root, "option", gravity="0 0 0", timestep=str(DT / 12))
    world = ET.SubElement(root, "worldbody")
    for name, mass, center, inertia in [
        ("service", 170., [0, 0, 0], np.diag([5., 6., 9.])),
        ("target", properties["mass_kg"], properties["center_body_m"],
         np.array(properties["inertia_com_body_kg_m2"])),
    ]:
        body = ET.SubElement(world, "body", name=name,
                             pos="0 0 0" if name == "service" else "17.4 .3 -.2")
        ET.SubElement(body, "freejoint", name=f"{name}_free")
        values = inertia[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]
        ET.SubElement(body, "inertial", mass=str(mass),
                      pos=" ".join(map(str, center)),
                      fullinertia=" ".join(map(str, values)))
        ET.SubElement(body, "geom", type="sphere", size=".1", contype="0", conaffinity="0")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def sample_gnss(sensor, readers, now):
    # Translation truth is consumed only by the GNSS sensor simulator.
    states = [np.r_[p.r_BN_N, p.v_BN_N] for p in [reader() for reader in readers]]
    raw = sensor.measure(states, now)
    return dict(source="gnss", stamp=raw["stamp"], z=raw["relative"],
                R=raw["relative_covariance"])


def run_case(model_path, destination, angle_deg, spin):
    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("pointing")
    process.addTask(sim.CreateNewTask("physics", PHYSICS_PERIOD_NS))
    scene = mujoco.MJScene.fromFile(str(model_path))
    configure_scene_integrator(scene)
    scene.addTorqueActuator("pointing_torque", "target_origin")
    torque_msg = messaging.TorqueAtSiteMsg()
    scene.getTorqueActuator("pointing_torque").torqueInMsg.subscribeTo(torque_msg)
    torque_msg.write(messaging.TorqueAtSiteMsgPayload())
    sim.AddModelToTask("physics", scene)
    sim.InitializeSimulation()
    try:
        axis = np.array([1., 2., 3.]) / np.sqrt(14.)
        q = np.r_[np.cos(np.deg2rad(angle_deg)/2), axis*np.sin(np.deg2rad(angle_deg)/2)]
        initial_rotation = reference(RELATIVE) @ rbk.EP2C(q).T
        # Physical initialization only; there are no state setters in the loop.
        target = scene.getBody("target")
        target.setAttitude(rbk.C2MRP(initial_rotation.T).tolist())
        target.setAttitudeRate(list(spin))
        # Keep COM initially at rest when the body's reference origin is offset.
        source_root = ET.parse(model_path).getroot()
        center = np.fromstring(source_root.find('./worldbody/body[@name="target"]/inertial').get("pos"), sep=" ")
        target.setVelocity((-initial_rotation @ np.cross(spin, center)).tolist())
        scene.postIntegration(0)
        scene.UpdateState(0)
        sim.ConfigureStopTime(0)
        sim.ExecuteSimulation()
        readers = [scene.getBody(n).getOrigin().stateOutMsg.addSubscriber()
                   for n in ("target", "service")]
        sensor = DifferentialGNSS(GNSSConfig(), seed=7)
        nav = None
        dwell = 0.
        dwell_start = None
        completed = None
        max_torque = 0.
        rows = []
        last_navigation = 0.
        for k in range(2401):
            nanos = k * CONTROL_PERIOD_NS
            now = nanos * 1e-9
            if k:
                sim.ConfigureStopTime(nanos)
                sim.ExecuteSimulation()
                nav.predict(CONTROL_PERIOD_NS * 1e-9, np.zeros(3), np.zeros((3, 3)))
            if k % 2 == 0:
                packet = sample_gnss(sensor, readers, now)
                if nav is None:
                    # The body origin is offset from COM: a slew accelerates
                    # that origin. Use mainline's unsettled-flight process
                    # noise, not the quiet, fixed-attitude navigation default.
                    nav = RelativeEKF(packet["z"], packet["R"], acceleration_sigma_m_s2=.05)
                    last_navigation = now
                elif nav.update(packet, now):
                    last_navigation = now
            desired = reference(nav.x[:3])
            # Separate, explicitly ideal cooperative attitude/gyro telemetry.
            telemetry = readers[0]()
            current = rbk.MRP2C(telemetry.sigma_BN).T
            omega = np.array(telemetry.omega_BN_B)
            error_deg = float(np.rad2deg(np.arccos(np.clip((np.trace(current.T@desired)-1)/2, -1, 1))))
            status = pointing_status(now_s=now,
                input_stamps_s=dict(navigation=last_navigation, service_attitude=now,
                                    target_telemetry=now, camera_kinematics=now),
                target_rotation_n_from_body=current, reference_rotation_n_from_body=desired,
                target_angular_rate_body_rad_s=omega, reference_angular_rate_n_rad_s=[0, 0, 0],
                navigation_position_sigma_m=nav.sigma_m)
            torque = bounded_pointing_torque(target_rotation_n_from_body=current,
                reference_rotation_n_from_body=desired, target_angular_rate_body_rad_s=omega,
                reference_angular_rate_n_rad_s=[0, 0, 0])
            payload = messaging.TorqueAtSiteMsgPayload()
            payload.torque_S = torque.tolist()
            torque_msg.write(payload, nanos)
            max_torque = max(max_torque, float(np.linalg.norm(torque)))
            if status.ready:
                if dwell_start is None:
                    dwell_start = now
                dwell = now - dwell_start
            else:
                dwell_start = None
                dwell = 0.
            rows.append(dict(time_s=now, attitude_error_deg=error_deg,
                angular_rate_rad_s=float(np.linalg.norm(omega)), sigma_m=nav.sigma_m,
                torque_body_nm=torque.tolist(), instantaneous_ready=status.ready,
                reason=status.reason, dwell_s=dwell))
            if dwell >= 5. - 1e-9:
                completed = now
                break
        # A fresh aligned orientation must cease to be ready on a stale input.
        stale = pointing_status(now_s=now + 1.,
            input_stamps_s=dict(navigation=last_navigation, service_attitude=now+1.,
                                target_telemetry=now+1., camera_kinematics=now+1.),
            target_rotation_n_from_body=current, reference_rotation_n_from_body=desired,
            target_angular_rate_body_rad_s=omega, reference_angular_rate_n_rad_s=[0, 0, 0],
            navigation_position_sigma_m=nav.sigma_m)
        result = dict(initial_error_deg=angle_deg, initial_rate_body_rad_s=list(spin),
            five_second_alignment_completed_s=completed,
            final_error_deg=error_deg,
            final_rate_error_rad_s=float(np.linalg.norm(omega)), max_torque_nm=max_torque,
            gnss_updates=nav.accepted["gnss"], gnss_rejected=nav.rejected,
            state_assignment="initialization only; loop writes torque messages",
            stale_navigation_revokes_readiness=not stale.ready and stale.reason == "input_stale")
        result["passed"] = (completed is not None and max_torque <= .02+1e-12
            and result["stale_navigation_revokes_readiness"])
        (destination / f"angle_{angle_deg:g}_trace.jsonl").write_text(
            "".join(json.dumps(row)+"\n" for row in rows), encoding="utf-8")
        return result
    finally:
        # SWIG objects own their native scene. Keep all message proxies alive
        # until the simulation finishes; no other dynamics process is touched.
        sim.TotalSim.deleteThreads()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-model", type=Path,
        default=SERVER / "model/SARM/platform/sarm_ground_target_self_collision.xml")
    parser.add_argument("--output", type=Path, default=SERVER / "run/cooperative_pointing")
    args = parser.parse_args()
    source = args.source_model.resolve()
    target = ET.parse(source).find('./worldbody/body[@name="capture_target"]')
    if target is None:
        raise ValueError("source model has no capture_target")
    properties = assembly_properties(target)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    model = output / "rigid_fixture.xml"
    fixture(model, properties)
    results = [run_case(model, output, angle, spin) for angle, spin in [
        (0., [0., 0., 0.]), (30., [0., 0., 0.]),
        (90., [.01, -.015, .005]), (180., [0., 0., 0.]),
    ]]
    summary = dict(scope="rigid, gravity-free native torque fixture; not full mission or visual validation",
        source_model=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        target_properties=properties, controller_hz=20, physics_hz=240,
        integrator="Basilisk MJScene RKF45; not LocalMujocoStepper",
        physics_period_ns=PHYSICS_PERIOD_NS, control_period_ns=CONTROL_PERIOD_NS,
        camera_midpoint_service_m=CAMERA.tolist(), marker_center_target_m=MARKER.tolist(),
        gnss_config=GNSSConfig().to_dict(), telemetry_model="ideal cooperative attitude/rate; frozen service and arm",
        navigation_process_acceleration_sigma_m_s2=.05,
        results=results, passed=all(result["passed"] for result in results))
    (output / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0 if summary["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
