"""Load the original articulated MJCF and verify a flight handoff without a state jump."""

import argparse
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from Basilisk.simulation import mujoco
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.orbital_rendezvous.handoff import apply_handoff, load_handoff
from simulation.orbital_rendezvous.model import DEFAULT_MODEL


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("handoff", type=Path)
    args = parser.parse_args()
    source = ROOT / DEFAULT_MODEL
    data = load_handoff(args.handoff, source)
    xml = ET.parse(source).getroot()
    files = [
        str(
            (
                source.parent / xml.find("compiler").get("meshdir", "") / m.get("file")
            ).resolve()
        )
        for m in xml.findall("./asset/mesh[@file]")
    ]
    scene = mujoco.MJScene.fromFile(str(source), files=files)
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("handoffVerification")
    process.addTask(simulation.CreateNewTask("handoff", macros.sec2nano(0.002)))
    simulation.AddModelToTask("handoff", scene)
    simulation.InitializeSimulation()
    epoch = apply_handoff(scene, data)
    def joint_message_error():
        errors = []
        for item in data["stowed_joints"]:
            joint = scene.getBody(item["body"]).getScalarJoint(item["joint"]["name"])
            errors.extend([abs(joint.stateOutMsg.read().state-item["position"]),
                           abs(joint.stateDotOutMsg.read().state-item.get("velocity", 0.))])
        return max(errors, default=0.)
    unpublished_joint_error = joint_message_error()
    scene.postIntegration(0)
    scene.UpdateState(0)
    initial_joint_message_error = joint_message_error()
    simulation.ConfigureStopTime(0)
    simulation.ExecuteSimulation()
    errors = []
    for b in data["state"]["bodies"]:
        name = (
            "cubesat_bus" if b["body_id"] == "rendezvous/chaser" else "capture_target"
        )
        p = scene.getBody(name).getOrigin().stateOutMsg.read()
        errors.append(
            {
                "body": name,
                "position_m": float(
                    np.linalg.norm(np.array(p.r_BN_N) - b["position_m"])
                ),
                "velocity_m_s": float(
                    np.linalg.norm(np.array(p.v_BN_N) - b["velocity_m_s"])
                ),
                "attitude_dcm": float(
                    np.linalg.norm(rbk.MRP2C(p.sigma_BN) - rbk.EP2C(b["attitude_wxyz"]))
                ),
                "angular_velocity_rad_s": float(
                    np.linalg.norm(np.asarray(p.omega_BN_B) - b["angular_velocity_rad_s"])
                ),
            }
        )
    joint_errors = []
    for item in data["stowed_joints"]:
        joint = scene.getBody(item["body"]).getScalarJoint(item["joint"]["name"])
        joint_errors.append(
            abs(float(joint.stateOutMsg.read().state) - item["position"])
        )
        joint_errors.append(abs(float(joint.stateDotOutMsg.read().state) - item.get("velocity", 0.0)))
    for wheel in data["reaction_wheels"]:
        joint = scene.getBody(wheel["body"]).getScalarJoint(wheel["joint"])
        joint_errors.append(abs(float(joint.stateOutMsg.read().state) - wheel["position_rad"]))
        joint_errors.append(
            abs(float(joint.stateDotOutMsg.read().state) - wheel["speed_rad_s"])
        )
    passed = (
        all(
            max(e["position_m"], e["velocity_m_s"], e["attitude_dcm"], e["angular_velocity_rad_s"]) < 1e-8
            for e in errors
        )
        and max(joint_errors) < 1e-8
        and initial_joint_message_error < 1e-8
    )
    result = {
        "passed": passed,
        "restored_original_model": DEFAULT_MODEL.as_posix(),
        "epoch_s": epoch,
        "body_errors": errors,
        "max_joint_error": max(joint_errors),
        "unpublished_joint_error": unpublished_joint_error,
        "initial_joint_message_error": initial_joint_message_error,
        "scope": "native articulated model initialization continuity, not subsequent near-field control",
    }
    (args.handoff.parent / "handoff_validation.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
