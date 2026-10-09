"""Headless single-pass IK/optional Basilisk dynamics validation. No network or live-scene control.

Run with the project's Basilisk Python, optionally --physics --scene-instance PATH.
The physical run is an independent scene without renderer/orbital ephemerides.
"""
from pathlib import Path
import argparse
import json
import sys
from collections import Counter
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]
from simulation.teleop_grasp_unreal import (CartesianTeleopTarget, CartesianIkControlModel,
    ARM_JOINT_NAMES, SARM_JOINT_NAMES, TELEOP_ARM_KP, TELEOP_ARM_KD, TELEOP_ARM_TORQUE_LIMIT)
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.control_defaults import BALANCED_TELEOP_HOME
from space_arm_platform.joint_limits import load_joint_limits


class Client:
    def __init__(self):
        self.command = np.zeros(6)
        self.deadman = False
        self.grip = 0.
    def latest_action(self):
        return {"deadman": self.deadman, "server_sequence": "1",
            "end_effector_linear_velocity_body_m_s": self.command[:3].tolist(),
            "end_effector_angular_velocity_body_rad_s": self.command[3:].tolist(),
            "gripper_velocity_m_s": self.grip}, False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--physics", action="store_true")
    parser.add_argument("--ik-mode", choices=("ik_pose", "strict"), default="ik_pose")
    parser.add_argument("--scene-instance", type=Path)
    parser.add_argument("--adapter-root", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--seconds-per-command", type=float, default=1.)
    args = parser.parse_args()
    model = ROOT / "model/SARM/platform/sarm_ground_target_self_collision.xml"
    initial = np.array(BALANCED_TELEOP_HOME, float)
    instance = None
    if args.scene_instance:
        instance = json.loads(args.scene_instance.read_text(encoding="utf-8-sig"))
        initial = np.array(instance["randomization"]["arm_joint_position_rad"], float)
        model = ROOT / instance["capture_target"]["runtime_model"]
    kin = SerialChainKinematics.from_mjcf(model,base_body="cubesat_bus",joint_names=ARM_JOINT_NAMES,tool_site="sarm_ee")
    client = Client()
    target = CartesianTeleopTarget(initial, client, kin, ik_mode=args.ik_mode,
                                  joint_limits=load_joint_limits(model,SARM_JOINT_NAMES))
    target.reset(0.)
    simulation = scene = None
    keep_alive = []
    if args.physics:
        if not args.adapter_root:
            parser.error("--physics requires --adapter-root")
        sys.path.insert(0,str(args.adapter_root / "Unreal/BskUnrealRenderer/examples"))
        from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
        from Basilisk.utilities import macros
        native = load_native_grasp_module(model.parent)
        native.MODEL_PATH = model
        native.PREGRASP = initial.copy()
        native.TIME_STEP = .002
        native.KP[:6], native.KD[:6], native.TORQUE_LIMITS[:6] = TELEOP_ARM_KP, TELEOP_ARM_KD, TELEOP_ARM_TORQUE_LIMIT
        if instance:
            native.TARGET_POS = np.array(instance["randomization"]["target_position_m"])
            native.TARGET_QUAT = np.array(instance["randomization"]["target_orientation_wxyz"])
        native.JointTrajectoryPublisher.reference = classmethod(lambda cls,seconds: target.cached_reference(seconds))
        simulation, scene, models, recorders = native._build_simulation(attitude_control_enabled=False)
        control = CartesianIkControlModel(target)
        task = simulation.CreateNewTask("validationIkTask", macros.sec2nano(.01))
        next(p for p in simulation.procList if p.Name == "graspProcess").addTask(task,100)
        simulation.AddModelToTask("validationIkTask",control)
        native._initialize_state(simulation,scene)
        joints = [scene.getBody(body).getScalarJoint(name) for body,name in native.JOINTS[:6]]
        target.bind_joint_state_provider(lambda: np.array([j.stateOutMsg.read().state for j in joints]))
        target.bind_joint_velocity_provider(lambda: np.array([j.stateDotOutMsg.read().state for j in joints]))
        bytag={m.ModelTag:m for m in models}
        target.bind_actuator_state_provider(lambda: {
            "requested_torque_nm":[float(bytag[f"{name}PID"].outputOutMsg.read().input) for name in ARM_JOINT_NAMES],
            "applied_torque_nm":[float(bytag[f"{name}TorqueLimiter"].actuatorOutMsg.read().input) for name in ARM_JOINT_NAMES],
            "torque_limits_nm":TELEOP_ARM_TORQUE_LIMIT.tolist()})
        keep_alive=[models,recorders,control]
    else:
        target.bind_joint_state_provider(lambda: target.position[:6].copy())
        target.bind_joint_velocity_provider(lambda: target.velocity[:6].copy())
    elapsed=0.
    times=[]
    rows=[]
    commands=[("settle",np.zeros(6))]
    for axis in range(6):
        for sign in (1,-1):
            command=np.zeros(6); command[axis]=sign*(.02 if axis<3 else .1)
            commands.append((f"{'xyzRPY'[axis]}{sign:+}",command))
    commands.extend([("combined",np.array([.01,-.01,.01,.03,-.02,.02])),("release",np.zeros(6))])
    commands.extend([("gripper_open",np.zeros(6)),("gripper_close",np.zeros(6))])
    for label,command in commands:
        client.command=command
        client.grip = .005 if label == "gripper_open" else -.005 if label == "gripper_close" else 0.
        client.deadman=bool(np.any(command) or client.grip)
        solves_before=target.ik_solve_count
        arm_before=target.position[:6].copy()
        max_arm_change=0.
        states=Counter(); ratios={"linear":[],"angular":[]}; reasons=Counter()
        pos_error=[]; rot_error=[]
        for step in range(round(args.seconds_per_command/.01)):
            elapsed+=.01
            if args.physics:
                simulation.ConfigureStopTime(macros.sec2nano(elapsed))
                simulation.ExecuteSimulation()
            else:
                target.update(elapsed)
            max_arm_change=max(max_arm_change,float(np.max(np.abs(target.position[:6]-arm_before))))
            times.append(target.solve_time_ms)
            states.update([target.solver_status])
            pos_error.append(float(np.linalg.norm(target.position_error)))
            rot_error.append(float(np.linalg.norm(target.orientation_error)))
            d=target.speed_monitor.latest
            for key in ratios:
                if d[key] and d[key].get("ratio") is not None and step>=35:
                    ratios[key].append(d[key]["ratio"])
                for reason in (d[key] or {}).get("reasons",[]): reasons.update([reason["code"]])
        solve_count=target.ik_solve_count-solves_before
        if label in ("settle","release","gripper_open","gripper_close"):
            assert solve_count == 0, f"{label}: idle arm unexpectedly ran IK"
            assert max_arm_change == 0., f"{label}: arm reference moved without arm input"
        rows.append(dict(command=label,ik_solves=solve_count,max_arm_reference_change_rad=max_arm_change,status_counts=dict(states),
                         median_speed_ratio={key:float(np.median(v)) if v else None for key,v in ratios.items()},
                         max_position_error_m=max(pos_error),max_orientation_error_rad=max(rot_error),
                         warning_evidence=dict(reasons)))
    report=dict(ik_mode=args.ik_mode, mode="independent_basilisk_dynamics" if args.physics else "ideal_kinematic_tracking",
                model=str(model.relative_to(ROOT)),seconds=elapsed,
                solve_time_ms={"median":float(np.median(times[1:])),"p99":float(np.percentile(times[1:],99)),"max":max(times)},
                cases=rows,notes="No renderer/network. Not a full orbital/contact-safety certification.")
    encoded=json.dumps(report,indent=2,ensure_ascii=False)
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(encoded,encoding="utf-8")
    print(encoded)
    _=keep_alive


if __name__ == "__main__":
    main()
