"""Independent telemetry/kinematics acceptance; no Basilisk import required.

Run --geometry in a Python-MuJoCo process, separately from native Basilisk.
All generated evidence belongs in an ignored run/ or validation/ directory.
"""
import argparse
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.near_field.model import prepare, JOINTS, GRASP_TARGET  # noqa: E402


def rotation(ep):
    w, x, y, z = ep
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def geometry():
    import mujoco
    with tempfile.TemporaryDirectory() as scratch:
        path, _, manifest, chain = prepare(ROOT, Path(scratch))
        m = mujoco.MjModel.from_xml_path(str(path))
        d = mujoco.MjData(m)
        plan = manifest["approach_plan"]
        nominal = np.array(plan["nominal_grasp_bus_m"])
        stowed, tool_rotation = chain.forward(np.zeros(6))
        axis = tool_rotation[:, 2]
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.jnt_bodyid[j])
                adr = m.jnt_qposadr[j]
                d.qpos[adr:adr+3] = nominal-GRASP_TARGET if name == "capture_target" else [0, 0, 0]
                d.qpos[adr+3:adr+7] = [1, 0, 0, 0]
        addresses = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS]
        d.qpos[addresses[6:]] = .0375
        from simulation.pose_ik import _iterate, ElbowPreference
        from simulation.near_field.navigation import stereo_geometry
        limits = np.array([3.1416] * 5 + [6.2832])
        pre_goal, end_goal = nominal-.15*axis, nominal-.018*axis
        pre = _iterate(chain, np.deg2rad([76.15, -64.33, 44.38, -70.05, 76.15, -90]),
                       pre_goal, tool_rotation, -limits, limits, ElbowPreference())
        end = None if pre is None else _iterate(chain, pre, end_goal, tool_rotation, -limits, limits, ElbowPreference())
        if pre is None or end is None:
            return {"passed": False, "checks": {"reachable": False}}
        contacts = []
        for start, stop in ((np.zeros(6), pre), (pre, end)):
            for fraction in np.linspace(0, 1, 201):
                d.qpos[addresses[:6]] = start*(1-fraction)+stop*fraction
                mujoco.mj_forward(m, d)
                contacts.append(d.ncon)
        checks = {"authored_physics_unchanged": manifest["joints_inertias_geometries_unchanged"],
                  "sampled_configuration_contacts_zero": max(contacts) == 0,
                  "stowed_tcp_half_meter": abs(axis @ (nominal-stowed)-.5) < 1e-10,
                  "both_marker_images_at_standoff": stereo_geometry(tool_rotation.T @ nominal)["valid"],
                  "pregrasp_reachable": np.linalg.norm(chain.forward(pre)[0]-pre_goal) < 1e-4,
                  "endpoint_reachable": np.linalg.norm(chain.forward(end)[0]-end_goal) < 1e-4}
        checks = {key: bool(value) for key, value in checks.items()}
        return {"checks": checks, "passed": all(checks.values()), "samples": len(contacts),
                "max_contacts": int(max(contacts)), "approach_plan": plan,
                "pregrasp_rad": pre.tolist(), "endpoint_rad": end.tolist(),
                "scope": "static samples only; not continuous collision proof or contact mechanics"}


def verify(folder):
    folder = folder.resolve()
    # NPZ is lazy: cache once, rather than decompressing full arrays per packet.
    with np.load(folder/"team_telemetry.npz") as source:
        data = {key: source[key] for key in source.files}
    report = json.loads((folder/"mission_report.json").read_text())
    from simulation.serial_chain_kinematics import SerialChainKinematics
    chain = SerialChainKinematics.from_mjcf(ROOT/report["model_manifest"]["source_model"],
                                          base_body="cubesat_bus", joint_names=JOINTS[:6], tool_site="sarm_ee")
    zero_r = chain.forward(np.zeros(6))[1]
    # Independently reconstruct TCP/grasp from source FK, physical packets and hinge encoders.
    evaluated = []
    clock_aligned = report.get("sensor_clock_contract") == "physical-message-epoch/v1"
    navigation_samples_match = []
    physical_samples_match = []
    optical_samples_match = []
    gap_samples_match = []
    for line in (folder/"state_packets.jsonl").open(encoding="utf-8"):
        packet = json.loads(line)
        i = int(round((packet["sim_time_s"]-data["t"][0])*20))
        bodies = {b["body_id"]: b for b in packet["bodies"]}
        a, b = bodies["rendezvous/target"], bodies["rendezvous/chaser"]
        if clock_aligned:
            physical = np.array([np.r_[body["position_m"], body["velocity_m_s"]] for body in (a, b)])
            navigation_samples_match.append(
                abs(data["navigation_prediction_time_s"][i]-packet["sim_time_s"]) < 1e-8
                and np.allclose(data["navigation_truth"][i], physical[0]-physical[1], rtol=0, atol=1e-8))
            if i+1 < len(data["t"]):
                physical_samples_match.append(
                    abs(data["physical_observation_time_s"][i+1]-packet["sim_time_s"]) < 1e-8
                    and np.allclose(data["truth"][i+1], physical, rtol=0, atol=1e-8))
        # Guidance runs before physics; packets are published after physics.
        # The NEXT guidance row therefore contains this packet's encoder sample.
        # Omit the very last packet, whose matching next row was not recorded.
        if i+1 >= len(data["t"]):
            continue
        i += 1
        Ra, Rb = rotation(a["attitude_wxyz"]), rotation(b["attitude_wxyz"])
        hinge = data["target_hinge_angle_rad"][i]
        c, s = np.cos(hinge), np.sin(hinge)
        panel = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        local = np.array([-.0145, -.07702, .04707870013])+panel@[-.36, .0046, 0]
        tcp, Rt = chain.forward(data["joints"][i, :6])
        e = np.array(a["position_m"])-b["position_m"]+Ra@local-Rb@tcp
        if "truth_optical_depth_m" in data:
            optical_depth = (Rb@zero_r)[:, 2] @ (e+Rb@tcp) + .10
            optical_samples_match.append(abs(optical_depth-data["truth_optical_depth_m"][i]) < 1e-8)
        goal_r = Ra@panel@zero_r
        axis = goal_r[:, 2]
        h = axis@e
        gap_samples_match.append(abs(h-data["truth_tcp_gap_m"][i]) < 1e-8)
        cosine = np.clip((np.trace(goal_r@(Rb@Rt).T)-1)/2, -1, 1)
        evaluated.append([h, np.linalg.norm(e-h*axis), np.rad2deg(np.arccos(cosine))])
        evaluated_index, evaluated_time = i, packet["sim_time_s"]
    h, lateral, angle = evaluated[-1]
    true_relative = data["truth"][:, 0]-data["truth"][:, 1]
    navigation_truth = data["navigation_truth"] if clock_aligned else true_relative
    error = np.linalg.norm(data["estimate"][:, :3]-navigation_truth[:, :3], axis=1)
    inertial_speed = float(np.linalg.norm(true_relative[-1, 3:]))
    velocity = true_relative[-1, 3:]
    if report.get("guidance_frame") == "orbital-vbar":
        target = data["truth"][-1, 0]
        omega = np.cross(target[:3], target[3:]) / (target[:3] @ target[:3])
        velocity = velocity - np.cross(omega, true_relative[-1, :3])
    speed = float(np.linalg.norm(velocity))
    phases = list(dict.fromkeys(data["phase"].tolist()))
    normal = not report["mission_options"]["vision_loss_phase"] and report["mission_options"]["communication_loss_s"] is None
    checks = {"finite": bool(all(np.isfinite(data[k]).all() for k in ("truth", "estimate", "covariance", "force_n", "joints"))),
              "time_contiguous": bool(np.allclose(np.diff(data["t"]), .05, atol=1e-8)),
              "covariance_psd": bool(np.linalg.eigvalsh(data["covariance"]).min() >= -1e-10),
              "force_bounded": bool(np.linalg.norm(data["force_n"], axis=1).max() <= 25.000001),
              "terminal_true_speed": speed < .003,
              "independent_gap_matches_telemetry": abs(h-data["truth_tcp_gap_m"][evaluated_index]) < 1e-8,
              "independent_navigation_matches": bool(np.allclose(error, data["navigation_error_m"], atol=1e-8))}
    if clock_aligned:
        checks.update(physical_acquisition_epochs=bool(physical_samples_match) and all(physical_samples_match)
                      and np.allclose(data["physical_observation_time_s"],
                                      data["t"][0]+np.maximum(0, data["elapsed_s"]-.05), rtol=0, atol=1e-8),
                      navigation_evaluated_at_prediction_epoch=bool(navigation_samples_match) and all(navigation_samples_match)
                      and np.allclose(data["navigation_prediction_time_s"], data["t"], rtol=0, atol=1e-8)
                      and np.isfinite(navigation_truth).all())
    if normal:
        capture = np.flatnonzero(data["phase"] == "CaptureHold")
        required = ["FarHandoff", "GNSSApproach", "VisualFusion", "VisualApproach", "PlatformHold", "ArmDeploy",
                    "ArmAlign", "FinalApproach", "ContactProxy", "Grasp", "CaptureConfirm", "CaptureHold"]
        checks.update(phase_order=phases == required, gap=.01 < h < .026, lateral=lateral < .006,
                      angle=angle < 1, hold_duration=bool(len(capture)) and float(data["elapsed_s"][-1]-data["elapsed_s"][capture[0]]) >= 5)
    else:
        safe = np.flatnonzero(data["phase"]=="SafeHold")
        checks.update(safe_hold=bool(len(safe)), never_latched="CaptureHold" not in phases)
        if len(safe):
            references = data["joint_reference"][safe]
            checks.update(closing_inhibited=bool(data["close_inhibited"][safe].all()),
                          fingers_only_open=bool((np.diff(references[:, 6:], axis=0) >= -1e-9).all()),
                          arm_reference_held=bool(np.max(np.ptp(references[:, :6], axis=0)) < 1e-7))
    checks = {key: bool(value) for key, value in checks.items()}
    plan = report["model_manifest"].get("approach_plan")
    if plan:
        checks["independent_gap_all_packets"] = bool(gap_samples_match) and all(gap_samples_match)
        checks["independent_optical_depth"] = bool(optical_samples_match) and all(optical_samples_match)
        checks["no_unplanned_dynamic_contact"] = report["local_dynamics"]["max_contacts"] == 0
        if normal:
            completed = np.flatnonzero(data["optical_checkpoint_complete"])
            checkpoint = np.arange(max(0, completed[0]-100), completed[0]+1) if len(completed) else np.array([], dtype=int)
            hold = np.flatnonzero(data["phase"] == "PlatformHold")
            # The ArmDeploy transition row still contains the pre-command
            # physical observation/encoders. Include it to check a full 5 s
            # interval (101 samples), not just 100 sample points spanning 4.95 s.
            if len(hold) and data["phase"][hold[-1]+1] == "ArmDeploy":
                hold = np.r_[hold, hold[-1]+1]
            hold = hold[-101:]
            # Reconstruct physical rotating velocity from physical body states.
            target = data["truth"][:, 0]
            omega = np.cross(target[:, :3], target[:, 3:]) / np.sum(target[:, :3]**2, axis=1)[:, None]
            physical_velocity = true_relative[:, 3:]
            if report.get("guidance_frame") == "orbital-vbar":
                physical_velocity = physical_velocity - np.cross(omega, true_relative[:, :3])
            physical_speed = np.linalg.norm(physical_velocity, axis=1)
            checks.update(
                optical_checkpoint_true_dwell=bool(len(checkpoint) == 101
                    and np.ptp(data["elapsed_s"][checkpoint]) >= 5.-1e-8
                    and np.all(np.abs(data["truth_optical_depth_m"][checkpoint]-17.4) < .10)
                    and np.all(physical_speed[checkpoint] < .003)),
                stowed_tcp_half_meter_true_dwell=bool(len(hold) == 101
                    and np.ptp(data["elapsed_s"][hold]) >= 5.-1e-8
                    and np.all(np.abs(data["truth_tcp_gap_m"][hold]-.5) < .010)
                    and np.all(physical_speed[hold] < .003)
                    and np.all(np.abs(data["joints"][hold, :6]) < .005)))
    return {"passed": all(checks.values()), "checks": checks, "phases": phases,
            "final_true_gap_m": float(h), "final_true_lateral_m": float(lateral), "final_true_angle_deg": float(angle),
            "kinematic_evaluation_time_s": float(evaluated_time),
            "kinematic_timing": "post-physics packet paired with next pre-physics encoder row; last unmatched packet omitted",
            "final_true_speed_m_s": speed, "final_inertial_speed_m_s": inertial_speed,
            "final_navigation_error_m": float(error[-1]),
            "max_force_n": float(np.linalg.norm(data["force_n"], axis=1).max()),
            "external_writes_instrumentation": report["local_dynamics"]["external_state_writes"],
            "scope": "independent kinematic/numeric evaluation; physical latch and image algorithm not validated"}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--run", type=Path)
    mode.add_argument("--geometry", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = geometry() if args.geometry else verify(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result["passed"] else 2)
