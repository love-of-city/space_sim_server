"""Replay recorded MJScene state with Python MuJoCo and report grasp contact."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco as mj
import numpy as np


def analyze(mjcf: Path, states: Path, start: float, stop: float) -> dict[str, float]:
    model = mj.MjModel.from_xml_path(str(mjcf))
    data = mj.MjData(model)
    saved = np.load(states)
    qpos = np.asarray(saved["qpos"], dtype=float).reshape(-1, model.nq)
    qvel = np.asarray(saved["qvel"], dtype=float).reshape(-1, model.nv)
    seconds = np.asarray(saved["times_ns"], dtype=float) * 1.0e-9
    selection = np.flatnonzero((seconds >= start) & (seconds <= stop))
    if selection.size == 0:
        raise RuntimeError("no MJScene samples exist in the requested contact interval")

    handle_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "capture_target_handle")
    fixed_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "so101_gripper")
    moving_body_id = mj.mj_name2id(
        model, mj.mjtObj.mjOBJ_BODY, "so101_moving_jaw_so101_v1"
    )
    gripper_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "so101_gripperframe")
    target_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "capture_target_grasp")
    bilateral: list[bool] = []
    fixed_contacts: list[bool] = []
    moving_contacts: list[bool] = []
    site_errors: list[float] = []
    site_offsets: list[np.ndarray] = []
    relative_linear_speeds: list[float] = []
    relative_angular_speeds: list[float] = []
    for sample in selection:
        data.qpos[:] = qpos[sample]
        data.qvel[:] = qvel[sample]
        mj.mj_forward(model, data)
        fixed_force = 0.0
        moving_force = 0.0
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if handle_id not in (geom1, geom2):
                continue
            other_geom = geom2 if geom1 == handle_id else geom1
            other_body = int(model.geom_bodyid[other_geom])
            contact_force = np.zeros(6)
            mj.mj_contactForce(model, data, contact_index, contact_force)
            normal_force = max(0.0, float(contact_force[0]))
            if other_body == fixed_body_id:
                fixed_force += normal_force
            elif other_body == moving_body_id:
                moving_force += normal_force
        bilateral.append(fixed_force > 0.01 and moving_force > 0.01)
        fixed_contacts.append(fixed_force > 0.01)
        moving_contacts.append(moving_force > 0.01)
        site_offset = data.site_xpos[target_site_id] - data.site_xpos[gripper_site_id]
        site_offsets.append(site_offset.copy())
        site_errors.append(float(np.linalg.norm(site_offset)))
        target_jacobian_position = np.zeros((3, model.nv))
        target_jacobian_rotation = np.zeros((3, model.nv))
        gripper_jacobian_position = np.zeros((3, model.nv))
        gripper_jacobian_rotation = np.zeros((3, model.nv))
        mj.mj_jacSite(
            model, data, target_jacobian_position, target_jacobian_rotation, target_site_id
        )
        mj.mj_jacSite(
            model, data, gripper_jacobian_position, gripper_jacobian_rotation, gripper_site_id
        )
        relative_linear_speeds.append(float(np.linalg.norm(
            (target_jacobian_position - gripper_jacobian_position) @ data.qvel
        )))
        relative_angular_speeds.append(float(np.linalg.norm(
            (target_jacobian_rotation - gripper_jacobian_rotation) @ data.qvel
        )))
    baseline_offset = site_offsets[0]
    return {
        "withdrawal_bilateral_contact_coverage": float(np.mean(bilateral)),
        "withdrawal_fixed_contact_coverage": float(np.mean(fixed_contacts)),
        "withdrawal_moving_contact_coverage": float(np.mean(moving_contacts)),
        "initial_withdrawal_grasp_site_error_m": float(site_errors[0]),
        "minimum_withdrawal_grasp_site_error_m": float(np.min(site_errors)),
        "maximum_withdrawal_grasp_site_error_m": float(np.max(site_errors)),
        "final_grasp_site_error_m": float(site_errors[-1]),
        "withdrawal_grasp_site_drift_m": float(np.max([
            np.linalg.norm(offset - baseline_offset) for offset in site_offsets
        ])),
        "final_gripper_target_relative_speed_m_s": float(relative_linear_speeds[-1]),
        "final_gripper_target_relative_rate_rad_s": float(relative_angular_speeds[-1]),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--states", type=Path, required=True)
    parser.add_argument("--start", type=float, required=True)
    parser.add_argument("--stop", type=float, required=True)
    args = parser.parse_args()
    print(json.dumps(analyze(args.mjcf, args.states, args.start, args.stop)))


if __name__ == "__main__":
    main()
