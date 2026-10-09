"""Compare the lightweight MJCF kinematics against native MuJoCo transforms."""

from __future__ import annotations

from pathlib import Path
import sys

import mujoco
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_ROOT = PROJECT_ROOT.parent
sys.path.insert(0, str(PROJECT_ROOT))

from simulation.serial_chain_kinematics import SerialChainKinematics


MODEL = (
    WORKSPACE_ROOT
    / "test"
    / "model"
    / "spacecraft_and_arm"
    / "assets"
    / "cubesat_so101_grasp"
    / "cubesat_so101_grasp.xml"
)
JOINTS = (
    "so101_shoulder_pan",
    "so101_shoulder_lift",
    "so101_elbow_flex",
    "so101_wrist_flex",
    "so101_wrist_roll",
)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    chain = SerialChainKinematics.from_mjcf(
        MODEL,
        base_body="cubesat_bus",
        joint_names=JOINTS,
        tool_site="so101_gripperframe",
    )
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cubesat_bus")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "so101_gripperframe")
    samples = (
        np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0]),
        np.array([0.25, -0.35, 0.3, -0.2, 0.4]),
        np.array([-0.4, 0.15, -0.2, 0.25, -0.3]),
    )
    for q in samples:
        for name, value in zip(JOINTS, q, strict=True):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[joint_id]] = value
        mujoco.mj_forward(model, data)
        world_from_base = data.xmat[base_id].reshape(3, 3)
        world_from_site = data.site_xmat[site_id].reshape(3, 3)
        native_position = world_from_base.T @ (data.site_xpos[site_id] - data.xpos[base_id])
        native_rotation = world_from_base.T @ world_from_site
        position, rotation = chain.forward(q)
        if not np.allclose(position, native_position, atol=2e-8):
            raise AssertionError(f"position mismatch: {position} vs {native_position}")
        if not np.allclose(rotation, native_rotation, atol=2e-7):
            raise AssertionError("orientation mismatch")
    print("MJCF forward kinematics matches native MuJoCo for 3 samples")


if __name__ == "__main__":
    main()
