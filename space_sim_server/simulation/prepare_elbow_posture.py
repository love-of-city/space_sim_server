"""Isolated startup-only SARM posture worker (no Basilisk imports).

Python MuJoCo must stay in this child process: importing a second MuJoCo DLL
inside the authoritative Basilisk simulator is deliberately avoided.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "backend"))

from simulation.pose_ik import ElbowPreference, elbow_height, select_elbow_preferred_pose
from simulation.serial_chain_kinematics import SerialChainKinematics
from space_arm_platform.joint_limits import load_joint_limits

ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
ALL_NAMES = ARM_NAMES + ("joint_finger1", "joint_finger2")


class SceneConfigurationValidator:
    """Static contacts from the selected MJCF, with the sampled target/fingers.

    This is endpoint screening, not swept-volume or dynamic collision proof.
    Experimental flex scenes are refused rather than silently ignoring flexes.
    """
    def __init__(self, model_path: Path, randomized: dict, chain: SerialChainKinematics):
        import mujoco  # Only imported by the offline worker, never live control.
        if mujoco.__version__ != "3.7.0":
            raise ValueError("posture validation requires runtime-matched MuJoCo 3.7.0")
        self.mj = mujoco
        self.model = model = mujoco.MjModel.from_xml_path(str(model_path))
        if model.nflex:
            raise ValueError("flex collision validation is not supported; preserving original start")
        self.data = data = mujoco.MjData(model)
        self.chain = chain
        self.addresses = [int(model.joint(name).qposadr[0]) for name in ALL_NAMES]
        q = np.asarray(randomized["arm_joint_position_rad"], dtype=float)
        if q.shape != (8,) or not np.all(np.isfinite(q)):
            raise ValueError("expected six arm joints and two fingers")
        data.qpos[self.addresses] = q
        target = model.joint("capture_target_free")
        address = int(target.qposadr[0])
        data.qpos[address:address + 3] = randomized["target_position_m"]
        data.qpos[address + 3:address + 7] = randomized["target_orientation_wxyz"]
        if "target_hinge_position_rad" in randomized:
            hinge = model.joint("outer_panel_hinge")
            data.qpos[int(hinge.qposadr[0])] = randomized["target_hinge_position_rad"]
        self.base_id = int(model.body("cubesat_bus").id)
        self.site_id = int(model.site("sarm_ee").id)
        arm_root = int(model.body("link1").id)
        self.arm_bodies = set()
        for body in range(model.nbody):
            parent = body
            while parent:
                if parent == arm_root:
                    self.arm_bodies.add(body)
                    break
                parent = int(model.body_parentid[parent])
        # Check that the parser and selected collision model use the same FK.
        self._forward(q[:6])

    def _forward(self, q):
        self.data.qpos[self.addresses[:6]] = q
        self.mj.mj_forward(self.model, self.data)
        base_rotation = self.data.xmat[self.base_id].reshape(3, 3)
        p = base_rotation.T @ (self.data.site_xpos[self.site_id] - self.data.xpos[self.base_id])
        r = base_rotation.T @ self.data.site_xmat[self.site_id].reshape(3, 3)
        expected_p, expected_r = self.chain.forward(q)
        if not (np.allclose(p, expected_p, atol=1e-6, rtol=0)
                and np.allclose(r, expected_r, atol=1e-6, rtol=0)):
            raise ValueError("kinematics/MuJoCo FK mismatch; preserving original start")

    def __call__(self, q: np.ndarray) -> bool:
        self._forward(q)
        for contact in self.data.contact:
            bodies = (int(self.model.geom_bodyid[contact.geom1]),
                      int(self.model.geom_bodyid[contact.geom2]))
            if contact.dist <= 0.0 and any(body in self.arm_bodies for body in bodies):
                return False
        return True


def prepare(model_path: Path, randomized: dict) -> dict:
    chain = SerialChainKinematics.from_mjcf(model_path, base_body="cubesat_bus",
                                           joint_names=ARM_NAMES, tool_site="sarm_ee")
    q = np.asarray(randomized["arm_joint_position_rad"], dtype=float)[:6]
    limits = load_joint_limits(model_path, ARM_NAMES)
    pref = ElbowPreference()
    validator = SceneConfigurationValidator(model_path, randomized, chain)
    selection = select_elbow_preferred_pose(
        chain, *chain.forward(q), q, joint_position_min=limits.lower,
        joint_position_max=limits.upper, configuration_is_valid=validator,
        preference=pref,
    )
    report = {
        "schema": "elbow-preferred-initialization/1", "mode": "prefer",
        "up_frame": "cubesat_bus", "up_axis": list(pref.up_axis),
        "preferred_height_m": pref.preferred_height_m,
        "original_joint_position_rad": q.tolist(),
        "original_elbow_height_m": elbow_height(chain, q, pref),
        "attempted_seeds": selection.attempted_seeds,
        "valid_candidates": len(selection.candidates),
        "rejected_collision": selection.rejected_collision,
        "collision_check": "selected_mjcf_static_arm_contacts",
        "mujoco_version": validator.mj.__version__,
    }
    chosen = selection.candidate
    if chosen is None:
        report.update(status="fallback", reason="no_valid_candidate", joint_position_rad=q.tolist())
    else:
        changed = not np.allclose(chosen.joint_position_rad, q, atol=1e-6, rtol=0)
        report.update(status="selected" if changed else "kept", reason="lowest_valid_cost",
                      joint_position_rad=chosen.joint_position_rad.tolist(),
                      selected_elbow_height_m=chosen.elbow_height_m,
                      position_error_m=chosen.position_error_m,
                      orientation_error_rad=chosen.orientation_error_rad,
                      minimum_singular_value=chosen.minimum_singular_value, cost=chosen.cost)
    return report


if __name__ == "__main__":
    try:
        payload = json.load(sys.stdin)
        result = prepare(Path(payload["model_path"]), payload["randomization"])
    except Exception as error:
        # Parent keeps the exact old state; never relabel an unchecked pose safe.
        result = {"schema": "elbow-preferred-initialization/1", "mode": "prefer",
                  "status": "fallback", "reason": f"{type(error).__name__}: {error}"}
    print(json.dumps(result, allow_nan=False))
