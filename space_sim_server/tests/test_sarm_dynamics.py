"""Regressions for orbital free flight and SARM prismatic finger actuation."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET

import numpy as np
import pytest

MODEL_ROOT = Path(__file__).resolve().parents[1] / "model/SARM/platform"


@pytest.mark.parametrize("name", ["cubesat_free", "capture_target_free"])
def test_free_flight_does_not_inherit_arm_passive_forces(name):
    root = ET.parse(MODEL_ROOT / "sarm_platform.xml").getroot()
    joint = root.find(f'.//joint[@name="{name}"]')
    assert joint is not None
    assert joint.get("type") == "free"
    for attribute in ("damping", "frictionloss", "stiffness", "armature"):
        assert joint.get(attribute) is not None, f"{name} must explicitly override {attribute}"
        assert float(joint.get(attribute)) == 0.0
    # Do not 'fix' free-flight drag by removing useful arm joint damping/friction.
    default = root.find("default/joint")
    assert float(default.get("damping")) == 0.5
    assert float(default.get("frictionloss")) == 0.05


def scenario_array(name):
    # Read the literal configuration without loading Python MuJoCo and the
    # Basilisk MuJoCo DLL into the same Windows process.
    tree = ast.parse((MODEL_ROOT / "scenarios/scenario_sarm_grasp.py").read_text(encoding="utf-8"))
    assignment = next(
        node for node in tree.body if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)
    )
    return np.asarray(ast.literal_eval(assignment.value.args[0]), dtype=float)


@pytest.mark.parametrize("index,name", [(6, "joint_finger1"), (7, "joint_finger2")])
def test_finger_pid_overcomes_friction_without_raising_force_cap(index, name):
    root = ET.parse(MODEL_ROOT / "sarm_platform.xml").getroot()
    joint = root.find(f'.//joint[@name="{name}"]')
    assert joint.get("type") == "slide"
    stiffness = scenario_array("KP")[index]
    force_limit = scenario_array("TORQUE_LIMITS")[index]
    friction = float(joint.get("frictionloss"))
    assert min(stiffness * 0.001, force_limit) > friction, "A 1 mm reference error must move the finger"
    assert force_limit == 0.05  # N, not Nm: retain the validated per-finger safety cap.
    assert friction == 0.01
    assert float(joint.get("damping")) == 1.0
