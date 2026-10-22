"""Offline reference geometry and frame conversions; no estimator truth input."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def rotation(element):
    if any(key in element.attrib for key in ("euler", "axisangle", "xyaxes", "zaxis")):
        raise ValueError("Reference loader requires explicit quaternion or identity")
    w, x, y, z = np.fromstring(element.get("quat", "1 0 0 0"), sep=" ")
    qnorm = np.linalg.norm([w, x, y, z])
    w, x, y, z = np.array([w, x, y, z])/qnorm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def position(element):
    return np.fromstring(element.get("pos", "0 0 0"), sep=" ")


def load_reference(model):
    """Current model only: fixed front mechanism and co-mounted camera/tool site."""
    model = Path(model)
    root = ET.parse(model).getroot()
    link = root.find(".//body[@name='link6']")
    camera = link.find("camera[@name='sarm_wrist_cam']")
    tool = link.find("site[@name='sarm_ee']")
    target = root.find(".//body[@name='capture_target']")
    geom = target.find("geom[@name='target_instance_0030_part_015_color_00']")
    if camera is None or tool is None or geom is None:
        raise ValueError("Reference frame hierarchy changed; review geometry before evaluation")
    asset = root.find(f"asset/mesh[@name='{geom.get('mesh')}']")
    compiler = root.find("compiler")
    mesh_path = (model.parent / compiler.get("meshdir", ".") / asset.get("file")).resolve()
    vertices = []
    with mesh_path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.startswith("v "):
                vertices.append([float(x) for x in line.split()[1:4]])
    # This exporter has no mesh reference transforms; fail rather than silently
    # treating a future transformed asset as the same geometric center.
    if any(key in asset.attrib for key in ("refpos", "refquat")):
        raise ValueError("Mesh reference transform needs explicit support")
    scale = np.fromstring(asset.get("scale", "1 1 1"), sep=" ")
    vertices = (np.asarray(vertices)*scale) @ rotation(geom).T + position(geom)
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    R_G_C = rotation(tool).T @ rotation(camera) @ np.diag([1., -1., -1.])
    t_G_C = rotation(tool).T @ (position(camera)-position(tool))
    return {"point_target_m": ((low+high)/2).tolist(),
            "bounds_target_m": [low.tolist(), high.tolist()],
            "rotation_gripper_from_optical": R_G_C.tolist(),
            "camera_in_gripper_m": t_G_C.tolist(), "mesh": str(mesh_path),
            "target_reference": "fixed WF10 separation assembly AABB center, not a grasp contact",
            "gripper_reference": "current sarm_ee site; provisional tool center, not future arm calibration"}


def point_in_gripper(R_C_S, t_C_S, reference):
    return (np.asarray(reference["rotation_gripper_from_optical"]) @
            (np.asarray(R_C_S) @ reference["point_target_m"] + t_C_S)
            + reference["camera_in_gripper_m"])


def center_errors(measured, truth):
    measured, truth = np.asarray(measured), np.asarray(truth)
    error = (measured-truth)*1000
    result = {"center_position_mm": float(np.linalg.norm(error)),
              "gripper_range_signed_mm": float((np.linalg.norm(measured)-np.linalg.norm(truth))*1000),
              "gripper_range_truth_m": float(np.linalg.norm(truth)),
              "gripper_range_measured_m": float(np.linalg.norm(measured))}
    for i, axis in enumerate("xyz"):
        result.update({f"center_{axis}_truth_m": float(truth[i]),
                       f"center_{axis}_measured_m": float(measured[i]),
                       f"center_{axis}_error_mm": float(error[i])})
    return result
