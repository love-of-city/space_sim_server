"""Explicit camera sets shared by model preparation, recording and validation."""
from copy import deepcopy
import xml.etree.ElementTree as ET

import numpy as np

SINGLE_ARM_CAMERA_IDS = ("teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam")
SIDE_CAMERA_ID = "teleop/camera/workbench_side"
PROFILES = {
    "dual-arm-three-view-v1": {
        "cameras": [dict(id="teleop/camera/spacecraft_overview", role="overview", label="全局相机"),
                    dict(id="teleop/camera/left_sarm_wrist_cam", role="left_wrist", label="左手眼相机"),
                    dict(id="teleop/camera/right_sarm_wrist_cam", role="right_wrist", label="右手眼相机")],
        "mounts": [],
    },
    "single-arm-two-view-v1": {
        "cameras": [dict(id=SINGLE_ARM_CAMERA_IDS[0], role="overview", label="全局相机"),
                    dict(id=SINGLE_ARM_CAMERA_IDS[1], role="wrist", label="手眼相机")],
        "mounts": [],
    },
    "single-arm-three-view-v1": {
        "cameras": [dict(id=SINGLE_ARM_CAMERA_IDS[0], role="overview", label="全局相机"),
                    dict(id=SINGLE_ARM_CAMERA_IDS[1], role="wrist", label="手眼相机"),
                    dict(id=SIDE_CAMERA_ID, role="fixed_aux", label="固定侧视相机")],
        "mounts": [dict(name="workbench_side", body="cubesat_bus",
                        position_m=[-.25, .85, .95], look_at_m=[.30, .45, .35],
                        fovy_deg=85., resolution=[1024, 1024])],
    },
}


def camera_profile(scene):
    key = scene.get("collection_camera_profile", "single-arm-two-view-v1")
    if key not in PROFILES:
        raise ValueError(f"Unknown collection_camera_profile: {key}")
    record = dict(schema="collection-cameras/1", id=key, **deepcopy(PROFILES[key]))
    saved = scene.get("collection_config", {}).get("camera_profile")
    if saved is not None and saved != record:
        raise ValueError("Frozen camera profile differs from the selected profile")
    return record


def camera_ids(scene):
    return [item["id"] for item in camera_profile(scene)["cameras"]]


def freeze_camera_profile(scene):
    profile = camera_profile(scene)
    scene.setdefault("collection_config", {})["camera_profile"] = profile
    return profile


def mount_rotation(mount):
    position, target = np.asarray(mount["position_m"]), np.asarray(mount["look_at_m"])
    # MJCF optical direction is -Z, with image up +Y.
    z = position - target
    z /= np.linalg.norm(z)
    x = np.cross([0., 0., 1.], z)
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


def validate_profile_capture(metadata, observation, scene):
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
    from .overview_camera import validate_body_fixed_capture
    for mount in camera_profile(scene)["mounts"]:
        # Adapter wire axes are +X forward, +Y left, +Z up.
        wire_rotation = mount_rotation(mount) @ np.array([[0., -1., 0.], [0., 0., 1.], [-1., 0., 0.]])
        expected = dict(position_body_m=mount["position_m"],
                        orientation_body_from_camera_wxyz=matrix_to_quaternion_wxyz(wire_rotation),
                        field_of_view_rad=[np.deg2rad(mount["fovy_deg"])] * 2)
        validate_body_fixed_capture(metadata, observation, expected, f'teleop/camera/{mount["name"]}')


def apply_camera_profile(root, scene):
    """Add body-fixed MJCF cameras; never move an existing camera silently."""
    from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
    profile = freeze_camera_profile(scene)
    for mount in profile["mounts"]:
        body = root.find(f'.//body[@name="{mount["body"]}"]')
        if body is None:
            raise ValueError(f"Missing camera parent {mount['body']}")
        rotation = mount_rotation(mount)
        attrs = dict(name=mount["name"], mode="fixed",
                     pos=" ".join(map(str, mount["position_m"])),
                     quat=" ".join(format(v, ".17g") for v in matrix_to_quaternion_wxyz(rotation)),
                     fovy=str(mount["fovy_deg"]), resolution=" ".join(map(str, mount["resolution"])))
        matches = root.findall(f'.//camera[@name="{mount["name"]}"]')
        if matches:
            if len(matches) != 1 or matches[0] not in list(body) or matches[0].attrib != attrs:
                raise ValueError("Existing fixed camera differs from the frozen mount")
        else:
            ET.SubElement(body, "camera", **attrs)
    for identifier in camera_ids(scene):
        if len(root.findall(f'.//camera[@name="{identifier.rsplit("/", 1)[-1]}"]')) != 1:
            raise ValueError(f"Collection camera missing or duplicated: {identifier}")
    return profile
