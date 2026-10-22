"""Target-body ArUco rig and image-only wrist-camera pose estimation (SI units).

No Basilisk or Python MuJoCo imports. OpenCV is loaded only by the vision worker.
The same layout drives rendering and PnP, including the physical black border.
"""
from __future__ import annotations

import json
import io
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from .camera_defaults import CAPTURE_CAMERA_RESOLUTION

CAMERA_ID = "teleop/camera/sarm_wrist_cam"
SUPPORTED_TEMPLATES = frozenset({
    "sarm-ground-validation-grasp", "sarm-ground-validation-self-collision-grasp",
    "sarm-ground-validation-mesh-grasp",
})
LAYOUT_PATH = Path(__file__).resolve().parents[2] / "model/ground_validation_satellite/aruco_rig.json"


def load_rig() -> dict:
    return json.loads(LAYOUT_PATH.read_text(encoding="utf-8"))


def marker_corners(marker: dict) -> np.ndarray:
    """OpenCV detection order: top-left, top-right, bottom-right, bottom-left.

    u is image-right, v is image-down on the printed face; u x v points INWARD.
    All points are expressed in the capture_target body frame, in metres.
    """
    center = np.asarray(marker["center_m"], dtype=float)
    u, v = np.asarray(marker["right"], float), np.asarray(marker["down"], float)
    half = marker["size_m"] / 2
    return np.array([center + half * (x*u + y*v) for x, y in
                     ((-1, -1), (1, -1), (1, 1), (-1, 1))])


def wrist_intrinsics(model_path: Path, width: int, height: int) -> np.ndarray:
    if not (160 <= width <= 1920 and 90 <= height <= 1080):
        raise ValueError("unsupported image resolution")
    camera = ET.parse(model_path).find('.//camera[@name="sarm_wrist_cam"]')
    if camera is None:
        raise ValueError("scene has no calibrated wrist camera")
    source_width, source_height = map(int, camera.get("resolution", "640 360").split())
    # The bridge overrides the output resolution, preserving MJCF horizontal
    # FOV and square pixels. Accept current preview and legacy MJCF-aspect
    # frames; reject other aspect ratios (cropped/stretched inputs).
    preview_width, preview_height = CAPTURE_CAMERA_RESOLUTION
    aspects = (preview_width / preview_height, source_width / source_height)
    if all(abs(width / height - aspect) > 0.01 for aspect in aspects):
        raise ValueError("wrist frame aspect ratio differs from camera calibration")
    if camera.get("fovy"):
        fov_x = 2 * math.atan(math.tan(math.radians(float(camera.get("fovy"))) / 2)
                             * source_width / source_height)
    else:
        sensor = float(camera.get("sensorsize", "0.00576 0.00324").split()[0])
        focal = float(camera.get("focal", "0.0036 0.0036").split()[0])
        fov_x = 2 * math.atan2(sensor, 2 * focal)
    focal_px = width / (2 * math.tan(fov_x / 2))
    # Matches BskSceneController capture metadata exactly.
    return np.array([[focal_px, 0, (width-1)/2], [0, focal_px, (height-1)/2], [0, 0, 1.]])


class FiducialEstimator:
    def __init__(self):
        import cv2
        self.cv = cv2
        self.rig = load_rig()
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.rig["dictionary"]))
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, params)
        self.markers = {m["id"]: m for m in self.rig["markers"]}

    def estimate(self, image: np.ndarray, K: np.ndarray) -> dict:
        cv = self.cv
        corners, ids, _ = self.detector.detectMarkers(image)
        found = []
        if ids is not None:
            for points, identifier in zip(corners, ids.ravel()):
                if int(identifier) in self.markers:
                    found.append((int(identifier), points.reshape(4, 2)))
        result = {"status": "lost", "camera_id": CAMERA_ID, "target_body": "capture_target",
                  "coordinate_convention": "camera_optical: x right, y down, z forward; metres",
                  "detections": [{"id": i, "corners": p.tolist()} for i, p in found],
                  "pose": None, "image_size": [image.shape[1], image.shape[0]]}
        if not found:
            return result
        if len({i for i, _ in found}) != len(found):
            result["status"] = "duplicate_ids"
            return result
        obj = np.concatenate([marker_corners(self.markers[i]) for i, _ in found])
        img = np.concatenate([p for _, p in found]).astype(np.float64)
        # IPPE explicitly returns BOTH planar solutions. Report unresolved
        # ambiguity instead of choosing an arbitrary flipped satellite pose.
        ok, rotations, translations, _ = cv.solvePnPGeneric(obj, img, K, None, flags=cv.SOLVEPNP_IPPE)
        candidates = []
        if ok:
            for r, t in zip(rotations, translations):
                R, _ = cv.Rodrigues(r)
                if np.any((obj @ R.T + t.reshape(3))[:, 2] <= 0):
                    continue
                projected, _ = cv.projectPoints(obj, r, t, K, None)
                error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-img)**2, axis=1))))
                candidates.append((error, r, t, R))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > 3.0:
            result["status"] = "reprojection_rejected"
            return result
        error, r, t, R = candidates[0]
        if len(candidates) > 1:
            alternate = candidates[1]
            delta = np.clip((np.trace(R @ alternate[3].T)-1)/2, -1., 1.)
            if math.degrees(math.acos(delta)) > 10 and alternate[0]-error < 0.25:
                result.update(status="ambiguous", reprojection_error_px=error)
                return result
        # Refine all visible corners together, preserving the selected branch.
        r, t = cv.solvePnPRefineLM(obj, img, K, None, r, t)
        R, _ = cv.Rodrigues(r)
        projected, _ = cv.projectPoints(obj, r, t, K, None)
        error = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2)-img)**2, axis=1))))
        if not np.isfinite(R).all() or not np.isfinite(t).all() or error > 3:
            result["status"] = "reprojection_rejected"
            return result
        # Rotation matrix maps target-body axes into optical camera axes.
        result.update(status="tracking", reprojection_error_px=error, marker_count=len(found),
                      pose={"translation_m": t.reshape(3).tolist(), "rotation_camera_from_target": R.tolist(),
                            "rvec_rad": r.reshape(3).tolist()})
        origin = np.mean(obj, axis=0)
        axes = np.array([origin, origin+[0.06, 0, 0], origin+[0, 0.06, 0], origin+[0, 0, 0.06]])
        axis_pixels, _ = cv.projectPoints(axes, r, t, K, None)
        result["axes_pixels"] = axis_pixels.reshape(-1, 2).tolist()
        return result

    def decode_and_estimate(self, payload: bytes, model_path: Path) -> dict:
        from PIL import Image, UnidentifiedImageError
        try:
            with Image.open(io.BytesIO(payload)) as header:
                width, height = header.size
                if header.format not in {"JPEG", "PNG"}:
                    raise ValueError("expected JPEG or PNG")
                K = wrist_intrinsics(model_path, width, height)
        except (UnidentifiedImageError, OSError, Image.DecompressionBombError) as error:
            raise ValueError("invalid image") from error
        image = self.cv.imdecode(np.frombuffer(payload, np.uint8), self.cv.IMREAD_COLOR)
        if image is None:
            raise ValueError("invalid image")
        if image.shape[:2] != (height, width):
            raise ValueError("image orientation must match the original wrist frame")
        return self.estimate(image, K)
