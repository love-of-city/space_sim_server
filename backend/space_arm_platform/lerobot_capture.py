"""Live LeRobot v3 sink. No post-hoc reader/converter of platform JSONL files.

One recording produces one self-contained dataset (episode_index=0). RGB is
video; segmentation is lossless image; original metric PFM depth is an auxiliary
asset referenced by a string feature. All products retain authoritative IDs.
"""
from __future__ import annotations

import io
import json
import re
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

SUPPORTED_FPS = (1, 2, 5, 10)
DEFAULT_CAMERAS = ["teleop/camera/spacecraft_overview", "teleop/camera/sarm_wrist_cam"]


def camera_key(camera: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", camera)


def sample_tick(sim_time_ns: int, fps: int) -> int | None:
    # Integer arithmetic; allow sub-microsecond integration rounding only.
    tick = (sim_time_ns * fps + 500_000_000) // 1_000_000_000
    return tick if abs(sim_time_ns * fps - tick * 1_000_000_000) <= fps * 1000 else None


class LiveLeRobotWriter:
    def __init__(self, root: Path, request: Any) -> None:
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
        if CODEBASE_VERSION != "v3.0":
            raise RuntimeError(f"Expected LeRobot v3.0, found {CODEBASE_VERSION}")
        self.dataset_class = LeRobotDataset
        self.root = root
        self.request = request
        self.fps = request.fps
        self.cameras = tuple(request.camera_ids)
        self.products = tuple(request.capture_products)
        self.dataset = None
        self.frames = 0
        self.last_tick: int | None = None
        self.camera_shapes: dict[str, tuple[int, ...]] = {}

    def append(self, observation: Any, captures: dict[str, tuple[dict, dict]]) -> None:
        tick = sample_tick(int(observation.sim_time_ns), self.fps)
        if tick is None:
            raise ValueError("observation is not on the dataset sampling grid")
        if self.last_tick is not None and tick != self.last_tick + 1:
            raise ValueError("missing/non-monotonic dataset sample; refusing to compress simulation time")
        if self.frames >= self.request.max_frames:
            raise ValueError("episode frame limit reached; stop and start a new recording")
        def f32(values):
            array = np.asarray(values, dtype=np.float32)
            if not np.isfinite(array).all():
                raise ValueError("non-finite dataset feature")
            return array
        frame = {
            "observation.state": f32(observation.joint_position_rad),
            "observation.joint_velocity": f32(observation.joint_velocity_rad_s),
            "observation.end_effector_pose_body": f32(observation.end_effector_position_body_m + observation.end_effector_orientation_body_wxyz),
            "observation.end_effector_twist_body": f32(observation.end_effector_twist_body),
            # Held joint servo targets, not the API's newest (possibly unapplied) command.
            "action": f32(observation.target_joint_position_rad),
            "observation.sim_time_ns": np.array([int(observation.sim_time_ns)], dtype=np.int64),
            "observation.source_frame_id": np.array([int(observation.render_frame_id)], dtype=np.int64),
            "observation.applied_action_sequence": np.array([int(observation.applied_action_sequence)], dtype=np.int64),
            "observation.platform_json": observation.model_dump_json(),
            "task": self.request.instruction or self.request.task,
        }
        for camera in self.cameras:
            meta, products = captures[camera]
            if set(self.products) - products.keys():
                raise ValueError(f"missing products for {camera}: {set(self.products) - products.keys()}")
            key = camera_key(camera)
            if meta.get("dataset_format") != "lerobot-v3":
                raise ValueError("UE peer must advertise dataset_format=lerobot-v3; rebuild the dataset branch")
            if meta.get("sampling_fps") != self.fps:
                raise ValueError("UE sampling_fps does not match recording fps")
            if meta.get("sample_index") != str(tick):
                raise ValueError("UE sample_index does not match simulation time")
            rgb = self._image(products["rgb"])
            h, w, _ = rgb.shape
            if w % 2 or h % 2:
                raise ValueError("RGB video resolution must have even dimensions")
            if list(meta.get("resolution", [])) != [w, h]:
                raise ValueError("capture resolution metadata does not match RGB bytes")
            previous_shape = self.camera_shapes.setdefault(camera, rgb.shape)
            if previous_shape != rgb.shape:
                raise ValueError("camera resolution changed during recording")
            frame[f"observation.images.{key}"] = rgb
            frame[f"observation.camera_metadata.{key}"] = json.dumps(meta, ensure_ascii=False)
            if "segmentation" in self.products:
                segmentation = self._image(products["segmentation"])
                if segmentation.shape != rgb.shape:
                    raise ValueError("segmentation/RGB resolution mismatch")
                frame[f"observation.segmentation.{key}"] = segmentation
            if "depth" in self.products:
                self._validate_depth(products["depth"], w, h)
                relative = Path("auxiliary") / "depth" / key / f"frame-{self.frames:06d}.pfm"
                # Dataset.create owns creating root, so defer asset write until below.
                frame[f"observation.depth_path.{key}"] = relative.as_posix()
        if self.dataset is None:
            features = {}
            for name, value in frame.items():
                if name == "task":
                    continue
                if isinstance(value, str):
                    features[name] = {"dtype": "string", "shape": (1,), "names": None}
                else:
                    dtype = "video" if name.startswith("observation.images.") else "image" if name.startswith("observation.segmentation.") else str(value.dtype)
                    features[name] = {"dtype": dtype, "shape": value.shape, "names": None}
            n = len(observation.joint_position_rad)
            names = [f"joint_{i + 1}_rad" for i in range(6)] + (["finger_1_m", "finger_2_m"] if n == 8 else [])
            features["observation.state"]["names"] = names
            features["action"]["names"] = names
            features["observation.end_effector_pose_body"]["names"] = ["x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
            self.dataset = self.dataset_class.create(
                repo_id=f"local/{self.root.parent.name}", root=self.root,
                robot_type="sarm", fps=self.fps, features=features,
                use_videos=bool(self.cameras), video_backend="pyav", vcodec="h264",
                image_writer_threads=0, encoder_threads=2,
            )
            # Explicit semantics alongside (not masquerading as) official metadata.
            (self.root / "meta" / "platform.json").write_text(json.dumps({
                "schema": "space-arm-lerobot/1", "body_frame": "cubesat_bus", "tool_site": "sarm_ee",
                "action_semantics": "joint servo target held at the observation time; six radians plus two finger metres",
                "timestamp_semantics": "episode-relative uniform sample time; original nanoseconds in observation.sim_time_ns",
                "depth_semantics": "relative auxiliary PFM path; float32 metres camera_z, bottom-up PFM rows",
                "segmentation_semantics": "lossless RGB instance IDs; mapping in camera_metadata, not a lossy video",
                "camera_keys": {c: camera_key(c) for c in self.cameras},
                "request": self.request.model_dump(mode="json"),
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        for camera in self.cameras:
            if "depth" in self.products:
                path = self.root / frame[f"observation.depth_path.{camera_key(camera)}"]
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(captures[camera][1]["depth"])
        self.dataset.add_frame(frame)
        self.frames += 1
        self.last_tick = tick

    @staticmethod
    def _image(blob: bytes) -> np.ndarray:
        with Image.open(io.BytesIO(blob)) as image:
            return np.asarray(image.convert("RGB"), dtype=np.uint8).copy()

    @staticmethod
    def _validate_depth(blob: bytes, width: int, height: int) -> None:
        stream = io.BytesIO(blob)
        if stream.readline().strip() != b"Pf":
            raise ValueError("expected a grayscale PFM depth product")
        if stream.readline().split() != [str(width).encode(), str(height).encode()]:
            raise ValueError("depth/RGB resolution mismatch")
        scale = float(stream.readline())
        values = stream.read()
        if scale != -1.0 or len(values) != width * height * 4:
            raise ValueError("expected little-endian float32 metre depth payload")
        depth = np.frombuffer(values, dtype="<f4")
        if not np.isfinite(depth).all() or np.any(depth < 0):
            raise ValueError("depth must contain finite, nonnegative metres")

    def finish(self, outcome: str = "unknown", note: str = "") -> None:
        if self.dataset is None:
            return
        try:
            self.dataset.save_episode()
            path = self.root / "meta" / "platform.json"
            metadata = json.loads(path.read_text(encoding="utf-8"))
            metadata["episode_result"] = {"outcome": outcome, "note": note}
            path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        finally:
            self.dataset.finalize()
            self.dataset.stop_image_writer()

    def close_failed(self) -> None:
        if self.dataset is not None:
            self.dataset.finalize()
            self.dataset.stop_image_writer()
