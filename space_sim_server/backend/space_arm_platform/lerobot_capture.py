"""Live LeRobot v3 sink. No post-hoc reader/converter of platform JSONL files.

One recording produces one self-contained dataset (episode_index=0). RGB is
the only visual product and is stored as video with authoritative frame IDs.
"""
from __future__ import annotations

import io
import json
import re
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .sampling import sample_tick
from .wrench_recording import WrenchStreamWriter
from .gripper_representation import opening_profile
from .recording_config import DEFAULT_CAMERA_IDS, VideoEncodingConfig
from .compact_storage import COMPACT_SCHEMA, CompactSidecars
DEFAULT_CAMERAS = list(DEFAULT_CAMERA_IDS)


def camera_key(camera: str) -> str:
    return re.sub(r"[^a-zA-Z0-9_]", "_", camera)


class LiveLeRobotWriter:
    def __init__(self, root: Path, request: Any, *, video_encoding: VideoEncodingConfig | None = None,
                 storage_profile: str = "diagnostic") -> None:
        from lerobot.datasets.lerobot_dataset import CODEBASE_VERSION, LeRobotDataset
        if CODEBASE_VERSION != "v3.0":
            raise RuntimeError(f"Expected LeRobot v3.0, found {CODEBASE_VERSION}")
        self.dataset_class = LeRobotDataset
        self.root = root
        self.request = request
        self.video_encoding = video_encoding or VideoEncodingConfig()
        if storage_profile not in ("compact", "diagnostic"):
            raise ValueError("Unknown storage profile")
        self.storage_profile = storage_profile
        self.sidecars = CompactSidecars(root) if storage_profile == "compact" else None
        self.fps = request.fps
        self.cameras = tuple(request.camera_ids)
        self.products = tuple(request.capture_products)
        self.dataset = None
        self.frames = 0
        self.last_tick: int | None = None
        self.camera_shapes: dict[str, tuple[int, ...]] = {}
        self.model_identity = None
        self.wrench_identity = None
        self.wrench_stream = WrenchStreamWriter(root, compact=self.sidecars is not None)

    def append(self, observation: Any, captures: dict[str, tuple[dict, dict]]) -> None:
        identity = (observation.model_sha256, observation.model_revision, observation.gripper_type)
        if self.model_identity is not None and self.model_identity != identity:
            raise ValueError('model or gripper schema changed within an episode')
        self.model_identity = identity
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
        profile = opening_profile(observation.gripper_type)
        state = list(observation.joint_position_rad)
        action = list(observation.target_joint_position_rad)
        if profile is not None:
            state = state[:6] + [observation.gripper_opening_normalized]
            action = action[:6] + [observation.target_gripper_opening_normalized]
        frame = {
            "observation.state": f32(state),
            "observation.joint_velocity": f32(observation.joint_velocity_rad_s),
            "observation.end_effector_pose_body": f32(observation.end_effector_position_body_m + observation.end_effector_orientation_body_wxyz),
            "observation.end_effector_twist_body": f32(observation.end_effector_twist_body),
            # Held joint servo targets, not the API's newest (possibly unapplied) command.
            "action": f32(action),
            "observation.sim_time_ns": np.array([int(observation.sim_time_ns)], dtype=np.int64),
            "observation.source_frame_id": np.array([int(observation.render_frame_id)], dtype=np.int64),
            "observation.applied_action_sequence": np.array([int(observation.applied_action_sequence)], dtype=np.int64),
            "task": self.request.instruction or self.request.task,
        }
        if self.sidecars is None:
            frame["observation.platform_json"] = observation.model_dump_json()
        if profile is not None:
            frame["observation.raw_joint_position"] = f32(observation.joint_position_rad)
            frame["observation.raw_target_joint_position"] = f32(observation.target_joint_position_rad)
        wrench = observation.wrist_wrench
        identity = None if wrench is None else (
            wrench.sensor_name, wrench.site_name, wrench.parent_body, wrench.child_body,
            wrench.frame, wrench.sign_convention, wrench.measurement, wrench.sample_phase)
        if self.frames and identity != self.wrench_identity:
            raise ValueError("wrist sensor availability or frame changed within an episode")
        self.wrench_identity = identity
        if wrench is not None:
            frame["observation.wrist_wrench_valid"] = np.array([int(wrench.valid)], dtype=np.int64)
            frame["observation.wrist_wrench"] = f32(wrench.force_n + wrench.torque_nm)
            frame["observation.wrist_wrench_sample_time_ns"] = np.array([int(wrench.sample_time_ns)], dtype=np.int64)
            frame["observation.wrist_wrench_interval_end_time_ns"] = np.array([int(wrench.interval_end_time_ns)], dtype=np.int64)
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
            if self.sidecars is None:
                frame[f"observation.camera_metadata.{key}"] = json.dumps(meta, ensure_ascii=False)
        if self.dataset is None:
            features = {}
            for name, value in frame.items():
                if name == "task":
                    continue
                if isinstance(value, str):
                    features[name] = {"dtype": "string", "shape": (1,), "names": None}
                else:
                    dtype = "video" if name.startswith("observation.images.") else str(value.dtype)
                    features[name] = {"dtype": dtype, "shape": value.shape, "names": None}
            n = len(observation.joint_position_rad)
            names = [f"joint_{i + 1}_rad" for i in range(6)] + (["finger_1_m", "finger_2_m"] if n == 8 else [])
            if profile is not None:
                raw_names = [f"joint_{i+1}_rad" for i in range(6)] + ["ee_link11_joint_rad", "ee_link12_joint_rad"]
                names = [f"joint_{i+1}_rad" for i in range(6)] + ["gripper_opening_normalized"]
                for key in ("observation.raw_joint_position", "observation.raw_target_joint_position"):
                    features[key]["names"] = raw_names
                features["observation.joint_velocity"]["names"] = [name + "_s" for name in raw_names]
            features["observation.state"]["names"] = names
            features["action"]["names"] = names
            if wrench is not None:
                features["observation.wrist_wrench"]["names"] = ["Fx_N", "Fy_N", "Fz_N", "Tx_Nm", "Ty_Nm", "Tz_Nm"]
            features["observation.end_effector_pose_body"]["names"] = ["x_m", "y_m", "z_m", "qw", "qx", "qy", "qz"]
            self.dataset = self.dataset_class.create(
                repo_id=f"local/{self.root.parent.name}", root=self.root,
                robot_type="sarm", fps=self.fps, features=features,
                use_videos=bool(self.cameras), video_backend="pyav", vcodec=self.video_encoding.codec,
                image_writer_threads=self.video_encoding.image_writer_threads,
                encoder_threads=self.video_encoding.encoder_threads,
                streaming_encoding=self.video_encoding.streaming_encoding,
                encoder_queue_maxsize=self.video_encoding.encoder_queue_maxsize,
            )
            if self.video_encoding.streaming_encoding and self.cameras:
                from .streaming_video import StrictStreamingVideoEncoder
                # Install before add_frame starts the episode. The upstream
                # implementation drops frames on a full queue.
                self.dataset._streaming_encoder = StrictStreamingVideoEncoder(
                    fps=self.fps, vcodec=self.video_encoding.codec,
                    g=self.video_encoding.keyframe_interval, crf=self.video_encoding.quality,
                    queue_maxsize=self.video_encoding.encoder_queue_maxsize,
                    encoder_threads=self.video_encoding.encoder_threads,
                )
                self.dataset._streaming_encoder.gpu_index = self.video_encoding.gpu_index
            # Explicit semantics alongside (not masquerading as) official metadata.
            (self.root / "meta" / "platform.json").write_text(json.dumps({
                "schema": "space-arm-lerobot/2" if profile is not None else "space-arm-lerobot/1", "body_frame": "cubesat_bus", "tool_site": "sarm_ee",
                "action_semantics": ('six held arm servo targets in radians plus target normalized gripper opening; not newest API command'
                                     if profile is not None else 'joint servo target held at the observation time; six radians plus two finger metres'),
                "state_semantics": ("six measured arm angles in radians plus measured normalized gripper opening"
                                    if profile is not None else "raw joint coordinates"),
                "gripper_normalization": profile.metadata() if profile is not None else None,
                "joint_velocity_semantics": "raw simulator joint velocities; retains both finger coordinates",
                "model_revision": observation.model_revision, "gripper_type": observation.gripper_type,
                "model_sha256": observation.model_sha256, "mujoco_runtime_version": observation.mujoco_runtime_version,
                "timestamp_semantics": "episode-relative uniform sample time; original nanoseconds in observation.sim_time_ns",
                "wrist_wrench": (wrench.model_dump(exclude={"force_n", "torque_nm", "sample_time_ns", "interval_end_time_ns", "valid"}) if wrench else None),
                "camera_keys": {c: camera_key(c) for c in self.cameras},
                "video_encoding": self.video_encoding.model_dump(mode="json"),
                "storage_profile": self.storage_profile,
                "compact_schema": COMPACT_SCHEMA if self.sidecars else None,
                "sidecars": {"context": "auxiliary/context", "cameras": "auxiliary/cameras"} if self.sidecars else {},
                "request": self.request.model_dump(mode="json"),
            }, ensure_ascii=False, indent=2), encoding="utf-8")
        self.wrench_stream.append(observation)
        self.dataset.add_frame(frame)
        if self.sidecars:
            self.sidecars.append(observation, captures, frame)
        self.frames += 1
        self.last_tick = tick

    @staticmethod
    def _image(blob: bytes) -> np.ndarray:
        with Image.open(io.BytesIO(blob)) as image:
            return np.asarray(image.convert("RGB"), dtype=np.uint8).copy()

    def finish(self, outcome: str = "unknown", note: str = "") -> None:
        if self.dataset is None:
            return
        try:
            self.wrench_stream.finish()
            if self.sidecars:
                self.sidecars.finish()
            self.dataset.save_episode()
            path = self.root / "meta" / "platform.json"
            metadata = json.loads(path.read_text(encoding="utf-8"))
            metadata["high_frequency_wrist_wrench"] = self.wrench_stream.metadata()
            metadata["episode_result"] = {"outcome": outcome, "note": note}
            path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        finally:
            self.dataset.finalize()
            self.dataset.stop_image_writer()

    def close_failed(self) -> None:
        if self.dataset is not None:
            self.wrench_stream.finish()
            if self.sidecars:
                self.sidecars.finish()
            self.dataset.finalize()
            self.dataset.stop_image_writer()
