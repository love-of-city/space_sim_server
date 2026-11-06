"""Live LeRobot v3 sink. No post-hoc reader/converter of platform JSONL files.

One recording produces one self-contained dataset (episode_index=0). RGB is
the only visual product and is stored as video with authoritative frame IDs.
"""
from __future__ import annotations

import io
import json
import re
import sys
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
# Below this conservative workload, starting fresh Windows encoder processes
# costs more than encoding in the already-warm isolated dataset worker.
IN_PROCESS_VIDEO_PIXEL_BUDGET = 200_000_000


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
        self.arm_wrench_streams = {}

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
        dual = getattr(observation, 'observation_layout', None) == 'dual-arm/1'
        profile = opening_profile(observation.gripper_type)
        arms = {side:observation.arms[side] for side in ('left','right')} if dual else {'': observation}
        state, action, raw, targets, velocity, pose, twist = [], [], [], [], [], [], []
        for arm in arms.values():
            state.extend(list(arm.joint_position_rad[:6]) + [arm.gripper_opening_normalized]
                         if profile else arm.joint_position_rad)
            action.extend(list(arm.target_joint_position_rad[:6]) + [arm.target_gripper_opening_normalized]
                          if profile else arm.target_joint_position_rad)
            raw.extend(arm.joint_position_rad)
            targets.extend(arm.target_joint_position_rad)
            velocity.extend(arm.joint_velocity_rad_s)
            pose.extend(arm.end_effector_position_body_m + arm.end_effector_orientation_body_wxyz)
            twist.extend(arm.end_effector_twist_body)
        frame = {
            "observation.state": f32(state),
            "observation.joint_velocity": f32(velocity),
            "observation.end_effector_pose_body": f32(pose),
            "observation.end_effector_twist_body": f32(twist),
            "action": f32(action),
            "observation.sim_time_ns": np.array([int(observation.sim_time_ns)], dtype=np.int64),
            "observation.source_frame_id": np.array([int(observation.render_frame_id)], dtype=np.int64),
            "observation.applied_action_sequence": np.array([int(observation.applied_action_sequence)], dtype=np.int64),
            "task": self.request.instruction or self.request.task,
        }
        if self.sidecars is None:
            frame["observation.platform_json"] = observation.model_dump_json()
        if profile is not None:
            frame["observation.raw_joint_position"] = f32(raw)
            frame["observation.raw_target_joint_position"] = f32(targets)
        wrench = None if dual else observation.wrist_wrench
        identities = {}
        for side, arm in arms.items():
            reading = arm.wrist_wrench
            identities[side] = None if reading is None else (
                reading.sensor_name, reading.site_name, reading.parent_body, reading.child_body,
                reading.frame, reading.sign_convention, reading.measurement, reading.sample_phase)
            if reading is not None:
                prefix = 'observation.' + (side + '.' if side else '')
                frame[prefix+"wrist_wrench_valid"] = np.array([int(reading.valid)], dtype=np.int64)
                frame[prefix+"wrist_wrench"] = f32(reading.force_n + reading.torque_nm)
                frame[prefix+"wrist_wrench_sample_time_ns"] = np.array([int(reading.sample_time_ns)], dtype=np.int64)
                frame[prefix+"wrist_wrench_interval_end_time_ns"] = np.array([int(reading.interval_end_time_ns)], dtype=np.int64)
        if self.frames and identities != self.wrench_identity:
            raise ValueError("wrist sensor availability or frame changed within an episode")
        self.wrench_identity = identities
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
            names, raw_names, pose_names = [], [], []
            for side, arm in arms.items():
                prefix = side+'_' if side else ''
                joint_names = [prefix+f"joint_{i+1}_rad" for i in range(6)]
                if profile:
                    names.extend(joint_names+[prefix+"gripper_opening_normalized"])
                    raw_names.extend(joint_names+[prefix+"ee_link11_joint_rad",prefix+"ee_link12_joint_rad"])
                else:
                    names.extend(joint_names+(["finger_1_m","finger_2_m"] if len(arm.joint_position_rad)==8 else []))
                pose_names.extend(prefix+name for name in ["x_m","y_m","z_m","qw","qx","qy","qz"])
                if arm.wrist_wrench:
                    key='observation.'+(side+'.' if side else '')+'wrist_wrench'
                    features[key]['names']=["Fx_N","Fy_N","Fz_N","Tx_Nm","Ty_Nm","Tz_Nm"]
            if profile:
                for key in ("observation.raw_joint_position", "observation.raw_target_joint_position"):
                    features[key]["names"] = raw_names
                features["observation.joint_velocity"]["names"] = [name+"_s" for name in raw_names]
            features["observation.state"]["names"] = names
            features["action"]["names"] = names
            features["observation.end_effector_pose_body"]["names"] = pose_names
            self.dataset = self.dataset_class.create(
                repo_id=f"local/{self.root.parent.name}", root=self.root,
                robot_type="sarm_dual" if dual else "sarm", fps=self.fps, features=features,
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
                    encoder_preset=self.video_encoding.encoder_preset,
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
            if dual:
                path=self.root / "meta/platform.json"
                metadata=json.loads(path.read_text())
                metadata.update(schema='space-arm-dual-lerobot/1', arm_order=['left','right'],
                    tool_site={'left':'left_sarm_ee','right':'right_sarm_ee'},
                    action_semantics='left six held joint servo targets in radians plus normalized opening, then right; same physics time',
                    state_semantics='left six measured joint angles in radians plus normalized opening, then right; same physics time',
                    wrist_wrench={side:arm.wrist_wrench.model_dump(exclude={'force_n','torque_nm','sample_time_ns','interval_end_time_ns','valid'}) for side,arm in arms.items()})
                path.write_text(json.dumps(metadata,ensure_ascii=False,indent=2))
                self.arm_wrench_streams={side:WrenchStreamWriter(self.root,compact=self.sidecars is not None,stream_name=side+'_wrist_wrench') for side in arms}
        if dual:
            for side,arm in arms.items():
                self.arm_wrench_streams[side].append(arm,sim_time_ns=observation.sim_time_ns)
        else:
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
            for stream in getattr(self, "arm_wrench_streams", {}).values():
                stream.finish()
            if self.sidecars:
                self.sidecars.finish()
            # LeRobot describes shape-(1,) features as scalar Hugging Face Values.
            # Its frame validator requires the arrays, but save_episode stacks
            # them into (frames, 1), which NumPy 2 cannot encode as scalars.
            scalar_keys = ['observation.sim_time_ns', 'observation.source_frame_id', 'observation.applied_action_sequence']
            for prefix in ('observation.', 'observation.left.', 'observation.right.'):
                scalar_keys.extend(prefix+name for name in ('wrist_wrench_valid','wrist_wrench_sample_time_ns','wrist_wrench_interval_end_time_ns'))
            for key in scalar_keys:
                if key not in self.dataset.episode_buffer:
                    continue
                self.dataset.episode_buffer[key] = [
                    value.reshape(()) for value in self.dataset.episode_buffer[key]
                ]
            pixels = self.frames * sum(shape[0] * shape[1] for shape in self.camera_shapes.values())
            parallel_encoding = sys.platform != "win32" or pixels > IN_PROCESS_VIDEO_PIXEL_BUDGET
            # Use the official batch path with the same images, statistics,
            # codec and encoder threads. Large/high-resolution episodes keep
            # parallel encoding; no streaming-statistics approximation is added.
            self.dataset.save_episode(parallel_encoding=parallel_encoding)
            path = self.root / "meta" / "platform.json"
            metadata = json.loads(path.read_text(encoding="utf-8"))
            encoder = getattr(self.dataset, "_streaming_encoder", None)
            if getattr(encoder, "episode_metrics", None) is not None:
                metadata["video_encoding_metrics"] = encoder.episode_metrics
            metadata["high_frequency_wrist_wrench"] = (
                {side:stream.metadata() for side,stream in self.arm_wrench_streams.items()}
                if getattr(self,"arm_wrench_streams",{}) else self.wrench_stream.metadata())
            metadata["episode_result"] = {"outcome": outcome, "note": note}
            path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
        finally:
            self.dataset.finalize()
            self.dataset.stop_image_writer()

    def close_failed(self) -> None:
        if self.dataset is not None:
            self.wrench_stream.finish()
            for stream in getattr(self, "arm_wrench_streams", {}).values():
                stream.finish()
            if self.sidecars:
                self.sidecars.finish()
            self.dataset.finalize()
            self.dataset.stop_image_writer()
