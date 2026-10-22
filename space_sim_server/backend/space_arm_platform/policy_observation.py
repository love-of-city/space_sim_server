"""Bounded authoritative RGB/state pairing for online policies, independent of recording."""
from __future__ import annotations

import base64
import threading
from collections import OrderedDict
from typing import Any

from .models import SimulationObservation

POLICY_CAMERAS = {
    "base": "teleop/camera/spacecraft_overview",
    "left_wrist": "teleop/camera/sarm_wrist_cam",
}
PERIOD_NS = 20_000_000


def observation_identity(value: dict) -> tuple[str, str, str]:
    session = value.get("render_session_id")
    frame, stamp = str(value.get("render_frame_id", "")), str(value.get("sim_time_ns", ""))
    if not isinstance(session, str) or not session or not frame.isdecimal() or not stamp.isdecimal():
        raise ValueError("authoritative observation requires session, decimal frame and simulation time")
    return session, frame, stamp


class PolicyObservationCache:
    """The two cameras and the state must share all three identity fields.

    CaptureReceiver calls from its worker thread; SimulationHub calls from asyncio.
    Never wait on this condition on the event loop or from a physics callback.
    """

    def __init__(self, *, capacity: int = 32, timeout: float = 45.0):
        self.capacity = capacity
        self.timeout = timeout
        self._condition = threading.Condition()
        self._capture_id = ""
        self._generation = 0
        self._entries: OrderedDict[tuple, dict] = OrderedDict()
        self._error: str | None = None

    def begin(self, capture_id: str) -> None:
        if not capture_id:
            raise ValueError("policy capture lease must have an identity")
        with self._condition:
            self._generation += 1
            self._capture_id = capture_id
            self._entries.clear()
            self._error = None
            self._condition.notify_all()

    def end(self) -> None:
        with self._condition:
            self._generation += 1
            self._capture_id = ""
            self._entries.clear()
            self._condition.notify_all()

    def fail(self, reason: str) -> None:
        with self._condition:
            if self._capture_id:
                self._error = reason
                self._condition.notify_all()

    def _entry(self, key):
        entry = self._entries.setdefault(key, {"state": None, "images": {}})
        # Bound both matched and early RGB packets. A missing requested identity
        # fails explicitly; it is never replaced with a newer state or image.
        while len(self._entries) > self.capacity:
            self._entries.popitem(last=False)
        return entry

    def record_observation(self, observation: SimulationObservation) -> None:
        with self._condition:
            if not self._capture_id or observation.capture_episode_id != self._capture_id:
                return
            if int(observation.sim_time_ns) % PERIOD_NS:
                self.fail("policy observations must lie on the 50 Hz simulation clock")
                return
            key = observation_identity(observation.model_dump())
            self._entry(key)["state"] = observation.model_copy(deep=True)
            self._condition.notify_all()

    def record_capture(self, metadata: dict, products: dict[str, bytes]) -> None:
        with self._condition:
            if not self._capture_id or metadata.get("capture_episode_id") != self._capture_id:
                return
            if metadata.get("type") == "capture_error":
                self.fail(str(metadata.get("error", "authoritative RGB capture failed")))
                return
            camera = str(metadata.get("camera_id", ""))
            if camera not in POLICY_CAMERAS.values():
                return
            if metadata.get("stream_kind") != "authoritative" or metadata.get("state_kind") != "authoritative":
                self.fail("online policy requires authoritative camera frames")
                return
            key = observation_identity({"render_session_id": metadata.get("session_id"),
                                        "render_frame_id": metadata.get("source_frame_id"),
                                        "sim_time_ns": metadata.get("sim_time_ns")})
            rgb = products.get("rgb")
            resolution = metadata.get("resolution", [])
            descriptor = next((p for p in metadata.get("products", []) if p.get("name") == "rgb"), {})
            filename = str(descriptor.get("file_name", "")).lower()
            encoding = "jpeg" if filename.endswith((".jpg", ".jpeg")) else "png" if filename.endswith(".png") else ""
            if (not rgb or not encoding or len(resolution) != 2
                    or any(type(v) is not int or not 0 < v <= 4096 for v in resolution)):
                self.fail("authoritative RGB packet has invalid encoding or resolution")
                return
            images = self._entry(key)["images"]
            if camera in images:
                self.fail("duplicate authoritative policy camera frame")
                return
            images[camera] = {"camera_id": camera, "encoding": encoding, "bytes": rgb,
                              "width": resolution[0], "height": resolution[1]}
            self._condition.notify_all()

    def wait_for_pair(self, observation: SimulationObservation) -> None:
        with self._condition:
            if not self._capture_id or observation.capture_episode_id != self._capture_id:
                return
        self.get(observation_identity(observation.model_dump()), encode_images=False)

    def get(self, key: tuple, *, encode_images: bool = True) -> dict[str, Any]:
        with self._condition:
            lease, generation = self._capture_id, self._generation
            def ready():
                entry = self._entries.get(key, {})
                return (self._error or generation != self._generation or lease != self._capture_id or not lease
                        or (entry.get("state") is not None
                            and set(POLICY_CAMERAS.values()).issubset(entry.get("images", {}))))
            if not self._condition.wait_for(ready, timeout=self.timeout):
                self.fail(f"timed out pairing policy RGB and state: frame={key[1]}, sim_time_ns={key[2]}")
            if self._error:
                raise RuntimeError(self._error)
            if not lease or generation != self._generation or lease != self._capture_id:
                raise RuntimeError("policy capture lease ended while waiting for observation")
            entry = self._entries[key]
            state = entry["state"]
            if not encode_images:
                return {}
            images = {view: dict(entry["images"][camera]) for view, camera in POLICY_CAMERAS.items()}
        opening = state.gripper_opening_normalized
        if opening is None:
            raise RuntimeError("policy requires the calibrated SARM 6+1 gripper representation")
        for image in images.values():
            image["data"] = base64.b64encode(image.pop("bytes")).decode("ascii")
        return {
            "render_session_id": key[0], "render_frame_id": key[1], "sim_time_ns": key[2],
            "state": list(state.joint_position_rad[:6]) + [opening], "images": images,
            "applied_action": list(state.target_joint_position_rad[:6]) + [state.target_gripper_opening_normalized],
            "wrist_wrench": state.wrist_wrench.model_dump() if state.wrist_wrench else None,
            "policy_task_state": getattr(state, "policy_task_state", {}),
            "model_sha256": state.model_sha256,
        }
