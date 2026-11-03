"""Basilisk ``SysModel`` that publishes renderer-neutral scene and state data."""

from __future__ import annotations

import os
import sys
import time
import uuid
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping

import numpy as np

from .descriptors import CameraVisual, SceneSettings, VisualElement
from .frames import FrameConverter
from .protocol import PROTOCOL_V2, RenderPublisher
from .recording import BskRecordingWriter


_conda_dll_directory = None
if os.name == "nt" and hasattr(os, "add_dll_directory"):
    _conda_library_bin = Path(sys.prefix) / "Library" / "bin"
    if _conda_library_bin.is_dir():
        _conda_dll_directory = os.add_dll_directory(str(_conda_library_bin))

try:
    from Basilisk.architecture import sysModel
except ImportError:
    sysModel = None


class _MissingBasiliskBase:
    def __init__(self, *_: Any, **__: Any) -> None:
        raise RuntimeError("Basilisk is unavailable; use the mujoco-dev Conda environment")


_BridgeBase = sysModel.SysModel if sysModel is not None else _MissingBasiliskBase


from . import scene_description
from .command_dispatch import CommandDispatch
from .device_registration import DeviceRegistration


class BasiliskRenderBridge(scene_description.SceneRegistration, DeviceRegistration, CommandDispatch, _BridgeBase):
    """Publish Basilisk state without making rendering simulation-critical."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 5558,
        *,
        origin_object: str | None = None,
        c_l_n: Iterable[Iterable[float]] | None = None,
        publisher: RenderPublisher | Any | None = None,
        reliable_frames: bool = False,
        capture_state_provider: Callable[[], dict[str, str]] | None = None,
        recording_path: str | Path | None = None,
        frame_period_ns: int | None = None,
        frame_rate_hz: int | None = None,
    ) -> None:
        super().__init__()
        if frame_period_ns is not None and int(frame_period_ns) <= 0:
            raise ValueError("frame_period_ns must be positive when provided")
        if frame_rate_hz is not None and (frame_rate_hz <= 0 or int(frame_rate_hz) != frame_rate_hz):
            raise ValueError("frame_rate_hz must be a positive integer")
        if frame_period_ns is not None and frame_rate_hz is not None:
            raise ValueError("choose frame_period_ns or frame_rate_hz, not both")
        self.frame_rate_hz = int(frame_rate_hz) if frame_rate_hz is not None else None
        self.ModelTag = "BasiliskRenderBridge"
        self.session_id = str(uuid.uuid4())
        self.origin_object = origin_object
        self.converter = FrameConverter(c_l_n)
        self.publisher = publisher or RenderPublisher(host, port, reliable_frames=reliable_frames)
        self.capture_state_provider = capture_state_provider
        self.last_capture_state: dict[str, str] = {}
        self.recorder = BskRecordingWriter(recording_path) if recording_path else None
        self._objects: list[scene_description.ObjectBinding] = []
        self._celestial: list[scene_description.CelestialBinding] = []
        self._visuals: list[VisualElement] = []
        self._cameras: list[CameraVisual] = []
        self._settings = SceneSettings(origin_object_id=origin_object or "")
        self._ui_settings: dict[str, list[dict[str, Any]]] = {
            "actuators": [],
            "instruments": [],
            "commands": [],
        }
        self._command_handlers: dict[str, Callable[[Mapping[str, Any], int], Mapping[str, Any] | None]] = {}
        self._frame_id = 0
        self._last_published_sim_time_ns = -1
        self._manifest_revision = 1
        self.frame_period_ns = int(frame_period_ns) if frame_period_ns is not None else None
        self._next_frame_ns = 0
        self._event_sequence = 0
        self.register_command_handler(
            "renderer.ping",
            lambda payload, sim_time_ns: {"echo": dict(payload), "sim_time_ns": str(sim_time_ns)},
            label="Ping BSK link",
            show_in_ui=False,
        )
        self.register_command_handler(
            "renderer.request_manifest",
            self._handle_manifest_request,
            label="Resend scene manifest",
            show_in_ui=False,
        )

    @property
    def last_published_frame_id(self) -> int:
        """Return the last authoritative render frame ID, or ``-1`` before Update."""

        return self._frame_id - 1

    @property
    def last_published_sim_time_ns(self) -> int:
        """Return the exact simulation timestamp carried by the last render frame."""

        return self._last_published_sim_time_ns


    def _record_and_retain_static(self) -> None:
        hello = self._hello_message()
        manifest = self._manifest_message()
        self.publisher.retain_hello(hello)
        self.publisher.retain_manifest(manifest)
        if self.recorder:
            self.recorder.write(hello)
            self.recorder.write(manifest)

    def Reset(self, CurrentSimNanos: int) -> None:
        """Start a new stream session and publish retained scene data."""

        self._frame_id = 0
        self._last_published_sim_time_ns = -1
        self._next_frame_ns = int(CurrentSimNanos)
        self._record_and_retain_static()
        self.publish_event(
            "session_started",
            {
                "severity": "info",
                "message": "Basilisk render session started",
                "sim_time_ns": str(int(CurrentSimNanos)),
            },
        )

    def _origin(self, raw_objects: list[dict[str, Any]]) -> np.ndarray:
        if not raw_objects:
            return np.zeros(3)
        if not self.origin_object:
            return np.asarray(raw_objects[0]["position_N_m"], dtype=np.float64)
        candidates = [self.origin_object]
        candidates.extend(item.object_id for item in self._objects if item.display_name == self.origin_object)
        for item in raw_objects:
            if item["object_id"] in candidates:
                return np.asarray(item["position_N_m"], dtype=np.float64)
        raise ValueError(f"origin object is not registered: {self.origin_object}")

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Sample registered messages and enqueue the newest render frame."""

        current_sim_ns = int(CurrentSimNanos)
        self._dispatch_commands(current_sim_ns)
        if self.frame_rate_hz is not None:
            rate = self.frame_rate_hz
            tick = (current_sim_ns * rate + 500_000_000) // 1_000_000_000
            expected = (tick * 1_000_000_000 + rate // 2) // rate
            if current_sim_ns != expected or self._last_published_sim_time_ns == current_sim_ns:
                return
        if self.frame_period_ns is not None and current_sim_ns < self._next_frame_ns:
            return

        states = [binding.reader() for binding in self._objects]
        position_values = [state.r_BN_N for state in states]
        origin_n = self._origin(
            [{"object_id": binding.object_id, "position_N_m": position}
             for binding, position in zip(self._objects, position_values)]
        )
        objects = []
        if states:
            # One batched conversion for all bodies (same checks as the per-body path).
            positions, orientations, velocities, rates = self.converter.bodies(
                position_values, [state.v_BN_N for state in states],
                [state.sigma_BN for state in states], [state.omega_BN_B for state in states], origin_n)
            objects = [
                {
                    "object_id": binding.object_id,
                    "position_m": position,
                    "orientation_wxyz": orientation,
                    "velocity_mps": velocity,
                    "angular_velocity_B_radps": rate,
                }
                for binding, position, orientation, velocity, rate in zip(
                    self._objects, positions.tolist(), orientations.tolist(), velocities.tolist(), rates.tolist())
            ]
        celestial = []
        for binding in self._celestial:
            state = binding.reader()
            celestial.append(
                {
                    "body_id": binding.visual.body_id,
                    "position_m": self.converter.position(state.PositionVector, origin_n).tolist(),
                    "velocity_mps": self.converter.velocity(state.VelocityVector).tolist(),
                    "orientation_wxyz": self.converter.fixed_orientation(
                        np.asarray(state.J20002Pfix, dtype=np.float64).reshape((3, 3))
                    ).tolist(),
                }
            )
        visual_states = []
        for visual in self._visuals:
            if visual.state_provider is not None:
                if visual.properties.get("state_provider_accepts_sim_time", False):
                    state = dict(visual.state_provider(int(CurrentSimNanos)))
                else:
                    state = dict(visual.state_provider())
                channels = state.get("channels")
                if channels is None:
                    channels = {
                        key: value
                        for key, value in state.items()
                        if key not in {"visible", "value", "status"}
                    }
                visual_states.append(
                    {
                        "visual_id": visual.visual_id,
                        **state,
                        "channels": dict(channels),
                    }
                )
        message = {
            "protocol": PROTOCOL_V2,
            "type": "frame",
            "session_id": self.session_id,
            "manifest_revision": str(self._manifest_revision),
            "frame_id": str(self._frame_id),
            "sim_time_ns": str(int(CurrentSimNanos)),
            "wall_time_ns": str(time.time_ns()),
            "origin_N_m": origin_n.tolist(),
            "c_LN": self.converter.c_l_n.reshape(9).tolist(),
            "objects": objects,
            "celestial_bodies": celestial,
            "visual_states": visual_states,
        }
        # Read the control state exactly once on the simulation thread. Its
        # episode/request IDs accompany both this render frame and its snapshot.
        capture_state = {}
        if self.capture_state_provider is not None:
            state = self.capture_state_provider()
            # This hook can annotate capture identity, never override poses,
            # timestamps or other authoritative frame payload fields.
            for key, limit in (("capture_episode_id", 128), ("capture_request_id", 64)):
                value = state.get(key, "")
                if not isinstance(value, str) or len(value) > limit:
                    raise ValueError(f"invalid {key}")
                capture_state[key] = value
        message.update(capture_state)
        self.publisher.publish_frame(message)
        self.last_capture_state = capture_state
        if self.recorder:
            self.recorder.write(message)
        self._last_published_sim_time_ns = current_sim_ns
        self._frame_id += 1
        if self.frame_period_ns is not None:
            elapsed = max(0, current_sim_ns - self._next_frame_ns)
            periods = elapsed // self.frame_period_ns + 1
            self._next_frame_ns += periods * self.frame_period_ns

    def publish_event(self, event_kind: str, payload: Mapping[str, Any] | None = None) -> bool:
        """Publish one bounded reliable event."""

        self._event_sequence += 1
        message = {
            "protocol": PROTOCOL_V2,
            "type": "event",
            "session_id": self.session_id,
            "sequence": str(self._event_sequence),
            "event_kind": event_kind,
            "payload": dict(payload or {}),
        }
        if self.recorder:
            self.recorder.write(message)
        return self.publisher.publish_event(message)

    def close(self) -> None:
        """Close network and recording resources."""

        self.publisher.close()
        if self.recorder:
            self.recorder.close()
