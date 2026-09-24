"""Reliable telemetry and latest-command transport; never accesses physics."""
from __future__ import annotations
from collections import OrderedDict
import logging
import math
import select
import socket
import threading
import time
import uuid
from typing import Any
from space_arm_platform.protocol import CONTROL_PROTOCOL, encode_packet, FramedSocketReader

class SimulationControlClient:
    """Latest actions plus bounded ACK/replay delivery of every authoritative state."""

    def __init__(self, host: str, port: int, simulation_id: str, *,
                 max_pending_observations: int = 128, observation_timeout_s: float = 30.) -> None:
        self.host = host
        self.port = int(port)
        self.simulation_id = simulation_id
        self._lock = threading.RLock()
        self._send_lock = threading.Lock()
        self._action: dict[str, Any] = self._neutral_action()
        self._received_monotonic = 0.0
        self._reset_generation = ""
        self._pending_reset: str | None = None
        self._resetting = False
        self._socket: socket.socket | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._pending_condition = threading.Condition(self._lock)
        self._pending_observations = OrderedDict()
        self._max_pending = max_pending_observations
        self._observation_timeout_s = observation_timeout_s
        self._stream_id = uuid.uuid4().hex
        self._next_sequence = 1
        self._acked_sequence = 0
        self._peer_ready = False
        self._transport_error = None
        self._reconnects = 0
        self._capture_state = {"capture_episode_id": "", "capture_request_id": ""}


    @property
    def connected(self) -> bool:
        with self._lock:
            return self._socket is not None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._worker, name="teleop-action-receiver", daemon=True)
        self._thread.start()

    def wait_until_ready(self, timeout: float = 10.0) -> None:
        """Do not build/advance physics until the backend accepts reliable telemetry."""
        with self._pending_condition:
            self._pending_condition.wait_for(
                lambda: self._stop.is_set() or (self._socket is not None and self._peer_ready), timeout=timeout)
            if self._stop.is_set() or self._socket is None or not self._peer_ready:
                raise RuntimeError(
                    "Backend did not confirm reliable_observations_v1 before physics startup; "
                    "restart/update the whole platform backend, not only the UE scene. "
                    f"Transport detail: {self._transport_error or 'handshake not received'}")

    def close(self) -> None:
        self._stop.set()
        with self._pending_condition:
            self._pending_condition.notify_all()
        with self._lock:
            connection = self._socket
            self._socket = None
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            connection.close()
        if self._thread:
            self._thread.join(timeout=2.0)

    def capture_state(self) -> dict[str, str]:
        """Sampled once by the render bridge, then shared with its observation."""
        with self._lock:
            return dict(self._capture_state)

    def latest_action(self) -> tuple[dict[str, Any], bool]:
        with self._lock:
            action = dict(self._action)
            action["end_effector_linear_velocity_body_m_s"] = list(
                self._action["end_effector_linear_velocity_body_m_s"]
            )
            action["end_effector_angular_velocity_body_rad_s"] = list(
                self._action["end_effector_angular_velocity_body_rad_s"]
            )
            stale = self._resetting or time.monotonic() - self._received_monotonic > 0.25
        if stale:
            action["deadman"] = False
            action["end_effector_linear_velocity_body_m_s"] = [0.0] * 3
            action["end_effector_angular_velocity_body_rad_s"] = [0.0] * 3
            action["gripper_velocity_m_s"] = 0.0
            action["gripper_velocity_rad_s"] = 0.0
        return action, stale

    @property
    def reset_generation(self) -> str:
        with self._lock:
            return self._reset_generation

    def take_reset(self) -> str | None:
        """Only the simulation thread may consume a reset, between advances."""
        with self._lock:
            request_id = self._pending_reset
            if request_id is not None:
                self._pending_reset = None
                self._reset_generation = request_id
            return request_id

    def complete_reset(self) -> None:
        with self._lock:
            # A reset received during initialization must stay pending.
            if self._pending_reset is None:
                self._resetting = False
            self._action = self._neutral_action()
            self._received_monotonic = 0.0

    def _accept_message(self, message: dict[str, Any]) -> None:
        with self._lock:
            if message.get("protocol") == CONTROL_PROTOCOL and message.get("type") == "capture_control":
                episode = message.get("episode_id")
                request = message.get("request_id")
                if (isinstance(episode, str) and len(episode) <= 128
                        and isinstance(request, str) and 1 <= len(request) <= 64):
                    self._capture_state = {"capture_episode_id": episode, "capture_request_id": request}
                return
            if message.get("protocol") == CONTROL_PROTOCOL and message.get("type") in {"observation_ack", "observation_ready"}:
                if message.get("observation_stream_id") != self._stream_id:
                    raise ValueError("observation ACK belongs to another stream")
                sequence = int(message.get("observation_sequence", "-1"))
                if not 0 <= sequence < self._next_sequence:
                    raise ValueError("observation ACK exceeds published sequence")
                self._peer_ready = True
                self._acked_sequence = max(self._acked_sequence, sequence)
                while self._pending_observations and next(iter(self._pending_observations)) <= self._acked_sequence:
                    self._pending_observations.popitem(last=False)
                self._transport_error = None
                self._pending_condition.notify_all()
                return

            if message.get("protocol") == CONTROL_PROTOCOL and message.get("type") == "reset":
                request_id = message.get("request_id")
                if not isinstance(request_id, str) or not 1 <= len(request_id) <= 64:
                    return
                if request_id == self._reset_generation or self._resetting:
                    return  # Reconnect/retry cannot reset an already applied generation twice.
                self._pending_reset = request_id
                self._resetting = True
                self._action = self._neutral_action()
                self._received_monotonic = 0.0
            elif (not self._resetting and self._valid_action(message)
                  and message.get("reset_generation", "") == self._reset_generation):
                self._action = message
                self._received_monotonic = time.monotonic()

    def send_observation(self, message: dict[str, Any]) -> bool:
        """Retain until acknowledged. Backpressure pauses physics, never loses a state."""
        with self._pending_condition:
            ready = self._pending_condition.wait_for(
                lambda: self._stop.is_set() or len(self._pending_observations) < self._max_pending,
                timeout=self._observation_timeout_s)
            if self._stop.is_set():
                raise RuntimeError("authoritative observation transport stopped before enqueue")
            if not ready:
                raise RuntimeError(f"authoritative observation ACK timeout: pending={len(self._pending_observations)}, "
                                   f"last_acked={self._acked_sequence}, error={self._transport_error}; refusing to drop a state")
            sequence = self._next_sequence
            packet = {**message, "observation_stream_id": self._stream_id, "observation_sequence": str(sequence)}
            self._pending_observations[sequence] = encode_packet(packet)
            self._next_sequence += 1
            return True

    def observation_transport_status(self):
        with self._lock:
            return {"observation_pending_ack": len(self._pending_observations),
                    "observation_last_acked": self._acked_sequence,
                    "observation_reconnects": self._reconnects,
                    "observation_transport_error": self._transport_error}

    def _worker(self) -> None:
        while not self._stop.is_set():
            connection = None
            try:
                connection = socket.create_connection((self.host, self.port), timeout=1.)
                connection.settimeout(1.)
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                reader = FramedSocketReader()
                with self._lock:
                    self._peer_ready = False
                    self._reconnects += 1
                    cursor = self._acked_sequence
                    hello = {"protocol": CONTROL_PROTOCOL, "type": "sim_hello", "simulation_id": self.simulation_id,
                             "reset_generation": self._reset_generation, "observation_stream_id": self._stream_id,
                             "observation_resume_after": str(self._acked_sequence),
                             "capabilities": ["reliable_observations_v1", "capture_on_demand_v1", "scene_reset", "arm_preparation",
                                 "cartesian_twist_6d", "damped_least_squares_ik", "gripper_velocity",
                                 "joint_observation", "cartesian_observation", "sarm_si_joint_observation",
                                 "reaction_wheel_attitude_control"]}
                    self._socket = connection
                connection.sendall(encode_packet(hello))
                handshake_deadline = time.monotonic() + 3.
                while not self._stop.is_set():
                    if not self._peer_ready and time.monotonic() >= handshake_deadline:
                        raise ConnectionError("backend did not negotiate reliable_observations_v1; restart/update backend")
                    with self._lock:
                        packets = [(seq, data) for seq, data in self._pending_observations.items()
                                   if self._peer_ready and seq > cursor][:8]
                    for sequence, data in packets:
                        connection.sendall(data)
                        cursor = sequence
                    # Reader retains bytes even when a payload spans receive timeouts.
                    if reader.buffer or select.select([connection], [], [], .01)[0]:
                        try:
                            self._accept_message(reader.receive(connection))
                        except socket.timeout:
                            continue
            except (OSError, EOFError, ValueError) as error:
                with self._lock:
                    self._transport_error = f"{type(error).__name__}: {error}"
                logging.getLogger(__name__).warning("Observation/control connection interrupted; retaining unacknowledged states: %s", error)
            finally:
                if connection:
                    self._drop(connection)
            self._stop.wait(.25)

    def _drop(self, connection: socket.socket) -> None:
        with self._lock:
            if self._socket is connection:
                self._socket = None
                self._peer_ready = False
                self._pending_condition.notify_all()
                self._action = self._neutral_action()
                self._received_monotonic = 0.0
        try:
            connection.close()
        except OSError:
            pass

    @staticmethod
    def _valid_action(message: dict[str, Any]) -> bool:
        preparation = message.get("arm_preparation")
        if preparation is not None:
            if not isinstance(preparation, dict):
                return False
            goal = preparation.get("joint_position_deg")
            request_id = preparation.get("request_id")
            if (not isinstance(request_id, str) or not 1 <= len(request_id) <= 80
                    or not isinstance(goal, list) or len(goal) != 6
                    or not all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x) for x in goal)):
                return False
        linear = message.get("end_effector_linear_velocity_body_m_s")
        angular = message.get("end_effector_angular_velocity_body_rad_s")
        return (
            message.get("protocol") == CONTROL_PROTOCOL
            and message.get("type") == "action"
            and message.get("control_frame", "spacecraft_body") == "spacecraft_body"
            and isinstance(linear, list)
            and len(linear) == 3
            and isinstance(angular, list)
            and len(angular) == 3
            and all(
                isinstance(value, (int, float)) and math.isfinite(value)
                for value in [*linear, *angular]
            )
            and (
                isinstance(message.get("gripper_velocity_m_s"), (int, float))
                or isinstance(message.get("gripper_velocity_rad_s"), (int, float))
            )
        )

    @staticmethod
    def _neutral_action() -> dict[str, Any]:
        return {
            "protocol": CONTROL_PROTOCOL,
            "type": "action",
            "server_sequence": "0",
            "deadman": False,
            "control_frame": "spacecraft_body",
            "end_effector_linear_velocity_body_m_s": [0.0] * 3,
            "end_effector_angular_velocity_body_rad_s": [0.0] * 3,
            "gripper_velocity_m_s": 0.0,
            "gripper_velocity_rad_s": 0.0,
        }
