"""Reproducible scene-instance generation and UE/simulation process orchestration."""

from __future__ import annotations

import json
import math
import os
import random
import secrets
import subprocess
import threading
import time
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .models import SceneInstanceCreate


SCENE_TEMPLATES: tuple[dict[str, Any], ...] = (
    {
        "id": "spacecraft-arm-teleop",
        "label": "CubeSat + SO-101 遥操作抓取",
        "description": "自由漂浮 CubeSat、SO-101 机械臂和可抓取目标。",
        "camera_ids": ["teleop/camera/spacecraft_overview", "teleop/camera/so101_wrist_cam"],
    },
)

RANDOMIZATION_PROFILES: tuple[dict[str, Any], ...] = (
    {
        "id": "none",
        "label": "固定基准",
        "description": "使用原始 PREGRASP、目标位姿与速度，便于回归测试。",
    },
    {
        "id": "training-v1",
        "label": "训练随机化 v1",
        "description": "在可抓取邻域内随机目标位置/姿态/自旋和机械臂初始关节。",
    },
)

_NATIVE_TARGET_POSITION = (0.38754456, -0.00109359, 0.42397138)
_NATIVE_TARGET_QUATERNION = (0.99967109, 0.02433362, -0.00809494, 0.00024763)
_NATIVE_TARGET_SPIN = (1.0, 0.0, 0.0)
_NATIVE_COMMON_VELOCITY = (0.01, -0.004, 0.002)
_NATIVE_PREGRASP = (0.0, -0.1790243, 0.2159404, -0.0368382, 0.0, 0.4)
_DEFAULT_EPHEMERIS_EPOCH_UTC = "2026 SEPTEMBER 02 00:00:00.000"
_DEFAULT_EPHEMERIS_CENTER = "Earth"
_DEFAULT_EPHEMERIS_FRAME = "J2000"
_DEFAULT_ORBIT = {
    "altitude_m": 500_000.0,
    "eccentricity": 0.0,
    "inclination_deg": 51.6,
    "raan_deg": 0.0,
    "argument_of_periapsis_deg": 0.0,
    "true_anomaly_deg": 180.0,
}


@dataclass(frozen=True)
class SceneLaunchConfig:
    project_root: Path
    adapter_root: Path
    model_root: Path
    unreal_root: Path
    powershell_exe: Path
    control_port: int
    capture_port: int
    render_port: int
    pixel_streamer_port: int
    pixel_streaming_id: str
    pixel_streaming_camera_ids: tuple[str, ...]
    pixel_streaming_camera_width: int
    pixel_streaming_camera_height: int
    preview_rate: float
    renderer_ready_timeout: int
    ik_rate: float
    simulation_rate: float
    capture_rate: float
    default_dataset_capture: bool


def _normalize_quaternion(values: list[float] | tuple[float, ...]) -> list[float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if norm <= 1.0e-12:
        return [1.0, 0.0, 0.0, 0.0]
    result = [float(value) / norm for value in values]
    if result[0] < 0.0:
        result = [-value for value in result]
    return result


def _multiply_quaternion(a: list[float], b: list[float]) -> list[float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return _normalize_quaternion(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ]
    )


def _sample_instance(request: SceneInstanceCreate, seed: int, created_by: dict[str, Any] | None = None) -> dict[str, Any]:
    rng = random.Random(seed)
    randomization: dict[str, Any] = {
        "target_position_m": list(_NATIVE_TARGET_POSITION),
        "target_orientation_wxyz": list(_NATIVE_TARGET_QUATERNION),
        "target_linear_velocity_m_s": list(_NATIVE_COMMON_VELOCITY),
        "target_angular_velocity_rad_s": list(_NATIVE_TARGET_SPIN),
        "arm_joint_position_rad": list(_NATIVE_PREGRASP),
    }
    if request.randomization_profile == "training-v1":
        randomization["target_position_m"] = [
            _NATIVE_TARGET_POSITION[0] + rng.uniform(-0.018, 0.018),
            _NATIVE_TARGET_POSITION[1] + rng.uniform(-0.020, 0.020),
            _NATIVE_TARGET_POSITION[2] + rng.uniform(-0.018, 0.018),
        ]
        axis = [rng.uniform(-1.0, 1.0) for _ in range(3)]
        axis_norm = math.sqrt(sum(value * value for value in axis)) or 1.0
        axis = [value / axis_norm for value in axis]
        half_angle = math.radians(rng.uniform(-4.0, 4.0)) * 0.5
        perturbation = [math.cos(half_angle), *(value * math.sin(half_angle) for value in axis)]
        randomization["target_orientation_wxyz"] = _multiply_quaternion(
            list(_NATIVE_TARGET_QUATERNION), perturbation
        )
        randomization["target_angular_velocity_rad_s"] = [
            _NATIVE_TARGET_SPIN[0] + rng.uniform(-0.15, 0.15),
            rng.uniform(-0.04, 0.04),
            rng.uniform(-0.04, 0.04),
        ]
        joint_spans = (0.04, 0.035, 0.035, 0.03, 0.04, 0.025)
        randomization["arm_joint_position_rad"] = [
            value + rng.uniform(-span, span)
            for value, span in zip(_NATIVE_PREGRASP, joint_spans, strict=True)
        ]

    return {
        "schema": "space-arm-scene-instance/1",
        "instance_id": f"scene-{time.strftime('%Y%m%d-%H%M%S')}-{uuid.uuid4().hex[:8]}",
        "template_id": request.template_id,
        "randomization_profile": request.randomization_profile,
        "seed": seed,
        "created_at_ns": str(time.time_ns()),
        "created_by": created_by,
        "environment": {
            "ephemeris_epoch_utc": _DEFAULT_EPHEMERIS_EPOCH_UTC,
            "ephemeris_center": _DEFAULT_EPHEMERIS_CENTER,
            "ephemeris_frame": _DEFAULT_EPHEMERIS_FRAME,
            "orbit": dict(_DEFAULT_ORBIT),
        },
        "runtime": {
            "simulation_rate": request.simulation_rate,
            "capture_rate_hz": request.capture_rate_hz,
            "ik_rate_hz": request.ik_rate_hz,
            "dataset_capture": request.dataset_capture,
        },
        "randomization": randomization,
    }


class SceneRuntimeManager:
    """Own at most one runtime launcher and persist its reproducible input."""

    def __init__(self, launch: SceneLaunchConfig | None, project_root: Path | None = None) -> None:
        self.launch = launch
        project_root = launch.project_root if launch else (project_root or Path.cwd())
        self.run_root = project_root / "run"
        self.scene_root = self.run_root / "scenes"
        self.state_path = self.run_root / "scene_runtime.json"
        self.log_root = project_root / "logs"
        self._process: subprocess.Popen[bytes] | None = None
        self._stdout = None
        self._stderr = None
        self._lock = threading.RLock()
        self._last_instance: dict[str, Any] | None = None
        self.scene_root.mkdir(parents=True, exist_ok=True)
        self.log_root.mkdir(parents=True, exist_ok=True)

    @property
    def enabled(self) -> bool:
        return self.launch is not None

    def catalog(self) -> dict[str, Any]:
        return {
            "templates": list(SCENE_TEMPLATES),
            "randomization_profiles": list(RANDOMIZATION_PROFILES),
            "defaults": {
                "template_id": "spacecraft-arm-teleop",
                "randomization_profile": "training-v1",
                "simulation_rate": self.launch.simulation_rate if self.launch else 1.0,
                "capture_rate_hz": self.launch.capture_rate if self.launch else 10.0,
                "ik_rate_hz": self.launch.ik_rate if self.launch else 100.0,
                "dataset_capture": self.launch.default_dataset_capture if self.launch else False,
            },
        }

    def create_instance(self, request: SceneInstanceCreate, created_by: dict[str, Any] | None = None) -> dict[str, Any]:
        if request.template_id not in {item["id"] for item in SCENE_TEMPLATES}:
            raise ValueError(f"unknown scene template: {request.template_id}")
        if request.randomization_profile not in {item["id"] for item in RANDOMIZATION_PROFILES}:
            raise ValueError(f"unknown randomization profile: {request.randomization_profile}")
        seed = request.seed if request.seed is not None else secrets.randbelow(2**31)
        instance = _sample_instance(request, seed, created_by)
        path = self.scene_root / f"{instance['instance_id']}.json"
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(instance, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        os.replace(temporary, path)
        instance["config_path"] = str(path.resolve())
        self._last_instance = instance
        return instance

    def start(self, request: SceneInstanceCreate, created_by: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self.launch:
            raise RuntimeError("scene runtime launching is not configured")
        if not self.launch.powershell_exe.is_file():
            raise RuntimeError(
                f"PowerShell executable does not exist: {self.launch.powershell_exe}"
            )
        launch_script = self.launch.project_root / "scripts" / "start_scene_instance.ps1"
        if not launch_script.is_file():
            raise RuntimeError(f"scene launch script does not exist: {launch_script}")
        with self._lock:
            status = self.status()
            if status["active"]:
                raise RuntimeError("a scene instance is already starting or running")
            instance = self.create_instance(request, created_by)
            instance_path = Path(instance["config_path"])
            self.state_path.unlink(missing_ok=True)
            script = launch_script
            command = [
                str(self.launch.powershell_exe), "-NoProfile", "-ExecutionPolicy", "Bypass",
                "-File", str(script),
                "-SceneInstancePath", str(instance_path),
                "-AdapterRoot", str(self.launch.adapter_root),
                "-ModelRoot", str(self.launch.model_root),
                "-UnrealRoot", str(self.launch.unreal_root),
                "-ControlPort", str(self.launch.control_port),
                "-CapturePort", str(self.launch.capture_port),
                "-RenderPort", str(self.launch.render_port),
                "-PixelStreamerPort", str(self.launch.pixel_streamer_port),
                "-PixelStreamingId", self.launch.pixel_streaming_id,
                "-PixelStreamingCameraIds", ",".join(self.launch.pixel_streaming_camera_ids),
                "-PixelStreamingCameraWidth", str(self.launch.pixel_streaming_camera_width),
                "-PixelStreamingCameraHeight", str(self.launch.pixel_streaming_camera_height),
                "-PreviewRate", str(self.launch.preview_rate),
                "-RendererReadyTimeout", str(self.launch.renderer_ready_timeout),
            ]
            stamp = instance["instance_id"]
            self._stdout = (self.log_root / f"{stamp}.launcher.out.log").open("wb")
            self._stderr = (self.log_root / f"{stamp}.launcher.err.log").open("wb")
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            try:
                self._process = subprocess.Popen(
                    command,
                    cwd=self.launch.project_root,
                    stdout=self._stdout,
                    stderr=self._stderr,
                    creationflags=flags,
                )
            except OSError as error:
                self._close_logs()
                self._process = None
                raise RuntimeError(f"unable to launch scene supervisor: {error}") from error
            return {**instance, "phase": "launching", "launcher_pid": self._process.pid}

    def stop(self) -> dict[str, Any]:
        if not self.launch:
            raise RuntimeError("scene runtime launching is not configured")
        with self._lock:
            script = self.launch.project_root / "scripts" / "stop_scene_instance.ps1"
            flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
            subprocess.run(
                [str(self.launch.powershell_exe), "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(script), "-Quiet"],
                cwd=self.launch.project_root,
                check=False,
                creationflags=flags,
            )
            if self._process and self._process.poll() is None:
                try:
                    self._process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    self._process.kill()
            self._close_logs()
            self._process = None
            return self.status()

    def status(self) -> dict[str, Any]:
        with self._lock:
            state: dict[str, Any] = {}
            if self.state_path.is_file():
                try:
                    state = json.loads(self.state_path.read_text(encoding="utf-8-sig"))
                except (OSError, ValueError):
                    state = {"phase": "unknown", "error": "scene runtime state is unreadable"}
            process_running = self._process is not None and self._process.poll() is None
            if self._process is not None and not process_running:
                state.setdefault("launcher_exit_code", self._process.returncode)
                if state.get("phase") in {"launching", "starting_renderer", "starting_simulation", "running"}:
                    state["phase"] = "failed" if self._process.returncode else "completed"
                self._close_logs()
            if process_running and not state:
                state["phase"] = "launching"
            phase = str(state.get("phase", "idle"))
            state["enabled"] = self.enabled
            state["active"] = process_running or phase in {"launching", "starting_renderer", "starting_simulation", "running"}
            if self._last_instance and "instance" not in state:
                state["instance"] = self._last_instance
            return state

    def close(self) -> None:
        if self.enabled and self.status()["active"]:
            self.stop()
        self._close_logs()

    def _close_logs(self) -> None:
        for stream_name in ("_stdout", "_stderr"):
            stream = getattr(self, stream_name)
            if stream:
                stream.close()
                setattr(self, stream_name, None)
