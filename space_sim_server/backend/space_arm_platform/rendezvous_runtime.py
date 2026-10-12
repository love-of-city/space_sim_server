"""Launch team-model orbital flights and retain the historical docking demo."""

from __future__ import annotations

import json
import os
import subprocess
import threading
import uuid
import socket
import time
from pathlib import Path

from .rendezvous_demo import RendezvousDemo
from .rendezvous_config import FlightOptions


class RendezvousRuntime:
    def __init__(
        self,
        repository: Path | None,
        python: Path | None,
        state_port: int,
        log_root: Path,
        team_repository: Path | None = None,
        render_port: int = 5558,
        launch_config=None,
    ):
        self.repository = repository
        self.python = python
        self.state_port = state_port
        self.render_port = render_port
        self.log_root = log_root
        self.team_repository = team_repository
        self.launch_config = launch_config
        self._renderer_identity = None
        self._options = FlightOptions()
        self._lock = threading.RLock()
        self._cancel_start = threading.Event()
        self._scenario = "legacy"
        self._process: subprocess.Popen | None = None
        self._log = None
        self._log_path: Path | None = None
        self._output_dir: Path | None = None
        self._demo: RendezvousDemo | None = None
        self._mode = "simulation"
        if repository is not None:
            runs = repository / "output" / "spacesim_platform"
            if runs.is_dir():
                completed = [
                    folder
                    for folder in runs.iterdir()
                    if (folder / "mission_report.json").is_file()
                ]
                if completed:
                    self._output_dir = max(
                        completed,
                        key=lambda folder: (
                            (folder / "mission_report.json").stat().st_mtime_ns
                        ),
                    )
        if team_repository is not None:
            team_runs = team_repository / "run" / "team_rendezvous"
            if team_runs.is_dir():
                reports = list(team_runs.glob("*/mission_report.json"))
                if reports:
                    latest = max(reports, key=lambda p: p.stat().st_mtime_ns)
                    self._output_dir = latest.parent
                    try:
                        saved = json.loads(latest.read_text(encoding="utf-8"))
                        self._scenario = "team-" + saved["scenario"]
                        self._options = FlightOptions.from_dict(saved.get("mission_options", {}))
                    except (OSError, ValueError, KeyError):
                        pass

    @property
    def output_dir(self) -> Path | None:
        return self._output_dir

    def prepare_start(self):
        self._cancel_start.clear()

    def cancel_start(self):
        self._cancel_start.set()

    def _check_start_cancelled(self):
        if self._cancel_start.is_set():
            raise RuntimeError("交会启动已取消")

    def status(self) -> dict:
        if self._mode == "demo" and self._demo is not None:
            demo = self._demo
            source = demo.data["source_report"]
            report = (
                {
                    "outcome": source["outcome"],
                    "basic_nominal_passed": source["basic_nominal_passed"],
                    "latch_time_s": source["capture"]["latch_time_s"],
                    "final_time_s": source["final_time_s"],
                    "final_body_origin_distance_m": source[
                        "final_body_origin_distance_m"
                    ],
                }
                if demo.completed
                else None
            )
            return {
                "configured": True,
                "mode": "demo",
                "demo_available": True,
                "running": demo.running,
                "completed": demo.completed,
                "pid": None,
                "exit_code": 0 if demo.completed else None,
                "error": demo.error,
                "elapsed_s": demo.elapsed,
                "duration_s": 15,
                "run_id": demo.run_id,
                "started_at": demo.started_at,
                "finished_at": demo.finished_at,
                "output_dir": str(demo.source),
                "log_file": None,
                "report": report,
            }
        process = self._process
        report_path = (
            self._output_dir / "mission_report.json" if self._output_dir else None
        )
        report = None
        if report_path and report_path.is_file():
            try:
                result = json.loads(report_path.read_text(encoding="utf-8"))
                report = {
                    "scenario": result.get("scenario", "legacy"),
                    "outcome": result.get("outcome"),
                    "basic_nominal_passed": result.get("basic_nominal_passed"),
                    "latch_time_s": (result.get("capture") or {}).get("latch_time_s"),
                    "final_time_s": result.get("final_time_s"),
                    "final_body_origin_distance_m": result.get(
                        "final_body_origin_distance_m"
                    ),
                    "final_center_distance_m": result.get("final_center_distance_m"),
                }
            except (OSError, ValueError):
                pass
        return {
            "scenario": self._scenario,
            "mode": "simulation",
            "demo_available": bool(
                self.repository
                and (self.repository / "output/spacesim_demo15/demo.json.gz").is_file()
            ),
            "configured": (
                self.repository is not None or self.team_repository is not None
            )
            and self.python is not None,
            "running": process is not None and process.poll() is None,
            "pid": process.pid if process else None,
            "exit_code": process.poll() if process else None,
            "output_dir": str(self._output_dir) if self._output_dir else None,
            "log_file": str(self._log_path) if self._log_path else None,
            "report": report,
            "mission_options": self._options.to_dict(),
        }

    def start(
        self, live: bool = False, mode: str = "simulation", scenario: str = "legacy", options=None
    ) -> dict:
        with self._lock:
            return self._start(live, mode, scenario, options)

    def _start(self, live: bool, mode: str, scenario: str, options=None) -> dict:
        resolved_options = FlightOptions.from_dict(options or {})
        if self._demo is not None and self._demo.running:
            raise RuntimeError("请先停止正在播放的演示")
        if self._process is not None and self._process.poll() is None:
            raise RuntimeError("A 7 km rendezvous run is already active")
        if mode == "demo":
            path = (
                self.repository / "output/spacesim_demo15/demo.json.gz"
                if self.repository
                else None
            )
            if path is None or not path.is_file():
                raise RuntimeError("尚未准备 15 秒演示数据")
            self.stop()
            if self._log is not None:
                self._log.close()
                self._log = None
            if self.launch_config is not None:
                self._ensure_renderer()
            try:
                if self._demo is None:
                    self._demo = RendezvousDemo(path, self.state_port, self.render_port)
                self._demo.start()
            except (OSError, ValueError) as error:
                raise RuntimeError(
                    f"演示未启动，请确认 UE 接收端已经启动：{error}"
                ) from error
            self._mode = "demo"
            self._output_dir = self._demo.source
            return self.status()
        if mode != "simulation":
            raise RuntimeError("Unknown rendezvous mode")
        if scenario not in ("legacy", "team-in-orbit", "team-orbit-raise"):
            raise RuntimeError("Unknown rendezvous scenario")
        if (
            not self.python
            or not self.state_port
            or (scenario == "legacy" and not self.repository)
        ):
            raise RuntimeError(
                "Configure the rendezvous repository, Python runtime and state port"
            )
        if (
            scenario == "legacy"
            and not (self.repository / "rendezvous" / "cli" / "spacesim.py").is_file()
        ):
            raise RuntimeError(
                "The configured rendezvous repository has no SpaceSim entry point"
            )
        if not self.python.is_file():
            raise RuntimeError(
                "The configured rendezvous Python executable does not exist"
            )
        if scenario != "legacy" and (
            self.team_repository is None
            or not (
                self.team_repository / "simulation/orbital_rendezvous/run.py"
            ).is_file()
        ):
            raise RuntimeError("Team flight entry point is unavailable")
        catalog = None
        if scenario != "legacy":
            catalog = Path(
                os.environ.get(
                    "SPACE_SIM_TEAM_CATALOG",
                    str(
                        self.team_repository.parent
                        / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_platform.catalog.json"
                    ),
                )
            )
            if live and not catalog.is_file():
                raise RuntimeError(
                    "Run platform asset preparation before starting UE flight"
                )
        self.stop()
        if self._log is not None:
            self._log.close()
            self._log = None
        if live and self.launch_config is not None:
            self._ensure_renderer()
        self._check_start_cancelled()
        self._mode = "simulation"
        self._options = resolved_options
        self._scenario = scenario
        self._process = None
        run_id = uuid.uuid4().hex
        self._output_dir = (
            self.repository / "output" / "spacesim_platform" / run_id
            if scenario == "legacy"
            else self.team_repository / "run" / "team_rendezvous" / run_id
        )
        self.log_root.mkdir(parents=True, exist_ok=True)
        self._log_path = self.log_root / f"{run_id}.log"
        self._log = self._log_path.open("wb")
        command = [
            str(self.python),
            "-m",
            "rendezvous.cli.spacesim",
            "--output-dir",
            str(self._output_dir),
            "--state-port",
            str(self.state_port),
        ]
        working_directory = self.repository
        if scenario != "legacy":
            working_directory = self.team_repository
            command = [
                str(self.python),
                "-u",
                "-m",
                "simulation.orbital_rendezvous.run",
                "--scenario",
                scenario.removeprefix("team-"),
                "--output-dir",
                str(self._output_dir),
                "--state-port",
                str(self.state_port),
            ]
            if catalog.is_file():
                command.extend(["--catalog", str(catalog)])
        if scenario != "legacy":
            self._output_dir.mkdir(parents=True, exist_ok=True)
            options_path = self._output_dir / "mission_options.json"
            options_path.write_text(json.dumps(self._options.to_dict(), indent=2), encoding="utf-8")
            command.extend(["--config", str(options_path)])
            command.extend(["--render-port", str(self.render_port)])
        if live:
            command.append("--live")
        flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
        try:
            self._process = subprocess.Popen(
                command,
                cwd=working_directory,
                stdin=subprocess.DEVNULL,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                creationflags=flags,
            )
        except BaseException:
            self._log.close()
            self._log = None
            raise
        return self.status()

    def _ensure_renderer(self):
        """Use the team's launcher in this checkout, so a fresh platform is usable."""
        def ready():
            try:
                with socket.create_connection(("127.0.0.1", self.render_port), timeout=0.5):
                    return True
            except OSError:
                return False
        cfg = self.launch_config
        adapter = cfg.runtime_adapter_root or self.team_repository.parent / "space_sim_UE_Adapter"
        if ready():
            self._remember_renderer(adapter)
            return
        script = adapter / "scripts/start_renderer.ps1"
        if not script.is_file() or cfg.runtime_unreal_root is None:
            raise RuntimeError("请先配置本平台的 UE 路径并完成资产准备")
        command = [str(cfg.runtime_powershell_exe or "pwsh"), "-NoProfile", "-File", str(script),
                   "-UnrealRoot", str(cfg.runtime_unreal_root), "-Port", str(self.render_port),
                   "-PixelStreamingURL", f"ws://127.0.0.1:{cfg.runtime_pixel_streamer_port}",
                   "-PixelStreamingId", cfg.pixel_streaming_streamer_id,
                   "-PixelStreamingFps", str(round(cfg.runtime_preview_rate)),
                   "-EncoderMinQuality", str(getattr(cfg, "runtime_encoder_min_quality", 60))]
        cameras = getattr(cfg, "runtime_pixel_streaming_camera_ids", ())
        if cameras:
            # The orbital manifest keeps the team's model under its flight
            # namespace; the console resolves these streams as the same cameras.
            cameras = tuple("team/" + camera.removeprefix("teleop/") if camera.startswith("teleop/") else camera
                            for camera in cameras)
            command.extend([
                "-PixelStreamingCameraIds", ",".join(cameras),
                "-PixelStreamingCameraWidth", str(cfg.runtime_pixel_streaming_camera_width),
                "-PixelStreamingCameraHeight", str(cfg.runtime_pixel_streaming_camera_height),
                "-PixelStreamingCameraFps", str(round(cfg.runtime_preview_rate)),
            ])
        try:
            self.log_root.mkdir(parents=True, exist_ok=True)
            # A Windows/RDP launch lets long-lived UE inherit the launcher's
            # stdout handles. PIPE/communicate then waits for UE to exit even
            # after PowerShell has finished (including timeout cleanup).
            # A regular log file preserves diagnostics without waiting on EOF.
            with (self.log_root / "renderer-start.log").open("ab") as output:
                result = subprocess.run(command, cwd=adapter, stdin=subprocess.DEVNULL,
                                        stdout=output, stderr=subprocess.STDOUT, timeout=60,
                                        creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
        except (OSError, subprocess.TimeoutExpired) as error:
            raise RuntimeError("无法启动本工作区的 UE 渲染器") from error
        if result.returncode:
            raise RuntimeError("UE 启动失败，请查看本工作区渲染日志")
        self._remember_renderer(adapter, require_listening=False)
        deadline = time.monotonic() + cfg.runtime_renderer_ready_timeout
        while time.monotonic() < deadline:
            self._check_start_cancelled()
            if ready():
                self._remember_renderer(adapter)
                return
            time.sleep(0.25)
        self.release_renderer()
        raise RuntimeError("UE 未在配置的渲染端口就绪，请检查本工作区渲染日志")

    def _remember_renderer(self, adapter, *, require_listening=True):
        import psutil
        pid_file = adapter / "Unreal/BskUnrealRenderer/Saved/BskRenderer.pid"
        try:
            process = psutil.Process(int(pid_file.read_text().strip()))
            project = str((adapter / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject").resolve()).replace("\\", "/").lower()
            command = " ".join(process.cmdline()).replace("\\", "/").lower()
            if project not in command:
                raise RuntimeError("渲染端口的进程不属于本平台工作区")
            if require_listening:
                owners = {connection.pid for connection in psutil.net_connections(kind="tcp")
                          if connection.status == psutil.CONN_LISTEN and connection.laddr.port == self.render_port}
                if owners != {process.pid}:
                    raise RuntimeError("渲染端口由另一进程占用，未接管该进程")
            self._renderer_identity = (process.pid, process.create_time())
        except (OSError, ValueError, psutil.Error) as error:
            raise RuntimeError("无法确认本工作区的 UE 进程，请检查渲染端口占用") from error

    def release_renderer(self):
        """Release only the verified PID/start-time pair before an arm session."""
        import psutil
        with self._lock:
            if self._renderer_identity is None:
                return
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("请先停止正在运行的交会任务")
            identity, self._renderer_identity = self._renderer_identity, None
            if identity is None:
                return
            try:
                process = psutil.Process(identity[0])
                if process.create_time() != identity[1]:
                    return
                children = process.children(recursive=True)
                process.terminate()
                psutil.wait_procs([process], timeout=10)
                for child in children:
                    try:
                        child.terminate()
                    except psutil.NoSuchProcess:
                        pass
            except psutil.NoSuchProcess:
                pass

    def stop(self) -> dict:
        with self._lock:
            return self._stop()

    def _stop(self) -> dict:
        if self._demo is not None:
            self._demo.stop()
        if self._process is not None and self._process.poll() is None:
            self._process.terminate()
            try:
                self._process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=5)
        return self.status()

    def close(self):
        self.stop()
        self.release_renderer()
        if self._log is not None:
            self._log.close()
            self._log = None
