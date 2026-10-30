"""Persistent orchestration of isolated production collectors (CPU + UE).

Task adapters own their CLI and validation; the manager only owns resources,
process lifecycle and durable progress. Native dynamics never run in the API.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time
from typing import Literal
import uuid
import xml.etree.ElementTree as ET
from simulation.scripted_tasks.floating_capture import FLOATING_TASKS

import psutil
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .collection_processes import ManagerLease, WindowsProcessGroup
from .collection_layouts import LayoutPool
from .collection_inputs import (
    fixed_layout_input, requires_color_layout, validate_fixed_snapshot, validate_saved_layout,
)
from .unreal_runtime import default_unreal_root, unreal_editor


@dataclass(frozen=True)
class CollectionTask:
    label: str
    script: str
    template: str
    arguments: tuple[str, ...] = ()
    description: str = "接近 → 抓取 → 抽出 → 放回 → 松爪 → 撤离。"


TASKS = {
    "drawer_cycle": CollectionTask(
        "抽屉取出、放回与撤离",
        "collection/collect_scripted_tasks.py",
        "sarm-task-box-20261012",
        arguments=("--task", "drawer_cycle"),
    ),
}
for _task, _label in (
    ("aviation_plug_insert", "航空插头"),
    ("guide_plug_insert", "导向插头"),
    ("cylindrical_plug_insert", "圆柱插头"),
):
    TASKS[_task] = CollectionTask(
        _label + "拿取与插入",
        "collection/collect_scripted_tasks.py",
        "sarm-task-box-20261012",
        arguments=("--task", _task),
        description=f"独立任务：跟踪并抓取{_label} → 搬到前方对应插孔 → 对准插入 → 松爪撤离 → 验收。",
    )

for _task, _label, _description in (
    ("aviation_bolt_insert", "航空螺丝旋出与装配",
     "抓取初始孔内螺丝 → 旋出取下 → 搬运对孔 → 旋入拧紧 → 松爪撤离 → 验收。"),
    ("locking_rod_cycle", "杆状机构放倒与复位",
     "旋松防脱紧固件 → 绕底部铰链放倒至卡扣 → 松爪再抓取 → 扶起复位 → 旋紧 → 撤离验收。"),
):
    TASKS[_task] = CollectionTask(
        _label, "collection/collect_scripted_tasks.py", "sarm-task-box-20261012",
        arguments=("--task", _task), description=_description,
    )

TERMINAL = {"complete", "failed", "cancelled", "interrupted", "attempt_limit"}

for _task, _asset in FLOATING_TASKS.items():
    TASKS[_task] = CollectionTask(
        _asset.label + "抓取", "collection/collect_scripted_tasks.py", "sarm-task-box-20261012",
        arguments=("--task", _task),
        description="随机释放物体 → 跟踪拦截 → 双指抓取 → 稳定保持；相机到抓取点初始距离 0.12–0.75 m。",
    )


from .collection_randomization import CollectionRandomization
from pydantic import StrictBool


class CollectionCreate(BaseModel):
    model_config = ConfigDict(extra="forbid")
    task: str = "drawer_cycle"
    count: int = Field(default=5, ge=1, le=1000)
    workers: int = Field(default=2, ge=1, le=4)
    max_attempts: int = Field(default=15, ge=1, le=1000)
    seed: int = Field(default=20261016, ge=0, le=2**31 - 1001)
    initialization_enabled: StrictBool = True
    layout_randomization: StrictBool = False
    randomize_overview_camera: StrictBool = False
    randomization: CollectionRandomization = Field(default_factory=CollectionRandomization)
    range_deg: float = Field(default=30, ge=0, le=50, allow_inf_nan=False)
    linear_speed_m_s: float | None = Field(
        default=None, gt=0, le=0.1, allow_inf_nan=False
    )
    threaded_speed_profile: Literal["default", "fast_collection"] | None = None

    @model_validator(mode="after")
    def valid_plan(self):
        if self.task not in TASKS:
            raise ValueError("未注册的采集任务")
        if self.task in FLOATING_TASKS:
            if self.randomize_overview_camera:
                raise ValueError("漂浮物任务使用固定相机配置")
            if self.layout_randomization:
                raise ValueError("漂浮物任务随机释放目标，操作台布局保持固定")
            self.range_deg = 0.
        if self.task == "drawer_cycle":
            if self.linear_speed_m_s is None:
                self.linear_speed_m_s = 0.1
        elif self.linear_speed_m_s is not None:
            raise ValueError("移动速度配置仅适用于抽屉完整循环任务")
        if self.task in ("aviation_bolt_insert", "locking_rod_cycle"):
            if self.threaded_speed_profile is None:
                self.threaded_speed_profile = "fast_collection"
        elif self.threaded_speed_profile is not None:
            raise ValueError("螺纹采集速度配置仅适用于航空螺丝和杆状机构任务")
        if not self.workers <= self.count <= self.max_attempts:
            raise ValueError("需满足 并发数 ≤ 成功条数 ≤ 最大尝试数")
        return self


def read_json(path, default=None):
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError):
        return default


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    temporary.replace(path)


def kill_owned_tree(pid, created):
    """Never terminate a reused PID or an unrelated interactive scene."""
    try:
        parent = psutil.Process(pid)
        if abs(parent.create_time() - created) > 0.01:
            return
        children = parent.children(recursive=True)
        # Stop the supervisor first so that it cannot spawn replacements.
        parent.suspend()
        for process in reversed(children):
            try:
                process.kill()
            except psutil.Error:
                pass
        parent.kill()
        psutil.wait_procs([parent, *children], timeout=3)
    except psutil.NoSuchProcess:
        pass


def free_ports(count):
    """Probe both protocols. Child listeners also fail closed if a port is taken."""
    for base in range(21000, 30000, 40):
        ports = list(range(base, base + count * 4))
        sockets = []
        try:
            for port in ports:
                for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM):
                    sock = socket.socket(socket.AF_INET, kind)
                    sockets.append(sock)
                    if os.name == "nt":
                        sock.setsockopt(
                            socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1
                        )
                    sock.bind(("127.0.0.1", port))
            return [ports[i : i + 4] for i in range(0, len(ports), 4)]
        except OSError:
            continue
        finally:
            for sock in sockets:
                sock.close()
    raise ValueError("没有足够空闲的采集端口")


class CollectionManager:
    def __init__(self, project_root, data_root, adapter=None, unreal=None, layout_pool=None,
                 fixed_model=None, fixed_catalog=None):
        self.project_root = Path(project_root).resolve()
        self.data_root = Path(data_root).resolve()
        self.root = self.data_root.parent / "collection_jobs"
        self.adapter = Path(
            adapter or self.project_root.parent / "space_sim_UE_Adapter"
        )
        self.unreal = Path(unreal or default_unreal_root())
        configured_pool = layout_pool or os.environ.get("SPACE_SIM_COLLECTION_LAYOUT_POOL")
        self.layout_pool = Path(configured_pool).expanduser().resolve() if configured_pool else None
        self.fixed_model = fixed_model or os.environ.get("SPACE_SIM_COLLECTION_FIXED_MODEL")
        self.fixed_catalog = fixed_catalog or os.environ.get("SPACE_SIM_COLLECTION_FIXED_CATALOG")
        self.lock = threading.RLock()
        self.thread = None
        self.closing = False
        self.jobs = {}
        self.root.mkdir(parents=True, exist_ok=True)
        self.lease = ManagerLease(self.root / "manager.lock")
        for path in self.root.glob("*/job.json"):
            job = read_json(path)
            if not job or job.get("id") != path.parent.name:
                continue
            if job["status"] not in TERMINAL:
                for worker in job["workers"]:
                    if worker.get("pid") and worker.get("created"):
                        kill_owned_tree(worker["pid"], worker["created"])
                    if worker["status"] not in TERMINAL:
                        worker["status"] = "interrupted"
                job.update(
                    status="interrupted",
                    error="后端重启中断；已清理原任务进程。请新建任务继续。",
                    finished_at=time.time(),
                )
                write_json(path, job)
            self.jobs[job["id"]] = job

    @staticmethod
    def visible(job, user):
        return user["role"] == "admin" or job["owner"]["user_id"] == user["user_id"]

    def _save(self, job):
        write_json(self.root / job["id"] / "job.json", job)

    def list(self, user):
        with self.lock:
            return [
                self._view(j)
                for j in sorted(
                    self.jobs.values(), key=lambda j: j["started_at"], reverse=True
                )
                if self.visible(j, user)
            ]

    def get(self, key, user):
        with self.lock:
            job = self.jobs.get(key)
            if not job or not self.visible(job, user):
                raise KeyError(key)
            return self._view(job)

    def _view(self, job):
        result = deepcopy(job)
        result["elapsed_seconds"] = round(
            result.get("finished_at", time.time()) - result["started_at"], 1
        )
        result["accepted_episodes"] = [
            episode
            for w in result["workers"]
            for episode in w.get("accepted_episodes", [])
        ]
        result["success_count"] = len(result["accepted_episodes"])
        for worker in result["workers"]:
            worker.pop("created", None)
        return result

    def fixed_layout_capability(self):
        if not self.fixed_model and not self.fixed_catalog:
            return dict(available=False, configured=False, description="尚未配置可区分孔位的固定操作台")
        try:
            record, _ = fixed_layout_input(self.fixed_model, self.fixed_catalog)
            return dict(available=True, configured=True, model_sha256=record["model_sha256"],
                        description="已校验固定操作台；本批次所有记录使用同一布局")
        except (OSError, ValueError, KeyError, ET.ParseError) as error:
            return dict(available=False, configured=True, description=f"固定采集布局校验失败：{error}")

    def start(self, request, owner, scene):
        with self.lock:
            if self.closing or any(
                j["status"] not in TERMINAL for j in self.jobs.values()
            ):
                raise ValueError("已有自动采集批次运行中，请等待完成或先停止该批次")
            task = TASKS[request.task]
            pool = None
            fixed = placement = None
            if request.layout_randomization:
                if self.layout_pool is None:
                    raise ValueError("尚未配置已校验的随机布局池，不能启用布局随机化")
                try:
                    pool = LayoutPool(self.layout_pool)
                except (OSError, ValueError, KeyError) as error:
                    raise ValueError(f"随机布局池校验失败：{error}") from error
            elif self.fixed_model or self.fixed_catalog:
                try:
                    fixed, placement = fixed_layout_input(self.fixed_model, self.fixed_catalog)
                    validate_saved_layout(scene, fixed)
                except (OSError, ValueError, KeyError, ET.ParseError) as error:
                    raise ValueError(f"固定采集布局校验失败：{error}") from error
            elif requires_color_layout(request.task):
                raise ValueError("该颜色指令任务需要可区分孔位的固定操作台；请配置 "
                                 "SPACE_SIM_COLLECTION_FIXED_MODEL 和 SPACE_SIM_COLLECTION_FIXED_CATALOG，"
                                 "或显式启用已校验的随机布局池")
            elif (scene.get("task_box_layout") or scene.get("collection_layout_randomization")
                  or (scene.get("collection_config") or {}).get("layout_randomization", {}).get("enabled")):
                raise ValueError("已保存布局场景需要配置与其匹配的固定 model 和 catalog，或显式启用随机布局池")
            try:
                editor = unreal_editor(self.unreal)
            except OSError as error:
                raise ValueError(str(error)) from error
            for path in (
                self.project_root / task.script,
                editor,
                Path(fixed["catalog"]) if fixed else self.adapter
                / "Unreal/BskUnrealRenderer/Saved/AssetImport/sarm_20261012.catalog.json",
            ):
                if not path.is_file():
                    raise ValueError(f"采集运行环境缺少文件：{path}")
            key = "collection-" + time.strftime("%Y%m%d-%H%M%S-") + uuid.uuid4().hex[:8]
            directory = self.root / key
            directory.mkdir()
            scene = deepcopy(scene)
            scene["collection_randomization"] = request.randomization.model_dump()
            scene["initialization_enabled"] = request.initialization_enabled
            scene["collection_layout_randomization"] = request.layout_randomization
            scene["randomize_overview_camera"] = request.randomize_overview_camera
            scene["overview_camera_sampling"] = "presets"
            from .overview_camera import initialize_overview_camera
            initialize_overview_camera(scene)
            scene["created_by"] = {k: owner[k] for k in ("user_id", "username", "role")}
            if fixed is not None:
                scene["task_box_layout"] = placement
                scene["collection_fixed_layout"] = fixed
                scene.setdefault("collection_config", {})["layout_randomization"] = {"enabled": False}
            write_json(directory / "scene.json", scene)
            if fixed is not None:
                write_json(directory / "fixed-layout.json", fixed)
            if pool is not None:
                # Each batch pins the validated membership before workers start.
                write_json(directory / "layout-pool.json", pool.document)
            ports = free_ports(request.workers)
            workers = []
            seed = request.seed
            for index in range(request.workers):
                count = request.count // request.workers + (
                    index < request.count % request.workers
                )
                attempts = request.max_attempts // request.workers + (
                    index < request.max_attempts % request.workers
                )
                workers.append(
                    dict(
                        index=index,
                        count=count,
                        max_attempts=attempts,
                        seed=seed,
                        ports=ports[index],
                        status="queued",
                        accepted_episodes=[],
                    )
                )
                seed += attempts
            job = dict(
                id=key,
                status="running",
                owner=scene["created_by"],
                config=request.model_dump(),
                fixed_layout=fixed,
                started_at=time.time(),
                workers=workers,
            )
            self.jobs[key] = job
            self._save(job)
            self.thread = threading.Thread(
                target=self._run, args=(job,), name=key, daemon=True
            )
            self.thread.start()
            return self._view(job)

    def stop(self, key, user):
        with self.lock:
            self.get(key, user)
            job = self.jobs[key]
            if job["status"] not in TERMINAL:
                job["status"] = "stopping"
                job.setdefault("stop_requested_at", time.time())
                (self.root / key / "cancel.request").touch()
                self._save(job)
            return self._view(job)

    def _refresh(self, job, worker):
        folder = self.root / job["id"] / f"worker-{worker['index']}"
        summary = read_json(folder / "summary.json", {})
        worker["attempts"] = summary.get("attempts", [])
        worker["accepted_episodes"] = [
            Path(p).name for p in summary.get("accepted_episodes", [])
        ]
        worker["collector_status"] = summary.get("status", "starting")
        if summary.get("error"):
            worker["error"] = summary["error"]
        if worker["attempts"]:
            attempt = worker["attempts"][-1]
            worker["phase"] = attempt["status"]
            # Tail only a bounded amount; progress records may be partially written.
            log = Path(attempt["directory"]) / "recording.log"
            try:
                with log.open("rb") as stream:
                    stream.seek(max(0, log.stat().st_size - 24000))
                    lines = stream.read().decode("utf-8", errors="replace").splitlines()
                for line in reversed(lines):
                    try:
                        item = json.loads(line)
                    except ValueError:
                        continue
                    if item.get("stage"):
                        worker["progress"] = item
                        break
            except OSError:
                pass

    def _run(self, job):
        active = []
        failure = None
        folder = self.root / job["id"]
        try:
            for worker in job["workers"]:
                with self.lock:
                    if job["status"] == "stopping":
                        break
                    cfg = job["config"]
                    command = [
                        sys.executable,
                        "-X",
                        "utf8",
                        str(self.project_root / TASKS[cfg["task"]].script),
                        "--scene",
                        str(folder / "scene.json"),
                        "--output",
                        str(folder / f"worker-{worker['index']}"),
                        "--episodes-root",
                        str(self.data_root),
                        "--count",
                        str(worker["count"]),
                        "--max-attempts",
                        str(worker["max_attempts"]),
                        "--seed",
                        str(worker["seed"]),
                        "--range-deg",
                        str(cfg["range_deg"] if cfg.get("initialization_enabled", True) else 0),
                        "--overview-camera-sampling",
                        "presets",
                        "--cancel-file",
                        str(folder / "cancel.request"),
                        "--adapter",
                        str(self.adapter),
                        "--unreal",
                        str(self.unreal),
                    ]
                    command += list(TASKS[cfg["task"]].arguments)
                    if cfg.get("layout_randomization"):
                        command += ["--layout-pool", str(folder / "layout-pool.json")]
                    elif job.get("fixed_layout") is not None:
                        fixed = job["fixed_layout"]
                        validate_fixed_snapshot(fixed)
                        command += ["--model", fixed["model"], "--catalog", fixed["catalog"]]
                    if cfg["task"] == "drawer_cycle":
                        command += [
                            "--linear-speed",
                            str(cfg.get("linear_speed_m_s") or 0.1),
                        ]
                    if cfg["task"] in ("aviation_bolt_insert", "locking_rod_cycle"):
                        command += [
                            "--threaded-speed-profile",
                            cfg.get("threaded_speed_profile") or "default",
                        ]
                    for name, port in zip(
                        ("control", "capture", "render", "signalling"),
                        worker["ports"],
                        strict=True,
                    ):
                        command += [f"--{name}-port", str(port)]
                    log = (folder / f"worker-{worker['index']}.log").open(
                        "w", encoding="utf-8"
                    )
                    try:
                        process = subprocess.Popen(
                            command,
                            cwd=self.project_root,
                            stdout=log,
                            stderr=subprocess.STDOUT,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
                        )
                    except BaseException:
                        log.close()
                        raise
                    try:
                        group = WindowsProcessGroup(process)
                    except BaseException:
                        kill_owned_tree(
                            process.pid, psutil.Process(process.pid).create_time()
                        )
                        log.close()
                        raise
                    active.append((worker, process, log, group))
                    worker.update(
                        status="running",
                        pid=process.pid,
                        created=psutil.Process(process.pid).create_time(),
                    )
                    self._save(job)
            while active:
                with self.lock:
                    for worker, process, log, group in list(active):
                        self._refresh(job, worker)
                        if (
                            job.get("stop_requested_at")
                            and time.time() - job["stop_requested_at"] > 180
                        ):
                            kill_owned_tree(process.pid, worker["created"])
                            worker["forced_stop"] = True
                        if process.poll() is None:
                            continue
                        # The child may have finalized after the refresh above.
                        # Read its durable final summary before counting results.
                        self._refresh(job, worker)
                        worker["returncode"] = process.returncode
                        worker["status"] = (
                            "cancelled"
                            if job["status"] == "stopping"
                            else "complete"
                            if process.returncode == 0
                            and len(worker["accepted_episodes"]) == worker["count"]
                            else "attempt_limit"
                            if process.returncode == 1
                            and worker["collector_status"] == "attempt_limit"
                            and not worker.get("error")
                            else "failed"
                        )
                        log.close()
                        if worker["status"] == "failed":
                            log_path = folder / f"worker-{worker['index']}.log"
                            with log_path.open("rb") as stream:
                                stream.seek(max(0, log_path.stat().st_size - 6000))
                                worker["log_tail"] = stream.read().decode(
                                    "utf-8", errors="replace"
                                )
                        group.close()
                        active.remove((worker, process, log, group))
                    self._save(job)
                if active:
                    time.sleep(0.5)
        except Exception as error:
            failure = f"{type(error).__name__}: {error}"
        finally:
            for worker, process, log, group in active:
                group.close()
                kill_owned_tree(process.pid, worker.get("created", 0))
                process.wait(timeout=5)
                log.close()
            with self.lock:
                # Publish a terminal state only after cleanup, and persist it
                # under the same lock before readers can observe completion.
                job["status"] = (
                    "failed" if failure is not None
                    else "cancelled" if job["status"] == "stopping"
                    else "complete" if all(w["status"] == "complete" for w in job["workers"])
                    else "attempt_limit"
                    if all(w["status"] in {"complete", "attempt_limit"} for w in job["workers"])
                    else "failed"
                )
                if failure is not None:
                    job["error"] = failure
                for worker in job["workers"]:
                    if worker["status"] in {"queued", "running"}:
                        worker["status"] = (
                            "cancelled" if job["status"] == "cancelled" else "failed"
                        )
                job["finished_at"] = time.time()
                self._save(job)

    def close(self):
        with self.lock:
            self.closing = True
            for job in self.jobs.values():
                if job["status"] not in TERMINAL:
                    self.stop(job["id"], job["owner"])
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=20)
            if self.thread.is_alive():
                with self.lock:
                    for job in self.jobs.values():
                        if job["status"] not in TERMINAL:
                            job["stop_requested_at"] = time.time() - 181
                self.thread.join(timeout=15)

        self.lease.close()
