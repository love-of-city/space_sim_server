"""One operator mission across orbital flight, scene handoff and capture.

The existing flight and scene executors retain their physics and process
ownership. This coordinator owns admission, user ownership and provenance.
All mutations run on the application's event loop; blocking launches run in
workers and never hold the coordinator while the UI reads progress.
"""
from __future__ import annotations

import asyncio
import copy
import json
import os
import re
import time
import uuid
from pathlib import Path

from .models import SceneInstanceCreate
from .rendezvous_config import FlightOptions
from .scene_targets import SELF_COLLISION_TEMPLATE


MISSION_MODES = (
    {"id": "teleop", "label": "直接遥操作", "template_ids": None},
    {"id": "team-in-orbit", "label": "7 km 交会 → 遥操作", "template_ids": [SELF_COLLISION_TEMPLATE]},
    {"id": "team-orbit-raise", "label": "300 → 500 km 升轨交会 → 遥操作", "template_ids": [SELF_COLLISION_TEMPLATE]},
)


class MissionRuntime:
    def __init__(self, root: Path, scenes, flight, telemetry, capture_busy):
        self.root = root
        self.scenes, self.flight, self.telemetry = scenes, flight, telemetry
        self.capture_busy = capture_busy
        self._record = None
        self._operation = None
        self._task = None
        self._stop_requested = False
        latest = root / "latest.json"
        if latest.is_file():
            try:
                record = json.loads(latest.read_text(encoding="utf-8"))
                if (isinstance(record, dict) and record.get("schema") == "spacesim-mission/1"
                        and re.fullmatch(r"mission-[0-9a-f]{32}", str(record.get("mission_id", "")))):
                    self._record = record
            except (OSError, ValueError):
                pass
        if self._record is None:
            self._adopt_existing()

    def _adopt_existing(self):
        scene = self.scenes.status()
        instance = scene.get("instance") or {}
        runtime = self.flight.status()
        if scene.get("active") and instance.get("instance_id"):
            self._new("teleop", instance.get("created_by"))
            self._record.update(stage="teleop", scene_instance_id=instance["instance_id"])
            self._save()
        elif runtime.get("output_dir") and runtime.get("report"):
            self._new(runtime.get("scenario", "legacy"), None)
            self._record.update(stage="flight", flight_output_dir=runtime["output_dir"],
                                options=runtime.get("mission_options", {}), restored=True)
            self._save()

    def _new(self, mode, creator):
        self._record = {"schema": "spacesim-mission/1", "mission_id": "mission-" + uuid.uuid4().hex,
                        "mode": mode, "stage": "flight" if mode != "teleop" else "teleop",
                        "created_by": {key: creator[key] for key in ("user_id", "username", "role") if key in creator} if creator else None,
                        "created_at_ns": str(time.time_ns()),
                        "flight_output_dir": None, "scene_instance_id": None, "episodes": [],
                        "error": None, "stopped": False}
        self._save()

    def _save(self):
        self.root.mkdir(parents=True, exist_ok=True)
        text = json.dumps(self._record, ensure_ascii=False, indent=2) + "\n"
        for path in (self.root / f"{self._record['mission_id']}.json", self.root / "latest.json"):
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(text, encoding="utf-8")
            os.replace(temporary, path)

    def can_manage(self, user):
        owner = (self._record or {}).get("created_by") or {}
        return bool(user and (user.get("role") == "admin" or owner.get("user_id") == user.get("user_id")))

    def require_owner(self, user):
        if self._record and not self.can_manage(user):
            raise PermissionError("只能管理自己创建的任务")

    def require_available(self):
        if self._operation or self._stop_requested or self.scenes.status().get("active") or self.flight.status().get("running"):
            raise RuntimeError("请先停止当前任务，再启动新任务")
        if self.capture_busy():
            raise RuntimeError("请先结束当前采集")

    def require_stable(self):
        if self._operation:
            raise RuntimeError("任务正在切换阶段，请稍后重试")

    def provenance(self):
        record = self._record or {}
        return {key: record.get(key) for key in ("mission_id", "mode", "flight_output_dir", "source_run_id")}

    async def _run(self, operation, work):
        try:
            return await work(), None
        except Exception as error:
            self._record["error"] = str(error)
            return None, error
        finally:
            if operation == "stopping":
                self._stop_requested = False
            self._operation = "stopping" if self._stop_requested else None
            self._save()

    async def _begin(self, operation, work, background):
        self._operation = operation
        self._record["error"] = None
        self._task = asyncio.create_task(self._run(operation, work), name=f"mission-{operation}")
        if background:
            return self.status()
        # A browser disconnect must not cancel the worker's state/cleanup while
        # its synchronous launcher continues running in a thread.
        result, error = await asyncio.shield(self._task)
        if error:
            raise error
        return result

    async def start_flight(self, user, *, scenario, options, live=True, mode="simulation", background=False):
        resolved = FlightOptions.from_dict(options).to_dict()
        if scenario not in {"legacy", "team-in-orbit", "team-orbit-raise"}:
            raise ValueError("未知交会任务模式")
        self.require_available()
        self._new(scenario, user)
        self._record.update(options=resolved, live=live)
        previous_run = self.telemetry.status().get("run_id")
        self.flight.prepare_start()

        async def work():
            result = await asyncio.to_thread(self.flight.start, live, mode, scenario, resolved)
            # Keep the previous evidence if admission/launch fails, and never
            # clear a fresh first packet that arrived while the worker ran.
            if self.telemetry.status().get("run_id") == previous_run:
                self.telemetry.clear_cached_state()
            self._record["flight_output_dir"] = result.get("output_dir")
            return result

        return await self._begin("starting", work, background)

    async def start_scene(self, request, user, *, background=False):
        self.require_available()
        self._new("teleop", user)

        async def work():
            await asyncio.to_thread(self.flight.release_renderer)
            result = await asyncio.to_thread(self.scenes.start, request, user, mission=self.provenance())
            self._record["scene_instance_id"] = result["instance_id"]
            return result

        return await self._begin("starting", work, background)

    async def handoff(self, user, *, background=False):
        self.require_owner(user)
        self.require_available()
        folder = self.flight.output_dir
        if folder is None:
            raise RuntimeError("没有可接续的交会结果")
        from simulation.orbital_rendezvous.handoff import load_handoff
        from simulation.orbital_rendezvous.model import DEFAULT_MODEL
        handoff = load_handoff(folder / "handoff.json", self.scenes.project_root / DEFAULT_MODEL)
        if self._record is None or self._record.get("flight_output_dir") != str(folder):
            # A pre-migration result can be adopted explicitly by an administrator.
            if user.get("role") != "admin":
                raise PermissionError("历史结果需要管理员接续")
            self._new(self.flight.status().get("scenario", "legacy"), user)
            self._record["flight_output_dir"] = str(folder)
        terminal_scene = self.scenes.status().get("phase") in {"stopped", "failed", "completed"}
        if self._record.get("scene_instance_id") and not (self._record.get("stopped") or terminal_scene):
            raise RuntimeError("本次结果已经接续，请使用当前场景或启动新任务")
        self._record["source_run_id"] = (handoff.get("state", {}).get("metadata") or {}).get("run_id")

        async def work():
            await asyncio.to_thread(self.flight.release_renderer)
            # The flight's source model is fixed and hash-validated above. Do
            # not let the platform's selected experimental template replace it.
            result = await asyncio.to_thread(self.scenes.start, SceneInstanceCreate(
                template_id=SELF_COLLISION_TEMPLATE, randomization_profile="none"),
                                             user, handoff=handoff, mission=self.provenance())
            self._record.update(stage="teleop", scene_instance_id=result.get("instance_id"), stopped=False)
            self._record.setdefault("scene_instance_ids", []).append(result.get("instance_id"))
            return result

        return await self._begin("handoff", work, background)

    async def stop(self, user, stop_scene):
        self.require_owner(user)
        if self._operation == "stopping" or self._stop_requested:
            raise RuntimeError("任务正在停止")
        self._stop_requested = True
        pending = self._task if self._operation else None
        if pending:
            self.flight.cancel_start()
        # No other admission can interleave before this operation is reserved.
        if self._record is None:
            self._new("teleop", user)

        async def work():
            if pending:
                await asyncio.shield(pending)
            if self.scenes.status().get("active") or self._record["stage"] == "teleop":
                result = await stop_scene()
            else:
                result = await asyncio.to_thread(self.flight.stop)
                await asyncio.to_thread(self.flight.release_renderer)
            self._record.update(stopped=True, error=None)
            return result

        return await self._begin("stopping", work, False)

    def add_episode(self, episode_id):
        if self._record and episode_id not in self._record["episodes"]:
            self._record["episodes"].append(episode_id)
            self._save()

    def artifact(self, mission_id, name):
        """Bind a download to its task even if another task starts meanwhile."""
        if not re.fullmatch(r"mission-[0-9a-f]{32}", mission_id):
            raise ValueError("无效任务编号")
        record = json.loads((self.root / f"{mission_id}.json").read_text(encoding="utf-8"))
        if record.get("mission_id") != mission_id or not record.get("flight_output_dir"):
            raise ValueError("该任务没有交会结果")
        folder = Path(record["flight_output_dir"]).resolve()
        allowed = [self.scenes.project_root / "run/team_rendezvous"]
        if getattr(self.flight, "repository", None):
            allowed.append(self.flight.repository / "output")
        if not any(folder.is_relative_to(path.resolve()) for path in allowed):
            raise ValueError("结果目录不属于本工作区")
        return folder / name

    def status(self, user=None):
        record = copy.deepcopy(self._record) if self._record else {}
        flight = self.flight.status()
        scene = self.scenes.status()
        telemetry = self.telemetry.status()
        is_flight = record.get("stage") == "flight"
        same_flight = record.get("flight_output_dir") == flight.get("output_dir")
        report = flight.get("report") if record.get("flight_output_dir") and same_flight else None
        if self._operation:
            phase = self._operation
        elif record.get("stopped"):
            phase = "stopped"
        elif record.get("error"):
            phase = "failed"
        elif is_flight:
            phase = "flight" if flight.get("running") else "ready" if report and report.get("basic_nominal_passed") else "failed"
        else:
            phase = scene.get("phase", "idle") if record else "idle"
        if not is_flight or not same_flight or self._operation == "starting":
            telemetry = {"connected": False, "state": None}
        active = bool(self._operation or scene.get("active") or flight.get("running"))
        can_handoff = bool(report and report.get("basic_nominal_passed") and not active
                           and (not record.get("scene_instance_id") or record.get("stopped")
                                or scene.get("phase") in {"stopped", "failed", "completed"}))
        return {**record, "phase": phase, "active": active,
                "can_manage": self.can_manage(user), "can_handoff": can_handoff,
                "report": report, "telemetry": telemetry,
                "runtime": {"configured": flight.get("configured"), "running": flight.get("running"),
                            "exit_code": flight.get("exit_code"), "output_dir": record.get("flight_output_dir")},
                "scene": scene}

    async def close(self):
        if self._task and not self._task.done():
            self.flight.cancel_start()
            await self._task
