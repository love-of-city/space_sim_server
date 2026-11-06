"""Authenticated-platform job ownership around the same benchmark CLI."""
from __future__ import annotations

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from pydantic import Field, model_validator

from .benchmark_batch import cancellation_grace_s, reconcile_batch, summarize
from .benchmark_catalog import load_catalog
from .benchmark_contract import Contract, Count, load_task
from .benchmark_execution import rescore_episode, sha256, source_identity, verify_episode_evidence, write_json
from .benchmark_paths import evidence_path
from .benchmark_external import ExternalMethodDefinition
from .benchmark_registry import capability_matrix, configuration_title, method_ids
from .collection_processes import ManagerLease, kill_owned_tree, launch_collector
from typing import Literal

TERMINAL = {"complete", "stopped", "interrupted", "failed"}
BUNDLED_METHODS = {
    "external-image-fiducial-pose-v1": ("spacesim-autonomy-rendered-pose/1", "examples/autonomy_methods/vision-pose.json"),
    "external-image-gyro-spin-v1": ("spacesim-autonomy-rendered-spin/1", "examples/autonomy_methods/vision-spin.json"),
    "external-image-stereo-approach-v1": ("spacesim-autonomy-visual-approach/1", "examples/autonomy_methods/vision-approach.json"),
    "external-ephemeris-pointing-v1": ("spacesim-autonomy-pointing/1", "examples/autonomy_methods/pointing.json"),
    "external-attitude-pd-v1": ("spacesim-autonomy-attitude/1", "examples/autonomy_methods/attitude-pd.json"),
    "external-attitude-sequence-v1": ("spacesim-autonomy-attitude/1", "examples/autonomy_methods/attitude-sequence.json"),
    "external-wheel-recovery-v1": ("spacesim-autonomy-wheel-recovery/1", "examples/autonomy_methods/wheel-recovery.json"),
    "external-catalog-avoidance-v1": ("spacesim-autonomy-debris-avoidance/1", "examples/autonomy_methods/debris-avoidance.json"),
    "external-gnss-pd-v1": ("spacesim-autonomy-waypoint/1", "examples/autonomy_methods/waypoint-pd.json"),
    "external-orbit-maneuver-v1": ("spacesim-autonomy-orbit/1", "examples/autonomy_methods/orbit-pd.json"),
    "external-gnss-estimator-v1": ("spacesim-autonomy-navigation-evaluation/1", "examples/autonomy_methods/navigation-estimator.json"),
    "external-relative-flight-v1": ("spacesim-autonomy-relative-flight/1", "examples/autonomy_methods/relative-flight.json"),
    "external-flyaround-v1": ("spacesim-autonomy-flyaround/1", "examples/autonomy_methods/flyaround.json"),
    "external-formation-v1": ("spacesim-autonomy-formation/1", "examples/autonomy_methods/formation.json"),
}


class BenchmarkCreate(Contract):
    configuration: str = Field(pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    method: str | None = Field(default=None, pattern=r"^[a-z0-9]+(?:-[a-z0-9]+)*$")
    split: Literal["train", "validation", "test"] = "test"
    selection: Literal["split", "single"] = "split"
    seed: Count | None = None

    @model_validator(mode="after")
    def selection_matches(self):
        if (self.selection == "single") != (self.seed is not None):
            raise ValueError("single selection requires a seed; a full split must not specify one")
        return self


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def dependency_check(root, *, rendered=False):
    """File checks only. Never import BSK/native dynamics into the web server."""
    problems = []
    if importlib.util.find_spec("Basilisk") is None:
        problems.append("未安装 Basilisk，需在仿真主机运行")
    try:
        from simulation.orbital_rendezvous.model import DEFAULT_MODEL
        model = root / DEFAULT_MODEL
        document = ET.parse(model).getroot()
        meshroot = model.parent / document.find("compiler").get("meshdir", "")
        for mesh in document.findall("./asset/mesh[@file]"):
            with (meshroot / mesh.get("file")).open("rb") as stream:
                if stream.read(64).startswith(b"version https://git-lfs.github.com/spec/v1"):
                    raise ValueError("团队模型的 Git LFS 资产尚未下载")
        native = root / "run/native_acceleration"
        manifest = read_json(native / "local_mujoco_stepper.json")
        library = (root / manifest["library"]).resolve()
        if (manifest.get("abi") != 1 or not library.is_relative_to(native.resolve())
                or manifest.get("source_sha256") != sha256(root / "native/local_mujoco_stepper.cpp")
                or manifest.get("library_sha256") != sha256(library)):
            raise ValueError("原生动力学构建与源码不一致，请重新构建")
    except (OSError, ValueError, KeyError, ET.ParseError) as error:
        problems.append(f"模型或原生动力学依赖未就绪：{error}")
    if rendered:
        try:
            from .benchmark_vision_resources import renderer_paths, validate_catalog
            if importlib.util.find_spec('cv2') is None:
                raise ValueError('未安装 OpenCV 图像估计依赖')
            project,_=renderer_paths(root)
            validate_catalog(root.resolve(),project,project/'Saved/AssetImport/sarm_platform.catalog.json')
        except (OSError, ValueError, RuntimeError) as error:
            problems.append(f"视觉仿真依赖未就绪：{error}")
    return {"available": not problems, "problems": problems, "scope": "dependency_files_only_not_native_acceptance"}


class BenchmarkManager:
    def __init__(self, project_root, data_root):
        self.project = Path(project_root).resolve()
        self.root = evidence_path(Path(data_root).resolve().parent / "benchmark_jobs")
        self.root.mkdir(parents=True, exist_ok=True)
        self.lease = ManagerLease(self.root / "manager.lock")
        self.lock, self.jobs, self.thread = threading.RLock(), {}, None
        self.closing = False
        try:
            for path in self.root.glob("*/job.json"):
                job = read_json(path)
                if not re.fullmatch(r"benchmark-[0-9a-f]{32}", path.parent.name) or job["id"] != path.parent.name:
                    continue
                if job["status"] not in TERMINAL:
                    identity = job.get("process")
                    if identity and identity["host"] != platform.node():
                        raise ValueError("active benchmark belongs to another host; stop it there before sharing this data directory")
                    if identity:
                        kill_owned_tree(identity["pid"], identity["created"], guarded=identity["guard"] == "linux_subreaper/1")
                    self._recover(job, "interrupted")
                self.jobs[job["id"]] = job
        except BaseException:
            self.lease.close()
            raise

    def _task(self, configuration):
        if not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", configuration):
            raise ValueError("未注册的评测配置")
        path = self.project / "config/benchmarks/tasks" / (configuration + ".json")
        if not path.is_file():
            raise ValueError("未注册的评测配置")
        return load_task(path)

    def _methods(self, task):
        values = {name: None for name in method_ids(task)}
        for name, (schema, path) in BUNDLED_METHODS.items():
            if task.schema_version == schema:
                values[name] = path
        return values

    def catalog(self):
        catalog = load_catalog(self.project / "config/benchmarks/autonomy-catalog-v1.json")
        tasks = [load_task(path, catalog) for path in sorted((self.project / "config/benchmarks/tasks").glob("*.json"))]
        native=dependency_check(self.project)
        rendered=dependency_check(self.project, rendered=True) if any(task.vision_mode!='V0' for task in tasks) else native
        return {"requirements": catalog.coverage(), "capability_matrix": capability_matrix(catalog, tasks),
                "dependencies": native,
                "configurations": [{"id": task.configuration_id, "source_id": task.source_id,
                    "title": configuration_title(catalog, task), "priority": catalog.task(task.requirement_id).priority,
                    "specification": task.model_dump(mode="json"), "fingerprint": task.fingerprint(),
                    "dependencies": rendered if task.vision_mode!='V0' else native,
                    "default_method": method_ids(task)[0], "methods": list(self._methods(task))} for task in tasks]}

    def _path(self, job):
        path = (self.root / job["id"]).resolve()
        if not path.is_relative_to(self.root) or path == self.root:
            raise ValueError("invalid benchmark storage path")
        return path

    def _save(self, job):
        write_json(self._path(job) / "job.json", job)

    def _owned(self, key, user):
        job = self.jobs.get(key)
        if job is None or (user["role"] != "admin" and user["user_id"] != job["owner"]["user_id"]):
            raise KeyError(key)
        return job

    def _view(self, job):
        path = self._path(job)
        summary_path = path / "batch/summary.json"
        summary = read_json(summary_path) if summary_path.is_file() else summarize([
            {"trial_id": f"{index:04d}-seed-{seed}", "seed": seed, "outcome": "pending"}
            for index, seed in enumerate(job["seeds"])])
        rows = deepcopy(summary["trials"])
        for row in rows:
            if row.get("result_sha256"):
                result_path = (path / "batch" / row["trial_id"] / "run/result.json").resolve()
                if not result_path.is_relative_to(path) or not result_path.is_file() or sha256(result_path) != row["result_sha256"]:
                    row.update(outcome="invalid_evidence", detail="评测结果文件已改变，无法验证原结论")
                else:
                    result = read_json(result_path)
                    row.update(physics_score=result.get("physics_score"), detail=result.get("detail"),
                               wall_time_s=result.get("wall_time_s"))
                    evidence = result.get("evidence_sha256", {})
                    row["artifacts"] = ["result", *(name for name in ("task", "manifest", "provenance", "context", "runtime_final") if name + ".json" in evidence),
                        *(name for name in ("observations", "actions") if name + ".jsonl" in evidence)]
                    if any(Path(name).name == "physics.jsonl" for name in evidence):
                        row["artifacts"].append("physics")
        view = {name: deepcopy(value) for name, value in job.items() if name not in {"process", "method_spec", "stop_file"}}
        view["summary"] = summarize(rows)
        view["can_resume"] = job["status"] in TERMINAL and not view["summary"]["complete"] and (path / "batch/plan.json").is_file()
        view["elapsed_seconds"] = max(0., job.get("finished_at", time.time()) - job["started_at"])
        return view

    def list(self, user):
        with self.lock:
            return [self._view(job) for job in sorted(self.jobs.values(), key=lambda item: item["started_at"], reverse=True)
                    if user["role"] == "admin" or job["owner"]["user_id"] == user["user_id"]]

    def get(self, key, user):
        with self.lock:
            return self._view(self._owned(key, user))

    def start(self, payload: BenchmarkCreate, user):
        payload = BenchmarkCreate.model_validate(payload)
        task = self._task(payload.configuration)
        method = payload.method or method_ids(task)[0]
        methods = self._methods(task)
        if method not in methods:
            raise ValueError("该方法不适用于所选任务")
        seeds = list(getattr(task.seeds, payload.split))
        if payload.seed is not None:
            if payload.seed not in seeds:
                raise ValueError("单次种子必须属于所选的冻结划分")
            seeds = [payload.seed]
        with self.lock:
            self._admit()
            capability = dependency_check(self.project, rendered=True) if task.vision_mode!='V0' else dependency_check(self.project)
            if not capability["available"]:
                raise ValueError("；".join(capability["problems"]))
            key = "benchmark-" + uuid.uuid4().hex
            job = {"id": key, "status": "starting", "configuration": payload.configuration, "method": method,
                   "title": configuration_title(load_catalog(self.project / "config/benchmarks/autonomy-catalog-v1.json"), task),
                   "method_spec": methods[method], "request": payload.model_dump(mode="json"), "seeds": seeds,
                   "task_fingerprint": task.fingerprint(), "owner": {name: user[name] for name in ("user_id", "username", "role")},
                   "started_at": time.time(), "launches": 0, "error": None}
            self._path(job).mkdir(exist_ok=False)
            write_json(self._path(job) / "task.json", task.model_dump(mode="json"))
            self.jobs[key] = job
            self._launch(job, resume=False)
            return self._view(job)

    def _admit(self):
        if self.closing or any(job["status"] not in TERMINAL for job in self.jobs.values()):
            raise ValueError("已有驾驶评测批次运行中，请先等待或停止该批次")

    def _launch(self, job, *, resume):
        directory = self._path(job)
        job["launches"] += 1
        suffix = str(job["launches"])
        job.update(status="starting", stop_file=f"stop-{suffix}.json", error=None)
        job.pop("finished_at", None)
        job.pop("stop_requested_at", None)
        command = [sys.executable, str(self.project / "tools/autonomy_benchmark.py"), "batch", job["configuration"],
                   "--split", job["request"]["split"], "--output", str(directory / "batch"),
                   "--stop-file", str(directory / job["stop_file"])]
        command += ["--method-spec", str(self.project / job["method_spec"])] if job["method_spec"] else ["--method", job["method"]]
        if job["request"]["seed"] is not None:
            command += ["--seed", str(job["request"]["seed"])]
        if resume:
            command.append("--resume")
        self._save(job)
        owner = None
        try:
            with (directory / f"worker-{suffix}.stdout.log").open("wb") as stdout, (directory / f"worker-{suffix}.stderr.log").open("wb") as stderr:
                process, owner = launch_collector(command, cwd=self.project, stdout=stdout, stderr=stderr,
                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            job.update(status="running", process={"host": platform.node(), "pid": process.pid,
                       "created": owner.created, "guard": getattr(owner, "kind", "windows_job/1")})
            self._save(job)
        except Exception as error:
            if owner is not None:
                owner.close()
            job.update(status="failed", error=str(error), finished_at=time.time())
            self._save(job)
            return
        self.thread = threading.Thread(target=self._monitor, args=(job, process, owner), daemon=True)
        self.thread.start()

    def _monitor(self, job, process, owner):
        try:
            # The coordinator must outlive the child worker's bounded evidence
            # finalization. Otherwise UI cancellation destroys an auditable run.
            stop_grace = cancellation_grace_s(load_task(self._path(job) / "task.json")) + 5.
            while process.poll() is None:
                with self.lock:
                    force_stop = job.get("stop_requested_at") and time.time() - job["stop_requested_at"] > stop_grace
                if force_stop:
                    break
                time.sleep(.2)
        finally:
            try:
                owner.close()
                process.wait(timeout=5)
            except Exception as error:
                # Keep the coordinator occupied if process ownership cannot be
                # released; never admit a second batch over a live predecessor.
                with self.lock:
                    job["error"] = f"评测进程清理失败：{error}"
                    self._save(job)
                return
            with self.lock:
                job["returncode"] = process.returncode
                self._recover(job, "stopped" if job.get("stop_requested_at") else "failed")

    def _recover(self, job, fallback):
        directory = self._path(job)
        try:
            if (directory / "batch/plan.json").is_file():
                summary = reconcile_batch(load_task(directory / "task.json"), job["method"], directory / "batch",
                    interrupted_outcome="cancelled" if fallback == "stopped" else "infrastructure_error")
                job["status"] = "complete" if summary["complete"] else fallback
            else:
                job["status"] = fallback
            if job["status"] == "failed":
                log = directory / f"worker-{job['launches']}.stderr.log"
                job["error"] = log.read_text(encoding="utf-8", errors="replace")[-4000:] if log.is_file() else "评测进程未生成完整结果"
        except Exception as error:
            job.update(status="failed", error=f"恢复评测记录失败：{error}")
        job["finished_at"] = time.time()
        self._save(job)

    def stop(self, key, user):
        with self.lock:
            job = self._owned(key, user)
            if job["status"] not in TERMINAL and job["status"] != "stopping":
                write_json(self._path(job) / job["stop_file"], {"requested_at": time.time(), "by": user["user_id"]})
                job.update(status="stopping", stop_requested_at=time.time())
                self._save(job)
            return self._view(job)

    def resume(self, key, user):
        with self.lock:
            job = self._owned(key, user)
            self._admit()
            if not self._view(job)["can_resume"]:
                raise ValueError("该批次没有可恢复的未启动样本")
            plan = read_json(self._path(job) / "batch/plan.json")
            task = self._task(job["configuration"])
            if plan["task_fingerprint"] != task.fingerprint() or plan["source_sha256"] != source_identity(self.project)["source_sha256"]:
                raise ValueError("任务或源码已改变，请建立新批次；历史样本保留原结果")
            if job["method_spec"] and plan.get("external_method") != ExternalMethodDefinition(self.project / job["method_spec"], task).identity:
                raise ValueError("方法实现已改变，请建立新批次")
            self._launch(job, resume=True)
            return self._view(job)

    def artifact(self, key, trial_id, name, user):
        with self.lock:
            job = self._owned(key, user)
            row = next((item for item in self._view(job)["summary"]["trials"] if item["trial_id"] == trial_id), None)
            if not row or not row.get("result_sha256") or row["outcome"] == "invalid_evidence":
                raise KeyError(trial_id)
            directory = self._path(job) / "batch" / trial_id / "run"
            paths = {item: item + ".json" for item in ("task", "manifest", "provenance", "result", "context", "runtime_final")}
            paths.update(observations="observations.jsonl", actions="actions.jsonl")
            if name == "physics":
                paths[name] = read_json(directory / "manifest.json")["physics_path"]
            if name not in paths:
                raise KeyError(name)
            path = (directory / paths[name]).resolve()
            if not path.is_relative_to(directory.resolve()) or not path.is_file():
                raise KeyError(name)
            if name != "result":
                evidence = read_json(directory / "result.json")["evidence_sha256"]
                if sha256(path) != evidence.get(path.relative_to(directory.resolve()).as_posix()):
                    raise ValueError("该原始记录已改变，无法验证其内容")
            return path

    def rescore(self, key, user):
        with self.lock:
            job = self._owned(key, user)
            if job["status"] not in TERMINAL:
                raise ValueError("请先等待批次结束或停止后再复算")
            summary = self._view(job)["summary"]
            results = []
            for trial in summary["trials"]:
                if trial.get("result_sha256"):
                    directory = self._path(job) / "batch" / trial["trial_id"] / "run"
                    manifest = read_json(directory / "manifest.json")
                    if not manifest.get("physics_path") and trial["outcome"] in {"infrastructure_error", "cancelled"}:
                        verify_episode_evidence(directory)
                        results.append({"trial_id": trial["trial_id"], "outcome": trial["outcome"],
                                        "replay": "no_complete_episode_evidence", "archive_hashes_verified": True})
                        continue
                    result = rescore_episode(directory)
                    if not result["matches_recorded_physics_score"] or result["outcome"] != trial["outcome"]:
                        raise ValueError("复算与原结果不一致，保留原始记录供检查")
                    results.append({"trial_id": trial["trial_id"], **result})
                else:
                    results.append({"trial_id": trial["trial_id"], "outcome": trial["outcome"], "replay": "no_complete_episode_evidence"})
            report = {"job_id": key, "rescored_at": time.time(), "trials": results}
            write_json(self._path(job) / "rescore.json", report)
            return report

    def export(self, key, user):
        with self.lock:
            job = self._owned(key, user)
            if job["status"] not in TERMINAL:
                raise ValueError("请先等待批次结束或停止后再导出")
            directory = self._path(job)
            plan = read_json(directory / "batch/plan.json") if (directory / "batch/plan.json").is_file() else None
            return {"schema_version": "spacesim-autonomy-export/1", "job": self._view(job),
                    "task": read_json(directory / "task.json"), "plan": plan}

    def close(self):
        with self.lock:
            self.closing = True
            for job in self.jobs.values():
                if job["status"] not in TERMINAL:
                    self.stop(job["id"], job["owner"])
            thread = self.thread
        if thread is not None:
            thread.join(timeout=35)
            if thread.is_alive():
                raise RuntimeError("benchmark coordinator has not released its owned processes")
        if any(job["status"] not in TERMINAL for job in self.jobs.values()):
            raise RuntimeError("benchmark process cleanup is incomplete; retain the coordinator lease")
        self.lease.close()
