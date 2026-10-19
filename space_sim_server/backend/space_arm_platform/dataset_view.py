"""Dataset read model shared by authenticated platform APIs.

Preserves native sample clocks; never interpolates sensor samples for export.
"""

from collections import OrderedDict
import json
import math
from pathlib import Path
import re
import statistics
import threading
from urllib.parse import quote

from .compact_storage import is_compact, read_observations, read_wrench

COLORS = ["#64d9c4", "#779cff", "#f2bb69", "#d397ed", "#ef8395", "#92c878"]


def number(value):
    if (
        isinstance(value, (float, int))
        and not isinstance(value, bool)
        and math.isfinite(value)
    ):
        return value
    return None


def get_path(obj, path):
    for key in path:
        if isinstance(obj, dict):
            obj = obj.get(key)
        elif isinstance(obj, list) and isinstance(key, int) and key < len(obj):
            obj = obj[key]
        else:
            return None
    return number(obj)


def specs():
    groups = []

    def add(key, title, unit, items, angle=False):
        groups.append(dict(id=key, title=title, unit=unit, angle=angle, series=items))

    def series(key, name, path, index, target=None, scale=1):
        return dict(
            id=key,
            label=name,
            path=path,
            color=COLORS[index % 6],
            target=target,
            scale=scale,
        )

    add(
        "joints",
        "关节角度",
        "rad",
        [
            series(
                f"j{i + 1}",
                f"J{i + 1}",
                ["joint_position_rad", i],
                i,
                ["target_joint_position_rad", i],
            )
            for i in range(6)
        ],
        True,
    )
    add(
        "velocity",
        "关节速度",
        "rad/s",
        [
            series(f"v{i + 1}", f"J{i + 1}", ["joint_velocity_rad_s", i], i)
            for i in range(6)
        ],
        True,
    )
    add(
        "force",
        "腕部三轴力",
        "N",
        [
            series("f" + a, "F" + a, ["wrist_wrench", "force_n", i], i)
            for i, a in enumerate("xyz")
        ],
    )
    add(
        "torque",
        "腕部三轴力矩",
        "N·m",
        [
            series("t" + a, "T" + a, ["wrist_wrench", "torque_nm", i], i)
            for i, a in enumerate("xyz")
        ],
    )
    add(
        "gripper_opening",
        "夹爪开合 · 0 闭合 / 1 张开",
        "0–1",
        [
            series(
                "grip_opening",
                "开合量",
                ["gripper_opening_normalized"],
                0,
                ["target_gripper_opening_normalized"],
            )
        ],
    )
    add(
        "gripper",
        "夹爪驱动角",
        "rad",
        [
            series(
                "grip",
                "驱动角",
                ["gripper_drive_position_rad"],
                0,
                ["target_gripper_drive_position_rad"],
            )
        ],
        True,
    )
    add(
        "gripper_m",
        "夹爪直线开度",
        "mm",
        [
            series(
                "grip_m",
                "开度",
                ["gripper_position_m"],
                0,
                ["target_gripper_position_m"],
                1000,
            )
        ],
    )
    add(
        "position",
        "末端位置 · 卫星本体系",
        "m",
        [
            series("p" + a, a.upper(), ["end_effector_position_body_m", i], i)
            for i, a in enumerate("xyz")
        ],
    )
    add(
        "drawer",
        "抽屉位移 · 抽出 / 放回",
        "mm",
        [
            series(
                "drawer_displacement",
                "相对初始位移",
                ["scripted_task", "drawer_displacement_m"],
                0,
                scale=1000,
            )
        ],
    )
    add(
        "drawer_buckle",
        "抽屉卡扣角度",
        "rad",
        [series("drawer_buckle", "卡扣", ["scripted_task", "buckle_rad"], 0)],
        True,
    )
    add(
        "insertion",
        "插头装配指标",
        "mm",
        [
            series(
                "depth",
                "插入深度",
                ["scripted_task", "insertion_depth_m"],
                0,
                scale=1000,
            ),
            series(
                "radial",
                "孔口径向误差",
                ["scripted_task", "radial_error_m"],
                1,
                scale=1000,
            ),
            series("gap", "法兰间隙", ["scripted_task", "seat_gap_m"], 2, scale=1000),
        ],
    )
    return groups


class Store:
    def __init__(self, root):
        self.root = Path(root).resolve()
        self.cache = OrderedDict()
        self.lock = threading.Lock()

    def episode(self, key):
        if not re.fullmatch(r"[\w.-]+", key) or key in (".", ".."):
            raise ValueError("无效的录制名称")
        path = (self.root / key).resolve()
        if not path.is_relative_to(self.root) or not path.is_dir():
            raise FileNotFoundError("找不到录制目录")
        return path

    def metadata(self, path):
        return json.loads((path / "metadata.json").read_text(encoding="utf-8-sig"))

    def listing(self):
        records = []
        if self.root.is_dir():
            for path in sorted(self.root.iterdir(), reverse=True):
                if not path.is_dir() or not (path / "metadata.json").is_file():
                    continue
                try:
                    meta = self.metadata(path)
                    records.append(
                        dict(
                            id=path.name,
                            task=meta.get("task", ""),
                            instruction=meta.get("instruction", ""),
                            outcome=meta.get("outcome", ""),
                            status=meta.get("dataset_status", meta.get("status", "")),
                            frames=meta.get("dataset_frame_count"),
                            fps=meta.get("dataset_fps"),
                            created=meta.get("created_wall_time_ns"),
                            readable=(path / "steps.jsonl").is_file() or is_compact(path),
                        )
                    )
                except (OSError, ValueError):
                    records.append(
                        dict(id=path.name, readable=False, task="元数据无法读取")
                    )
        return dict(root=str(self.root), exists=self.root.is_dir(), episodes=records)

    def load(self, key):
        path = self.episode(key)
        source_files = [path / "metadata.json"]
        if is_compact(path):
            source_files += sorted((path / "lerobot/data").rglob("*.parquet"))
            source_files += sorted((path / "lerobot/auxiliary").rglob("*.parquet"))
            source_files += sorted((path / "lerobot/high_frequency").rglob("*.parquet"))
        else:
            source_files += [path / "steps.jsonl"]
            high_file = path / "lerobot/high_frequency/wrist_wrench.jsonl"
            if high_file.is_file():
                source_files.append(high_file)
        if any(not file.resolve().is_relative_to(path) for file in source_files):
            raise ValueError("Dataset source path escapes episode")
        signature = (key, tuple((str(file), file.stat().st_mtime_ns, file.stat().st_size)
                                for file in source_files))
        with self.lock:
            if signature in self.cache:
                return self.cache[signature]
        meta = self.metadata(path)
        groups = specs()
        flat = [s for g in groups for s in g["series"]]
        values = {s["id"]: [] for s in flat}
        targets = {s["id"]: [] for s in flat if s["target"]}
        times, frames, stages, valid_wrench = [], [], [], []
        start = None
        absolute_times = []
        for line_no, obs in enumerate(read_observations(path), 1):
            try:
                stamp = int(obs["sim_time_ns"])
            except (KeyError, ValueError, TypeError) as exc:
                raise ValueError(f"第 {line_no} 行缺少有效仿真时间") from exc
            if start is None:
                start = stamp
            if absolute_times and stamp <= absolute_times[-1]:
                raise ValueError(f"第 {line_no} 行仿真时间未递增，不能直接回放")
            absolute_times.append(stamp)
            times.append((stamp - start) / 1e9)
            frames.append(str(obs.get("render_frame_id", "")))
            stages.append((obs.get("scripted_task") or {}).get("stage", ""))
            valid = (obs.get("wrist_wrench") or {}).get("valid") is True
            valid_wrench.append(valid)
            for s in flat:
                value = get_path(obs, s["path"])
                if s["path"][0] == "wrist_wrench" and not valid:
                    value = None
                values[s["id"]].append(
                    value * s["scale"] if value is not None else None
                )
                if s["target"]:
                    value = get_path(obs, s["target"])
                    targets[s["id"]].append(
                        value * s["scale"] if value is not None else None
                    )
        if not times:
            raise ValueError("该记录还没有可显示的观察数据")
        for g in groups:
            for s in g["series"]:
                s["values"] = values[s["id"]]
                s["targetValues"] = targets.get(s["id"])
                s["available"] = any(v is not None for v in s["values"])
                del s["path"], s["target"], s["scale"]
        # Auxiliary sensor stream has its own clock; never upsample RGB values.
        high_rows = list(read_wrench(path))
        if high_rows:
            high_times = [(int(row["sim_time_ns"]) - start) / 1e9 for row in high_rows]
            if any(
                abs(b - a - 0.005) > 1e-8 for a, b in zip(high_times, high_times[1:])
            ):
                raise ValueError("200 Hz 六维力数据存在缺失或重复采样")
            for group_key, title, field, unit in (
                ("force_200hz", "六维力 · 200 Hz 原始力", "force_n", "N"),
                ("torque_200hz", "六维力 · 200 Hz 原始力矩", "torque_nm", "N·m"),
            ):
                groups.append(
                    dict(
                        id=group_key,
                        title=title,
                        unit=unit,
                        angle=False,
                        times=high_times,
                        sample_rate_hz=200,
                        series=[
                            dict(
                                id=group_key + axis,
                                label=axis.upper(),
                                color=["#64d9c4", "#6eb5ff", "#e7ad6b"][i],
                                values=[
                                    row[field][i] if row["valid"] else None
                                    for row in high_rows
                                ],
                                targetValues=None,
                                available=bool(high_rows),
                            )
                            for i, axis in enumerate("xyz")
                        ],
                    )
                )
        videos = []
        video_root = path / "lerobot/videos"
        if video_root.is_dir():
            for file in sorted(video_root.rglob("*.mp4")):
                if not file.resolve().is_relative_to(path):
                    continue
                relative = file.relative_to(path).as_posix()
                camera = file.relative_to(video_root).parts[0]
                videos.append(
                    dict(
                        camera=camera,
                        file=relative,
                        url="/api/datasets/"
                        + quote(key)
                        + "/media/"
                        + quote(relative, safe="/"),
                    )
                )
        differences = [b - a for a, b in zip(times, times[1:])]
        median = statistics.median(differences) if differences else 0
        fps = number(meta.get("dataset_fps")) or (1 / median if median else 30)
        contiguous = all(
            b == a + 1
            for a, b in zip(
                [int(f) for f in frames if f.isdigit()],
                [int(f) for f in frames if f.isdigit()][1:],
            )
        ) and all(f.isdigit() for f in frames)
        complete = meta.get("dataset_status") == "complete"
        declared_frames = meta.get("dataset_frame_count")
        sync_ok = (
            complete
            and declared_frames == len(times)
            and contiguous
            and all(abs(dt - 1 / fps) < 1e-6 for dt in differences)
        )
        # Multipart datasets require their own episode offsets; do not silently assume one clip.
        cameras = [v["camera"] for v in videos]
        sync_ok = sync_ok and len(set(cameras)) == len(cameras)
        result = dict(
            id=key,
            metadata=meta,
            times=times,
            frames=frames,
            stages=stages,
            groups=groups,
            videos=videos,
            fps=fps,
            duration=times[-1],
            count=len(times),
            start_sim_time_ns=str(start),
            valid_wrench_count=sum(valid_wrench),
            sync_ok=sync_ok,
            warning=""
            if sync_ok
            else "该记录的完整性、连续帧或视频分片条件不满足自动同步要求，曲线可查看，视频联动已关闭。",
        )
        payload = json.dumps(
            result, ensure_ascii=False, allow_nan=False, separators=(",", ":")
        ).encode("utf-8")
        with self.lock:
            self.cache[signature] = payload
            while len(self.cache) > 3:
                self.cache.popitem(last=False)
        return payload
