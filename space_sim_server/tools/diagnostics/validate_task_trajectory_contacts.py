"""Check actual scripted-task contacts without changing native dynamics.

Runs the production CPU task runner. Contact snapshots are read after every
400 Hz native tick; legal finger/object grasp and object/own-mount contacts are
kept. Task success, observed trajectory clearance and sampling coverage are
reported independently. No Python ``mujoco`` module is loaded by this tool.
"""

from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
from collections import Counter
from datetime import datetime, timezone
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
TASKS = ("drawer_cycle", "drawer_lift", "aviation_plug_insert",
         "guide_plug_insert", "cylindrical_plug_insert")
POLICY_VERSION = "task-trajectory-contacts/2"
FINGERS = frozenset(("ee_finger1", "ee_finger2"))
PLUGS = {name + "_plug_insert": name + "_plug"
         for name in ("aviation", "guide", "cylindrical")}
MODULES = tuple(name + "_pickup" for name in ("aviation", "guide", "cylindrical")) + tuple(
    name + "_insert_" + size for name in ("aviation", "guide", "cylindrical")
    for size in ("40p1", "40p5")) + ("bolt_storage_hole", "bolt_assembly_hole",
                                    "drawer", "locking_mechanism")
# Production stage names: PlugInsertTask.latch_stages (v11),
# DrawerCycleTask.stages (v5), DrawerLiftTask.stages. A short failed plan is
# never evidence that the full extraction/insertion/retraction path is clear.
REQUIRED_OPERATION_STAGES = {
    **{task: ("settle", "approach", "grip", "extract", "lift", "transfer", "above_hole",
              "insert", "release", "retract", "verify") for task in PLUGS},
    "drawer_cycle": ("settle", "approach", "grip", "lift", "return", "release", "retract", "verify"),
    "drawer_lift": ("settle", "raise", "above_drawer", "align", "open", "descend", "grip", "lift", "hold"),
}


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n",
                          encoding="utf-8")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ContactPolicy:
    """An explicit body/installation ownership policy, independent of the task.

    Background contacts that involve neither the robot nor the active object
    are inventoried, but do not claim that an unrelated mechanism is collision
    free. Robot self contacts are reported separately from layout interference.
    No stage, penetration tolerance or success criterion weakens this policy.
    """

    def __init__(self, model, task, target_site=None, ignore_free_bolt=False):
        if task not in TASKS:
            raise ValueError(f"Unsupported task: {task}")
        self.task = task
        self.ignore_free_bolt = bool(ignore_free_bolt)
        self.model = Path(model).resolve()
        root = ET.parse(self.model).getroot()
        arm = root.find('.//body[@name="link1"]')
        if arm is None:
            raise ValueError("Expected the delivered robot subtree rooted at link1")
        self.robot_bodies = {node.get("name") for node in arm.iter("body")}
        # The bus owns the delivered arm-base geometry; its deck is a distinct
        # body and must never become a legal gripper contact by ancestry.
        self.robot_bodies.add("cubesat_bus")
        if not FINGERS.issubset(self.robot_bodies):
            raise ValueError("Expected both original gripper finger bodies")
        self.active_bodies = ({PLUGS[task]} if task in PLUGS
                              else {"drawer_base", "drawer_buckle"})
        kind = PLUGS[task].removesuffix("_plug") if task in PLUGS else None
        self.target_site = target_site or (kind + "_insert_40p5" if kind else None)
        if kind and self.target_site not in {kind + "_insert_40p1", kind + "_insert_40p5"}:
            raise ValueError("Target site must be one of this plug's original holes")
        self.legal_modules = ({kind + "_pickup", self.target_site} if kind else {"drawer"})
        self.body_modules = {"drawer_base": "drawer", "drawer_buckle": "drawer",
                             "asm_board3": "drawer", "locking_rod": "locking_mechanism",
                             "fastener": "locking_mechanism", "hook": "locking_mechanism",
                             "limit_clip": "locking_mechanism", "bolt": "bolt_storage_hole"}
        self.body_modules.update({body: body.removesuffix("_plug") + "_pickup"
                                  for body in PLUGS.values()})
        self.geom_modules = {}
        manifest = self.model.with_suffix(".layout.json")
        self.manifest = manifest if manifest.is_file() else None
        if self.manifest:
            record = json.loads(manifest.read_text(encoding="utf-8-sig"))
            self.geom_modules.update(record.get("precision_bore_ownership", {}))
            for item in record.get("translated_bodies", []):
                self.body_modules[item["body"]] = item["module"]
        for body in root.findall(".//body"):
            name = body.get("name", "")
            module = next((m for m in MODULES if name in ("layout_" + m,
                                                           "layout_" + m + "_bores")), None)
            if module:
                for child in body.iter("body"):
                    self.body_modules[child.get("name", "")] = module
        for geom in root.findall(".//geom"):
            name = geom.get("name", "")
            match = re.fullmatch(r"(aviation|guide|cylindrical)_(?:bore|pin_floor)_(40p1|40p5)_.+", name)
            if match:
                self.geom_modules[name] = match[1] + "_insert_" + match[2]
        self.geom_modules["board_thread_sdf"] = "bolt_assembly_hole"
        self.ownership_assets = {}
        self._assign_delivered_shared_mounts(root)

    def _assign_delivered_shared_mounts(self, root):
        """Identify wholly owned original convex plate pieces conservatively.

        The delivered model predates separate installation bodies. A convex
        collider belongs to a site only when *all* its vertices lie strictly
        inside that site's XY Voronoi cell, as used by the layout partitioner.
        A piece spanning cells stays unknown. This never allows a shared board
        wholesale, and generated layouts use their explicit body ownership.
        """
        assets = {node.get("name"): node for node in root.findall("./asset/mesh")}
        compiler = root.find("compiler")
        meshdir = compiler.get("meshdir", ".") if compiler is not None else "."
        for board, sites in (("asm_board2", MODULES[:3]), ("asm_board1", MODULES[3:11])):
            body = root.find(f'.//body[@name="{board}"]')
            if body is None:
                continue
            points = []
            for name in sites:
                node = body.find(f'site[@name="{name}"]')
                if node is not None:
                    points.append((name, tuple(map(float, node.get("pos", "0 0 0").split()))))
            if len(points) != len(sites):
                continue
            for geom in body.findall("geom"):
                if (geom.get("name") in self.geom_modules
                        or geom.get("pos", "0 0 0") != "0 0 0"
                        or geom.get("quat", "1 0 0 0") != "1 0 0 0"
                        or any(geom.get(tag) for tag in ("euler", "axisangle", "xyaxes", "zaxis"))):
                    continue
                asset = assets.get(geom.get("mesh"))
                if asset is None or any(asset.get(tag) for tag in ("refpos", "refquat")):
                    continue
                filename = asset.get("file", "")
                if not filename.lower().endswith(".stl"):
                    continue
                path = (self.model.parent / meshdir / filename).resolve()
                data = path.read_bytes()
                if len(data) < 84:
                    continue
                triangles = struct.unpack_from("<I", data, 80)[0]
                if len(data) != 84 + triangles * 50:
                    continue
                self.ownership_assets[str(path)] = hashlib.sha256(data).hexdigest()
                scale = tuple(map(float, asset.get("scale", "1 1 1").split()))
                owner = None
                for i in range(triangles):
                    for j in range(3):
                        vertex = struct.unpack_from("<fff", data, 84 + i * 50 + 12 + j * 12)
                        x, y = vertex[0] * scale[0], vertex[1] * scale[1]
                        ranked = sorted(((x - p[0]) ** 2 + (y - p[1]) ** 2, name)
                                        for name, p in points)
                        if (ranked[1][0] - ranked[0][0] <= 1e-12
                                or owner is not None and owner != ranked[0][1]):
                            owner = "ambiguous"
                            break
                        owner = ranked[0][1]
                    if owner == "ambiguous":
                        break
                if owner not in (None, "ambiguous"):
                    self.geom_modules[geom.get("name", "")] = owner

    def module(self, body, geom):
        return self.geom_modules.get(geom, self.body_modules.get(body))

    def classify(self, body_a, body_b, geom_a, geom_b, distance):
        if not math.isfinite(distance):
            return "invalid_contact"
        if distance > 0:
            return "positive_candidate"
        a_robot, b_robot = body_a in self.robot_bodies, body_b in self.robot_bodies
        if a_robot and b_robot:
            return "robot_self_contact"
        if a_robot or b_robot:
            robot, other = (body_a, body_b) if a_robot else (body_b, body_a)
            if robot in FINGERS and other in self.active_bodies:
                return "legal_grasp"
            return ("deferred_bolt_contact" if self.ignore_free_bolt and "bolt" in (body_a, body_b)
                    else "forbidden_robot_contact")
        a_active, b_active = body_a in self.active_bodies, body_b in self.active_bodies
        if a_active and b_active:
            return "legal_active_internal"
        if a_active or b_active:
            other_body, other_geom = ((body_b, geom_b) if a_active else (body_a, geom_a))
            if self.module(other_body, other_geom) in self.legal_modules:
                return "legal_mating"
            return ("deferred_bolt_contact" if self.ignore_free_bolt and "bolt" in (body_a, body_b)
                    else "forbidden_active_object_contact")
        return "background_contact"

    def metadata(self):
        return dict(version=POLICY_VERSION, task=self.task, target_site=self.target_site,
                    robot_bodies=sorted(self.robot_bodies), active_bodies=sorted(self.active_bodies),
                    legal_object_mount_modules=sorted(self.legal_modules),
                    contact_threshold_m=0.0,
                    robot_self_contacts="reported separately; excluded from layout decision",
                    background_contacts="inventoried; not an unrelated-object stability test",
                    unknown_active_object_mount="forbidden",
                    ignore_free_bolt=self.ignore_free_bolt,
                    deferred_bolt_reason=("User explicitly deferred the free grey bolt because it will be fixed later; "
                                          "only otherwise forbidden contacts involving body 'bolt' are deferred; "
                                          "native physics is unchanged" if self.ignore_free_bolt else None),
                    delivered_shared_mount_ownership="all convex vertices strictly inside one site's XY Voronoi cell",
                    force_or_impulse_available=False)


class ContactAccumulator:
    def __init__(self, policy):
        self.policy = policy
        self.names = {"bodies": {}, "geoms": {}}
        self.pairs = {}
        self.counts = Counter()
        self.samples = 0
        self.first_ns = self.last_ns = None
        self.max_interval_ns = 0
        self.nonmonotonic_samples = 0
        self.invalid_rows = 0
        self.substeps = set()
        self.observed_stages = []
        self.task_state_observed_stages = []

    def observe_task_stage(self, stage):
        if stage not in self.task_state_observed_stages:
            self.task_state_observed_stages.append(stage)

    def observe(self, stamp, stage, contacts, names=None, stepper=None):
        stamp = int(stamp)
        if self.last_ns is not None:
            self.max_interval_ns = max(self.max_interval_ns, stamp - self.last_ns)
            self.nonmonotonic_samples += int(stamp <= self.last_ns)
        else:
            self.first_ns = stamp
        self.last_ns = stamp
        self.samples += 1
        if stage not in self.observed_stages:
            self.observed_stages.append(stage)
        if names is not None:
            self.names = names
        if stepper is not None:
            self.substeps.add(int(stepper.substeps))
        seen = set()
        for row in contacts:
            if len(row) != 5 or not all(math.isfinite(float(x)) for x in row):
                self.invalid_rows += 1
                continue
            ga, gb, ba, bb = (int(value) for value in row[:4])
            if stepper is not None:
                for kind, object_type, ids in (("geoms", 5, (ga, gb)), ("bodies", 1, (ba, bb))):
                    for index in ids:
                        if str(index) not in self.names[kind]:
                            self.names[kind][str(index)] = stepper._name(object_type, index) if index >= 0 else ""
            body_a, body_b = (self.names["bodies"].get(str(index), "") for index in (ba, bb))
            geom_a, geom_b = (self.names["geoms"].get(str(index), "") for index in (ga, gb))
            distance = float(row[4])
            category = self.policy.classify(body_a, body_b, geom_a, geom_b, distance)
            self.counts[category] += 1
            if category == "positive_candidate":
                continue
            # Canonical order allows multiple contact points and pair reversals
            # to share one inventory entry without inflating tick durations.
            ends = sorted(((body_a, geom_a), (body_b, geom_b)))
            key = (category, *ends[0], *ends[1])
            if key not in self.pairs:
                self.pairs[key] = dict(category=category, bodies=[x[0] for x in ends],
                                       geoms=[x[1] for x in ends], first_time_ns=stamp,
                                       last_time_ns=stamp, minimum_separation_m=distance,
                                       contact_point_samples=0, observed_ticks=0, stages={})
            record = self.pairs[key]
            record["last_time_ns"] = stamp
            record["minimum_separation_m"] = min(record["minimum_separation_m"], distance)
            record["contact_point_samples"] += 1
            if key not in seen:
                record["observed_ticks"] += 1
                record["stages"][stage] = record["stages"].get(stage, 0) + 1
                seen.add(key)

    def report(self, stock_summary, native_returncode, expected_hz=400, error=None):
        expected_period = 1_000_000_000 // expected_hz
        last_expected = round(stock_summary.get("simulated_seconds", -1) * 1e9)
        expected_samples = last_expected // expected_period + 1 if last_expected >= 0 else None
        coverage = (self.samples > 0 and self.first_ns == 0 and self.last_ns == last_expected
                    and self.samples == expected_samples and self.nonmonotonic_samples == 0
                    and self.max_interval_ns <= expected_period and self.substeps == {1}
                    and self.invalid_rows == 0 and error is None and expected_hz == 400)
        forbidden = sum(count for kind, count in self.counts.items() if kind.startswith("forbidden"))
        clear = forbidden == 0 and self.invalid_rows == 0 and self.samples > 0
        success = stock_summary.get("result") == "success"
        required = REQUIRED_OPERATION_STAGES[self.policy.task]
        missing = [stage for stage in required if stage not in self.observed_stages]
        positions = [self.observed_stages.index(stage) for stage in required if stage in self.observed_stages]
        ordered = positions == sorted(positions)
        planning_failed = "planning_failed" in str(stock_summary.get("reason", ""))
        operational_path_complete = not missing and ordered and not planning_failed
        rows = sorted(self.pairs.values(), key=lambda row: (not row["category"].startswith("forbidden"),
                                                           row["category"], row["bodies"], row["geoms"]))
        return dict(schema=POLICY_VERSION, task_success=success,
                    task_result=stock_summary.get("result"), task_reason=stock_summary.get("reason"),
                    trajectory_clear=clear, coverage_complete=coverage,
                    observed_stages=self.observed_stages,
                    task_state_observed_stages=self.task_state_observed_stages,
                    required_operation_stages=list(required), missing_operation_stages=missing,
                    operational_path_complete=operational_path_complete,
                    operation_stages_in_order=ordered, planning_failed=planning_failed,
                    accepted=success and clear and coverage and operational_path_complete and native_returncode == 0,
                    native_returncode=native_returncode, execution_error=error,
                    forbidden_contact_point_samples=forbidden, categories=dict(self.counts),
                    policy=self.policy.metadata(),
                    coverage=dict(observed_hz=expected_hz, expected_physics_hz=400,
                                  samples=self.samples, expected_samples=expected_samples,
                                  first_time_ns=self.first_ns, last_time_ns=self.last_ns,
                                  expected_last_time_ns=last_expected,
                                  max_interval_ns=self.max_interval_ns,
                                  nonmonotonic_samples=self.nonmonotonic_samples,
                                  substeps=sorted(self.substeps), invalid_rows=self.invalid_rows,
                                  scope="native contact snapshots at each integration tick; no continuous-time or force claim"),
                    contact_pairs=rows)


def analyze_existing(args, policy, output):
    source = args.analyze_existing.resolve()
    names = json.loads((source / "names.json").read_text())
    summary = json.loads((source / "run/summary.json").read_text())
    accumulator = ContactAccumulator(policy)
    accumulator.substeps.add(int(summary.get("substeps", 1)))
    native_ticks = (source / "contacts-400hz.jsonl.gz").is_file()
    trace = source / ("contacts-400hz.jsonl.gz" if native_ticks else "state-contact-200hz.jsonl.gz")
    with gzip.open(trace, "rt") as stream:
        for line in stream:
            row = json.loads(line)
            stamp = row["interval_end_ns"] if native_ticks else row["sim_time_ns"]
            accumulator.observe(stamp, row["stage"], row["contacts"], names=names)
    state_trace = source / "states-200hz.jsonl.gz"
    if state_trace.is_file():
        with gzip.open(state_trace, "rt") as stream:
            for line in stream:
                accumulator.observe_task_stage(json.loads(line)["stage"])
    launch_path = source / "launch.json"
    launch = json.loads(launch_path.read_text()) if launch_path.is_file() else {}
    previous_path = source / "acceptance.json"
    previous = json.loads(previous_path.read_text()) if previous_path.is_file() else {}
    native_code = launch.get("native_returncode", launch.get("returncode",
                              0 if summary.get("result") == "success" else 1))
    report = accumulator.report(summary, native_code, expected_hz=400 if native_ticks else 200,
                                error=previous.get("execution_error"))
    report["analyzed_existing"] = str(source)
    report["analysis_source_sha256"] = {str(path): sha256(path) for path in
                                        (trace, source / "names.json", source / "run/summary.json",
                                         Path(__file__), policy.model)}
    report["ownership_asset_sha256"] = policy.ownership_assets
    if state_trace.is_file():
        report["analysis_source_sha256"][str(state_trace)] = sha256(state_trace)
    if launch_path.is_file():
        # Preserve the actual native launch unchanged. The analysis provenance
        # and effective policy above bind the new classification separately.
        (output / "launch.json").write_bytes(launch_path.read_bytes())
        report["original_launch"] = dict(path=str(launch_path), sha256=sha256(launch_path))
    if not native_ticks:
        report["coverage"]["scope"] = "historical 200 Hz task snapshots; 400 Hz physics coverage unavailable"
    write_json(output / "acceptance.json", report)
    return report


def run_native(args, policy, output):
    sys.path[:0] = [str(ROOT), str(ROOT / "backend"),
                   str(ROOT.parent / "space_sim_UE_Adapter/Adapters"),
                   str(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples")]
    # Preserve the official DLL load order before importing the stepper.
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
    load_native_grasp_module(ROOT / "model/SARM/platform")
    from simulation.local_mujoco_stepper import LocalMujocoStepper
    from simulation.scripted_tasks.native_state import DrawerState, PlugState
    from collection import run_scripted_task

    accumulator = ContactAccumulator(policy)
    task_holder = {}
    contact_stream = gzip.open(output / "contacts-400hz.jsonl.gz", "wt", compresslevel=1)
    state_stream = gzip.open(output / "states-200hz.jsonl.gz", "wt", compresslevel=1)
    originals = {cls: (cls.__init__, cls.sample) for cls in (DrawerState, PlugState)}
    original_update = LocalMujocoStepper.UpdateState

    def update(stepper, now):
        value = original_update(stepper, now)
        task = task_holder.get("task")
        stage = str(task.stage) if task is not None else "initializing"
        contacts = stepper.contact_snapshot()
        previous_ns = accumulator.last_ns
        accumulator.observe(now, stage, contacts, stepper=stepper)
        contact_stream.write(json.dumps(dict(interval_start_ns=previous_ns,
                                            interval_end_ns=int(now), stage=stage,
                                            native_step_count=int(stepper.step_count),
                                            contacts=contacts.tolist()), separators=(",", ":")) + "\n")
        return value

    def wrap_init(original):
        def initialize(reader, *values, **keywords):
            original(reader, *values, **keywords)
            task_holder["task"] = reader.task
        return initialize

    def wrap_sample(original):
        def sample(reader, time_s, q, v, wrench):
            value = original(reader, time_s, q, v, wrench)
            accumulator.observe_task_stage(reader.task.stage)
            state_stream.write(json.dumps(dict(sim_time_ns=round(float(time_s) * 1e9),
                                               stage=reader.task.stage, result=reader.task.result,
                                               joint_position=q.tolist(), joint_velocity=v.tolist(),
                                               qpos=reader.stepper.qpos.tolist(),
                                               qvel=reader.stepper.qvel.tolist(),
                                               metrics=reader.metrics(value)),
                                              separators=(",", ":")) + "\n")
            return value
        return sample

    argv = ["--model", str(args.model.resolve()), "--scene", str(args.scene.resolve()),
            "--task", args.task, "--output", str(output / "run"),
            "--max-sim-seconds", str(args.max_sim_seconds), "--no-save-trace"]
    if args.target_site:
        argv += ["--target-site", args.target_site]
    if args.initial_joints_deg:
        argv += ["--initial-joints-deg", *map(str, args.initial_joints_deg)]
    if args.native_geometry:
        argv += ["--native-geometry"]
    sources = [Path(__file__), ROOT / "collection/run_scripted_task.py",
               ROOT / "simulation/local_mujoco_stepper.py",
               ROOT / "simulation/scripted_tasks/native_state.py", args.model, args.scene]
    if policy.manifest:
        sources.append(policy.manifest)
    launch = dict(schema=POLICY_VERSION, started_utc=datetime.now(timezone.utc).isoformat(),
                  argv=[sys.executable, str(ROOT / "collection/run_scripted_task.py"), *argv],
                  source_sha256={str(path.resolve()): sha256(path) for path in sources},
                  ownership_asset_sha256=policy.ownership_assets,
                  contact_policy=policy.metadata(),
                  observation="read-only wrappers after original native tick and task sample",
                  physics_control_and_success_thresholds_unchanged=True, ue_gpu_started=False)
    write_json(output / "launch.json", launch)
    code, error = 2, None
    try:
        LocalMujocoStepper.UpdateState = update
        for cls, (initialize, sample) in originals.items():
            cls.__init__, cls.sample = wrap_init(initialize), wrap_sample(sample)
        code = run_scripted_task.main(argv=argv)
    except Exception:
        error = traceback.format_exc()
        (output / "execution-error.txt").write_text(error, encoding="utf-8")
    finally:
        LocalMujocoStepper.UpdateState = original_update
        for cls, (initialize, sample) in originals.items():
            cls.__init__, cls.sample = initialize, sample
        contact_stream.close()
        state_stream.close()
        write_json(output / "names.json", accumulator.names)
    summary_path = output / "run/summary.json"
    summary = json.loads(summary_path.read_text()) if summary_path.is_file() else {}
    report = accumulator.report(summary, code, error=error)
    write_json(output / "acceptance.json", report)
    launch.update(finished_utc=datetime.now(timezone.utc).isoformat(), native_returncode=code,
                  accepted=report["accepted"])
    write_json(output / "launch.json", launch)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--scene", type=Path)
    parser.add_argument("--task", choices=TASKS, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--target-site")
    parser.add_argument("--ignore-free-bolt", action="store_true",
                        help="Explicitly defer otherwise forbidden contacts involving only the free grey bolt; physics unchanged")
    parser.add_argument("--initial-joints-deg", type=float, nargs=6)
    parser.add_argument("--native-geometry", action="store_true")
    parser.add_argument("--max-sim-seconds", type=float, default=360.0)
    parser.add_argument("--analyze-existing", type=Path,
                        help="Read saved native 400 Hz or historical 200 Hz contacts; no simulation")
    args = parser.parse_args(argv)
    if args.scene is None and args.analyze_existing is None:
        parser.error("--scene is required for a native run")
    policy = ContactPolicy(args.model, args.task, args.target_site,
                           ignore_free_bolt=args.ignore_free_bolt)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = (analyze_existing(args, policy, output) if args.analyze_existing
              else run_native(args, policy, output))
    print(json.dumps({key: report[key] for key in ("task_success", "trajectory_clear",
                                                  "coverage_complete", "operational_path_complete", "accepted")}
                     | {"report": str(output / "acceptance.json")}), flush=True)
    return 0 if report["accepted"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
