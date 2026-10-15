"""Offline reproduction and evidence index for the user's AOC implementations.

Uses subprocesses so native Basilisk and optional Python MuJoCo cannot mix DLLs.
No network calls, remote Git operations, or Feishu writes are performed.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
TASKS = [
    ("recvuDDwIMmgvr", "BSK 调研与官方上帝视角远程交会 Demo", ["official"]),
    ("recvuDxoc2CQT1", "姿轨控控制（卡尔曼滤波＋PD 控制）", ["native", "in-orbit", "orbit-raise"]),
    ("recvuDyq2I8Wud", "BSK 飞轮动力属性与 MuJoCo 静态属性", ["native"]),
    ("recvuGFbfikK8K", "真实飞轮动力学驱动的远程交会 Demo", ["native", "in-orbit"]),
    ("recvuDDwIMSVVc", "团队模型远程交会接入 SpaceSim", ["in-orbit", "orbit-raise", "render"]),
    ("recvw5TEOevemU", "千米级交会至 100 m 外，包含双星 GNSS", ["in-orbit"]),
    ("recvw5Z4qlhkrf", "300→500 km 升轨后交会至 100 m 外，包含双星 GNSS", ["orbit-raise"]),
    ("reczz28KuW3GWuBG", "GNSS 误差与随机轨道误差", ["native", "in-orbit", "orbit-raise"]),
]
NATIVE_FILES = ["tests/test_attitude_control.py", "tests/test_team_orbital_reference.py",
                "tests/test_team_rendezvous_runtime.py", "tests/test_team_asset_relocation.py",
                "tests/test_flight_environment.py", "tests/test_rendezvous_options.py"]


def execute(output, label, arguments):
    command = [sys.executable, *map(str, arguments)]
    print(f"[{label}] running", flush=True)
    env = os.environ.copy()
    env["MPLBACKEND"] = "Agg"
    env["PYTEST_ADDOPTS"] = ""
    with (output / f"{label}.log").open("w", encoding="utf-8") as stream:
        subprocess.run(command, cwd=ROOT, env=env, stdout=stream, stderr=subprocess.STDOUT, check=True)
    print(f"[{label}] passed", flush=True)


def passed_json(path):
    return json.loads(path.read_text(encoding="utf-8")).get("passed") is True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--catalog", type=Path, help="Local SARM UE mesh catalog; required for full reproduction")
    parser.add_argument("--verify-existing", action="store_true", help="Recheck prior local outputs without rerunning dynamics")
    args = parser.parse_args()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if not args.verify_existing and not args.catalog:
        parser.error("--catalog is required for the SpaceSim model/render task")
    # Never leave an old successful receipt visible after a failed rerun.
    (output / "acceptance.json").write_text(
        json.dumps({"local_complete": False, "status": "verification_in_progress"}), encoding="utf-8"
    )
    if not args.verify_existing:
        execute(output, "official", ["tools/run_official_rendezvous.py", "--output-dir", output / "official"])
        execute(output, "native-tests", ["-m", "pytest", *NATIVE_FILES, "-q", f"--junitxml={output / 'native-tests.xml'}"])
    # Verify-existing re-runs the raw-data, original-model and recording verifiers.
    for scenario in ("in-orbit", "orbit-raise"):
        folder = output / scenario
        if not args.verify_existing:
            execute(output, scenario, ["-m", "simulation.orbital_rendezvous.run", "--scenario", scenario,
                                      "--output-dir", folder, "--catalog", args.catalog.resolve()])
        execute(output, f"{scenario}-independent", ["tools/verify_team_flight.py", folder])
        execute(output, f"{scenario}-handoff", ["tools/verify_team_handoff.py", folder / "handoff.json"])
        execute(output, f"{scenario}-recording", ["tools/verify_team_recording.py", folder / "team_rendezvous.bskrec"])
    xml = ET.parse(output / "native-tests.xml").getroot()
    cases = list(xml.iter("testcase"))
    test_names = {case.get("name", "") for case in cases}
    required = {"test_native_arm_disturbance_rejection_enabled_vs_disabled",
                "test_native_attitude_impulse_recovers_and_reports_torque_saturation",
                "test_native_rate_disturbance_is_damped_without_losing_reference",
                "test_navigation_initialization_requires_observation_and_filters_noise",
                "test_model_preserves_team_assets_and_mass"}
    official = json.loads((output / "official/official_report.json").read_text(encoding="utf-8"))
    statuses = {"official": official.get("passed") is True and official["source_sha256"] == hashlib.sha256(
                    (ROOT / "tools/vendor/scenarioRendezVous.py").read_bytes()).hexdigest(),
                "native": len(cases) >= 25 and required <= test_names and not any(
                    list(case) for case in cases)}
    for scenario in ("in-orbit", "orbit-raise"):
        statuses[scenario] = all(passed_json(output / scenario / name)
                                 for name in ("independent_validation.json", "handoff_validation.json"))
    statuses["render"] = all(passed_json(output / scenario / "recording_validation.json")
                              for scenario in ("in-orbit", "orbit-raise"))
    tasks = [{"record_id": record_id, "task": name, "local_complete": all(statuses[e] for e in evidence),
              "evidence_groups": evidence} for record_id, name, evidence in TASKS]
    # Hash only deliverable reports/plots/numerical artifacts, never accounts or secrets.
    extensions = {".json", ".xml", ".npz", ".png", ".log"}
    hashes = {p.relative_to(output).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in output.rglob("*") if p.is_file() and p.suffix in extensions
              and p.name != "acceptance.json"}
    report = {"schema": "spacesim-aoc-local-acceptance/1", "local_complete": all(t["local_complete"] for t in tasks),
              "tasks": tasks, "evidence": statuses, "native_test_count": len(cases),
              "artifact_sha256": hashes,
              "scope": "Listed AOC implementations, including the GNSS/orbital-error task and retained 7 km scenario; offline evidence only. No remote status/merge, shared-team completion or autonomous capture claim.",
              "render_scope": "Full native recordings and team mesh bindings revalidated here; actual UE/WebRTC evidence indexed in AOC_SEVEN_TASKS.md."}
    (output / "acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"local_complete": report["local_complete"], "completed_tasks": sum(t["local_complete"] for t in tasks),
                      "output": str(output)}, ensure_ascii=False))
    return 0 if report["local_complete"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
