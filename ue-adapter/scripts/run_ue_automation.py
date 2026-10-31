"""Run UE Automation with explicit reports, timeout, and nonempty success checks."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess

from build_ue_project import ADAPTER, ENGINE_POLICY, host_platform, validate_engine


def assert_report(report: Path) -> dict:
    data = json.loads((report / "index.json").read_text(encoding="utf-8-sig"))
    tests = data.get("tests", [])
    if not tests:
        raise RuntimeError("UE Automation executed no tests; this is not a passing validation")
    failures = [test for test in tests if test.get("state", "").lower() != "success"]
    if failures or data.get("failed", 0) or data.get("notRun", 0):
        names = [test.get("fullTestPath", test.get("testDisplayName", "unknown")) for test in failures]
        raise RuntimeError(f"UE Automation has failed, skipped, or incomplete tests: {names}")
    return {"tests": len(tests), "status": "passed", "report": str(report)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=ADAPTER / "Unreal/BskUnrealRenderer/BskUnrealRenderer.uproject")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--tests", default="BskUnreal")
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--gpu", action="store_true", help="Use GPU instead of NullRHI; not a full dataset acceptance")
    parser.add_argument("--engine-policy", type=Path, default=ENGINE_POLICY)
    args = parser.parse_args(argv)
    engine, report = args.engine.resolve(), args.report.resolve()
    validate_engine(engine, args.engine_policy)
    if report.exists() and any(report.iterdir()):
        parser.error("Use an empty report directory so stale results cannot pass a new run")
    report.mkdir(parents=True, exist_ok=True)
    platform = host_platform()
    editor = engine / f"Engine/Binaries/{platform}" / ("UnrealEditor-Cmd" if platform == "Linux" else "UnrealEditor-Cmd.exe")
    if not editor.is_file():
        raise FileNotFoundError(editor)
    command = [str(editor), str(args.project.resolve()), "-unattended", "-nop4", "-nosplash",
               f"-ExecCmds=Automation RunTests {args.tests};Quit",
               f"-ReportExportPath={report}", f"-abslog={report / 'automation.log'}"]
    command.append("-RenderOffscreen" if args.gpu else "-NullRHI")
    if platform == "Linux":
        command.append("-DDC=InstalledNoZenLocalFallback")
    subprocess.run(command, check=True, timeout=args.timeout)
    result = assert_report(report)
    (report / "validation-result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
