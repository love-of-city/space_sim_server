"""Cook an isolated project and validate dynamic assets in its packaged runtime."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from build_ue_project import ADAPTER, ENGINE_POLICY, host_platform, toolchain_identity, validate_engine
from prepare_cook_staging import DEFAULT_MANIFESTS, digest, prepare


def assert_runtime_report(report: Path, manifest: Path) -> dict:
    result = json.loads(report.read_text())
    expected = [item["object_path"] for item in json.loads(manifest.read_text())["assets"]]
    actual = result.get("assets", [])
    paths = [item.get("object_path") for item in actual]
    if (result.get("schema") != "ue-cook-runtime-validation/1" or result.get("passed") is not True
            or result.get("errors") != [] or not expected or len(set(expected)) != len(expected)
            or len(paths) != len(expected) or set(paths) != set(expected)
            or any(item.get("loaded") is not True for item in actual)):
        raise RuntimeError(f"Packaged runtime asset validation failed or incomplete: {report}")
    return result


def validate_packaged_runtime(binary: Path, manifest: Path, report_directory: Path) -> Path:
    if report_directory.exists() and any(report_directory.iterdir()):
        raise ValueError("Runtime validation output must be empty; refusing to overwrite evidence")
    report_directory.mkdir(parents=True, exist_ok=True)
    report = report_directory / "runtime-asset-validation.json"
    with (report_directory / "package-runtime.log").open("w") as log:
        subprocess.run([str(binary), "-unattended", "-nosplash", "-nosound", "-NullRHI",
                        f"-BskValidateCookAssets={manifest}", f"-BskCookValidationReport={report}"],
                       cwd=binary.parent, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    assert_runtime_report(report, manifest)
    return report


def package_evidence(output: Path, staging: Path, binary: Path, report: Path, platform: str, engine_version: dict) -> dict:
    return {"schema": "ue-package-validation/1", "platform": platform, "engine": engine_version,
            "cook_inputs_sha256": digest(staging / "cook-input-manifest.json"),
            "build_manifest_sha256": digest(output / "ue-build-manifest.json"),
            "packaged_executable_sha256": digest(binary),
            "container_files": {str(path.relative_to(output)): digest(path) for path in sorted(output.rglob("*"))
                                if path.suffix in (".pak", ".utoc", ".ucas")},
            "runtime_report_sha256": digest(report), "packaged_assets": "passed", "gpu_capture": "not-run"}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--engine", type=Path, required=True)
    parser.add_argument("--project", type=Path, default=ADAPTER / "Unreal/BskUnrealRenderer")
    parser.add_argument("--staging", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, action="append")
    parser.add_argument("--engine-policy", type=Path, default=ENGINE_POLICY)
    parser.add_argument("--timeout", type=int, default=14400)
    args = parser.parse_args(argv)
    engine, staging, output = args.engine.resolve(), args.staging.resolve(), args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("Package output must be empty; refusing to overwrite a release")
    if output.is_relative_to(staging) or staging.is_relative_to(output):
        parser.error("Staging and package output must be separate directories")
    engine_version = validate_engine(engine, args.engine_policy)
    manifests = args.manifest or DEFAULT_MANIFESTS
    prepare(args.project, staging, manifests)
    platform = host_platform()
    output.mkdir(parents=True, exist_ok=True)
    script = engine / "Engine/Build/BatchFiles" / ("RunUAT.sh" if platform == "Linux" else "RunUAT.bat")
    command = (["bash", str(script)] if platform == "Linux" else [str(script)]) + [
        "BuildCookRun", f"-project={staging / 'BskUnrealRenderer.uproject'}", "-noP4", f"-platform={platform}",
        "-clientconfig=Development", "-build", "-cook", "-cookall", "-stage", "-pak", "-archive", "-ubtargs=-NoUBA", f"-archivedirectory={output}"]
    if platform == "Linux":
        command.append("-AdditionalCookerOptions=-DDC=InstalledNoZenLocalFallback")
    with (output / "package-build.log").open("w") as log:
        subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT, timeout=args.timeout)
    toolchain = output / "package-toolchain.json"
    toolchain.write_text(json.dumps(toolchain_identity((output / "package-build.log").read_text(), engine), indent=2) + "\n")
    subprocess.run([sys.executable, str(ADAPTER / "scripts/write_ue_build_manifest.py"),
                    "--project", str(staging), "--engine", str(engine), "--platform", platform,
                    "--target", "BskUnrealRenderer", "--configuration", "Development",
                    "--output", str(output / "ue-build-manifest.json"), "--toolchain-json", str(toolchain)], check=True)
    binary_name = "BskUnrealRenderer" if platform == "Linux" else "BskUnrealRenderer.exe"
    binaries = [path for path in output.rglob(binary_name) if f"Binaries/{platform}" in path.as_posix()]
    if len(binaries) != 1:
        raise RuntimeError(f"Expected one packaged game binary, got {binaries}")
    report = validate_packaged_runtime(binaries[0], staging / "cook-runtime-assets.json", output / "runtime-validation")
    evidence = package_evidence(output, staging, binaries[0], report, platform, engine_version)
    (output / "package-validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
