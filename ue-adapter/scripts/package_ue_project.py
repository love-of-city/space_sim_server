"""Cook an isolated project and validate dynamic assets in its packaged runtime."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

from build_ue_project import ADAPTER, ENGINE_POLICY, host_platform, toolchain_identity, validate_engine
from prepare_cook_staging import REPO, digest, prepare


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
    manifests = args.manifest or [REPO / "manifests/assets/ue-cook-base-assets.json"]
    prepare(args.project, staging, manifests)
    platform = host_platform()
    output.mkdir(parents=True, exist_ok=True)
    script = engine / "Engine/Build/BatchFiles" / ("RunUAT.sh" if platform == "Linux" else "RunUAT.bat")
    command = (["bash", str(script)] if platform == "Linux" else [str(script)]) + [
        "BuildCookRun", f"-project={staging / 'BskUnrealRenderer.uproject'}", "-noP4", f"-platform={platform}",
        "-clientconfig=Development", "-build", "-cook", "-stage", "-pak", "-archive", "-ubtargs=-NoUBA", f"-archivedirectory={output}"]
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
    report = output / "runtime-asset-validation.json"
    with (output / "package-runtime.log").open("w") as log:
        subprocess.run([str(binaries[0]), "-unattended", "-nosplash", "-NullRHI",
                        f"-BskValidateCookAssets={staging / 'cook-runtime-assets.json'}",
                        f"-BskCookValidationReport={report}"], cwd=binaries[0].parent,
                       check=True, stdout=log, stderr=subprocess.STDOUT, timeout=180)
    result = json.loads(report.read_text())
    if result.get("schema") != "ue-cook-runtime-validation/1" or result.get("passed") is not True:
        raise RuntimeError(f"Packaged runtime asset validation failed: {report}")
    evidence = {"schema": "ue-package-validation/1", "platform": platform, "engine": engine_version,
                "cook_inputs_sha256": digest(staging / "cook-input-manifest.json"),
                "build_manifest_sha256": digest(output / "ue-build-manifest.json"),
                "packaged_executable_sha256": digest(binaries[0]),
                "pak_files": {str(path.relative_to(output)): digest(path) for path in sorted(output.rglob("*.pak"))},
                "runtime_report_sha256": digest(report), "packaged_assets": "passed", "gpu_capture": "not-run"}
    (output / "package-validation.json").write_text(json.dumps(evidence, indent=2) + "\n")
    print(json.dumps(evidence))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
