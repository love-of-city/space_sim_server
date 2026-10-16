"""Isolated, two-world MuJoCo/Warp CONTACT-KERNEL benchmark.

Uses the complete platform XML, including all custom SDF geometry and wrist
sensors. This is not a BSK/UE collector: orbital force generation, attitude
control, online IK, cameras, transport and encoding are outside this test.
CPU and GPU use the SAME MuJoCo package version, snapshot and held controls.
Never run this Python environment inside the production Basilisk process.
"""

from __future__ import annotations
import argparse
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model/SARM/platform/sarm_20261012.xml"


def build_plugin(out):
    import mujoco
    from build_native_acceleration import compiler_environment, BUILD_FLAGS

    work = out / "native"
    work.mkdir(exist_ok=True)
    source = ROOT / "native/sarm_thread_sdf.cpp"
    headers = list((Path(mujoco.__file__).parent / "include").rglob("mujoco.h"))
    if len(headers) != 1:
        raise RuntimeError("Cannot identify headers for the installed MuJoCo runtime")
    include = headers[0].parent.parent
    env = compiler_environment()
    binary = Path(env["VCToolsInstallDir"]) / "bin/Hostx64/x64"
    definition = work / "mujoco.def"
    definition.write_text(
        "LIBRARY mujoco.dll\nEXPORTS\nmj_getPluginConfig\nmjp_defaultPlugin\nmjp_registerPlugin\n",
        encoding="ascii",
    )
    # Include the unmodified production formula and expose it for numerical checks.
    check = work / "sdf_probe.cpp"
    check.write_text(
        '#include "' + source.as_posix() + '"\n'
        'extern "C" __declspec(dllexport) void benchmark_sdf(int n, const double* p, const int* mode, double* d, double* g) {\n'
        "for (int i=0;i<n;++i) { d[i]=distance(p+3*i,mode[i]); gradient(g+3*i,p+3*i,mode[i]); }\n}\n",
        encoding="utf-8",
    )
    subprocess.run(
        [
            str(binary / "lib.exe"),
            "/nologo",
            f"/def:{definition}",
            f"/out:{work / 'mujoco.lib'}",
            "/machine:x64",
        ],
        cwd=work,
        env=env,
        check=True,
    )
    dll = work / "sarm_thread_probe.dll"
    subprocess.run(
        [
            str(binary / "cl.exe"),
            *BUILD_FLAGS,
            str(check),
            f"/I{include}",
            f"/Fo:{work / 'probe.obj'}",
            f"/Fe:{dll}",
            "/link",
            str(work / "mujoco.lib"),
        ],
        cwd=work,
        env=env,
        check=True,
    )
    return dll


def load_model(plugin):
    import mujoco

    mujoco.mj_loadPluginLibrary(str(plugin))
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    model.opt.timestep = 0.0025
    return model


def snapshot(model, trace, at):
    import mujoco

    previous = None
    selected = None
    with trace.open(encoding="utf-8") as stream:
        for line in stream:
            row = json.loads(line)
            if int(row["sim_time_ns"]) * 1e-9 >= at:
                selected = row
                break
            previous = row
    if selected is None or previous is None:
        raise ValueError("Snapshot and previous sample required")
    data = mujoco.MjData(model)
    data.qpos[:] = selected["local_qpos"]
    dt = (int(selected["sim_time_ns"]) - int(previous["sim_time_ns"])) * 1e-9
    mujoco.mj_differentiatePos(
        model,
        data.qvel,
        dt,
        np.asarray(previous["local_qpos"]),
        np.asarray(selected["local_qpos"]),
    )
    for i in range(6):
        data.ctrl[model.actuator(f"joint{i + 1}_position").id] = selected[
            "joint_reference"
        ][i]
    data.ctrl[model.actuator("ee_fingers_position").id] = 0.5 * (
        selected["joint_reference"][6] - selected["joint_reference"][7]
    )
    mujoco.mj_forward(model, data)
    return data, selected


def sdf_check(plugin):
    import warp as wp
    import benchmark_warp_sdf as sdf

    rng = np.random.default_rng(123)
    points = rng.uniform(
        [-0.025, -0.025, -0.115], [0.025, 0.025, 0.01], size=(10000, 3)
    ).astype(np.float32)
    modes = (np.arange(len(points)) % 5).astype(np.int32)
    native_points = points.astype(np.float64)
    native_d = np.empty(len(points), np.float64)
    native_g = np.empty((len(points), 3), np.float64)
    lib = ctypes.CDLL(str(plugin))
    lib.benchmark_sdf.argtypes = [ctypes.c_int, *([ctypes.c_void_p] * 4)]
    lib.benchmark_sdf(
        len(points),
        native_points.ctypes.data,
        modes.ctypes.data,
        native_d.ctypes.data,
        native_g.ctypes.data,
    )
    dist = wp.empty(len(points), dtype=float)
    grad = wp.empty(len(points), dtype=wp.vec3)
    wp.launch(
        sdf.evaluate,
        dim=len(points),
        inputs=[
            wp.array(points, dtype=wp.vec3),
            wp.array(modes, dtype=int),
            dist,
            grad,
        ],
    )
    wp.synchronize()
    derr = float(np.max(np.abs(native_d - dist.numpy())))
    gerr = float(np.max(np.abs(native_g - grad.numpy())))
    result = dict(
        samples=len(points),
        modes=5,
        max_distance_error_m=derr,
        max_gradient_component_error=gerr,
        distance_tolerance_m=1e-7,
        gradient_component_tolerance=0.01,
        passed=derr < 1e-7 and gerr < 0.01,
    )
    if not result["passed"]:
        raise RuntimeError("SDF port check failed: " + json.dumps(result))
    return result


def cpu_worker(plugin, trace, at, steps, ready, start_file, label):
    import mujoco

    m = load_model(plugin)
    d, _ = snapshot(m, trace, at)
    # Retain a separate initial state; a warmup step must not shift the timed workload.
    q, v, ctrl = d.qpos.copy(), d.qvel.copy(), d.ctrl.copy()
    mujoco.mj_step(m, d)
    mujoco.mj_resetData(m, d)
    d.qpos[:], d.qvel[:], d.ctrl[:] = q, v, ctrl
    mujoco.mj_forward(m, d)
    ready.write_text("ready")
    while not start_file.exists():
        time.sleep(0.01)
    start = time.perf_counter()
    for _ in range(steps):
        mujoco.mj_step(m, d)
    elapsed = time.perf_counter() - start
    return dict(
        label=label,
        step_wall_seconds=elapsed,
        qpos=d.qpos.tolist(),
        sensordata=d.sensordata.tolist(),
        finite=bool(np.isfinite(d.qpos).all() and np.isfinite(d.sensordata).all()),
        contacts=int(d.ncon),
        warning_count=int(d.warning.number.sum()),
        final_time=float(d.time),
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--trace", type=Path, required=True)
    p.add_argument("--at", type=float, default=83)
    p.add_argument("--steps", type=int, default=400)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    started = time.perf_counter()

    def gpu_status():
        result = subprocess.run(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,utilization.gpu,memory.used,memory.total",
                "--format=csv,noheader",
            ],
            capture_output=True,
            text=True,
            timeout=10,
            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
        )
        return dict(
            at=datetime.now().astimezone().isoformat(), values=result.stdout.strip()
        )

    report = dict(
        scope="contact kernel only; no BSK, IK, UE, dataset encoding",
        complete=False,
        worlds=2,
        steps=args.steps,
        timestep_s=0.0025,
        model_sha256=hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        snapshot_time_s=args.at,
        production_equivalence_verified=False,
        sdf_cpp_sha256=hashlib.sha256(
            (ROOT / "native/sarm_thread_sdf.cpp").read_bytes()
        ).hexdigest(),
        sdf_warp_sha256=hashlib.sha256(
            (ROOT / "tools/benchmark_warp_sdf.py").read_bytes()
        ).hexdigest(),
    )
    try:
        import mujoco
        import mujoco_warp as mjw
        import warp as wp
        import benchmark_warp_sdf as sdf

        report["gpu_before"] = gpu_status()
        wp.init()
        wp.set_device("cuda:0")
        sdf.install()
        report["versions"] = dict(
            mujoco=mujoco.__version__, mujoco_warp=mjw.__version__, warp=wp.__version__
        )
        report["device"] = str(wp.get_device())
        plugin = build_plugin(out)
        m = load_model(plugin)
        d, row = snapshot(m, args.trace, args.at)
        report["model"] = dict(
            nq=m.nq,
            nv=m.nv,
            nbody=m.nbody,
            ngeom=m.ngeom,
            nmesh=m.nmesh,
            neq=m.neq,
            nsensor=m.nsensor,
            nplugin=m.nplugin,
            initial_contacts=d.ncon,
            initial_constraints=d.nefc,
            source_stage=row["stage"],
        )
        report["sdf_validation"] = sdf_check(plugin)
        print(json.dumps(dict(stage="loaded", **report)), flush=True)
        t = time.perf_counter()
        wm = mjw.put_model(m)
        wd = mjw.put_data(m, d, nworld=2, nconmax=2048, njmax=4096)
        wp.synchronize()
        report["gpu_transfer_seconds"] = time.perf_counter() - t
        report["gpu_after_transfer"] = gpu_status()
        print(
            json.dumps(
                dict(stage="transferred", seconds=report["gpu_transfer_seconds"])
            ),
            flush=True,
        )
        t = time.perf_counter()
        mjw.step(wm, wd)
        wp.synchronize()
        report["gpu_first_step_compile_seconds"] = time.perf_counter() - t
        print(
            json.dumps(
                dict(stage="compiled", seconds=report["gpu_first_step_compile_seconds"])
            ),
            flush=True,
        )
        # Fresh state after JIT, then capture without changing the state.
        wd = mjw.put_data(m, d, nworld=2, nconmax=2048, njmax=4096)
        wp.synchronize()
        with wp.ScopedCapture() as capture:
            mjw.step(wm, wd)
        wp.synchronize()
        t = time.perf_counter()
        for _ in range(args.steps):
            wp.capture_launch(capture.graph)
        wp.synchronize()
        gpu_seconds = time.perf_counter() - t
        report["gpu_after_steps"] = gpu_status()
        gpu_q = wd.qpos.numpy()
        gpu_sensor = wd.sensordata.numpy()
        report["gpu"] = dict(
            step_wall_seconds=gpu_seconds,
            simulated_seconds_per_world=args.steps * 0.0025,
            finite=bool(np.isfinite(gpu_q).all() and np.isfinite(gpu_sensor).all()),
            qpos=gpu_q.tolist(),
            sensordata=gpu_sensor.tolist(),
            final_time=wd.time.numpy().tolist(),
            worlds_agree_max_abs=float(np.max(np.abs(gpu_q[0] - gpu_q[1]))),
            overflow=wd.overflow.numpy().tolist(),
        )
        print(json.dumps(dict(stage="gpu_finished", seconds=gpu_seconds)), flush=True)
        # GPU is idle during the CPU comparison; no simultaneous benchmarks.
        start_file = out / "cpu.start"
        with ProcessPoolExecutor(max_workers=2) as pool:
            ready = [out / f"cpu-{i}.ready" for i in range(2)]
            futures = [
                pool.submit(
                    cpu_worker,
                    plugin,
                    args.trace,
                    args.at,
                    args.steps,
                    ready[i],
                    start_file,
                    i,
                )
                for i in range(2)
            ]
            deadline = time.perf_counter() + 180
            try:
                while not all(x.exists() for x in ready):
                    for future in futures:
                        if future.done():
                            future.result()
                    if time.perf_counter() > deadline:
                        raise TimeoutError("CPU workers did not initialize")
                    time.sleep(0.1)
            finally:
                t = time.perf_counter()
                start_file.write_text("start")
            report["cpu"] = [f.result() for f in futures]
            report["cpu_pair_step_wall_seconds"] = time.perf_counter() - t
        cpu_q = np.asarray(report["cpu"][0]["qpos"])
        report["comparison"] = dict(
            cpu_over_gpu_speed_ratio=report["cpu_pair_step_wall_seconds"] / gpu_seconds,
            max_abs_qpos_difference=float(np.max(np.abs(cpu_q - gpu_q[0]))),
            max_abs_sensor_difference=float(
                np.max(
                    np.abs(np.asarray(report["cpu"][0]["sensordata"]) - gpu_sensor[0])
                )
            ),
            note="These are held-control contact windows; not a drawer task success test or calibrated force equivalence.",
        )
        report["complete"] = (
            report["gpu"]["finite"]
            and not any(report["gpu"]["overflow"])
            and all(x["finite"] and x["warning_count"] == 0 for x in report["cpu"])
        )
    except BaseException as exc:
        report["error"] = type(exc).__name__ + ": " + str(exc)
        raise
    finally:
        report["total_wall_seconds"] = time.perf_counter() - started
        (out / "benchmark.json").write_text(
            json.dumps(report, indent=2), encoding="utf-8"
        )
    print(
        json.dumps(
            dict(
                stage="finished",
                comparison=report["comparison"],
                report=str(out / "benchmark.json"),
            )
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
