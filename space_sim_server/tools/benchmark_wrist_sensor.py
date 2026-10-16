"""Offline comparison; no UE, no pacing, same MuJoCo and five substeps."""

from pathlib import Path
import sys, time, json, copy, statistics, xml.etree.ElementTree as ET
import numpy as np


def main():
    ROOT = Path(__file__).resolve().parents[1]
    sys.path[:0] = [
        str(ROOT),
        str(ROOT.parent / "space_sim_UE_Adapter/Unreal/BskUnrealRenderer/examples"),
    ]
    from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module

    n = load_native_grasp_module(ROOT / "model/SARM/platform")
    from simulation.sarm_model_profile import load_model_profile
    from simulation.joint_reference_publisher import HeldJointReferencePublisher
    from Basilisk.utilities import macros
    from simulation.wrench_sensors import WRIST_SENSOR

    folder = ROOT / "run/wrist-ft-development"
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "attitude_control.json").write_bytes(
        (ROOT / "model/SARM/platform/attitude_control.json").read_bytes()
    )
    source = ROOT / "model/SARM/platform/sarm_20261012.xml"
    root = ET.parse(source).getroot()
    root.find("compiler").set("meshdir", str(source.parent))
    models = {}
    for variant in ("baseline", "sensor"):
        model = copy.deepcopy(root)
        if variant == "baseline":
            sensors = model.find("sensor")
            for item in list(sensors):
                if item.get("name") in ("wrist_ft_force", "wrist_ft_torque"):
                    sensors.remove(item)
            if not list(sensors):
                model.remove(sensors)
            flange = model.find('.//body[@name="ee_base"]')
            tool = flange.find('body[@name="wrist_ft_tool"]')
            for item in list(tool):
                if item.tag != "site" or item.get("name") != "wrist_ft_site":
                    flange.append(item)
            flange.remove(tool)
            for exclusion in model.findall("./contact/exclude"):
                for attribute in ("body1", "body2"):
                    if exclusion.get(attribute) == "wrist_ft_tool":
                        exclusion.set(attribute, "ee_base")
        path = folder / f"benchmark_{variant}.xml"
        ET.ElementTree(model).write(path, encoding="utf-8", xml_declaration=True)
        models[variant] = path
    profile = load_model_profile(source)
    n.JOINTS = profile.joints
    n.ACTUATORS = profile.arm_actuators
    n.KP = np.full(6, 500.0)
    n.KD = np.full(6, 50.0)
    n.TORQUE_LIMITS = np.full(6, 100.0)
    n.TIME_STEP = 1 / 240
    results = []
    for variant in ("baseline", "sensor", "sensor", "baseline", "baseline", "sensor"):
        n.MODEL_PATH = models[variant]
        q = np.zeros(8)
        ref = HeldJointReferencePublisher(lambda t: (q.copy(), np.zeros(8)), 8)
        sim, scene, keep, _ = n._build_simulation(
            external_reference=ref,
            dynamics_backend="local",
            local_substeps=5,
            record_history=False,
            attitude_control_enabled=False,
        )
        sim.AddModelToTask("graspTask", ref, 9000)
        sim.InitializeSimulation()
        sim.ConfigureStopTime(macros.sec2nano(1))
        sim.ExecuteSimulation()
        started = time.perf_counter()
        for tick in range(31, 121):
            q[6] = 0.15 * (tick / 30 - 1)
            q[7] = -q[6]
            sim.ConfigureStopTime(macros.sec2nano(tick / 30))
            sim.ExecuteSimulation()
            if variant == "sensor":
                end = sim.local_stepper.wrench_interval_end_ns
                WRIST_SENSOR.observation(sim.local_stepper, end)
        elapsed = time.perf_counter() - started
        stepper = sim.local_stepper
        row = dict(
            variant=variant,
            elapsed_s=elapsed,
            simulated_s=3,
            real_time_factor=3 / elapsed,
            nq=stepper.nq,
            nv=stepper.nv,
            qpos=stepper.qpos.tolist(),
            qvel=stepper.qvel.tolist(),
            max_contacts=stepper.max_contacts,
            max_solver_iterations=stepper.max_solver_iterations,
        )
        results.append(row)
        print(
            json.dumps({k: v for k, v in row.items() if k not in ("qpos", "qvel")}),
            flush=True,
        )
        del stepper, sim, scene, keep, ref
    base = [r for r in results if r["variant"] == "baseline"]
    sensor = [r for r in results if r["variant"] == "sensor"]
    b = statistics.median(r["elapsed_s"] for r in base)
    s = statistics.median(r["elapsed_s"] for r in sensor)
    report = dict(
        scope="native local dynamics, no BSK orbital environment, no UE, 240 Hz outer tick x 5 substeps; 1 s warmup + 3 s gripper opening; alternating 3 trials",
        baseline_median_s=b,
        sensor_median_s=s,
        overhead_percent=100 * (s / b - 1),
        max_abs_qpos_difference=float(
            np.max(np.abs(np.array(base[0]["qpos"]) - sensor[0]["qpos"]))
        ),
        max_abs_qvel_difference=float(
            np.max(np.abs(np.array(base[0]["qvel"]) - sensor[0]["qvel"]))
        ),
        trials=results,
    )
    (folder / "benchmark.json").write_text(
        json.dumps(report, indent=2), encoding="utf-8"
    )
    print(json.dumps({k: v for k, v in report.items() if k != "trials"}), flush=True)


if __name__ == "__main__":
    main()
