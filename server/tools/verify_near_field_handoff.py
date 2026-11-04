"""Native two-second continuity probe; NOT a completed distant/near-field flight."""
import argparse
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.near_field.model import prepare  # noqa: E402
from simulation.near_field.run import fixture, run  # noqa: E402
from simulation.near_field.navigation import origin_navigation  # noqa: E402
from Basilisk.utilities import RigidBodyKinematics as rbk  # noqa: E402


def check(output):
    output = output.resolve()
    _, _, manifest, _ = prepare(ROOT, output/"source")
    handoff = fixture(ROOT, manifest)
    handoff["source_kind"] = "continuity probe fixture; no phases 1-3 completion claim"
    handoff["state"]["sim_time_s"] = 5120.
    handoff["body_properties"] = manifest["bodies"]
    for i, name in enumerate(("capture_target", "cubesat_bus")):
        body = handoff["state"]["bodies"][i]
        body["attitude_wxyz"] = [float(np.cos(.0005)), 0, 0, float(np.sin(.0005))]
        body["angular_velocity_rad_s"] = [0, 0, .0001]
        R = rbk.EP2C(np.array(body["attitude_wxyz"])).T
        center = np.array(manifest["bodies"][name]["center_body_m"])
        handoff["gnss_estimates_eci"][i] = np.r_[np.array(body["position_m"])+R@center,
            np.array(body["velocity_m_s"])+R@np.cross(body["angular_velocity_rad_s"], center)].tolist()
    handoff["reaction_wheels"] = [dict(body=f"rw_{a}", joint=f"rw_{a}_spin", position_rad=p, speed_rad_s=v)
        for a, p, v in zip("xyz", [.1, -.2, .3], [10., 20., -15.])]
    handoff["covariances"] = [np.diag([9.] * 3+[.25]*3).tolist()] * 2
    path = output/"probe_handoff.json"
    path.write_text(json.dumps(handoff, indent=2))
    args = SimpleNamespace(handoff=path, output_dir=output/"propagation", seed=7, vision_loss_phase=None,
                           communication_loss=None, catalog=None, live=False, state_port=0, render_port=5558, duration=2.)
    exit_code = run(args)  # Expected 2: deliberately stops long before CaptureHold.
    report = json.loads((args.output_dir/"mission_report.json").read_text())
    state, covariance = origin_navigation(handoff)
    checks = {"initial_continuity": report["checks"]["initial_continuity"],
              "large_covariance_inherited": np.allclose(report["initial_navigation"]["covariance"], covariance, atol=0, rtol=0),
              "origin_navigation_inherited": np.allclose(report["initial_navigation"]["state"], state[0]-state[1], atol=0, rtol=0),
              "nonzero_time_continues": report["final_time_s"] == 5122.,
              "no_external_state_writes": report["local_dynamics"]["external_state_writes"] == 0,
              "probe_stopped_before_terminal": exit_code == 2 and report["outcome"] == "FarHandoff"}
    result = {"checks": checks, "passed": all(checks.values()), "continuity": report["continuity"],
              "scope": "native synthetic handoff continuity only; not real distant-flight acceptance"}
    (output/"continuity_result.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    raise SystemExit(check(parser.parse_args().output_dir))
