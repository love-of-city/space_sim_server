"""Recompute local flight acceptance from numerical telemetry, independent of report booleans."""

import argparse
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulation.orbital_rendezvous.reference import EARTH_RADIUS, FlightProfile
from space_arm_platform.rendezvous_config import FlightOptions


def verify(folder, repository=ROOT):
    folder, repository = Path(folder), Path(repository)
    report = json.loads((folder / "mission_report.json").read_text(encoding="utf-8"))
    handoff = json.loads((folder / "handoff.json").read_text(encoding="utf-8"))
    with np.load(folder / "team_telemetry.npz", allow_pickle=False) as archive:
        d = {key: archive[key] for key in archive.files}
    t, truth = d["time_s"], d["truth"]
    count = len(t)
    expected_shapes = {"time_s": (count,), "truth": (count, 2, 6),
                       "gnss": (count, 2, 6), "estimate": (count, 2, 6),
                       "force_n": (count, 3), "range_m": (count,),
                       "altitude_m": (count,), "tracking_error_m": (count,),
                       "relative_speed_hill_m_s": (count,),
                       "wheel_speed_rad_s": (count, 3), "wheel_torque_nm": (count, 3),
                       "attitude_error_rad": (count,), "angular_velocity_rad_s": (count, 3),
                       "navigation_valid": (count,)}
    for name, shape in expected_shapes.items():
        if name not in d or d[name].shape != shape or not np.isfinite(d[name]).all():
            raise ValueError(f"Missing, malformed or nonfinite telemetry: {name}")
    if count < 2 or not np.allclose(np.diff(t), 0.5, rtol=0, atol=1e-8):
        raise ValueError("Incomplete 2 Hz telemetry")
    integrated = "mission_options" in report
    options = FlightOptions.from_dict(report["mission_options"]) if integrated else None
    profile = FlightProfile(**report["profile"])
    target, chaser = truth[:, 0], truth[:, 1]
    relative = chaser[:, :3] - target[:, :3]
    distance = np.linalg.norm(relative, axis=1)
    altitude = np.linalg.norm(chaser[:, :3], axis=1) - EARTH_RADIUS
    orbit_rate = np.cross(target[:, :3], target[:, 3:]) / np.sum(target[:, :3]**2, axis=1)[:, None]
    speed = np.linalg.norm(chaser[:, 3:] - target[:, 3:] - np.cross(orbit_rate, relative), axis=1)
    references = np.array([profile.chaser(float(value))[0] for value in t])
    if integrated:
        references += target[:, :3] - np.array([profile.target(float(value))[0] for value in t])
    tracking = np.linalg.norm(chaser[:, :3] - references, axis=1)
    hold = t >= profile.duration_s - profile.hold_duration_s - 1e-8
    source = repository / report["model_manifest"]["source_model"]
    xml = ET.parse(source).getroot()
    max_speeds = np.fromstring(xml.find('./custom/numeric[@name="rw_max_speed_rad_s"]').get("data"), sep=" ")
    max_torques = np.array([float(xml.find(f'./actuator/motor[@name="rw_{a}_motor"]').get("ctrlrange").split()[1]) for a in "xyz"])
    raw_error, estimate_error = d["gnss"] - truth, d["estimate"] - truth
    raw_rmse = np.sqrt(np.mean(np.sum(raw_error[20:, :, :3]**2, axis=2), axis=0))
    filtered_rmse = np.sqrt(np.mean(np.sum(estimate_error[20:, :, :3]**2, axis=2), axis=0))
    source_match = all(
        hashlib.sha256((repository / "simulation/orbital_rendezvous" / name).read_bytes()).hexdigest() == digest
        for name, digest in report["source_hashes"].items()
    )
    if integrated:
        source_match = source_match and bool(report.get("integration_source_hashes")) and all(
            hashlib.sha256((repository / name).read_bytes()).hexdigest() == digest
            for name, digest in report["integration_source_hashes"].items())
    checks = {
        "mission_profile_matches_scenario": vars(profile) == vars(FlightProfile(
            scenario=report["scenario"], handoff_distance_m=options.handoff_distance_m if integrated else 100.5)),
        "code_fingerprints_match": source_match,
        "model_fingerprint_matches": hashlib.sha256(source.read_bytes()).hexdigest() == report["model_manifest"]["source_sha256"] == handoff["source_sha256"],
        "full_duration": abs(t[0]) < 1e-9 and abs(t[-1] - profile.duration_s) < 1e-8,
        "range_matches_raw_states": np.allclose(distance, d["range_m"], rtol=0, atol=1e-6),
        "altitude_matches_raw_states": np.allclose(altitude, d["altitude_m"], rtol=0, atol=1e-6),
        "tracking_matches_raw_states": np.allclose(tracking, d["tracking_error_m"], rtol=0, atol=1e-6),
        "speed_matches_raw_states": np.allclose(speed, d["relative_speed_hill_m_s"], rtol=0, atol=1e-8),
        "minimum_range_100m": np.min(distance) >= 100.0,
        "hold_120s": np.sum(hold) >= 241,
        "hold_range": np.any(hold) and np.all(np.abs(distance[hold] - (profile.handoff_distance_m if integrated else 100)) < (5 if integrated else 1)),
        "hold_speed": np.any(hold) and np.all(speed[hold] < (0.2 if integrated else 0.02)),
        "tracking_after_acquisition": np.any(t >= 600) and np.max(tracking[t >= 600]) < (5 if integrated else 2),
        "final_target_orbit": abs(altitude[-1] - (np.linalg.norm(target[-1, :3]) - EARTH_RADIUS if integrated else 500000)) < (10 if integrated else 2),
        "initial_orbit": abs(altitude[0] - (np.linalg.norm(report["initial_states_eci"][1][:3])-EARTH_RADIUS if integrated else (300000 if profile.scenario == "orbit-raise" else 500000))) < (0.01 if integrated else 1),
        "force_limit": np.max(np.linalg.norm(d["force_n"], axis=1)) <= report["model_manifest"]["propulsion"]["limit_n"] + 1e-9,
        "wheel_torque_limits": np.all(np.abs(d["wheel_torque_nm"]) <= max_torques + 1e-12),
        "wheel_speed_limits": np.all(np.abs(d["wheel_speed_rad_s"]) < max_speeds),
        "attitude_hold": np.max(d["attitude_error_rad"]) < np.deg2rad(.5),
        "both_gnss_available": np.all(d["navigation_valid"]),
        "independent_receiver_noise": not np.allclose(raw_error[:, 0], raw_error[:, 1]),
        "both_filters_reduce_position_noise": np.all(filtered_rmse < raw_rmse),
        "handoff_estimate_matches": np.allclose(handoff["gnss_estimates_eci"], d["estimate"][-1], rtol=0, atol=1e-8),
        "handoff_wheels_match": np.allclose([w["speed_rad_s"] for w in handoff["reaction_wheels"]], d["wheel_speed_rad_s"][-1], rtol=0, atol=1e-8),
        "report_final_distance_matches": abs(report["final_center_distance_m"] - distance[-1]) < 1e-6,
        "producer_report_passed": report["basic_nominal_passed"] is True and all(report["checks"].values()) and handoff["accepted"] is True,
    }
    if integrated:
        force_parts = d["environment_force_n"]
        offsets = np.random.default_rng(np.random.SeedSequence([options.seed, 1])).normal(size=(2, 6)) * np.array(
            [options.initial_position_sigma_m] * 3 + [options.initial_velocity_sigma_m_s] * 3)
        initial = np.array([np.r_[*reference(0.)[:2]] for reference in (profile.target, profile.chaser)]) + offsets
        illumination, density = d["illumination"], d["density_kg_m3"]
        checks.update({
            "shared_local_backend": report["local_dynamics"]["dynamics_backend"] == "local",
            "no_runtime_state_overwrite": report["local_dynamics"]["external_state_writes"] == 0,
            "native_bsk_gravity": report["local_dynamics"]["orbital_environment"].get("mode") == "bsk",
            "gravity_at_every_substep": report["local_dynamics"]["orbital_environment"].get("force_stride_steps") == 1,
            "environment_telemetry": force_parts.shape == (count, 2, 9) and np.isfinite(force_parts).all(),
            "drag_active": bool(np.any(np.linalg.norm(force_parts[..., :3], axis=-1) > 0)) if options.drag_enabled and options.density_300km_kg_m3 else bool(np.all(force_parts[..., :3] == 0)),
            "srp_active": bool(np.any(np.linalg.norm(force_parts[..., 3:6], axis=-1) > 0)) if options.srp_enabled else bool(np.all(force_parts[..., 3:6] == 0)),
            "disturbance_active": bool(np.any(np.linalg.norm(force_parts[..., 6:], axis=-1) > 0)) if options.orbit_acceleration_sigma_m_s2 else bool(np.all(force_parts[..., 6:] == 0)),
            "gnss_position_sigma": np.allclose(np.std(raw_error[:, :, :3], axis=0), options.gnss_position_sigma_m, rtol=.08),
            "gnss_velocity_sigma": np.allclose(np.std(raw_error[:, :, 3:], axis=0), options.gnss_velocity_sigma_m_s, rtol=.08),
            "initial_state_matches": np.allclose(truth[0], report["initial_states_eci"], atol=1e-5, rtol=0),
            "seeded_initial_orbit": np.allclose(truth[0], initial, atol=1e-5, rtol=0),
            "seeded_initial_offsets": np.allclose(report["initial_offsets_eci"], offsets, atol=1e-12, rtol=0),
            "density_valid": density.shape == (count, 2) and np.isfinite(density).all() and np.all(density >= 0),
            "illumination_valid": illumination.shape == (count, 2) and np.isfinite(illumination).all() and np.all((illumination >= 0) & (illumination <= 1)),
            "srp_zero_in_full_shadow": np.all(force_parts[..., 3:6][illumination == 0] == 0),
        })
    checks = {name: bool(value) for name, value in checks.items()}
    return {"passed": all(checks.values()), "checks": checks, "samples": count,
            "scenario": profile.scenario, "final_range_m": float(distance[-1]),
            "minimum_range_m": float(distance.min()), "max_tracking_error_m": float(tracking.max()),
            "initial_chaser_altitude_m": float(altitude[0]), "final_chaser_altitude_m": float(altitude[-1]),
            "final_target_altitude_m": float(np.linalg.norm(target[-1, :3]) - EARTH_RADIUS),
            "max_tracking_after_acquisition_m": float(tracking[t >= 600].max()) if np.any(t >= 600) else None,
            "hold_max_speed_m_s": float(speed[hold].max()) if np.any(hold) else None,
            "raw_position_rmse_m": raw_rmse.tolist(), "filtered_position_rmse_m": filtered_rmse.tolist(),
            "max_wheel_torque_nm": np.max(np.abs(d["wheel_torque_nm"]), axis=0).tolist(),
            "max_wheel_speed_rad_s": np.max(np.abs(d["wheel_speed_rad_s"]), axis=0).tolist(),
            "scope": "Local numerical criteria at 2 Hz; thresholds documented in TEAM_RENDEZVOUS.md, not flight-hardware acceptance."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    try:
        result = verify(args.folder)
    except (ValueError, KeyError, OSError, IndexError) as error:
        result = {"passed": False, "error": str(error)}
    (args.folder / "independent_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result))
    return 0 if result["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
