"""Audit pre-image attitude observability without changing any sensor model.

This is a stationary, ideal-innovation covariance experiment, not a flight.
Fresh 50 Hz gyros and 10 Hz paired COM GNSS exercise the production estimator;
an independent discrete covariance calculation checks its attitude growth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'backend')]
from simulation.gyroscope import GyroConfig  # noqa: E402
from simulation.pose_navigation import PoseNavigation  # noqa: E402


def independent_sigma(time_s, gyro, *, dt=.02):
    n = round(time_s/dt)
    single_variance = (np.deg2rad(.05)**2
        + time_s**2*gyro.initial_bias_std_rad_s**2
        + gyro.white_noise_std_rad_s**2*.02*time_s
        + gyro.bias_random_walk_rad_s_sqrt_s**2*dt**3*(n-1)*n*(2*n-1)/6)
    return float(np.sqrt(2*single_variance))


def audit(duration_s=600.):
    gyro = GyroConfig()
    prior = np.diag(([.002**2]*6+[np.deg2rad(.05)**2]*3
                     +[gyro.initial_bias_std_rad_s**2]*3)*2)
    positions, velocities = np.array([[0., 110., 0.], [0., 0., 0.]]), np.zeros((2, 3))
    nav = PoseNavigation(positions, velocities, [np.eye(3)]*2, prior,
        gyro_white_sigma=gyro.white_noise_std_rad_s,
        gyro_bias_walk=gyro.bias_random_walk_rad_s_sqrt_s)
    measurement = np.hstack([positions, velocities])
    samples, first_failure, max_difference = [], None, 0.
    gnss_attitude_change, accepted_gnss = 0., 0
    for tick in range(round(duration_s/.02)+1):
        now = tick*20_000_000
        nav.advance(now)
        for body in (0, 1):
            assert nav.gyro(body, dict(sample_time_ns=now,
                angular_velocity_body_rad_s=[0., 0., 0.], valid=True, saturated_axes=[False]*3))
        assert all(nav.gyro_health(now))
        if tick % 5 == 0:
            before = nav.relative_covariances()[1].copy()
            assert nav.gnss(measurement, np.eye(12)*.002**2, now, tick//5)
            accepted_gnss += 1
            gnss_attitude_change = max(gnss_attitude_change,
                float(np.max(np.abs(before-nav.relative_covariances()[1]))))
        sigma = float(np.sqrt(np.linalg.eigvalsh(nav.relative_covariances()[1])[-1]))
        expected = independent_sigma(now*1e-9, gyro)
        max_difference = max(max_difference, abs(sigma-expected))
        if first_failure is None and sigma >= np.deg2rad(.3):
            first_failure = now*1e-9
        if tick % 500 == 0 or tick == round(duration_s/.02):
            samples.append(dict(time_s=now*1e-9, relative_attitude_sigma_deg=float(np.rad2deg(sigma)),
                independent_sigma_deg=float(np.rad2deg(expected)), gyros_healthy=True))
    if max_difference > 1e-10 or gnss_attitude_change > 1e-15:
        raise ValueError('independent covariance audit disagrees with production estimator')
    return dict(schema='space-sim-capture26-attitude-observability-audit/1',
        integration_complete=False, scope='stationary covariance diagnostic; not a continuous flight',
        attitude_accuracy_required_for_release=False,
        threshold_note='0.3 deg is a historical diagnostic; capture26 has no attitude accuracy gate',
        no_visual_observations=True, gyro_hz=50, gnss_hz=10,
        gyro_configuration=vars(gyro), numeric_duration_s=duration_s,
        accepted_gnss_updates=accepted_gnss, max_gnss_attitude_covariance_change=gnss_attitude_change,
        max_independent_sigma_difference_rad=max_difference,
        first_relative_sigma_at_least_0p3deg_s=first_failure,
        projected_without_image=[dict(time_s=t, relative_attitude_sigma_deg=float(
            np.rad2deg(independent_sigma(t, gyro)))) for t in (5000., 5120., 11120.)],
        source_sha256={name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in
            ('simulation/pose_navigation.py', 'simulation/gyroscope.py', 'simulation/capture26_runtime.py')},
        samples=samples)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--duration', type=float, default=600.)
    args = p.parse_args()
    if not np.isfinite(args.duration) or not 0 < args.duration <= 12000:
        p.error('duration must be in (0, 12000]')
    result = audit(args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k != 'samples'}, indent=2))


if __name__ == '__main__':
    main()
