"""Physical-state/FK trajectory audit, separate from controller release decisions."""
import xml.etree.ElementTree as ET
import json

import numpy as np

from .center_kinematics import CenterKinematics
from .model import JOINTS
from .parameters import PARAMETERS as P


def audit_vbar(data, source_model, plan, *, link_age_limit=.5):
    """Reconstruct all COMs from source mass, physical origins and encoders.

    Controller timestamps only select the claimed release windows. Their
    physical and observation conditions are independently recomputed at 20 Hz.
    """
    root = ET.parse(source_model).getroot()
    fk = [CenterKinematics(root.find(f'./worldbody/body[@name="{name}"]'))
          for name in ('capture_target', 'cubesat_bus')]
    truth = np.asarray(data['truth'])
    rotations = np.asarray(data['truth_rotations'])
    angular = np.asarray(data['truth_angular_rates'])
    joints, rates = np.asarray(data['joints']), np.asarray(data['joint_rates'])
    times = np.asarray(data['elapsed_s'])
    guidance = [json.loads(s) for s in data['near_guidance_json']]
    observations = [json.loads(s) for s in data['vision_updates_json']]
    rho, velocities, pointing, rotation_rates, platform_errors, transverse_sigma, optical_depths = [], [], [], [], [], [], []
    for i in range(len(times)):
        target = truth[i, 0]
        radial = target[:3]/np.linalg.norm(target[:3])
        normal = np.cross(target[:3], target[3:])
        normal /= np.linalg.norm(normal)
        Q = np.column_stack((np.cross(normal, radial), normal, radial))
        omega = np.cross(target[:3], target[3:])/(target[:3]@target[:3])
        centers = [fk[0].evaluate({'outer_panel_hinge': float(data['target_hinge_angle_rad'][i])},
                                 {'outer_panel_hinge': float(data['target_hinge_rate_rad_s'][i])}),
                   fk[1].evaluate(dict(zip(JOINTS, joints[i])), dict(zip(JOINTS, rates[i])))]
        p, v = [], []
        angles, rate_errors = [], []
        for k in range(2):
            R = rotations[i, k]
            lever, derivative = centers[k]
            p.append(truth[i, k, :3]+R@lever)
            v.append(truth[i, k, 3:]+R@(np.cross(angular[i, k], lever)+derivative))
            angles.append(np.arccos(np.clip((np.trace(Q@R.T)-1)/2, -1., 1.)))
            rate_errors.append(np.linalg.norm(angular[i, k]-R.T@omega))
        delta = p[0]-p[1]
        rho.append(Q.T@delta)
        velocities.append(Q.T@(v[0]-v[1]-np.cross(omega, delta)))
        pointing.append(max(angles))
        rotation_rates.append(max(rate_errors))
        hinge = float(data['target_hinge_angle_rad'][i])
        c, s = np.cos(hinge), np.sin(hinge)
        panel = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        grasp_local = np.array([-.0145, -.07702, .04707870013])+panel@[-.36, .0046, 0.]
        grasp = truth[i, 0, :3]-truth[i, 1, :3]+rotations[i, 0]@grasp_local
        optical_depths.append((rotations[i, 1]@np.asarray(plan['grasp_axis_bus']))@grasp+.10)
        platform_errors.append(np.linalg.norm(grasp-rotations[i, 1]@np.array(plan['nominal_grasp_bus_m'])))
        rotated_cov = Q.T@np.asarray(data['covariance'])[i, :3, :3]@Q
        transverse_sigma.append(np.sqrt(max(0., np.linalg.eigvalsh(rotated_cov[1:, 1:]).max())))
    rho, velocities = np.array(rho), np.array(velocities)
    physical_speed = np.linalg.norm(velocities, axis=1)
    sigma = np.sqrt(np.maximum(0., np.linalg.eigvalsh(np.asarray(data['covariance'])[:, :3, :3]).max(axis=1)))
    common = ((np.array(pointing) < np.deg2rad(1)) & (np.array(rotation_rates) < .001)
              & (times-np.asarray(data['last_gnss_stamp_s']) <= link_age_limit+1e-9))
    streak, last_stamp, last_receive, last_geometry = 0, None, None, {}
    readiness = []
    for i, updates in enumerate(observations):
        for sample in updates:
            stamp, geometry = sample['stamp'], sample['geometry']
            good = (sample['accepted'] and sample['nis'] <= 20. and 0 <= times[i]-stamp <= P.visual_age_s+1e-9
                    and sample['sigma_m'] < P.visual_sigma_m and geometry['valid'] and geometry['common_fov']
                    and geometry['marker_px'] >= P.marker_px and geometry['depth_sigma_m'] < P.depth_sigma_m)
            different = last_stamp is None or stamp > last_stamp+1e-9
            if not good or not different:
                streak = 0
            else:
                streak = streak+1 if last_stamp is None or stamp-last_stamp <= P.max_update_gap_s+1e-9 else 1
            if different:
                last_stamp, last_geometry = stamp, geometry if sample['accepted'] else {}
                last_receive = times[i]
        current = (last_stamp is not None and 0 <= times[i]-last_stamp <= P.visual_age_s+1e-9
                   and last_receive is not None and 0 <= times[i]-last_receive <= P.max_update_gap_s+1e-9
                   and sigma[i] < P.visual_sigma_m and last_geometry.get('valid', False)
                   and last_geometry.get('common_fov', False) and last_geometry.get('marker_px', 0) >= P.marker_px
                   and last_geometry.get('depth_sigma_m', np.inf) < P.depth_sigma_m)
        if not current:
            streak = 0
        readiness.append(bool(current and streak >= P.visual_samples))
    readiness = np.array(readiness)
    checks = {'physical_com_finite': bool(np.isfinite(rho).all() and np.isfinite(velocities).all()),
              'physical_com_matches_dynamics': bool(np.allclose(np.linalg.norm(rho, axis=1), data['center_distance_m'], rtol=0, atol=1e-8)),
              'physical_optical_depth_reconstructed': bool(np.allclose(optical_depths, data['truth_optical_depth_m'], rtol=0, atol=1e-8)),
              'position_sigma_reconstructed': bool(np.allclose(sigma, data['position_sigma_m'], rtol=0, atol=1e-10)),
              'readiness_recomputed': bool(np.array_equal(readiness, data['visual_ready']))}
    windows = {}
    last = guidance[-1]
    for name, tolerance in [('Vbar50m', .15), ('Optical17p4m', .10), ('VisualReady5p5m', .05), ('PlatformStandoff', .010)]:
        completed = last.get('nodes', {}).get(name, {}).get('completed_at_s')
        j = int(round(completed*20)) if completed is not None else -1
        indices = np.arange(j-100, j+1) if j >= 100 else np.array([], dtype=int)
        qualified = False
        if len(indices) == 101 and indices[-1] < len(times):
            # Optical node COM intersections are reconstructed from physical
            # depth and along-track COM separation, not controller references.
            if name == 'Vbar50m':
                errors = np.linalg.norm(rho[indices]-[50., 0., 0.], axis=1)
            elif name in ('Optical17p4m', 'VisualReady5p5m'):
                depth = 17.4 if name == 'Optical17p4m' else 5.5
                errors = np.linalg.norm(np.column_stack((np.asarray(optical_depths)[indices]-depth,
                                                         rho[indices, 1:])), axis=1)
            else:
                errors = np.maximum(np.abs(np.asarray(data['truth_tcp_gap_m'])[indices]-.5), np.array(platform_errors)[indices])
            qualified = bool(times[indices[-1]]-times[indices[0]] >= 5.-1e-8
                             and np.all(errors < tolerance) and np.all(physical_speed[indices] < .003)
                             and common[indices].all() and np.all(sigma[indices] < (.04 if name in ('VisualReady5p5m', 'PlatformStandoff') else .05))
                             and np.all(np.abs(joints[indices, :6]) < .005)
                             and np.all(np.abs(np.asarray(data['target_hinge_angle_rad'])[indices]) < .005)
                             and np.all(np.abs(np.asarray(data['target_hinge_rate_rad_s'])[indices]) < .001)
                             and (name not in ('VisualReady5p5m', 'PlatformStandoff') or readiness[indices].all()))
            windows[name] = {'release_s': completed, 'max_physical_error_m': float(errors.max()),
                             'max_physical_speed_m_s': float(physical_speed[indices].max()), 'samples': 101}
        checks[name+'_physical_joint_dwell'] = qualified
    acquire_completed = last.get('acquire', {}).get('completed_at_s')
    release = last.get('nodes', {}).get('VisualReady5p5m', {}).get('completed_at_s')
    if acquire_completed is not None and release is not None:
        mask = (times >= acquire_completed+.05) & (times <= release)
        width = np.minimum(1., np.maximum(.10, .01*rho[:, 0]))
        checks['strict_com_vbar_before_visual_release'] = bool(mask.any() and np.all(
            np.linalg.norm(rho[mask, 1:], axis=1)+3*np.array(transverse_sigma)[mask] < width[mask]))
        checks['no_reference_bias_before_visual_release'] = all(
            np.linalg.norm(g['reference_m'][1:]) < 1e-10 and g['offset_progress'] == 0.
            for t, g in zip(times, guidance) if acquire_completed+.05 <= t <= release)
    else:
        checks['strict_com_vbar_before_visual_release'] = False
        checks['no_reference_bias_before_visual_release'] = False
    ref_v = np.array([g['reference_velocity_m_s'] for g in guidance])
    ref_a = np.array([g['reference_acceleration_m_s2'] for g in guidance])
    normal = np.array([g['subsegment'] not in ('SafeHold', 'SafeHoldBrake', 'NearAcquire') for g in guidance])
    checks['reference_acceleration_bounded'] = bool(np.all(np.linalg.norm(ref_a[normal], axis=1) <= .002+1e-8))
    checks['reference_jerk_bounded'] = bool(np.all(np.linalg.norm(np.diff(ref_a, axis=0)[normal[1:] & normal[:-1]], axis=1)/.05 <= .002+1e-6))
    checks['reference_speed_bounded'] = bool(np.all(np.linalg.norm(ref_v, axis=1) <= .100001))
    return {'passed': all(checks.values()), 'checks': checks, 'nodes': windows,
            'scope': 'physical states, original model mass/FK and raw accepted observation time series; logical capture only'}
