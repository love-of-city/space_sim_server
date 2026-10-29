"""Inspect current GNSS noise versus distance and actual no-vision policy.

Sensor samples and direct VbarApproach calls only. No authoritative dynamics,
flight trajectory, new camera geometry, filtering performance or flight safety
verification is claimed. Never changes navigation/vision thresholds.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

SERVER = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(SERVER),str(SERVER/'backend')]
from space_arm_platform.gnss_config import GNSSConfig
from simulation.differential_gnss import DifferentialGNSS
from simulation.near_field.parameters import PARAMETERS as P
from simulation.near_field.vbar_approach import VbarApproach


def sensor_check(distance, samples):
    sensor = DifferentialGNSS(GNSSConfig(),seed=18)
    # Fixed body origins in J2000 axes; truth is used by the simulated sensor
    # and scorer only. Body-origin separation is NOT optical checkpoint depth.
    truth = np.zeros((2,6))
    truth[0,0] = distance
    errors = []
    covariance = None
    for i in range(samples):
        packet = sensor.measure(truth,i/sensor.config.sample_rate_hz)
        errors.append(packet['relative'][:3]-[distance,0,0])
        covariance = packet['relative_covariance'][:3,:3]
    error = np.asarray(errors)
    norm = np.linalg.norm(error,axis=1)*1000
    return dict(body_origin_separation_m=distance,samples=samples,
        theoretical_per_axis_sigma_mm=(np.sqrt(np.diag(covariance))*1000).tolist(),
        theoretical_position_3d_rms_mm=float(np.sqrt(np.trace(covariance))*1000),
        sampled_position_3d_rms_mm=float(np.sqrt(np.mean(norm**2))),
        sampled_position_error_p95_mm=float(np.percentile(norm,95)),
        sampled_position_error_max_mm=float(norm.max()))


def controller_case(*, depth, speed, visual_valid, visual_ready, at_checkpoint=False,
                    start_at_node=False, steps=1):
    """Independent branch probes, not a simulated flight from one to another.

    Synthetic coordinates identify along-track with optical depth solely for
    exercising the existing guard, with no camera/COM installation offsets.
    The caller's position/velocity covariance and current estimate are fixtures.
    """
    state = np.array([depth,0.,0.,-speed,0.,0.])
    covariance = np.diag([.03**2]*3+[.0015**2]*3)
    control = VbarApproach()
    control.initial = state.copy()
    control.acquire.completed_at_s = 0.
    control.index = 1 if at_checkpoint else 2
    control.targets = [np.array([d,0.,0.]) for d in (50.,17.4,5.5,1.)]
    control.next_start = depth if at_checkpoint or start_at_node else 17.4
    control.offset_end_s = 2.
    for tick in range(steps):
        control.update(tick*.05,state,covariance,common=True,sigma=.03,
            visual_valid=visual_valid,visual_ready=visual_ready,optical_depth=depth,
            optical_s=lambda d:d,hold=np.array([1.,0.,0.]),offset_end_s=2.,
            geometry_ready=True,available_acceleration=.005,observation_age=.1)
    telemetry = control.telemetry
    return dict(optical_depth_m=depth,approach_speed_m_s=speed,
        visual_valid=visual_valid,visual_ready=visual_ready,
        input_position_sigma_m=.03,input_velocity_sigma_m_s=.0015,observation_age_s=.1,
        node=telemetry['node_id'],subsegment=telemetry['subsegment'],
        active_boundary_m=telemetry['optical_boundary_m'],
        stopping_distance_and_uncertainty_m=telemetry['stopping_distance_m'],
        braking_margin_m=telemetry['braking_margin_m'],failure_reason=telemetry['failure_reason'],
        optical17_completed=control.gates['Optical17p4m'].completed_at_s is not None,
        visual5p5_completed=control.gates['VisualReady5p5m'].completed_at_s is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=SERVER/'run/gnss_proximity_inspection')
    parser.add_argument('--samples',type=int,default=10000)
    args = parser.parse_args()
    if args.samples<100:
        parser.error('at least 100 sensor samples required')
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=True)
    ranges = [sensor_check(d,args.samples) for d in (110.,50.,17.4,12.,5.5,3.36,.5,.05)]
    cases = dict(checkpoint17_without_vision=controller_case(depth=17.4,speed=0.,
        visual_valid=False,visual_ready=False,at_checkpoint=True,steps=101))
    for depth in (17.4,13.,12.3,12.2,12.05):
        cases[f'no_vision_{depth:g}m'] = controller_case(depth=depth,speed=.02,
            visual_valid=False,visual_ready=False)
    cases['valid_but_not_ready_5p5m'] = controller_case(depth=5.5,speed=0.,
        visual_valid=True,visual_ready=False,start_at_node=True,steps=102)
    cases['ready_5p5m'] = controller_case(depth=5.5,speed=0.,
        visual_valid=True,visual_ready=True,start_at_node=True,steps=102)
    example = cases['no_vision_12.2m']
    checks = dict(checkpoint_does_not_require_vision=cases['checkpoint17_without_vision']['optical17_completed'],
        no_vision_boundary_is_12m=example['active_boundary_m']==12.,
        brakes_before_boundary=example['subsegment']=='OpticalBoundaryHold' and example['failure_reason'] is None,
        boundary_breach_fault=cases['no_vision_12.05m']['failure_reason']=='optical_braking_margin',
        visual5p5_not_released_without_readiness=not cases['valid_but_not_ready_5p5m']['visual5p5_completed'],
        valid_vision_but_not_ready_boundary_is_5m=cases['valid_but_not_ready_5p5m']['active_boundary_m']==5.,
        ready_visual_removes_optical_wait_boundary=cases['ready_5p5m']['active_boundary_m'] is None,
        visual5p5_released_only_in_qualified_control_case=cases['ready_5p5m']['visual5p5_completed'])
    sources = [SERVER/'backend/space_arm_platform/gnss_config.py',
        SERVER/'simulation/differential_gnss.py',SERVER/'simulation/near_field/parameters.py',
        SERVER/'simulation/near_field/vbar_approach.py']
    summary = dict(scope=__doc__,main_commit=subprocess.check_output(
        ['git','rev-parse','origin/main'],cwd=SERVER,text=True).strip(),
        source_sha256={str(p.relative_to(SERVER)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources},
        gnss_config=GNSSConfig().to_dict(),safety_parameters=P.metadata(),sensor_ranges=ranges,
        controller_probes=cases,checks=checks,passed=all(checks.values()),
        illustrative_braking_trigger_depth_m=12.+example['stopping_distance_and_uncertainty_m'],
        filter_accuracy_validated=False,full_flight_validated=False,
        interpretation='noise model has no minimum separation cutoff; 12 m is an optical safety-policy boundary, not a GNSS hardware limit')
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2))
    return 0 if summary['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
