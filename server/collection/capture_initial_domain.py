"""Freeze calibrated sampling bounds; production poses are not replayed twice."""
import hashlib
import json
from pathlib import Path
import numpy as np

from simulation.scripted_tasks.capture_initialization import CaptureInitialState
from simulation.scripted_tasks.capture_grasp import TARGET_FROM_INTERFACE
from simulation.scripted_tasks.capture_platform import platform_contract

BOUND_KEYS = ('gap_range_m', 'bearing_cone_deg', 'tilt_cone_deg', 'roll_range_deg')


def sampling_bounds(config):
    return {key:config[key] for key in (*BOUND_KEYS,'arm_joint_offset_range_deg','arm_joint_baseline_rad')
            if key in config}


def contains_initial(state, bounds):
    initial = CaptureInitialState(**state)
    if 'arm_joint_offset_range_deg' in bounds:
        if initial.arm_joint_position_rad is None: return False
        offset = np.rad2deg(np.asarray(initial.arm_joint_position_rad)-bounds['arm_joint_baseline_rad'])
        limits = np.asarray(bounds['arm_joint_offset_range_deg'])
        if np.any(offset < limits[:,0]-1e-8) or np.any(offset > limits[:,1]+1e-8): return False
    elif initial.arm_joint_position_rad is not None:
        return False
    p, r = initial.geometry(np.zeros(3), np.eye(3))
    del p
    direction = np.r_[np.tan(np.deg2rad([initial.bearing_x_deg, initial.bearing_y_deg])), 1.]
    bearing = np.rad2deg(np.arccos(1./np.linalg.norm(direction)))
    tilt = np.rad2deg(np.arccos(np.clip((r@TARGET_FROM_INTERFACE)[2, 2], -1., 1.)))
    return (bounds['gap_range_m'][0] <= initial.gap_m <= bounds['gap_range_m'][1]
            and bearing <= bounds['bearing_cone_deg']+1e-8
            and tilt <= bounds['tilt_cone_deg']+1e-8
            and bounds['roll_range_deg'][0] <= initial.roll_deg <= bounds['roll_range_deg'][1])


def verify_calibration(document):
    evidence = Path(document['calibration_report'])
    if hashlib.sha256(evidence.read_bytes()).hexdigest() != document['calibration_sha256']:
        raise ValueError('capture domain calibration evidence changed')
    report = json.loads(evidence.read_text())
    if (report.get('schema') != 'capture-domain-calibration/1'
            or report.get('capture_platform_contract') != platform_contract()
            or report.get('approved_bounds') != sampling_bounds(document['sampling_config'])):
        raise ValueError('capture domain differs from its calibrated platform/bounds')
    if not report.get('passed_trials'):
        raise ValueError('capture domain needs real physical calibration trials')
    if report.get('model_identity'):
        model=report['model_identity']
        if hashlib.sha256(Path(model['path']).read_bytes()).hexdigest()!=model['sha256']:
            raise ValueError('capture calibration model changed')
    if 'arm_joint_offset_range_deg' in document['sampling_config']:
        scan=report.get('workspace_scan')
        if not scan:
            raise ValueError('random arm postures require a workspace scan')
        path=Path(scan['path'])
        if hashlib.sha256(path.read_bytes()).hexdigest()!=scan['sha256']:
            raise ValueError('workspace scan evidence changed')
        data=json.loads(path.read_text())
        if data.get('status')!='complete' or data.get('capture_platform_contract')!=platform_contract():
            raise ValueError('workspace scan belongs to an incomplete or different platform')
    for trial in report['passed_trials']:
        path = Path(trial['physical_report'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != trial['report_sha256']:
            raise ValueError('calibration physical report changed')
        data = json.loads(path.read_text())
        if report.get('model_identity') and data.get('source_model_sha256') != report['model_identity']['sha256']:
            raise ValueError('capture calibration mixes different source models')
        if (trial['process_returncode'] != 0 or data.get('result') != 'success'
                or not data.get('preflight', {}).get('accepted')
                or data.get('capture_platform_contract') != platform_contract()):
            raise ValueError('capture domain requires successful trials under this platform')
    return report


def make_range_pool(domain_path, output, *, model, catalog):
    domain_path, output = Path(domain_path), Path(output)
    document = json.loads(domain_path.read_text())
    if document.get('schema') != 'capture-initial-domain/1':
        raise ValueError('unsupported capture initial domain')
    calibration=verify_calibration(document)
    if calibration.get('model_identity') and hashlib.sha256(Path(model).read_bytes()).hexdigest()!=calibration['model_identity']['sha256']:
        raise ValueError('capture pool model differs from the calibrated source')
    from tools.prepare_capture_initial_pool import sample_candidates
    states = sample_candidates(document['sampling_config'])
    if not all(contains_initial(state, document['sampling_config']) for state in states):
        raise ValueError('sample outside frozen domain')
    result = dict(schema='capture-initial-range-pool/1', model=str(Path(model).resolve()),
        catalog=str(Path(catalog).resolve()), domain=document,
        capture_platform_contract=platform_contract(),
        layouts=[dict(id=f'random-capture-{index:05d}', initial_state=state) for index,state in enumerate(states)],
        scope='random candidates within calibrated bounds; each is physically run once while recording; '
              'geometric rejects and physical failures are excluded from accepted training data')
    output.write_text(json.dumps(result, indent=2))
    return result
