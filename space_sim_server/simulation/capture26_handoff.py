"""Strict versioned capture26 continuation, separate from legacy handoffs."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from simulation.capture26_mission import MODEL_ID, MODEL_PATH
from space_arm_platform.visual_navigation import covariance

SCHEMA = 'space-sim-capture26-handoff/1'
COMMON_CHECKS = {'completed_stage', 'native_state_not_overwritten', 'no_spacecraft_contact',
                 'gyros_healthy', 'gnss_healthy', 'common_sensor_epoch'}
# Older /1 documents put these diagnostics in checks. Keep reading them, but
# functional admission no longer depends on real-time processing capacity.
LEGACY_PERFORMANCE_CHECKS = {'processing_capacity_1x', 'time_backlog_not_growing', 'realtime_paced'}


def validate(document, root, *, accepted=True):
    d = deepcopy(document)
    if (d.get('schema') != SCHEMA or d.get('model_id') != MODEL_ID
            or d.get('source_sha256') != hashlib.sha256((Path(root)/MODEL_PATH).read_bytes()).hexdigest()
            or accepted and d.get('accepted') is not True):
        raise ValueError('capture26 handoff model, schema or acceptance mismatch')
    if d.get('phase') not in ('FarReady', 'NearParked'):
        raise ValueError('capture26 handoff requires a completed parking phase')
    epoch = d.get('time_ns')
    if type(epoch) is not int or epoch < 0 or epoch % 100_000_000:
        raise ValueError('capture26 handoff must preserve a common physical/sensor epoch')
    for key in ('physical', 'navigation', 'gnss_receiver', 'gnss_pending', 'gyros',
                'retention', 'control', 'calibration_id', 'mission_id', 'environment'):
        if key not in d:
            raise ValueError('missing capture26 handoff extension: '+key)
    physical = d['physical']
    if (physical.get('time_ns') != epoch or d['navigation']['state']['time_ns'] != epoch
            or physical.get('model_xml_sha256') != d['source_sha256']):
        raise ValueError('capture26 handoff epoch mismatch')
    q, v = np.array(physical['qpos_inertial']), np.array(physical['qvel_inertial'])
    if (q.shape != (physical['nq'],) or v.shape != (physical['nv'],)
            or not np.isfinite(np.r_[q, v]).all()):
        raise ValueError('invalid capture26 physical coordinates')
    for qa in physical['free_qpos']:
        if not np.isclose(np.linalg.norm(q[qa+3:qa+7]), 1., atol=1e-8, rtol=0):
            raise ValueError('invalid capture26 physical attitude')
    covariance(d['navigation']['state']['covariance'], 24)
    if len(d['gyros']) != 2:
        raise ValueError('two gyro internal states required')
    for state, body, name in zip(d['gyros'], ('cubesat_bus', 'capture_target'), ('service_gyro', 'target_gyro')):
        if (state.get('body_name') != body or state.get('sensor_name') != name
                or state.get('last') is None or int(state['last']['sample_time_ns']) != epoch):
            raise ValueError('capture26 gyro identity or epoch mismatch')
        from simulation.gyroscope import Gyroscope, GyroConfig
        Gyroscope(GyroConfig.from_mapping(state['config']), seed=state['seed'],
                  body_name=body, sensor_name=name).restore(state)
    last = d['gnss_receiver'].get('last_time')
    if last is None or abs(last-epoch*1e-9) > 1e-7:
        raise ValueError('capture26 GNSS sampling history mismatch')
    for packet in d['gnss_pending']:
        if not packet['stamp'] <= epoch*1e-9 < packet['due']:
            raise ValueError('capture26 GNSS in-flight epoch mismatch')
        covariance(packet['covariance'], 12)
        if np.shape(packet['observations']) != (2, 6) or not np.isfinite(packet['observations']).all():
            raise ValueError('invalid capture26 GNSS in-flight observation')
    for key, count in (('operating_reference', 8), ('capture_reference', 12), ('force_n', 3)):
        value = np.asarray(d['control'].get(key), float)
        if value.shape != (count,) or not np.isfinite(value).all():
            raise ValueError('capture26 control reference mismatch')
    from simulation.capture26_mission import load_vision
    from space_arm_platform.visual_navigation import calibration_id
    if not d['mission_id'] or d['calibration_id'] != calibration_id(load_vision(root)):
        raise ValueError('capture26 mission and calibration identity required')
    required = COMMON_CHECKS | ({'actual_120s_hold', 'actual_tracking_2m', 'actual_velocity_2cm_s'}
        if d['phase'] == 'FarReady' else {'current_visual_ready', 'vision_nodes_prevalidated',
                                        'five_second_dwell', 'prepared_arm_geometry_certified'})
    if d.get('accepted') and (not required <= set(d.get('checks', {}))
                             or not all(v is True for k, v in d['checks'].items()
                                        if k not in LEGACY_PERFORMANCE_CHECKS)):
        raise ValueError('capture26 accepted handoff has failed independent checks')
    return d


def load(path, root):
    return validate(json.loads(Path(path).read_text('utf-8')), root)
