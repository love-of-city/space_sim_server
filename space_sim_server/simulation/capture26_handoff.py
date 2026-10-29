"""Strict versioned capture26 continuation, separate from legacy handoffs."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from simulation.capture26_mission import MODEL_ID, MODEL_PATH
from space_arm_platform.visual_navigation import covariance

SCHEMA = 'space-sim-capture26-handoff/2'
DEMO_SCHEMA = 'space-sim-capture26-handoff/3'
LEGACY_SCHEMA = 'space-sim-capture26-handoff/1'
COMMON_CHECKS = {'completed_stage', 'native_state_not_overwritten', 'no_spacecraft_contact',
                 'gyros_healthy', 'gnss_healthy', 'common_sensor_epoch'}
# Older /1 documents put these diagnostics in checks. Keep reading them, but
# functional admission no longer depends on real-time processing capacity.
LEGACY_PERFORMANCE_CHECKS = {'processing_capacity_1x', 'time_backlog_not_growing', 'realtime_paced'}


def validate(document, root, *, accepted=True):
    d = deepcopy(document)
    demo = d.get('schema') == DEMO_SCHEMA
    if (d.get('schema') not in (SCHEMA, LEGACY_SCHEMA, DEMO_SCHEMA) or d.get('model_id') != MODEL_ID
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
    if not demo and (last is None or abs(last-epoch*1e-9) > 1e-7):
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
    if d['schema'] == SCHEMA and d['phase'] == 'FarReady':
        required.add('actual_service_rate_0p001rad_s')
    if demo:
        required = {'completed_stage', 'native_state_not_overwritten', 'no_spacecraft_contact',
                    'common_sensor_epoch', 'authoritative_simulation_feedback'}
        required |= ({'actual_5s_hold', 'actual_tracking_2m', 'actual_velocity_1cm_s', 'continuous_physical_evidence'}
                     if d['phase'] == 'FarReady' else {'all_four_actual_five_second_dwells', 'working_reference_bound'})
    if d.get('accepted') and (not required <= set(d.get('checks', {}))
                             or not all(v is True for k, v in d['checks'].items()
                                        if k not in LEGACY_PERFORMANCE_CHECKS)):
        raise ValueError('capture26 accepted handoff has failed independent checks')
    if d['schema'] in (SCHEMA, DEMO_SCHEMA):
        from simulation.capture26_mission import load_mission
        from simulation.sensor_delivery import FixedStereoDelivery
        configuration = load_mission(root)
        if d.get('mission_configuration') != configuration.to_dict() or not isinstance(d.get('sensor_delivery'), dict):
            raise ValueError('capture26 v2 handoff lacks frozen configuration or scheduled sensors')
        scheduler = FixedStereoDelivery(round(configuration.stereo_delay_s*1e9), configuration.vision_task_timeout_wall_s)
        scheduler.restore(d['sensor_delivery'])
        if configuration.geometry_certificate_sha256 and not demo:
            from simulation.capture26_mission import load_certified_preparation
            preparation = load_certified_preparation(root, configuration)
            preparation.restore(d['control'].get('preparation', {}))
            if not preparation.complete or preparation.failed:
                raise ValueError('capture26 certified preparation is not complete at handoff')
        if any(p['exposure_ns'] > epoch or p['due_ns'] <= epoch for p in scheduler.pending):
            raise ValueError('capture26 stereo in-flight epoch mismatch')
        control = d['control']
        if any(key not in control for key in ('gyro_pending', 'last_cooperative_ns', 'platform_stability', 'last_gnss_ns')):
            raise ValueError('capture26 v2 handoff lacks sensor or stability history')
        for packet in control['gyro_pending']:
            body, sample = packet['body'], packet['observation']
            sample_time = int(sample['sample_time_ns'])
            if type(body) is not int or body not in (0, 1):
                raise ValueError('capture26 scheduled gyro identity mismatch')
            delay = configuration.gyro_delay_s+(configuration.cooperative_delay_s if body == 1 else 0.)
            if (sample.get('body_name') != ('cubesat_bus', 'capture_target')[body]
                    or not sample_time <= epoch < packet['due_ns']
                    or packet['due_ns'] != sample_time+round(delay*1e9)):
                raise ValueError('capture26 gyro in-flight epoch mismatch')
        if any((control[key] is not None or not demo) and (type(control[key]) is not int or not 0 <= control[key] <= epoch)
               for key in ('last_cooperative_ns', 'last_gnss_ns')):
            raise ValueError('capture26 received sensor history mismatch')
    if demo:
        from simulation.capture26_mission import load_mission
        configuration = load_mission(root)
        if (not configuration.is_demo or d.get('acceptance_profile') != configuration.acceptance_profile
                or d.get('feedback_source') != configuration.feedback_source):
            raise ValueError('handoff does not carry current simulation-state demonstration rules')
        parked = d['control'].get('parking_completion', {})
        expected = ['FarHandoff110m'] if d['phase'] == 'FarReady' else [
            'Vbar50m', 'Optical17p4m', 'VisualReady5p5m', 'PlatformStandoff']
        records = parked.get('records', [])
        if (parked.get('acceptance_profile') != configuration.acceptance_profile
                or parked.get('working_reference_sha256') != configuration.operating_geometry_sha256
                or [record.get('node') for record in records] != expected):
            raise ValueError('demonstration parking history or operating reference changed')
        previous = -1
        for record in records:
            start, end = record.get('dwell_start_ns'), record.get('release_time_ns')
            if (type(start) is not int or type(end) is not int or not previous < start <= end <= epoch
                    or end-start < 5_000_000_000 or record.get('dwell_s', 0.) < 5.):
                raise ValueError('demonstration requires actual sequential five-second parking intervals')
            previous = end
        if d['phase'] == 'NearParked':
            from simulation.capture26_approach import Capture26DemoApproach
            from simulation.capture26_demo_geometry import load_geometry
            approach = Capture26DemoApproach(np.zeros(3), np.zeros(3), load_vision(root),
                config=configuration, geometry=load_geometry(root, configuration))
            approach.restore(d['control'].get('approach', {}))
            if (not approach.released or approach.safe_hold or approach.audit != records
                    or approach.stability.snapshot() != d['control']['platform_stability']):
                raise ValueError('demonstration final parking record was not inherited intact')
    elif d['phase'] == 'NearParked':
        from simulation.capture26_mission import load_mission
        from simulation.platform_stability import PlatformStability
        configuration = load_mission(root)
        if (d['schema'] != SCHEMA or not configuration.geometry_certificate_sha256
                or d.get('mission_configuration') != configuration.to_dict()
                or d.get('checks', {}).get('five_second_platform_stability') is not True
                or not d['control'].get('platform_stability')):
            raise ValueError('capture26 NearParked lacks certified version 2 stability/configuration')
        stable = PlatformStability(configuration.dwell_s)
        stable.restore(d['control']['platform_stability'])
        if (stable.last_ns != epoch or not stable.status(epoch)['platform_stable']
                or stable.reference != ['optical_depth_m', configuration.terminal_depth_m]):
            raise ValueError('capture26 platform stability/reference does not match handoff')
    return d


def load(path, root):
    document = validate(json.loads(Path(path).read_text('utf-8')), root)
    from simulation.capture26_mission import load_mission
    configuration = load_mission(root)
    if configuration.is_demo and document['schema'] != DEMO_SCHEMA:
        raise ValueError('old image-navigation results cannot acquire demonstration acceptance')
    return document
