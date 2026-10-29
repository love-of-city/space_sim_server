"""Receiving clocks get no new dwell credit; completed work survives intact."""
from copy import deepcopy
import json

import pytest

from tools.check_capture26_continuity import check


def documents():
    physical = dict(time_ns=10_000_000_000, qpos_inertial=[1., 0., 0., 0.],
        qvel_inertial=[0., 0., 0.], origin_position_m=[7.e6, 0., 0.], origin_velocity_m_s=[0., 7500., 0.])
    source = dict(phase='FarReady', acceptance_profile='simulation_state_demo_v1', physical=physical,
        navigation={key: value for key, value in [('state', dict(time_ns=physical['time_ns'])),
            ('base', {}), ('events', []), ('snapshots', []), ('accepted', {}),
            ('base_committed', False), ('last_visual_epoch', None)]},
        gyros=[], gnss_receiver={}, gnss_pending=[], retention=None, environment=[],
        control=dict(operating_reference=[0.]*8, capture_reference=[0.]*12,
                     gyro_pending=[], last_cooperative_ns=physical['time_ns'], platform_stability=None,
                     preparation=None))
    return source, deepcopy(source)


def audit(tmp_path, source, receiving):
    incoming = tmp_path/'handoff.json'
    incoming.write_text(json.dumps(source), encoding='utf-8')
    (tmp_path/'restored-state.json').write_text(json.dumps(receiving), encoding='utf-8')
    return check(incoming, tmp_path)


def test_new_near_reference_cannot_inherit_far_dwell_credit(tmp_path):
    source, receiving = documents()
    receiving['control']['platform_stability'] = dict(schema='space-sim-platform-stability/1',
        duration_ns=5_000_000_000, start_ns=None, last_ns=None, reference=None, failures=[])
    assert audit(tmp_path, source, receiving)['continuity_passed']
    receiving['control']['platform_stability'].update(start_ns=0, last_ns=10_000_000_000)
    assert not audit(tmp_path, source, receiving)['continuity_passed']


@pytest.mark.parametrize('change', ['completion', 'physical_state', 'time'])
def test_operation_handoff_rejects_lost_completion_or_physical_discontinuity(tmp_path, change):
    source, receiving = documents()
    source['phase'] = receiving['phase'] = 'NearParked'
    source['control']['parking_completion'] = dict(records=[dict(node='PlatformStandoff', dwell_s=5.)])
    receiving['control']['parking_completion'] = deepcopy(source['control']['parking_completion'])
    assert audit(tmp_path, source, receiving)['continuity_passed']
    if change == 'completion':
        receiving['control']['parking_completion']['records'][0]['dwell_s'] = 0.
    elif change == 'physical_state':
        receiving['physical']['qpos_inertial'][0] += .01
    else:
        receiving['physical']['time_ns'] += 1
    assert not audit(tmp_path, source, receiving)['continuity_passed']
