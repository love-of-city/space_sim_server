import copy

import pytest
from pydantic import ValidationError

from simulation.gyroscope import Gyroscope
from space_arm_platform.models import GyroObservation, SimulationObservation


@pytest.mark.parametrize('change', ['nan', 'variance', 'axes', 'frame', 'timestamp', 'truth', 'valid'])
def test_invalid_sensor_packets_are_rejected(change):
    row = Gyroscope().sample(0, [0.] * 3)
    if change == 'nan':
        row['angular_velocity_body_rad_s'][0] = float('nan')
    elif change == 'variance':
        row['error_variance_rad2_s2'][1] = -.1
    elif change == 'axes':
        row['saturated_axes'] = [False]
    elif change == 'frame':
        row['frame'] = 'camera'
    elif change == 'timestamp':
        row['sample_time_ns'] = '1'
    elif change == 'truth':
        row['ground_truth_rate'] = [0.] * 3
    else:
        row['valid'] = False
    with pytest.raises(ValidationError):
        GyroObservation.model_validate(row)


def test_optional_packet_roundtrip_age_and_no_truth_substitution():
    payload = dict(protocol='space-arm-control/1', type='observation', simulation_id='gyro',
                   step_id='1', render_frame_id='1', sim_time_ns='20000000', wall_time_ns='0',
                   applied_action_sequence='0', joint_position_rad=[0.]*6,
                   joint_velocity_rad_s=[0.]*6, target_joint_position_rad=[0.]*6)
    assert SimulationObservation.model_validate(payload).service_gyro is None
    row = Gyroscope().sample(0, [0., .01, -.01])
    payload['service_gyro'] = row
    assert SimulationObservation.model_validate(payload).service_gyro.model_dump() == row
    future = copy.deepcopy(payload)
    future['service_gyro']['sample_time_ns'] = '40000000'
    with pytest.raises(ValidationError, match='newer'):
        SimulationObservation.model_validate(future)
