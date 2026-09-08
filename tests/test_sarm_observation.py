"""SARM mixed-unit compatibility, wheel telemetry, real Hub and recording."""
import asyncio
import json
from pathlib import Path

import pytest
from pydantic import ValidationError
from space_arm_platform.models import EpisodeStart, EpisodeStop, SimulationObservation
from space_arm_platform.protocol import read_async, write_async
from space_arm_platform.recorder import EpisodeRecorder
from space_arm_platform.simulation_hub import SimulationHub


def observation():
    return {
        'protocol': 'space-arm-control/1', 'type': 'observation', 'simulation_id': 'sarm-test',
        'step_id': '1', 'render_frame_id': '1', 'sim_time_ns': '10000000',
        'wall_time_ns': '100', 'applied_action_sequence': '0', 'jacobian_rank': 6,
        'joint_position_rad': [0.]*6+[.01,.01], 'joint_velocity_rad_s': [0.]*8,
        'target_joint_position_rad': [0.]*6+[.01,.01],
        'arm_joint_position_rad': [0.]*6, 'arm_joint_velocity_rad_s': [0.]*6,
        'target_arm_joint_position_rad': [0.]*6, 'gripper_position_m': [.01,.01],
        'gripper_velocity_m_s': [0.,0.], 'target_gripper_position_m': [.01,.01],
        'attitude_control': {
            'enabled': True, 'mode': 'initial_inertial_hold', 'reference_frame': 'J2000',
            'reference_initialized': True, 'state_time_ns': '10000000', 'control_time_ns': '10000000',
            'orientation_inertial_wxyz': [1,0,0,0], 'reference_orientation_inertial_wxyz': [1,0,0,0],
            'angular_velocity_body_rad_s': [0,0,0], 'attitude_error_angle_rad': 0.,
            'control_rate_hz': 100., 'saturated': False,
        },
        'reaction_wheels': {
            'names': ['rw_x','rw_y','rw_z'], 'speed_rad_s': [1,2,3],
            'relative_momentum_nms': [.01,.02,.03], 'requested_motor_torque_nm': [0,0,0],
            'applied_motor_torque_nm': [0,0,0], 'max_motor_torque_nm': [.2]*3,
            'max_speed_rad_s': [628.318530717959]*3, 'torque_limited': [False]*3,
            'speed_limited': [False]*3, 'overspeed': [False]*3,
        },
    }


def test_explicit_si_and_wheel_telemetry_survive_model_roundtrip():
    payload = observation()
    value = SimulationObservation.model_validate(payload).model_dump(mode='json')
    for key in payload:
        assert value[key] == payload[key]
    assert len(value['joint_position_rad']) == 8
    assert len(value['reaction_wheels']['speed_rad_s']) == 3


@pytest.mark.parametrize('change', ['seven', 'mismatch', 'units', 'partial', 'nan', 'rank', 'wheels'])
def test_invalid_observations_are_rejected(change):
    payload = observation()
    if change == 'seven':
        payload['joint_position_rad'] = [0.]*7
    elif change == 'mismatch':
        payload['joint_velocity_rad_s'] = [0.]*6
    elif change == 'units':
        payload['gripper_position_m'] = [10.,10.]
    elif change == 'partial':
        del payload['arm_joint_position_rad']
    elif change == 'nan':
        payload['reaction_wheels']['speed_rad_s'][0] = float('nan')
    elif change == 'rank':
        payload['jacobian_rank'] = 7
    else:
        del payload['reaction_wheels']
    with pytest.raises(ValidationError):
        SimulationObservation.model_validate(payload)


def test_schema_and_backend_agree_on_sarm_shapes_and_fields():
    schema = json.loads((Path(__file__).resolve().parents[1] / 'contracts/space-arm-control-v1.schema.json').read_text())
    source = SimulationObservation.model_json_schema()
    properties = schema['$defs']['observation']['allOf'][1]['properties']
    for key in source['properties']:
        for constraint in ('minItems','maxItems','maximum','minimum','anyOf','$ref'):
            assert properties[key].get(constraint) == source['properties'][key].get(constraint)
    assert schema['$defs']['ReactionWheelObservation'] == source['$defs']['ReactionWheelObservation']
    assert len(schema['$defs']['observation']['allOf'][2]['oneOf']) == 2


def test_real_hub_accepts_repeated_sarm_frames_and_records_wheels(tmp_path):
    async def run():
        recorder = EpisodeRecorder(tmp_path)
        episode = recorder.start(EpisodeStart())
        hub = SimulationHub()
        async def record(obs):
            recorder.record_observation(obs, None)
        hub.on_observation = record
        await hub.start('127.0.0.1', 0)
        reader, writer = await asyncio.open_connection('127.0.0.1', hub.bound_port)
        try:
            await write_async(writer, {'protocol':'space-arm-control/1','type':'sim_hello',
                'simulation_id':'sarm-test','capabilities':['reaction_wheel_attitude_control']})
            revision = 0
            for step in range(1, 4):
                payload = observation()
                payload['step_id'] = payload['render_frame_id'] = str(step)
                await write_async(writer, payload)
                revision, received = await asyncio.wait_for(hub.wait_for_observation(revision), 2)
                assert received.step_id == str(step)
                assert received.jacobian_rank == 6
                assert hub.connected
            closed = recorder.stop(EpisodeStop())
            assert closed['step_count'] == 3
            rows = [json.loads(line) for line in (tmp_path/episode['episode_id']/'steps.jsonl').read_text().splitlines()]
            assert rows[-1]['observation']['reaction_wheels']['speed_rad_s'] == [1,2,3]
            assert rows[-1]['observation']['gripper_position_m'] == [.01,.01]
        finally:
            writer.close()
            await writer.wait_closed()
            await hub.close()
    asyncio.run(run())
