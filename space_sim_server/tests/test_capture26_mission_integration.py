"""Real contracts at mission, image-route and private-state boundaries."""
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from simulation.capture26_flight import Capture26Flight
from simulation.capture26_mission import load_vision
from simulation.plug_retention import PlugFriction, RetentionSocket, RetentionParameters
from space_arm_platform.mission_vision import MissionVisionRouter
from space_arm_platform.vision_channel import VisionResultReceiver
from space_arm_platform.vision_service import StereoVisionService
from space_arm_platform.visual_navigation import calibration_id
from space_arm_platform.rendezvous_config import FlightOptions
from space_arm_platform.near_field_config import NearFieldOptions

ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('scenario', ['in-orbit', 'orbit-raise'])
def test_distant_release_uses_estimates_and_requires_continuous_120s(scenario):
    flight = Capture26Flight(scenario)
    start = flight.profile.start_approach_s+flight.profile.approach_duration_s

    def step(t, healthy=True):
        p0, v0, _ = flight.profile.chaser(t)
        p1, v1, _ = flight.profile.target(t)
        return flight.step(round(t*1e9), dict(p=np.array([p0, p1]), v=np.array([v0, v1]),
            rates=np.zeros((2, 3)), b=np.zeros((2, 3))), healthy=healthy)

    assert not step(start)['released']
    assert not step(start+119.95)['released']
    assert step(start+120)['released']
    assert step(start+120.05, False)['phase'] == 'SafeHold'
    assert not step(start+250)['released']


def test_held_plug_restarts_with_identical_wrench_and_cannot_recapture_after_release():
    socket = RetentionSocket('seat', np.zeros(3), np.eye(3), np.eye(3), True)
    a = PlugFriction((socket,), RetentionParameters())
    a.wrench([0., 0., .003], np.eye(3), [0., 0., .001], np.zeros(3))
    b = PlugFriction((socket,), RetentionParameters())
    b.restore(json.loads(json.dumps(a.snapshot())))
    inputs = ([.0001, 0., .004], np.eye(3), [0., 0., .001], np.zeros(3))
    for x, y in zip(a.wrench(*inputs), b.wrench(*inputs)):
        np.testing.assert_array_equal(x, y)
    b.wrench([0., 0., .01], np.eye(3), np.zeros(3), np.zeros(3))
    saved = b.snapshot()
    c = PlugFriction((socket,), RetentionParameters())
    c.restore(saved)
    c.wrench([0., 0., .003], np.eye(3), np.zeros(3), np.zeros(3))
    assert c.active is None
    invalid = dict(saved, active='other-seat')
    before = c.snapshot()
    with pytest.raises(ValueError):
        c.restore(invalid)
    assert c.snapshot() == before


def test_distant_fault_brakes_against_a_latched_reference_and_cancels_feedforward():
    flight = Capture26Flight('in-orbit')
    state = dict(p=np.array([[7.e6, 100., 0.], [7.e6, 0., 0.]]),
                 v=np.array([[0., 7500.1, 0.], [0., 7500., 0.]]), rates=np.zeros((2, 3)), b=np.zeros((2, 3)))
    first = flight.step(10_000_000_000, state, healthy=False)
    np.testing.assert_array_equal(first['velocity_goal'], state['v'][1])
    np.testing.assert_array_equal(first['differential_acceleration'], np.zeros(3))
    state['p'][0, 1] += 3.
    state['p'][1, 1] += 2.
    second = flight.step(11_000_000_000, state, healthy=True)
    np.testing.assert_array_equal(second['position_goal']-state['p'][1], [0., 100., 0.])
    assert second['phase'] == 'SafeHold' and not second['released']
    flight.hold('telemetry connection lost')
    assert flight.reason == 'telemetry connection lost'


def test_shared_image_route_keeps_flight_identity_and_returns_to_teleop(tmp_path):
    config = load_vision(ROOT)
    cal = calibration_id(config)
    runtime = SimpleNamespace(output_dir=tmp_path, status=lambda: dict(running=True))
    vision = StereoVisionService(tmp_path, config=config,
        estimator=lambda left, right: dict(status='rejected', reason='explicit image failure', ambiguous=False))
    router = MissionVisionRouter(ROOT, runtime, vision)
    receiver = VisionResultReceiver('mission', 'flight-session', cal)
    vision.start()
    try:
        route = dict(schema='space-sim-vision-route/1', mission_id='mission', session_id='flight-session',
                     calibration_id=cal, result_port=receiver.port)
        (tmp_path/'vision-route.json').write_text(json.dumps(route))
        router.update()
        assert (tmp_path/'vision-ready.json').is_file()
        assert router.accepts(dict(session_id='flight-session'))
        assert not router.accepts(dict(session_id='old-teleop'))
        for eye in ('left', 'right'):
            packet = dict(session_id='flight-session', camera_id=config['camera_ids'][eye],
                          source_frame_id='3', sim_time_ns='100000000', stream_kind='vision', state_kind='authoritative')
            vision.on_capture(packet, dict(rgb=eye.encode()))
        assert vision.wait_idle()
        result, = receiver.poll()
        assert result['mission_id'] == 'mission' and result['session_id'] == 'flight-session'
        assert result['status'] == 'rejected'
        runtime.status = lambda: dict(running=False)
        router.update()
        assert not router.accepts(dict(session_id='flight-session'))
        assert vision._bound_session is None
        assert vision.config_path.name == 'capture26_stereo.json'
    finally:
        router.close()
        vision.close()
        receiver.close()


@pytest.mark.parametrize('cls', [FlightOptions, NearFieldOptions])
def test_model_is_an_explicit_reproducible_option(cls):
    options = cls(model_id='sarm-dual-capture26', seed=19)
    assert cls.from_dict(options.to_dict()) == options
    with pytest.raises(ValueError, match='model_id'):
        cls(model_id='arbitrary.xml')


def test_long_flight_streams_every_record_but_bounds_working_memory(tmp_path):
    from simulation.capture26_evidence import EvidenceRows
    path = tmp_path/'score.jsonl'
    rows = EvidenceRows(path, retain=3)
    try:
        for i in range(25):
            rows.append(dict(epoch=i, values=np.array([i, i+1])))
        assert len(rows.rows) == 3 and rows[-1]['epoch'] == 24
        assert rows.count == 25
    finally:
        rows.close()
    records = [json.loads(line) for line in path.read_text('utf-8').splitlines()]
    assert [r['epoch'] for r in records] == list(range(25))
    assert records[-1]['values'] == [24, 25]
    with pytest.raises(FileExistsError):
        EvidenceRows(path)


def test_capture26_does_not_launch_a_synthetic_near_fixture(tmp_path):
    from space_arm_platform.rendezvous_runtime import RendezvousRuntime
    runtime = RendezvousRuntime(None, Path('python.exe'), 1234, tmp_path/'logs', tmp_path)
    with pytest.raises(ValueError, match='accepted far-flight'):
        runtime.start(scenario='near-field', options=NearFieldOptions(model_id='sarm-dual-capture26').to_dict())


def test_operators_cannot_move_either_arm_before_receiving_sensor_gate():
    from simulation.capture26_operations import OperationsArmClient
    action = dict(deadman=True, active_arm='capture', allow_reference_recovery=True)
    gate = OperationsArmClient(SimpleNamespace(latest_action=lambda: (action, False)))
    held, stale = gate.latest_action()
    assert stale and not held['deadman'] and not held['allow_reference_recovery']
    assert action['deadman']  # Inhibition must not mutate the shared input.
    gate.enabled = True
    permitted, stale = gate.latest_action()
    assert permitted == action and not stale


def test_operation_observation_conforms_to_backend_schema_and_exposes_sensor_health():
    from simulation.capture26_operations import OperationsControlBridge
    from space_arm_platform.models import SimulationObservation
    delivered = []
    client = SimpleNamespace(connected=True, take_reset=lambda: None,
        has_observation_capacity=lambda: True, reset_generation='', send_observation=delivered.append)
    bridge = OperationsControlBridge.__new__(OperationsControlBridge)
    bridge.client = client
    bridge.args = SimpleNamespace(scene_instance=dict(instance_id='receiving-scene'), seed=7)
    bridge.target = SimpleNamespace(applied_sequence='0', position=np.zeros(8))
    bridge.positions = bridge.rates = [lambda: SimpleNamespace(state=0.)]*8
    bridge.kinematics = SimpleNamespace(forward=lambda q: (np.zeros(3), np.eye(3)),
                                        jacobian=lambda q: np.zeros((6, 6)))
    renderer = SimpleNamespace(session_id='session', last_published_frame_id=5, last_capture_state={})
    nav = SimpleNamespace(base=dict(time_ns=0), state=dict(p=np.zeros((2, 3)), r=np.array([np.eye(3)]*2)),
                          gyro_health=lambda now: [True, True])
    gyro = SimpleNamespace(observation=lambda: None)
    bridge.publish(SimpleNamespace(model_xml_sha256='model'), nav, [gyro, gyro],
        SimpleNamespace(telemetry=lambda: {}), SimpleNamespace(status=lambda now, wall: dict(ready=False, count=0)),
        100_000_000, renderer, dict(phase='OperationsHold', reason='acquiring', gnss_healthy=True), 1.1)
    observation = SimulationObservation.model_validate(delivered[0])
    assert observation.capture26_navigation['gnss_healthy']
    assert observation.capture26_navigation['gyro_health'] == [True, True]
    assert observation.scene_instance_id == 'receiving-scene'


def test_operations_vision_rebinds_without_reusing_retired_flight_session(tmp_path):
    config = load_vision(ROOT)
    config_path = tmp_path/'config/vision/capture26_rendezvous_stereo.json'
    config_path.parent.mkdir(parents=True)
    config_path.write_text(json.dumps(config))
    folder = tmp_path/'run/capture26-operations/receiving-instance'
    folder.mkdir(parents=True)
    runtime = SimpleNamespace(output_dir=None, status=lambda: dict(running=False))
    instance = dict(instance_id='receiving-instance', capture26_output_directory=str(folder))
    scenes = SimpleNamespace(status=lambda: dict(active=True, instance=instance))
    vision = StereoVisionService(tmp_path, config=config,
        estimator=lambda left, right: dict(status='rejected', reason='image failure', ambiguous=False))
    router = MissionVisionRouter(tmp_path, runtime, vision, scenes=scenes)
    receiver = VisionResultReceiver('continuous-mission', 'receiving-session', calibration_id(config))
    vision.start()
    try:
        (folder/'vision-route.json').write_text(json.dumps(dict(schema='space-sim-vision-route/1',
            mission_id='continuous-mission', session_id='receiving-session',
            calibration_id=calibration_id(config), result_port=receiver.port)))
        router.update()
        assert (folder/'vision-ready.json').is_file()
        assert router.accepts(dict(session_id='receiving-session'))
        assert not router.accepts(dict(session_id='retired-flight-session'))
        instance['capture26_output_directory'] = str(tmp_path/'another-task')
        with pytest.raises(ValueError, match='outside its instance'):
            router.update()
        instance['capture26_output_directory'] = str(folder)
        scenes.status = lambda: dict(active=False, instance=instance)
        router.update()
        assert not router.accepts(dict(session_id='receiving-session'))
    finally:
        router.close()
        vision.close()
        receiver.close()


def test_capture26_continuation_requires_complete_checked_state_and_matching_calibration():
    """A unit-level transport fixture; it is not a flown accepted result."""
    import hashlib
    from copy import deepcopy
    from simulation.capture26_handoff import validate, SCHEMA, COMMON_CHECKS, LEGACY_PERFORMANCE_CHECKS
    from simulation.capture26_mission import MODEL_PATH, MODEL_ID, load_mission
    from simulation.sensor_delivery import FixedStereoDelivery
    from simulation.pose_navigation import PoseNavigation
    from simulation.gyroscope import Gyroscope
    from simulation.differential_gnss import DifferentialGNSS
    from space_arm_platform.gnss_config import GNSSConfig
    cal = calibration_id(load_vision(ROOT))
    nav = PoseNavigation(np.zeros((2, 3)), np.zeros((2, 3)), [np.eye(3)]*2, np.eye(24),
                         vision_identity=('unit-contract', 'previous-render-session', cal))
    gyros = [Gyroscope(seed=7), Gyroscope(seed=8, body_name='capture_target', sensor_name='target_gyro')]
    for i, gyro in enumerate(gyros):
        nav.gyro(i, gyro.sample(0, np.zeros(3)))
    receiver = DifferentialGNSS(GNSSConfig(), 7)
    receiver.measure(np.zeros((2, 6)), 0.)
    digest = hashlib.sha256((ROOT/MODEL_PATH).read_bytes()).hexdigest()
    required = COMMON_CHECKS | {'actual_120s_hold', 'actual_tracking_2m', 'actual_velocity_2cm_s', 'actual_service_rate_0p001rad_s'}
    document = dict(schema=SCHEMA, model_id=MODEL_ID, source_sha256=digest,
        accepted=True, checks=dict.fromkeys(required, True), phase='FarReady', time_ns=0,
        mission_id='unit-contract', calibration_id=cal,
        physical=dict(time_ns=0, model_xml_sha256=digest, nq=14, nv=12, free_qpos=[0, 7],
            qpos_inertial=[0., 0., 0., 1., 0., 0., 0.]*2, qvel_inertial=[0.]*12),
        navigation=nav.snapshot(), gyros=[g.snapshot() for g in gyros],
        mission_configuration=load_mission(ROOT).to_dict(), sensor_delivery=FixedStereoDelivery().snapshot(),
        gnss_receiver=receiver.snapshot(), gnss_pending=[], retention={}, environment=[],
        control=dict(operating_reference=[0.]*8, capture_reference=[0.]*12, force_n=[0.]*3,
                     gyro_pending=[], last_cooperative_ns=0, last_gnss_ns=0, platform_stability=None))
    assert validate(json.loads(json.dumps(document)), ROOT)['mission_id'] == 'unit-contract'
    # A slower run may pass functional admission. Legacy /1 performance flags
    # remain readable, while failures of any functional check still reject it.
    slower = deepcopy(document)
    slower['performance'] = dict(required_for_acceptance=False,
                                 processing_real_time_factor_after_warmup=.9, achieved_1x=False)
    assert validate(slower, ROOT)['accepted']
    slower['checks'].update(dict.fromkeys(LEGACY_PERFORMANCE_CHECKS, False))
    assert validate(slower, ROOT)['accepted']
    slower['checks']['gnss_healthy'] = False
    with pytest.raises(ValueError, match='failed independent checks'):
        validate(slower, ROOT)
    slower['checks']['gnss_healthy'] = True
    slower['checks']['unknown_functional_check'] = False
    with pytest.raises(ValueError, match='failed independent checks'):
        validate(slower, ROOT)
    for change in ('missing_check', 'physical_epoch', 'calibration', 'gyro_identity', 'model'):
        bad = deepcopy(document)
        if change == 'missing_check':
            del bad['checks']['gyros_healthy']
        elif change == 'physical_epoch':
            bad['physical']['time_ns'] = 100_000_000
        elif change == 'calibration':
            bad['calibration_id'] = 'outdated'
        elif change == 'gyro_identity':
            bad['gyros'][1]['body_name'] = 'cubesat_bus'
        else:
            bad['source_sha256'] = 'other-model'
        with pytest.raises(ValueError):
            validate(bad, ROOT)
    # Validation operates on a copy; a failed admission cannot mutate the sender.
    assert document['gyros'][1]['body_name'] == 'capture_target'
    legacy = deepcopy(document)
    legacy['schema'] = 'space-sim-capture26-handoff/1'
    del legacy['mission_configuration'], legacy['sensor_delivery']
    assert validate(legacy, ROOT)['accepted']  # Readable, never admitted by the v2 runtime.
    for missing in ('mission_configuration', 'sensor_delivery'):
        bad = deepcopy(document)
        del bad[missing]
        with pytest.raises(ValueError, match='v2 handoff lacks'):
            validate(bad, ROOT)


@pytest.mark.parametrize('paced', [False, True])
def test_stage_scoring_keeps_slow_processing_informational(tmp_path, monkeypatch, paced):
    """Unit scoring inputs only; these ideal rows are not flight evidence."""
    from simulation import capture26_results
    from simulation.capture26_mission import MODEL_ID, Capture26Mission
    from simulation.orbital_rendezvous.reference import FlightProfile
    profile = FlightProfile(scenario='in-orbit', handoff_distance_m=110.)
    start = profile.start_approach_s+profile.approach_duration_s
    rows = []
    for t in (start, start+120.):
        p0, v0, _ = profile.chaser(t)
        p1, v1, _ = profile.target(t)
        rows.append(dict(time_s=t, guidance=dict(released=True, phase='FarReady'),
            truth_com_position_n_m=[p0, p1], truth_com_velocity_n_m_s=[v0, v1],
            truth_relative_position_m=p0-p1, gyro_health=[True, True], gnss_healthy=True,
            truth_angular_velocity_body_rad_s=[[.001, 0., 0.], [0., .01, 0.]],
            visual=dict(ready=False)))
    windows = [dict(sim_time_s=t, compute_s=t/.9, wall_s=t/.8) for t in (2., 122.)]
    args = SimpleNamespace(mission_stage='far', scenario='in-orbit', realtime=paced,
        simulation_rate=.8, flight_options=FlightOptions(model_id=MODEL_ID))
    summary = dict(dynamics=dict(external_state_writes=0), source_sha256='unit-model',
                   mission_configuration=Capture26Mission().to_dict(),
                   calibration_id='unit-calibration', body_properties={})
    sensor = SimpleNamespace(snapshot=lambda: {})
    stepper = SimpleNamespace(physical_snapshot=lambda **kw: {})
    control = SimpleNamespace(force_n=np.zeros(3), last_gnss_ns=round(rows[-1]['time_s']*1e9))
    validated = []
    # Transport validity is exercised separately with the complete contract
    # above; here isolate independent scoring from native serialization.
    monkeypatch.setattr(capture26_results, 'validate', lambda d, root: validated.append(d))
    report = capture26_results.write_stage_result(tmp_path, args, summary, rows, windows,
        sensor, [SimpleNamespace(sensor=sensor)]*2, sensor, [], None,
        SimpleNamespace(target=SimpleNamespace(position=np.zeros(12))), stepper, control,
        None, [], None, 'unit-contract', 0, 'unit-epoch')
    assert report['mission_success'] and len(validated) == 1
    assert report['performance']['achieved_1x'] is False
    assert report['performance']['processing_real_time_factor_after_warmup'] == pytest.approx(.9)
    assert report['performance']['required_for_acceptance'] is False
    assert report['performance']['wall_clock_pacing'] is paced
    handoff = json.loads((tmp_path/'handoff.json').read_text())
    assert handoff['accepted'] and handoff['performance'] == report['performance']
    # Performance cannot mask a functional failure.
    rows[-1]['gnss_healthy'] = False
    failed = capture26_results.write_stage_result(tmp_path, args, summary, rows, windows,
        sensor, [SimpleNamespace(sensor=sensor)]*2, sensor, [], None,
        SimpleNamespace(target=SimpleNamespace(position=np.zeros(12))), stepper, control,
        None, [], None, 'unit-contract', 0, 'unit-epoch')
    assert not failed['mission_success'] and len(validated) == 1
    rows[-1]['gnss_healthy'] = True
    rows[-1]['truth_angular_velocity_body_rad_s'][0][0] = .001000001
    failed = capture26_results.write_stage_result(tmp_path, args, summary, rows, windows,
        sensor, [SimpleNamespace(sensor=sensor)]*2, sensor, [], None,
        SimpleNamespace(target=SimpleNamespace(position=np.zeros(12))), stepper, control,
        None, [], None, 'unit-contract', 0, 'unit-epoch')
    assert not failed['checks']['actual_service_rate_0p001rad_s']
    assert not failed['mission_success'] and len(validated) == 1
