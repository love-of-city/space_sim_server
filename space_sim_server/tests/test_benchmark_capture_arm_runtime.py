"""Native port integrity with explicit fake messages; no physics pass claimed."""
import hashlib
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

import simulation.benchmark_capture_arm_runtime as runtime
from simulation.benchmark_capture_arm_control import PERIOD_NS
from simulation.benchmark_capture_model import MODEL_PATH, _capture_joints
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.physics_ports import PhysicsPorts
from space_arm_platform.benchmark_capture_arm import CaptureArmCommand

SERVER = Path(__file__).resolve().parents[1]
EPOCH = 8_000_000_000_000
CHANNELS = _capture_joints(ET.parse(SERVER/MODEL_PATH).getroot())


class Message:
    def __init__(self, value):
        self.value, self.stamp, self.written = value, 0, True

    def addSubscriber(self): return self
    def isWritten(self): return self.written
    def timeWritten(self): return self.stamp
    def __call__(self): return SimpleNamespace(state=self.value, input=self.value)


class Ports(PhysicsPorts):
    def __init__(self):
        names = ('wheel', *(row['actuator'] for row in reversed(CHANNELS)))
        core = SimpleNamespace(_handle=object(), _last_nanos=None, step_count=0, external_writes=0,
            servos_finalized=False, servo_index=[], actuator_names=names, requestedOutMsgs=[], appliedOutMsgs=[],
            actuator_force=[0.]*len(names), model_xml_sha256=hashlib.sha256((SERVER/MODEL_PATH).read_bytes()).hexdigest())
        super().__init__(None, stepper=core)
        self.positions = {joint: Message(initial) for _, joint, initial in INITIAL_JOINTS}
        self.velocities = {joint: Message(index*.001) for index, (_, joint, _) in enumerate(INITIAL_JOINTS)}
        self.connections = []

    def joint_state(self, joint): return self.positions[joint]
    def joint_rate(self, joint): return self.velocities[joint]

    def drive_servo(self, actuator, position, velocity, **options):
        self.connections.append((actuator, position, velocity, options))
        self._servo_owners[actuator] = options['owner']
        index = self.stepper.actuator_names.index(actuator)
        self.stepper.servo_index.append(index)
        self.stepper.requestedOutMsgs.append(Message(index+.5))
        self.stepper.appliedOutMsgs.append(Message(index*.01))
        self.stepper.actuator_force[index] = index*.01

    def publish(self, nanos):
        self.stepper._last_nanos, self.stepper.step_count = nanos, nanos//PERIOD_NS
        for message in (*self.positions.values(), *self.velocities.values(),
                        *self.stepper.requestedOutMsgs, *self.stepper.appliedOutMsgs):
            message.stamp = nanos


def reader(ports):
    return runtime.NativeCaptureEncoderReader(ports, episode_id='capture-a', epoch_ns=EPOCH)


def bind(monkeypatch, ports):
    # Model resource validation is exercised by the complete-model test suite.
    # Here only replace verified manifest acquisition and native message creation.
    monkeypatch.setattr(runtime, 'verify_capture_model', lambda *_: dict(
        generated_sha256=hashlib.sha256((SERVER/MODEL_PATH).read_bytes()).hexdigest(), capture_joints=CHANNELS))
    monkeypatch.setattr(runtime, '_publisher', lambda _: SimpleNamespace(
        positionOutMsgs=[object() for _ in range(12)], velocityOutMsgs=[object() for _ in range(12)]))
    return runtime.NativeCaptureArm(ports, SERVER/MODEL_PATH,
        repository=SERVER, episode_id='capture-a', epoch_ns=EPOCH)


def test_reader_keeps_all_joint_units_and_current_time_without_exposing_body_truth():
    ports = Ports()
    ports.publish(3*PERIOD_NS)
    observed = reader(ports).sample(EPOCH+3*PERIOD_NS)
    assert observed.positions() == tuple(value for _, _, value in INITIAL_JOINTS)
    assert observed.velocities() == tuple(index*.001 for index in range(12))
    assert observed.sample_time_ns == EPOCH+3*PERIOD_NS
    assert not {'model_xml_sha256', 'native_step_count', 'external_state_writes', 'effort', 'reference'} & observed.model_dump().keys()
    assert ports.stepper.step_count == 3 and not ports.connections


@pytest.mark.parametrize('fault', ['missing_position', 'stale_position', 'missing_velocity', 'stale_velocity',
    'nan', 'wrong_step', 'wrong_time', 'state_write', 'closed', 'off_grid', 'bool_time', 'past_epoch'])
def test_reader_rejects_invalid_native_publication_before_any_control(fault):
    ports = Ports()
    ports.publish(PERIOD_NS)
    stamp = EPOCH+PERIOD_NS
    if fault.startswith('missing_'):
        messages = ports.positions if fault.endswith('position') else ports.velocities
        messages['bottom_joint6'].written = False
    elif fault.startswith('stale_'):
        messages = ports.positions if fault.endswith('position') else ports.velocities
        messages['cam_angle'].stamp -= 1
    elif fault == 'nan': ports.velocities['jaw_slide_3'].value = float('nan')
    elif fault == 'wrong_step': ports.stepper.step_count += 1
    elif fault == 'wrong_time': ports.stepper._last_nanos += 1
    elif fault == 'state_write': ports.stepper.external_writes = 1
    elif fault == 'closed': ports.stepper._handle = None
    elif fault == 'off_grid': stamp += 1
    elif fault == 'bool_time': stamp = True
    else: stamp = EPOCH-1
    with pytest.raises(ValueError):
        reader(ports).sample(stamp)
    assert not ports.connections


@pytest.mark.parametrize('kwargs', [dict(epoch_ns=True), dict(epoch_ns=-1), dict(episode_id=''), dict(episode_id=5)])
def test_reader_rejects_invalid_initial_identity_before_subscribing(kwargs):
    with pytest.raises(ValueError):
        runtime.NativeCaptureEncoderReader(Ports(), **dict(dict(episode_id='x', epoch_ns=0), **kwargs))


def test_all_twelve_original_servos_bind_with_exact_authored_gains_and_mixed_units(monkeypatch):
    ports = Ports()
    arm = bind(monkeypatch, ports)
    assert len(ports.connections) == 12
    for index, (name, position, velocity, options) in enumerate(ports.connections):
        row = CHANNELS[index]
        assert name == row['actuator'] and options == dict(owner='benchmark_capture_arm',
            joint=row['joint'], kp=row['kp'], kd=row['kd'], limit=row['effort_limit'])
        assert position is arm.publisher.positionOutMsgs[index] and velocity is arm.publisher.velocityOutMsgs[index]
    assert 'wheel' not in ports._actuator_owners and 'wheel' not in ports._servo_owners
    assert arm.reference_at(0)[0] == tuple(value for _, _, value in INITIAL_JOINTS)


@pytest.mark.parametrize('fault', ['already_driven', 'already_served', 'directly_connected', 'wrong_model',
    'started', 'closed', 'finalized', 'missing_actuator'])
def test_installation_preflights_all_channels_before_claiming_any_servo(monkeypatch, fault):
    ports = Ports()
    last = CHANNELS[-1]['actuator']
    if fault == 'already_driven': ports._actuator_owners[last] = 'someone'
    elif fault == 'already_served': ports._servo_owners[last] = 'someone'
    elif fault == 'directly_connected': ports.stepper.servo_index.append(ports.stepper.actuator_names.index(last))
    elif fault == 'wrong_model': ports.stepper.model_xml_sha256 = '0'*64
    elif fault == 'started': ports.stepper._last_nanos = 0
    elif fault == 'closed': ports.stepper._handle = None
    elif fault == 'finalized': ports.stepper.servos_finalized = True
    else: ports.stepper.actuator_names = tuple(name for name in ports.stepper.actuator_names if name != last)
    with pytest.raises(ValueError):
        bind(monkeypatch, ports)
    assert not ports.connections


def test_private_receipt_separates_reference_requested_pid_and_actual_native_force(monkeypatch):
    ports = Ports()
    arm = bind(monkeypatch, ports)
    arm.check_initial_reset(0)
    arm.reference_at(0)
    ports.publish(0)
    initial = arm.evidence(EPOCH)
    assert initial['effort'] is None and initial['reference'] is None
    arm.submit(CaptureArmCommand(enabled=True, joint6_velocity_rad_s=.1), EPOCH)
    reference, _ = arm.reference_at(PERIOD_NS)
    assert reference[5] > 0 and ports.positions['bottom_joint6'].value == 0
    ports.publish(PERIOD_NS)
    receipt = arm.evidence(EPOCH+PERIOD_NS)
    assert receipt['reference']['encoder_sample_time_ns'] == EPOCH
    assert receipt['reference']['control_time_ns'] == receipt['sample_time_ns']
    assert receipt['native_step_count'] == 1 and receipt['external_state_writes'] == 0
    assert [row['unit'] for row in receipt['effort']] == ['N m']*6+['N', 'N m']+['N']*4
    for row in receipt['effort']:
        index = ports.stepper.actuator_names.index(row['actuator'])
        assert row['requested_post_step_pid'] == index+.5 and row['applied_native'] == index*.01
    assert arm.evidence(EPOCH+PERIOD_NS) == receipt
    with pytest.raises(ValueError, match='fresh native core'):
        arm.check_initial_reset(0)


@pytest.mark.parametrize('fault', ['missing', 'stale', 'nan', 'wrong_native_force'])
def test_private_force_receipt_rejects_stale_or_wrong_actuation(monkeypatch, fault):
    ports = Ports()
    arm = bind(monkeypatch, ports)
    ports.publish(0)
    arm.reference_at(PERIOD_NS)
    ports.publish(PERIOD_NS)
    message = ports.stepper.appliedOutMsgs[3]
    if fault == 'missing': message.written = False
    elif fault == 'stale': message.stamp = 0
    elif fault == 'nan': message.value = float('nan')
    else: message.value += 1
    with pytest.raises(ValueError):
        arm.evidence(EPOCH+PERIOD_NS)


def test_missed_control_tick_latches_failure_and_requires_rebuild(monkeypatch):
    ports = Ports()
    arm = bind(monkeypatch, ports)
    ports.publish(0)
    with pytest.raises(ValueError, match='current contiguous'):
        arm.reference_at(2*PERIOD_NS)
    assert arm.controller.last_reference_time_ns == EPOCH
    for action in (lambda: arm.reference_at(PERIOD_NS),
                   lambda: arm.submit(CaptureArmCommand(), EPOCH), lambda: arm.evidence(EPOCH)):
        with pytest.raises(ValueError, match='publisher failed'):
            action()
