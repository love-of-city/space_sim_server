"""Servo readback must follow connection order even with mixed MJCF actuators."""
from types import SimpleNamespace

import pytest

from simulation.physics_ports import PhysicsPorts, PortError


def test_servo_effort_uses_connected_servo_order_not_global_actuator_index():
    core = SimpleNamespace(actuator_names=('operating', 'wheel', 'capture', 'jaw'),
        servo_index=[3, 0, 2], requestedOutMsgs=['jaw-request', 'arm-request', 'capture-request'],
        appliedOutMsgs=['jaw-actual', 'arm-actual', 'capture-actual'])
    ports = PhysicsPorts(None, stepper=core)
    assert ports.servo_effort('capture') == ('capture-request', 'capture-actual')
    assert ports.servo_effort('operating') == ('arm-request', 'arm-actual')
    assert ports.servo_effort('jaw') == ('jaw-request', 'jaw-actual')
    with pytest.raises(PortError, match='not served'):
        ports.servo_effort('wheel')
    with pytest.raises(PortError, match='unknown actuator'):
        ports.servo_effort('missing')


def test_unconnected_or_incomplete_servo_publication_fails_instead_of_using_another_motor():
    core = SimpleNamespace(actuator_names=('a', 'b'), servo_index=[1],
        requestedOutMsgs=[], appliedOutMsgs=[])
    ports = PhysicsPorts(None, stepper=core)
    with pytest.raises(PortError, match='not served'):
        ports.servo_effort('a')
    with pytest.raises(PortError, match='not served'):
        ports.servo_effort('b')
