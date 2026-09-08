"""Three-wheel inertial attitude hold for the authoritative SARM MJScene.

Only joint motors inject torque.  No independently integrated spacecraft or
additional bus torque is created.  Hardware is read from MJCF; gains from JSON.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.fswAlgorithms import attTrackingError, inertial3D, mrpFeedback, rwMotorTorque
from Basilisk.simulation import (
    arrayMotorTorqueToSingleActuators, scalarJointStatesToRWSpeed, simpleNav,
)
from Basilisk.utilities import RigidBodyKinematics as rbk, macros


@dataclass(frozen=True)
class Wheel:
    body: str
    joint: str
    motor: str
    axis: tuple[float, float, float]
    spin_inertia: float  # [kg*m^2]
    max_torque: float  # [N*m]
    max_speed: float  # [rad/s], rotor relative to bus


def _vector(element, name, default):
    if element is None:
        raise ValueError(f'Missing MJCF element containing {name}')
    values = np.asarray([float(v) for v in element.get(name, default).split()])
    if not np.all(np.isfinite(values)):
        raise ValueError(f'MJCF {name} must contain finite numbers')
    return values


def _inertia(element):
    """Return an authored central inertia tensor in its body's local axes."""
    if element is None:
        raise ValueError('Every controlled-spacecraft body needs explicit inertia')
    if 'fullinertia' in element.attrib:
        xx, yy, zz, xy, xz, yz = _vector(element, 'fullinertia', '')
        return np.array([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
    quaternion = _vector(element, 'quat', '1 0 0 0')
    if quaternion.shape != (4,) or np.linalg.norm(quaternion) <= 0:
        raise ValueError('Inertial quaternion must have four components and nonzero norm')
    # MJCF normalizes authored quaternions during compilation; match it here.
    rotation = rbk.EP2C(quaternion / np.linalg.norm(quaternion)).T
    return rotation @ np.diag(_vector(element, 'diaginertia', '')) @ rotation.T


def load_hardware(model_path: Path):
    """Read and validate the supported balanced, body-axis wheel arrangement."""
    root = ET.parse(model_path).getroot()
    bus = root.find('./worldbody/body[@name="cubesat_bus"]')
    if bus is None:
        raise ValueError('Missing cubesat_bus in platform MJCF')
    speed = _vector(root.find('./custom/numeric[@name="rw_max_speed_rad_s"]'), 'data', '')
    if speed.shape != (3,) or not np.all(np.isfinite(speed)) or np.any(speed <= 0):
        raise ValueError('rw_max_speed_rad_s must contain three positive finite speeds')
    wheels = []
    for i, label in enumerate('xyz'):
        name = f'rw_{label}'
        body = bus.find(f'./body[@name="{name}"]')
        if body is None:
            raise ValueError(f'Missing body {name}')
        joint = body.find(f'./joint[@name="{name}_spin"]')
        motor = root.find(f'./actuator/motor[@name="{name}_motor"]')
        axis = _vector(joint, 'axis', '0 0 1')
        inertia = _inertia(body.find('inertial'))
        expected = np.eye(3)[i]
        if (axis.shape != (3,) or not np.allclose(axis, expected)
                or any(key in body.attrib for key in ('quat', 'euler', 'axisangle', 'xyaxes', 'zaxis'))
                or not np.allclose(_vector(body.find('inertial'), 'pos', '0 0 0'), [0, 0, 0])
                or not np.allclose(_vector(joint, 'pos', '0 0 0'), [0, 0, 0])
                or joint.get('type') != 'hinge' or joint.get('limited') != 'false'
                or any(float(joint.get(key, 'nan')) != 0 for key in
                       ('damping', 'frictionloss', 'stiffness', 'armature'))):
            raise ValueError(f'{name}: expected a frictionless, unlimited body-axis hinge')
        if not np.allclose(inertia, np.diag(np.diag(inertia))):
            raise ValueError(f'{name}: unbalanced rotor inertia is unsupported')
        transverse = np.delete(np.diag(inertia), i)
        if not np.isclose(*transverse) or np.any(np.linalg.eigvalsh(inertia) <= 0):
            raise ValueError(f'{name}: rotor must have positive, axisymmetric inertia')
        limits = _vector(motor, 'ctrlrange', '')
        if (motor is None or motor.get('joint') != joint.get('name') or motor.get('gear', '1') != '1'
                or limits.shape != (2,) or not np.all(np.isfinite(limits))
                or limits[1] <= 0 or not np.isclose(limits[0], -limits[1])
                or motor.get('ctrllimited') != 'true'
                or motor.get('forcelimited') != 'true'
                or not np.allclose(_vector(motor, 'forcerange', ''), limits)):
            raise ValueError(f'{name}: expected matching symmetric unit-gear motor limits')
        wheels.append(Wheel(name, joint.get('name'), motor.get('name'), tuple(axis),
                            float(axis @ inertia @ axis), float(limits[1]), float(speed[i])))
    # Exclude the separately free-flying capture target from vehicle inertia.
    parts = []
    for body in [bus, *bus.findall('.//body')]:
        inertial = body.find('inertial')
        if inertial is None or not math.isfinite(float(inertial.get('mass', 'nan'))) or float(inertial.get('mass', '0')) <= 0:
            raise ValueError(f'{body.get("name")}: explicit positive finite body mass required')
        parts.append((body.get('name'), float(inertial.get('mass')),
                      _vector(inertial, 'pos', '0 0 0'), _inertia(inertial)))
    return wheels, parts


def load_settings(path: Path):
    settings = json.loads(path.read_text(encoding='utf-8-sig'))
    expected = {'mode', 'enabled', 'control_rate_hz', 'mrp_proportional_gain_nm',
                'rate_derivative_gain_nm_s', 'speed_guard_fraction'}
    if set(settings) != expected or settings['mode'] != 'initial_inertial_hold':
        raise ValueError('unsupported attitude control configuration')
    if not isinstance(settings['enabled'], bool):
        raise ValueError('attitude enabled must be boolean')
    for key in expected - {'enabled', 'mode'}:
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{key} must be finite and positive')
    rate = settings['control_rate_hz']
    if rate > 500 or not math.isclose(500 / rate, round(500 / rate)):
        raise ValueError('attitude control rate must divide the 500 Hz dynamics rate')
    if settings['speed_guard_fraction'] >= 1:
        raise ValueError('speed_guard_fraction must be below one')
    return settings


def limit_wheel_torque(requested, speed, wheel: Wheel, enabled, guard_fraction):
    """Clip motor torque and inhibit acceleration near the speed boundary.

    This is a drive limit, not an unphysical velocity clamp.  Braking is allowed;
    externally induced overspeed is reported and cannot be instantaneously reset.
    """
    if not math.isfinite(requested) or not math.isfinite(speed):
        raise FloatingPointError('non-finite reaction-wheel command or state')
    torque = float(np.clip(requested, -wheel.max_torque, wheel.max_torque)) if enabled else 0.0
    torque_limited = enabled and not math.isclose(torque, requested, abs_tol=1e-12)
    speed_limited = enabled and abs(speed) >= guard_fraction * wheel.max_speed and torque * speed > 0
    return (0.0 if speed_limited else torque), torque_limited, speed_limited


class WheelDrive(sysModel.SysModel):
    """Bridge a scalar BSK torque to a speed-protected MJScene joint motor."""
    def __init__(self, owner, index, torque_message):
        super().__init__()
        self.ModelTag = f'{owner.wheels[index].body}Drive'
        self.owner, self.index = owner, index
        self.torque_message = torque_message
        self.actuatorOutMsg = messaging.SingleActuatorMsg()
        self.requested = self.applied = 0.0  # [N*m]
        self.torque_limited = self.speed_limited = False

    def UpdateState(self, current_sim_nanos):
        i, owner = self.index, self.owner
        self.requested = float(self.torque_message.read().input)
        speed = float(owner.joints[i].stateDotOutMsg.read().state)
        self.applied, self.torque_limited, self.speed_limited = limit_wheel_torque(
            self.requested, speed, owner.wheels[i], owner.enabled,
            owner.settings['speed_guard_fraction'])
        payload = messaging.SingleActuatorMsgPayload()
        payload.input = self.applied
        self.actuatorOutMsg.write(payload, current_sim_nanos, self.moduleID)


class InitialReference(sysModel.SysModel):
    """Latch initial inertial attitude and locked articulated-vehicle inertia."""
    def __init__(self, owner):
        super().__init__()
        self.ModelTag = 'sarmInitialAttitudeReference'
        self.owner = owner

    def UpdateState(self, current_sim_nanos):
        owner = self.owner
        owner.control_time_ns = current_sim_nanos
        if owner.reference_mrp is not None:
            return
        state = owner.bus.getOrigin().stateOutMsg.read()
        owner.reference_mrp = np.asarray(state.sigma_BN, dtype=float)
        owner.reference.sigma_R0N = owner.reference_mrp.tolist()
        c_bn = rbk.MRP2C(owner.reference_mrp)
        origin = np.asarray(state.r_BN_N)
        parts = []
        for name, mass, com, inertia in owner.parts:
            body_state = owner.scene.getBody(name).getOrigin().stateOutMsg.read()
            r_ni = rbk.MRP2C(body_state.sigma_BN).T
            r_bi = c_bn @ r_ni
            position = c_bn @ (np.asarray(body_state.r_BN_N) - origin) + r_bi @ com
            parts.append((mass, position, r_bi @ inertia @ r_bi.T))
        total_mass = sum(m for m, _, _ in parts)
        center = sum(m * r for m, r, _ in parts) / total_mass
        tensor = np.zeros((3, 3))  # [kg*m^2]
        for mass, position, inertia in parts:
            delta = position - center
            tensor += inertia + mass * (delta @ delta * np.eye(3) - np.outer(delta, delta))
        if not np.all(np.isfinite(tensor)) or np.any(np.linalg.eigvalsh(tensor) <= 0):
            raise ValueError('invalid assembled spacecraft inertia')
        payload = messaging.VehicleConfigMsgPayload()
        payload.ISCPntB_B = tensor.flatten().tolist()
        owner.vehicle_config.write(payload, current_sim_nanos, self.moduleID)
        owner.initial_inertia = tensor
        owner.controller.Reset(current_sim_nanos)


class AttitudeControl:
    """Own the BSK guidance/control chain and the three native wheel drives."""
    def __init__(self, simulation, process, scene, model_path: Path, *, enabled=None):
        self.scene = scene
        self.wheels, self.parts = load_hardware(model_path)
        self.settings = load_settings(model_path.with_name('attitude_control.json'))
        self.enabled = self.settings['enabled'] if enabled is None else bool(enabled)
        self.bus = scene.getBody('cubesat_bus')
        self.state_reader = self.bus.getOrigin().stateOutMsg.addSubscriber()
        self.control_time_ns = 0
        self.joints = [scene.getBody(w.body).getScalarJoint(w.joint) for w in self.wheels]
        self.reference_mrp = None
        self.initial_inertia = None
        self.models = []
        self.drives = []
        task = 'sarmAttitudeTask'
        # Dynamics runs first at coincident ticks; FSW commands are held until
        # the next dynamics evaluation.  IK retains its independent task.
        process.addTask(simulation.CreateNewTask(task, macros.sec2nano(
            1.0 / self.settings['control_rate_hz'])), -10)

        def add(model, tag, priority):
            model.ModelTag = tag
            simulation.AddModelToTask(task, model, priority)
            self.models.append(model)
            return model

        payload = messaging.RWArrayConfigMsgPayload()
        payload.numRW = len(self.wheels)
        payload.GsMatrix_B = np.asarray([w.axis for w in self.wheels]).flatten().tolist()
        payload.JsList = [w.spin_inertia for w in self.wheels]
        payload.uMax = [w.max_torque for w in self.wheels]
        self.wheel_config = messaging.RWArrayConfigMsg().write(payload)
        # Replaced with the actual assembled inertia before the first control
        # evaluation.  mrpFeedback caches this message in Reset, so reset it
        # once more after InitialReference publishes the startup tensor.
        self.vehicle_config = messaging.VehicleConfigMsg().write(
            messaging.VehicleConfigMsgPayload(ISCPntB_B=np.eye(3).flatten().tolist()))
        self.reference = add(inertial3D.inertial3D(), 'sarmInertial3D', 800)
        add(InitialReference(self), 'sarmInitialAttitudeReference', 1000)
        nav = add(simpleNav.SimpleNav(), 'sarmTruthNavigation', 900)
        nav.scStateInMsg.subscribeTo(self.bus.getCenterOfMass().stateOutMsg)
        error = add(attTrackingError.attTrackingError(), 'sarmAttitudeError', 700)
        error.attNavInMsg.subscribeTo(nav.attOutMsg)
        error.attRefInMsg.subscribeTo(self.reference.attRefOutMsg)
        speeds = add(scalarJointStatesToRWSpeed.ScalarJointStatesToRWSpeed(), 'sarmWheelSpeeds', 600)
        speeds.setNumJoints(len(self.wheels))
        for i, joint in enumerate(self.joints):
            speeds.jointStateInMsgs[i].subscribeTo(joint.stateDotOutMsg)
        controller = add(mrpFeedback.mrpFeedback(), 'sarmMrpPD', 500)
        self.controller = controller
        controller.K = self.settings['mrp_proportional_gain_nm']
        controller.P = self.settings['rate_derivative_gain_nm_s']
        controller.Ki = -1.0  # Integral action disabled; no saturation windup.
        controller.integralLimit = 0.0
        controller.guidInMsg.subscribeTo(error.attGuidOutMsg)
        controller.vehConfigInMsg.subscribeTo(self.vehicle_config)
        controller.rwParamsInMsg.subscribeTo(self.wheel_config)
        controller.rwSpeedsInMsg.subscribeTo(speeds.rwSpeedOutMsg)
        allocator = add(rwMotorTorque.rwMotorTorque(), 'sarmWheelAllocation', 400)
        allocator.controlAxes_B = np.eye(3).flatten().tolist()
        allocator.rwParamsInMsg.subscribeTo(self.wheel_config)
        allocator.vehControlInMsg.subscribeTo(controller.cmdTorqueOutMsg)
        singles = add(arrayMotorTorqueToSingleActuators.ArrayMotorTorqueToSingleActuators(), 'sarmWheelTorqueAdapter', 300)
        singles.setNumActuators(len(self.wheels))
        singles.torqueInMsg.subscribeTo(allocator.rwMotorTorqueOutMsg)
        for i, wheel in enumerate(self.wheels):
            drive = WheelDrive(self, i, singles.actuatorOutMsgs[i])
            # Fresh joint speed after FK; enforce safety at dynamics substeps,
            # even though the attitude controller only runs at 100 Hz.
            scene.AddModelToDynamicsTask(drive, 6500 - i)
            scene.getSingleActuator(wheel.motor).actuatorInMsg.subscribeTo(drive.actuatorOutMsg)
            self.drives.append(drive)

    def telemetry(self):
        """Keep wheel states separate from the eight arm/finger coordinates."""
        state = self.bus.getOrigin().stateOutMsg.read()
        sigma = np.asarray(state.sigma_BN, dtype=float)
        error = (rbk.subMRP(sigma, self.reference_mrp)
                 if self.reference_mrp is not None else np.zeros(3))
        quaternion = np.asarray(rbk.MRP2EP(sigma))
        reference = np.asarray(rbk.MRP2EP(self.reference_mrp)) if self.reference_mrp is not None else np.array([1., 0., 0., 0.])
        speeds = [float(j.stateDotOutMsg.read().state) for j in self.joints]
        angle = 4 * math.atan(float(np.linalg.norm(error)))
        angle = min(angle, 2 * math.pi - angle)
        return {
            'attitude_control': {
                'enabled': self.enabled,
                'mode': self.settings['mode'],
                'reference_frame': 'J2000',
                'reference_initialized': self.reference_mrp is not None,
                'state_time_ns': str(self.state_reader.timeWritten()),
                'control_time_ns': str(self.control_time_ns),
                'orientation_inertial_wxyz': quaternion.tolist(),
                'reference_orientation_inertial_wxyz': reference.tolist(),
                'angular_velocity_body_rad_s': list(state.omega_BN_B),
                'attitude_error_angle_rad': angle,
                'control_rate_hz': self.settings['control_rate_hz'],
                'saturated': any(d.torque_limited or d.speed_limited for d in self.drives)
                    or any(abs(v) >= w.max_speed for v, w in zip(speeds, self.wheels)),
            },
            'reaction_wheels': {
                'names': [w.body for w in self.wheels],
                'speed_rad_s': speeds,
                'relative_momentum_nms': [w.spin_inertia * v for w, v in zip(self.wheels, speeds)],
                'requested_motor_torque_nm': [d.requested for d in self.drives],
                'applied_motor_torque_nm': [d.applied for d in self.drives],
                'max_motor_torque_nm': [w.max_torque for w in self.wheels],
                'max_speed_rad_s': [w.max_speed for w in self.wheels],
                'torque_limited': [d.torque_limited for d in self.drives],
                'speed_limited': [d.speed_limited for d in self.drives],
                'overspeed': [abs(v) > w.max_speed for v, w in zip(speeds, self.wheels)],
            },
        }
