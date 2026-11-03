"""Bounded differential IK and independent capture tool reference (no physics overrides)."""
import math
import xml.etree.ElementTree as ET
import numpy as np
from simulation.serial_chain_kinematics import SerialChainKinematics

INITIAL_JOINTS = [(f'bottom_link{i}', f'bottom_joint{i}', 0.) for i in range(1, 7)] + [
    ('axial_carriage', 'axial', .02), ('cam', 'cam_angle', math.pi/3),
    *[(f'jaw_{i}', f'jaw_slide_{i}', -.012) for i in range(4)]]


class CaptureArmTarget:
    def __init__(self, model):
        self.position = np.array([x[2] for x in INITIAL_JOINTS])
        self.velocity = np.zeros(12)
        self.was_active = False
        self.status = 'hold'
        self.kinematics = SerialChainKinematics.from_mjcf(model, base_body='cubesat_bus',
            joint_names=tuple(x[1] for x in INITIAL_JOINTS[:6]), tool_site='grip_frame')
        joints = {j.get('name'): j for j in ET.parse(model).iter('joint')}
        ranges = np.array([[float(x) for x in joints[j].get('range').split()] for _,j,_ in INITIAL_JOINTS])
        self.lower, self.upper = ranges.T

    def step(self, dt, action, stale, measured):
        dt = float(np.clip(dt, 0., .02))
        measured = np.asarray(measured)
        valid = measured.shape == (12,) and np.all(np.isfinite(measured))
        enabled = valid and not stale and bool(action.get('deadman')) and action.get('active_arm', 'operating') == 'capture'
        enabled = enabled and not action.get('arm_preparation')
        self.velocity.fill(0)
        if not enabled:
            if self.was_active and valid:
                # Latch measured arm pose once, never reset to the folded pose.
                self.position[:] = np.clip(measured, self.lower, self.upper)
            self.was_active = False
            self.status = 'hold' if valid else 'invalid_measurement'
            return self.position.copy(), self.velocity.copy()
        if not self.was_active:
            self.position[:] = np.clip(measured, self.lower, self.upper)
        self.was_active = True
        if dt <= 0: return self.position.copy(), self.velocity.copy()
        _, rotation = self.kinematics.forward(measured[:6])
        linear = np.asarray(action.get('end_effector_linear_velocity_body_m_s', [0.]*3))
        angular = np.asarray(action.get('end_effector_angular_velocity_body_rad_s', [0.]*3)) + rotation @ np.asarray(action.get('end_effector_angular_velocity_tool_rad_s', [0.]*3))
        result = self.kinematics.inverse_velocity_ik_pose(measured[:6], np.r_[linear, angular],
            joint_velocity_limits=np.full(6,.25), joint_position_min=self.lower[:6],
            joint_position_max=self.upper[:6], dt=dt)
        dq = result.joint_velocity_rad_s.copy()
        dq[5] += float(action.get('joint6_velocity_rad_s', 0))
        dq = np.clip(dq, -.25, .25)
        self.status = 'tracking' if np.linalg.norm(result.residual_twist) < .01 else 'limited_or_singular'
        # Bound reference lead to 3 degrees; do not integrate through a blocked arm.
        old = self.position.copy()
        self.position[:6] = np.clip(self.position[:6] + dq*dt, measured[:6]-.05, measured[:6]+.05)
        self.position[6] += np.clip(action.get('capture_axial_velocity_m_s',0),-.003,.003)*dt
        radial_rate = np.clip(action.get('capture_radial_velocity_m_s',0),-.003,.003)
        if radial_rate != 0:
            radial = np.clip(float(np.mean(self.position[8:])) + radial_rate*dt,-.012,0)
            self.position[8:] = radial
            self.position[7] = -radial/.012*(math.pi/3)
        self.position = np.clip(self.position, self.lower, self.upper)
        self.velocity = (self.position-old)/dt
        return self.position.copy(), self.velocity.copy()

    def telemetry(self, measured, rates):
        q, dq = np.asarray(measured), np.asarray(rates)
        p, _ = self.kinematics.forward(q[:6])
        return dict(joint_position_rad=q[:6].tolist(), joint_velocity_rad_s=dq[:6].tolist(),
                    arm_joint_limits_rad=np.column_stack((self.lower[:6], self.upper[:6])).tolist(),
                    joint_reference_rad=self.position[:6].tolist(), end_effector_position_body_m=p.tolist(),
                    end_effector_twist_body=(self.kinematics.jacobian(q[:6])@dq[:6]).tolist(),
                    axial_position_m=float(q[6]), radial_position_m=float(np.mean(q[8:])),
                    controller_status=self.status)
