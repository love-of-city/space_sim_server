"""Hold an already latched capture arm using its ordinary actuator servos."""
import xml.etree.ElementTree as ET
import numpy as np

from simulation.capture_arm_target import CaptureArmTarget, INITIAL_JOINTS

HOLD_GAINS = dict(kp=1500., kd=80., torque_limit_nm=80.)


def hold_gains(values=None):
    result = dict(HOLD_GAINS if values is None else values)
    if (set(result) != set(HOLD_GAINS) or not all(np.isfinite(v) for v in result.values())
            or not 0 < result['kp'] <= 5000 or not 0 <= result['kd'] <= 160
            or result['torque_limit_nm'] != 80):
        raise ValueError('Invalid capture hold gains; the actuator limit must remain 80 Nm')
    return result


def validate_preheld(model, values):
    q = np.asarray(values, dtype=float)
    target = CaptureArmTarget(model)
    if q.shape != (12,) or not np.isfinite(q).all() or np.any(q < target.lower) or np.any(q > target.upper):
        raise ValueError('Invalid preheld capture joint references')
    root = ET.parse(model).getroot()
    body = root.find('./worldbody/body[@name="capture_target"]')
    lock = root.find('./equality/weld[@name="preheld_capture_lock"]')
    if (body is None or (body.find('joint[@type="free"]') is None and body.find('freejoint') is None) or lock is None
            or lock.get('body1') != 'gripper' or lock.get('body2') != 'capture_target'
            or lock.get('active', 'true') != 'true'):
        raise ValueError('Preheld target must be an independent free body locked to the capture gripper')
    return q


class PreheldCaptureComponent:
    name = 'preheld_capture_hold'

    def __init__(self, model, values, gains=None):
        self.q = validate_preheld(model, values)
        self.gains = hold_gains(gains)

    def install(self, ctx):
        from simulation.assembly import Slot
        from simulation.joint_reference_publisher import HeldJointReferencePublisher
        publisher = HeldJointReferencePublisher(lambda t: (self.q.copy(), np.zeros(12)), 12)
        specs = [(f'bottom_joint{i}_act', self.gains['kp'], self.gains['kd'], self.gains['torque_limit_nm']) for i in range(1, 7)] + [
            ('axial_act', 10000., 80., 15.), ('cam_act', 10., 1., 1.),
            *[(f'jaw_act_{i}', 8000., 40., 5.) for i in range(4)]]
        for i, ((_, joint, _), (act, kp, kd, limit)) in enumerate(zip(INITIAL_JOINTS, specs, strict=True)):
            ctx.ports.drive_servo(act, publisher.positionOutMsgs[i], publisher.velocityOutMsgs[i],
                                  owner=self.name, joint=joint, kp=kp, kd=kd, limit=limit)
        ctx.add(publisher, Slot.COMMAND)
        ctx.keep_alive(publisher)
