"""Independent capture-arm servos; only explicit selected-arm commands move it."""
import numpy as np
from simulation.assembly import Slot
from simulation.joint_reference_publisher import HeldJointReferencePublisher
from simulation.capture_arm_target import CaptureArmTarget, INITIAL_JOINTS


class Capture26Component:
    name = 'capture26_hold'

    def __init__(self, model, client, initial=None):
        self.target = CaptureArmTarget(model)
        if initial is not None:
            q = np.asarray(initial, dtype=float)
            if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < self.target.lower[:6]) or np.any(q > self.target.upper[:6]):
                raise ValueError('invalid capture arm initial joints')
            self.target.position[:6] = q
        self.initial = self.target.position.copy()
        self.client = client
        self.last_time = None

    def install(self, ctx):
        joints = [ctx.scene.getBody(b).getScalarJoint(j) for b,j,_ in INITIAL_JOINTS]
        self.positions = [j.stateOutMsg.addSubscriber() for j in joints]
        self.rates = [j.stateDotOutMsg.addSubscriber() for j in joints]
        publisher = HeldJointReferencePublisher(self.reference, 12)
        specs = [(f'bottom_joint{i}_act',1500.,80.,80.) for i in range(1,7)] + [
            ('axial_act',10000.,80.,15.), ('cam_act',10.,1.,1.),
            *[(f'jaw_act_{i}',8000.,40.,5.) for i in range(4)]]
        for i, ((_, joint, _), (act, kp, kd, limit)) in enumerate(zip(INITIAL_JOINTS, specs, strict=True)):
            ctx.ports.drive_servo(act, publisher.positionOutMsgs[i], publisher.velocityOutMsgs[i],
                                  owner=self.name, joint=joint, kp=kp, kd=kd, limit=limit)
        ctx.add(publisher, Slot.COMMAND)
        ctx.keep_alive(publisher)

    def reference(self, time_s):
        reset = self.last_time is not None and time_s < self.last_time
        if reset:
            self.target.position = self.initial.copy()
            self.target.was_active = False
        dt = 0. if self.last_time is None or reset else max(0., time_s-self.last_time)
        self.last_time = time_s
        action, stale = self.client.latest_action()
        measured = np.array([float(reader().state) for reader in self.positions])
        return self.target.step(dt, action, stale, measured)

    def telemetry(self):
        return self.target.telemetry([float(reader().state) for reader in self.positions],
                                     [float(reader().state) for reader in self.rates])
