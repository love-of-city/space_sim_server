"""Angular reference for the source MJCF tendon position actuator."""
from dataclasses import dataclass
import math

@dataclass
class LinkageGripperCommand:
    position: float = 0.0
    speed_rad_s: float = 0.2

    def advance(self, direction: float, dt_s: float, enabled: bool) -> float:
        if not math.isfinite(direction) or not math.isfinite(dt_s) or dt_s < 0:
            raise ValueError('invalid gripper command')
        if enabled:
            self.position = min(1., max(0., self.position + max(-1., min(1., direction))*self.speed_rad_s*dt_s))
        return self.position

def tendon_reference_publisher(scene, trajectory, actuator_name):
    from Basilisk.architecture import messaging, sysModel

    class Publisher(sysModel.SysModel):
        def __init__(self):
            super().__init__()
            self.ModelTag = 'sarmLinkageTendonReference'
            self.reference = trajectory.positionOutMsgs[6].addSubscriber()
            self.output = messaging.SingleActuatorMsg()
            scene.getSingleActuator(actuator_name).actuatorInMsg.subscribeTo(self.output)

        def UpdateState(self, now):
            # gear=.5 * (q11 - q12), with equality q12=-q11 => q11.
            self.output.write(messaging.SingleActuatorMsgPayload(input=float(self.reference().state)), now, self.moduleID)

        def Reset(self, now):
            self.UpdateState(now)

    return Publisher()
