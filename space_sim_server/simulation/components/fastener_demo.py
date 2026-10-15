"""Opt-in native physics demonstration; no kinematic pose assignments."""
import json

from Basilisk.architecture import messaging, sysModel

from simulation.assembly import Slot
from simulation.fastener_demo_schedule import demo_command


class FastenerTorqueDemo(sysModel.SysModel):
    def __init__(self, travel, angle):
        super().__init__()
        self.ModelTag = 'captiveFastenerTorqueDemo'
        self.travel = travel.addSubscriber()
        self.angle = angle.addSubscriber()
        self.nut = messaging.CmdTorqueBodyMsg()
        self.rod = messaging.CmdTorqueBodyMsg()
        self.last_second = -1

    def UpdateState(self, nanos):
        seconds = nanos * 1e-9
        phase, torque = demo_command(seconds)
        # Nut +Z is rod -Y: these moments cancel in world coordinates.
        self.nut.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., 0., torque]), nanos)
        self.rod.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=[0., torque, 0.]), nanos)
        if int(seconds) != self.last_second:
            self.last_second = int(seconds)
            print(json.dumps(dict(type='fastener_demo', time_s=seconds, phase=phase,
                torque_nm=torque, travel_mm=float(self.travel().state)*1000,
                angle_rad=float(self.angle().state))), flush=True)


class FastenerDemoComponent:
    name = 'fastener_demo'

    def install(self, ctx):
        if ctx.ports.stepper is None:
            raise RuntimeError('Fastener demo requires local dynamics')
        demo = FastenerTorqueDemo(ctx.ports.joint_state('fastener_slide'),
                                  ctx.ports.joint_state('fastener_spin'))
        ctx.ports.add_body_wrench('fastener', source=self.name + '_nut', torque_body=demo.nut)
        ctx.ports.add_body_wrench('locking_rod', source=self.name + '_rod', torque_body=demo.rod)
        ctx.add(demo, Slot.ACTUATOR)
        ctx.keep_alive(demo)
        ctx.extra['fastener_demo'] = demo
