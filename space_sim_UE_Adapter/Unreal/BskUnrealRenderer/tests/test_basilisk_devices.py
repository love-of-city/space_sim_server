"""Integration coverage using real Basilisk device modules and messages."""

from __future__ import annotations

import unittest

from bsk_render_adapter import enableUnrealVisualization
from Basilisk.architecture import messaging
from Basilisk.simulation import (
    coarseSunSensor,
    reactionWheelStateEffector,
    spacecraft,
    thrusterDynamicEffector,
)
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    simIncludeRW,
    simIncludeThruster,
)


class _Publisher:
    def __init__(self):
        self.manifest = None
        self.frames = []

    def retain_hello(self, _message):
        pass

    def retain_manifest(self, message):
        self.manifest = message

    def publish_frame(self, message):
        self.frames.append(message)

    def publish_event(self, _message):
        return True

    def close(self):
        pass


class BasiliskDeviceIntegrationTests(unittest.TestCase):
    def test_real_rw_thruster_and_css_messages(self):
        simulation = SimulationBaseClass.SimBaseClass()
        process = simulation.CreateNewProcess("deviceProcess")
        process.addTask(simulation.CreateNewTask("deviceTask", macros.sec2nano(0.01)))

        vehicle = spacecraft.Spacecraft()
        vehicle.ModelTag = "deviceSat"
        simulation.AddModelToTask("deviceTask", vehicle)

        wheels = reactionWheelStateEffector.ReactionWheelStateEffector()
        wheel_factory = simIncludeRW.rwFactory()
        wheel_factory.create(
            "custom",
            [0.0, 0.0, 1.0],
            Omega=100.0,
            Omega_max=6000.0,
            Js=0.1,
            rWB_B=[0.1, 0.0, 0.0],
            u_max=0.2,
            label="RW1",
        )
        wheel_factory.addToSpacecraft("RW", wheels, vehicle)
        simulation.AddModelToTask("deviceTask", wheels)

        thrusters = thrusterDynamicEffector.ThrusterDynamicEffector()
        thruster_factory = simIncludeThruster.thrusterFactory()
        thruster_factory.create("MOOG_Monarc_1", [0.0, 0.0, -0.5], [0.0, 0.0, 1.0])
        thruster_factory.addToSpacecraft("THR", thrusters, vehicle)
        simulation.AddModelToTask("deviceTask", thrusters)
        command = messaging.THRArrayOnTimeCmdMsgPayload()
        command.OnTimeRequest = [0.2]
        command_message = messaging.THRArrayOnTimeCmdMsg().write(command)
        thrusters.cmdsInMsg.subscribeTo(command_message)

        css = coarseSunSensor.CoarseSunSensor()
        css.ModelTag = "CSS1"
        css.r_B = [0.0, 0.0, 0.5]
        css.nHat_B = [1.0, 0.0, 0.0]
        css.maxOutput = 2.0
        css.stateInMsg.subscribeTo(vehicle.scStateOutMsg)
        sun = messaging.SpicePlanetStateMsgPayload()
        sun.PositionVector = [1.0e11, 0.0, 0.0]
        # The SWIG reader does not own its publisher; retain it through the run.
        sun_message = messaging.SpicePlanetStateMsg().write(sun)
        css.sunInMsg.subscribeTo(sun_message)
        simulation.AddModelToTask("deviceTask", css)

        publisher = _Publisher()
        bridge = enableUnrealVisualization(
            simulation,
            "deviceTask",
            vehicle,
            rwEffectorList=wheels,
            thrEffectorList=thrusters,
            thrColors=[255, 255, 255, 255],
            cssList=[[css]],
            publisher=publisher,
            liveStream=False,
        )
        try:
            simulation.InitializeSimulation()
            simulation.ConfigureStopTime(macros.sec2nano(0.1))
            simulation.ExecuteSimulation()
        finally:
            bridge.close()

        self.assertEqual(
            {item["kind"] for item in publisher.manifest["visuals"]},
            {"reaction_wheel", "thruster", "css"},
        )
        states = {item["visual_id"]: item for item in publisher.frames[-1]["visual_states"]}
        self.assertGreater(states["deviceSat/reaction_wheel/0"]["channels"]["angle_rad"], 0.5)
        self.assertAlmostEqual(
            states["deviceSat/reaction_wheel/0"]["channels"]["omega_rad_s"],
            100.0 * macros.RPM,
            places=6,
        )
        self.assertGreater(states["deviceSat/thruster/0"]["channels"]["thrust_N"], 0.0)
        self.assertGreater(states["deviceSat/css/0"]["channels"]["signal"], 0.0)


if __name__ == "__main__":
    unittest.main()
