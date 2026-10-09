"""Atmospheric drag: the first component added through the documented extension path.

Checks the force law and the two properties the extension path is supposed to give
for free: an ENVIRONMENT-slot rate that is phase-aligned with state publication, and
an external wrench that coexists with another source on the same body.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("Basilisk")
from Basilisk.architecture import messaging, sysModel  # noqa: E402
from Basilisk.utilities import SimulationBaseClass, macros  # noqa: E402

from simulation.assembly import AssemblyContext, SessionConfig, Slot  # noqa: E402
from simulation.components.drag import (  # noqa: E402
    AtmosphericDragComponent, DRAG_EVERY, PlateDrag, atmosphere_velocity_n, drag_force_n,
)
from simulation.local_mujoco_stepper import LocalMujocoStepper, servo_specs  # noqa: E402
from simulation.physics_ports import PhysicsPorts, PortError  # noqa: E402

DT = 1.0 / 240.0
MJCF = """<mujoco model="drag_test">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="bus" pos="0 0 0">
      <freejoint name="bus_free"/>
      <geom name="bus_geom" type="box" size=".2 .2 .2" mass="160"/>
    </body>
  </worldbody>
</mujoco>
"""


def test_drag_force_is_opposite_the_relative_wind():
    force = drag_force_n(1e-11, [7000.0, 0.0, 0.0], 0.06, 2.2)
    expected = -0.5 * 1e-11 * 2.2 * 0.06 * 7000.0 * 7000.0
    assert force[0] == pytest.approx(expected, rel=1e-12)
    assert force[1] == 0.0 and force[2] == 0.0
    assert np.allclose(drag_force_n(1e-11, [0.0, 0.0, 0.0], 0.06, 2.2), 0.0)


def test_atmosphere_velocity_is_the_corotating_flow():
    position = np.array([6.9e6, 0.0, 0.0])
    velocity = atmosphere_velocity_n(position)
    assert velocity[1] == pytest.approx(6.9e6 * 7.2921150e-5, rel=1e-12)
    assert velocity[0] == 0.0 and velocity[2] == 0.0


def test_drag_rejects_nonsense_parameters():
    density = messaging.AtmoPropsMsg()
    state = messaging.SCStatesMsg()
    with pytest.raises(ValueError, match="area"):
        PlateDrag(density, state, area_m2=0.0, drag_coefficient=2.2)
    with pytest.raises(ValueError, match="coefficient"):
        PlateDrag(density, state, area_m2=0.1, drag_coefficient=-1.0)


def test_plate_drag_publishes_both_force_and_offset_torque():
    density, state = messaging.AtmoPropsMsg(), messaging.SCStatesMsg()
    drag = PlateDrag(density, state, area_m2=0.06, drag_coefficient=2.2,
                     center_of_pressure_body_m=(0.1, 0.0, 0.0))
    density.write(messaging.AtmoPropsMsgPayload(neutralDensity=1e-11))
    state.write(messaging.SCStatesMsgPayload(
        r_BN_N=[6.9e6, 0.0, 0.0], v_BN_N=[0.0, 7600.0, 0.0], sigma_BN=[0.0, 0.0, 0.0]))
    drag.UpdateState(0)
    # Relative wind is the orbital motion minus the co-rotating atmosphere.
    relative_y = 7600.0 - 6.9e6 * 7.2921150e-5
    assert drag.last_force_n[1] == pytest.approx(
        -0.5 * 1e-11 * 2.2 * 0.06 * relative_y * abs(relative_y), rel=1e-9)
    # r_cp x F with both along y, so the torque is about z.
    assert drag.last_torque_nm[2] == pytest.approx(0.1 * drag.last_force_n[1], rel=1e-12)
    assert drag.forceOutMsg.read().forceRequestInertial[1] == pytest.approx(
        drag.last_force_n[1], rel=1e-12)


@pytest.fixture
def model_path(tmp_path: Path) -> Path:
    path = tmp_path / "drag_test.xml"
    path.write_text(MJCF, encoding="utf-8")
    return path


def build(model_path: Path, *, publish_stride: int = 2):
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("graspProcess")
    process.addTask(simulation.CreateNewTask("graspTask", macros.sec2nano(DT)))
    from Basilisk.simulation import mujoco

    scene = mujoco.MJScene.fromFile(str(model_path))
    process.addTask(simulation.CreateNewTask("scene_init", 10**18))
    simulation.AddModelToTask("scene_init", scene)
    reference = messaging.ScalarJointStateMsg()
    stepper = LocalMujocoStepper(scene, model_path, [], publish_every=publish_stride)
    simulation.AddModelToTask("graspTask", stepper, int(Slot.PHYSICS))
    simulation.local_stepper = stepper
    ports = PhysicsPorts(scene, stepper=stepper)
    ctx = AssemblyContext(
        simulation, scene,
        config=SessionConfig(dynamics_backend="local", publish_stride=publish_stride,
                             physics_task="graspTask"),
        physics_ports=ports)
    return simulation, scene, stepper, ctx


def test_environment_component_places_a_divided_phase_aligned_module(model_path):
    simulation, scene, stepper, ctx = build(model_path)
    component = AtmosphericDragComponent("bus")
    # The planet-state port needs a provider; this scene has no ephemeris, so give
    # the component a stub that satisfies both read ports.
    ports = ctx.ports
    ports.set_ephemeris(type("E", (), {"planetStateOutMsgs": [messaging.SpicePlanetStateMsg(),
                                                             messaging.SpicePlanetStateMsg()]})())
    ctx.install([component])
    entry = [item for item in ctx.describe() if item["component"] == "atmospheric_drag"]
    assert [item["module"] for item in entry] == ["exponentialAtmosphere", "plateDrag"]
    assert all(item["every"] == DRAG_EVERY for item in entry)
    # A multiple of the publication stride, so the drag never reads a stale body.
    assert DRAG_EVERY % ctx.config.publish_stride == 0
    assert all(item["phase"] == 1 for item in entry)
    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(0.5))
    simulation.ExecuteSimulation()
    assert component.drag.last_force_n is not None


def test_drag_coexists_with_a_second_wrench_source_on_one_body(model_path):
    simulation, scene, stepper, ctx = build(model_path)
    ports = ctx.ports
    other = messaging.CmdForceInertialMsg()
    other.write(messaging.CmdForceInertialMsgPayload(forceRequestInertial=[0.0, 1.0, 0.0]))
    ports.add_body_wrench("bus", source="thruster", force_inertial=other)
    ports.set_ephemeris(type("E", (), {"planetStateOutMsgs": [messaging.SpicePlanetStateMsg(),
                                                             messaging.SpicePlanetStateMsg()]})())
    AtmosphericDragComponent("bus").install(ctx)
    kept = [x for x in ctx._kept]
    assert kept  # both components retained
    # Two sources on one body are summed, not rejected.
    assert len(stepper._wrench_channels) == 2
    assert {channel.source for channel in stepper._wrench_channels} == {"thruster", "atmospheric_drag"}


def test_drag_refuses_the_basilisk_backend():
    scene = type("Scene", (), {})()
    ctx = AssemblyContext(
        SimulationBaseClass.SimBaseClass(), scene,
        config=SessionConfig(dynamics_backend="basilisk"), physics_ports=PhysicsPorts(scene))
    with pytest.raises(RuntimeError, match="local-backend component"):
        AtmosphericDragComponent("bus").install(ctx)


def test_drag_refuses_an_unknown_body(model_path):
    simulation, scene, stepper, ctx = build(model_path)
    ctx.ports.set_ephemeris(type("E", (), {
        "planetStateOutMsgs": [messaging.SpicePlanetStateMsg(), messaging.SpicePlanetStateMsg()]})())
    with pytest.raises(RuntimeError, match="not in this scene"):
        AtmosphericDragComponent("no_such_body").install(ctx)


_ = (sysModel, servo_specs, PortError)
