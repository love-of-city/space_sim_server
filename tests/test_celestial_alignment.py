from types import SimpleNamespace

import numpy as np
import pytest

from simulation.teleop_grasp_unreal import (
    _create_ephemeris_interface,
    _register_celestial_bodies,
    DEFAULT_EPHEMERIS_EPOCH_UTC,
    CELESTIAL_FIXED_FRAMES,
)


def test_ephemeris_translation_and_fixed_frames_are_not_conflated():
    class Factory:
        def createSpiceInterface(self, **kwargs):
            self.kwargs = kwargs
            return SimpleNamespace()

    factory = Factory()
    spice = _create_ephemeris_interface(factory, DEFAULT_EPHEMERIS_EPOCH_UTC)
    assert spice.referenceBase == "J2000"
    assert spice.zeroBase == "Earth"
    assert factory.kwargs == {
        "time": DEFAULT_EPHEMERIS_EPOCH_UTC,
        "spicePlanetFrames": ["IAU_EARTH", "IAU_SUN"],
        "epochInMsg": True,
    }


def test_native_spice_earth_spin_and_sun_share_the_simulation_clock():
    from Basilisk.utilities import SimulationBaseClass, macros, simIncludeGravBody

    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth()
    sun = factory.createSun()
    try:
        spice = _create_ephemeris_interface(factory, DEFAULT_EPHEMERIS_EPOCH_UTC)
        simulation = SimulationBaseClass.SimBaseClass()
        process = simulation.CreateNewProcess("ephemerisTestProcess")
        process.addTask(simulation.CreateNewTask("ephemerisTestTask", macros.sec2nano(3600)))
        simulation.AddModelToTask("ephemerisTestTask", spice)
        simulation.InitializeSimulation()
        simulation.ConfigureStopTime(0)
        simulation.ExecuteSimulation()
        before = np.asarray(earth.planetBodyInMsg().J20002Pfix).copy().reshape(3, 3)
        sun_before = np.asarray(sun.planetBodyInMsg().PositionVector).copy()
        simulation.ConfigureStopTime(macros.sec2nano(3600))
        simulation.ExecuteSimulation()
        after = np.asarray(earth.planetBodyInMsg().J20002Pfix).reshape(3, 3)
        sun_after = np.asarray(sun.planetBodyInMsg().PositionVector)
        assert list(spice.planetFrames) == list(CELESTIAL_FIXED_FRAMES)
        assert np.allclose(earth.planetBodyInMsg().PositionVector, 0)
        assert np.linalg.det(before) == pytest.approx(1.0)
        assert np.linalg.det(after) == pytest.approx(1.0)
        assert not np.allclose(before, np.eye(3))
        rotation = np.arccos(np.clip((np.trace(after @ before.T) - 1) / 2, -1, 1))
        assert 0.25 < rotation < 0.28  # About 15 degrees of Earth rotation in one hour.
        assert 1.4e11 < np.linalg.norm(sun_before) < 1.6e11
        assert np.linalg.norm(sun_after - sun_before) > 1e7
    finally:
        factory.unloadSpiceKernels()


def test_renderer_registers_the_same_earth_and_sun_objects_as_gravity():
    earth, sun = object(), object()
    calls = []
    bridge = SimpleNamespace(add_celestial_bodies=lambda bodies, **kwargs: calls.append((bodies, kwargs)))
    _register_celestial_bodies(bridge, earth, sun)
    assert len(calls) == 1
    bodies, kwargs = calls[0]
    assert bodies == [earth, sun]
    assert kwargs["visual_overrides"]["sun"]["drives_directional_light"] is True
    assert kwargs["visual_overrides"]["sun"]["luminous"] is True
