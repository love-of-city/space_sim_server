"""Use SpaceSim's celestial assets and lighting with the flight's J2000 states."""

from .reference import EARTH_RADIUS, MU


def configure_celestials(simulation, bridge, gravity):
    from Basilisk.utilities import simIncludeGravBody
    from simulation.teleop_grasp_unreal import (
        DEFAULT_EPHEMERIS_EPOCH_UTC,
        _create_ephemeris_interface,
        _register_celestial_bodies,
    )

    factory = simIncludeGravBody.gravBodyFactory()
    earth, sun = factory.createEarth(), factory.createSun()
    earth.mu, earth.radEquator = MU, EARTH_RADIUS
    ephemeris = _create_ephemeris_interface(factory, DEFAULT_EPHEMERIS_EPOCH_UTC)
    # The existing central Earth remains the only gravity source. Sun is
    # registered for SpaceSim's illumination and display, not as a new force.
    gravity.getGravitySource("Earth").stateInMsg.subscribeTo(ephemeris.planetStateOutMsgs[0])
    simulation.AddModelToTask("flightNavigation", ephemeris, 100)
    _register_celestial_bodies(bridge, earth, sun)
    return (factory, earth, sun, ephemeris), DEFAULT_EPHEMERIS_EPOCH_UTC


def flight_scene_settings(sunlight_intensity_scale=1.0, bodies=None):
    from bsk_render_adapter.descriptors import RelativeView, SceneSettings

    bodies = bodies or {}

    return SceneSettings(
        origin_object_id="team/cubesat_bus",
        default_camera_target="team/cubesat_bus",
        default_camera_distance_m=2.8,
        sunlight_intensity_scale=sunlight_intensity_scale,
        orbit_lines=False,
        trajectory_history=False,
        relative_view=RelativeView(
            primary_object_id="team/cubesat_bus", secondary_object_id="team/capture_target",
            primary_center_body_m=bodies.get("cubesat_bus", {}).get("center_body_m", (0, 0, 0)),
            secondary_center_body_m=bodies.get("capture_target", {}).get("center_body_m", (0, 0, 0)),
        ),
    )
