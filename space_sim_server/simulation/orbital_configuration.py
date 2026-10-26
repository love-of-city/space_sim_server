"""Configure native BSK gravity and resolve reproducible scene settings."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from space_arm_platform.orbital_physics import OrbitalPhysics


def resolve_orbital_physics(scene_instance, *, override=None, backend="local", mode="bsk",
                            require_drag=False) -> OrbitalPhysics:
    if override is not None:
        settings = OrbitalPhysics.model_validate_json(Path(override).read_text(encoding="utf-8-sig"))
    elif scene_instance is not None:
        environment = scene_instance.get("environment", {})
        settings = (OrbitalPhysics.model_validate(environment["physics"])
                    if "physics" in environment else OrbitalPhysics.legacy())
    else:
        settings = OrbitalPhysics() if backend == "local" and mode == "bsk" else OrbitalPhysics.legacy()
    if scene_instance is not None and scene_instance.get("rendezvous_handoff") is not None:
        from space_arm_platform.rendezvous_config import FlightOptions
        inherited = FlightOptions.from_dict(scene_instance["rendezvous_handoff"].get("mission_options", {})).physics
        if settings != inherited:
            raise ValueError("handoff must preserve the flight orbital physics configuration; an override cannot change it")
    if (settings.earth_gravity != "point_mass" or settings.drag is not None) and (
        backend != "local" or mode != "bsk"
    ):
        raise ValueError("J2/configured drag requires local + bsk; use an explicit point_mass, drag=null baseline for legacy backends")
    if require_drag and settings.drag is None:
        raise ValueError("--atmospheric-drag requires explicit density and body parameters in environment.physics.drag or --orbital-config")
    return settings


def configure_earth_gravity(earth, settings: OrbitalPhysics):
    """Configure the shared GravBodyData before either propagation path starts.

    BSK's normalized C20 = -J2/sqrt(5); the model already contains point mass.
    Its input is Earth-fixed, supplied by the common SPICE orientation.
    """
    from Basilisk.simulation import pointMassGravityModel, sphericalHarmonicsGravityModel

    if settings.earth_gravity == "point_mass":
        model = pointMassGravityModel.PointMassGravityModel()
        model.muBody = float(earth.mu)
    else:
        model = sphericalHarmonicsGravityModel.SphericalHarmonicsGravityModel()
        model.maxDeg = 2
        model.muBody = float(earth.mu)
        model.radEquator = float(earth.radEquator)
        model.cBar = [[1.0], [0.0, 0.0], [-settings.j2_coefficient / np.sqrt(5.0), 0.0, 0.0]]
        model.sBar = [[0.0], [0.0, 0.0], [0.0, 0.0, 0.0]]
    earth.gravityModel = model
    return model


def environment_metadata(settings: OrbitalPhysics, earth, *, epoch_utc: str) -> dict:
    return {
        "configuration": settings.model_dump(mode="json"),
        "epoch_utc": epoch_utc, "center": "Earth", "frame": "J2000",
        "local_axes": "J2000_parallel_translating", "sources": ["earth", "sun"],
        "earth_mu_m3_s2": float(earth.mu), "earth_radius_m": float(earth.radEquator),
        "earth_fixed_frame": "IAU_EARTH",
        "c20_normalized": (-settings.j2_coefficient / np.sqrt(5.0)
                           if settings.earth_gravity == "j2" else 0.0),
        "gravity_gradient_torque": False,
    }
