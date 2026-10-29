"""Offline native-BSK orbit acceptance, independent of UE and running scenes.

Two-body invariants, J2 nodal regression, and point-field analytic checks.
These validate the BSK environment/reference path, not long-term MuJoCo accuracy.
"""
from __future__ import annotations

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))


import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "backend")]

from Basilisk.architecture import messaging
from Basilisk.simulation import spacecraft
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, simIncludeGravBody

from simulation.orbital_configuration import configure_earth_gravity
from space_arm_platform.orbital_physics import OrbitalPhysics


def propagate(earth_gravity: str, *, dt: float = 1.0, periods: int = 2) -> dict:
    settings = OrbitalPhysics(earth_gravity=earth_gravity)
    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("orbitProcess")
    proc.addTask(sim.CreateNewTask("orbitTask", macros.sec2nano(dt)))
    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth()
    earth.isCentralBody = True
    field = configure_earth_gravity(earth, settings)
    # Fixed, aligned Earth pole isolates the analytical J2 comparison from SPICE.
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.J20002Pfix = np.eye(3).tolist()
    planet = messaging.SpicePlanetStateMsg().write(payload)
    earth.planetBodyInMsg.subscribeTo(planet)
    craft = spacecraft.Spacecraft()
    craft.ModelTag = "orbitAcceptance"
    craft.hub.mHub = 1.0
    factory.addBodiesTo(craft)
    elements = orbitalMotion.ClassicElements()
    elements.a = earth.radEquator + 500000.
    elements.e = 0.0
    elements.i = math.radians(51.6)
    elements.Omega, elements.omega, elements.f = 0., 0., 0.
    r0, v0 = orbitalMotion.elem2rv(earth.mu, elements)
    craft.hub.r_CN_NInit, craft.hub.v_CN_NInit = r0, v0
    sim.AddModelToTask("orbitTask", craft)
    recorder = craft.scStateOutMsg.recorder(macros.sec2nano(max(dt, 10.0)))
    sim.AddModelToTask("orbitTask", recorder)
    sim.InitializeSimulation()
    period = 2 * math.pi * math.sqrt(elements.a**3 / earth.mu)
    sim.ConfigureStopTime(macros.sec2nano(periods * period))
    sim.ExecuteSimulation()
    r, v = np.asarray(recorder.r_BN_N), np.asarray(recorder.v_BN_N)
    energy = .5 * np.sum(v*v, axis=1) - earth.mu / np.linalg.norm(r, axis=1)
    total_energy = energy.copy()
    if earth_gravity == "j2":
        radius = np.linalg.norm(r, axis=1)
        total_energy += (earth.mu / radius * settings.j2_coefficient * (earth.radEquator/radius)**2
                         * .5 * (3*(r[:, 2]/radius)**2 - 1))
    h = np.cross(r, v)
    h_norm = np.linalg.norm(h, axis=1)
    raan = np.unwrap(np.arctan2(h[:, 0], -h[:, 1]))
    elapsed = float((recorder.times()[-1] - recorder.times()[0]) * 1e-9)
    rate = float((raan[-1] - raan[0]) / elapsed)
    predicted = -1.5 * math.sqrt(earth.mu / elements.a**3) * settings.j2_coefficient * (
        earth.radEquator / elements.a)**2 * math.cos(elements.i)
    result = {
        "earth_gravity": earth_gravity, "dt_s": dt, "elapsed_s": elapsed,
        "periods_requested": periods, "raan_rate_rad_s": rate,
        "j2_secular_rate_rad_s": predicted,
        "max_two_body_energy_relative_drift": float(np.max(np.abs(energy/energy[0] - 1))),
        "max_angular_momentum_norm_relative_drift": float(np.max(np.abs(h_norm/h_norm[0] - 1))),
        "max_model_energy_relative_drift": float(np.max(np.abs(total_energy/total_energy[0] - 1))),
        "max_polar_angular_momentum_relative_drift": float(np.max(np.abs(h[:, 2]/h[0, 2] - 1))),
    }
    assert result["max_model_energy_relative_drift"] < 1e-10, result
    assert result["max_polar_angular_momentum_relative_drift"] < 1e-10, result
    if earth_gravity == "point_mass":
        assert result["max_two_body_energy_relative_drift"] < 1e-10, result
        assert result["max_angular_momentum_norm_relative_drift"] < 1e-10, result
        assert abs(rate) < 1e-12, result
    else:
        # Two-body energy/|h| are NOT invariants of the J2 field. Check the
        # secular node rate, allowing short-period terms and first-order theory.
        result["raan_relative_error"] = abs(rate/predicted - 1)
        assert result["raan_relative_error"] < .02, result
    return result


def check_fields() -> dict:
    settings = OrbitalPhysics()
    factory = simIncludeGravBody.gravBodyFactory()
    earth = factory.createEarth()
    field = configure_earth_gravity(earth, settings)
    earth.initBody(0)
    rng = np.random.default_rng(123)
    directions = rng.normal(size=(200, 3))
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    positions = directions * (earth.radEquator + rng.uniform(400e3, 600e3, 200))[:, None]
    max_error = 0.
    for r in positions:
        radius = np.linalg.norm(r)
        z2 = (r[2]/radius)**2
        j2 = (1.5 * settings.j2_coefficient * earth.mu * earth.radEquator**2 / radius**5
              * np.array([r[0]*(5*z2-1), r[1]*(5*z2-1), r[2]*(5*z2-3)]))
        expected = -earth.mu*r/radius**3 + j2
        max_error = max(max_error, float(np.linalg.norm(np.asarray(field.computeField(r)).reshape(3)-expected)))
    assert max_error < 1e-12
    return {"samples": len(positions), "max_acceleration_error_m_s2": max_error,
            "j2": settings.j2_coefficient, "coefficient_source": settings.coefficient_source}


def verify(*, dt=1., periods=2):
    return {"scope": "Native BSK static field and isolated 500 km, 51.6 degree reference orbits",
            "field": check_fields(),
            "orbits": [propagate(mode, dt=dt, periods=periods) for mode in ("point_mass", "j2")]}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--dt", type=float, default=1.)
    parser.add_argument("--periods", type=int, default=2)
    args = parser.parse_args()
    if not math.isfinite(args.dt) or not 0 < args.dt <= 2 or not 1 <= args.periods <= 10:
        parser.error("dt must be in (0, 2] s and periods in [1, 10]")
    result = verify(dt=args.dt, periods=args.periods)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
