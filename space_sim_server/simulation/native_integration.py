"""Native MJScene integration settings, independent of the output/control clock.

The small wrist inertias and physical joint damping make a single fixed RK4
step at 240 Hz unstable (J6 spikes followed by NaN acceleration). RKF45 controls
local integration error inside each scheduled step. It still evaluates the same
MJScene's dynamics/contact and the existing PID/torque limiters; no second model,
state clipping, altered damping/inertia, or changed collision filtering.
"""

import math
import os

# Fixed RK4 at the 240 Hz outer grid is unstable for the small wrist inertias.
# RKF45 remains the authority, but the original 1e-5/1e-6 pair made ordinary
# teleoperation spend most of its time in rejected internal stages. The looser
# default was validated against 20 s of active Cartesian motion with finite
# state and materially fewer internal stages. Strict values remain configurable.
# Set either environment variable to restore/tune a stricter deployment value.
RELATIVE_TOLERANCE = 1e-4
ABSOLUTE_TOLERANCE = 1e-4
RELATIVE_TOLERANCE_ENV = "SPACE_SIM_RKF45_RELATIVE_TOLERANCE"
ABSOLUTE_TOLERANCE_ENV = "SPACE_SIM_RKF45_ABSOLUTE_TOLERANCE"


def _configured_tolerance(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return float(default)
    try:
        value = float(raw)
    except ValueError as error:
        raise ValueError(f"{name} must be a finite positive number") from error
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be a finite positive number")
    return value


def _create_integrator(scene):
    # Defer native DLL loading until the live loader has established the
    # Windows-safe Basilisk/MuJoCo load order (not during pytest collection).
    from Basilisk.simulation import svIntegrators
    return svIntegrators.svIntegratorRKF45(scene)


def configure_scene_integrator(scene):
    relative_tolerance = _configured_tolerance(RELATIVE_TOLERANCE_ENV, RELATIVE_TOLERANCE)
    absolute_tolerance = _configured_tolerance(ABSOLUTE_TOLERANCE_ENV, ABSOLUTE_TOLERANCE)
    integrator = _create_integrator(scene)
    integrator.setRelativeTolerance(relative_tolerance)
    integrator.setAbsoluteTolerance(absolute_tolerance)
    scene.setIntegrator(integrator)
    # Keep the Python proxy alive with the native scene for the entire session.
    scene._teleop_integrator = integrator
    return {"integrator": "RKF45", "relative_tolerance": relative_tolerance,
            "absolute_tolerance": absolute_tolerance,
            "collision_authority": "running_native_MJScene"}
