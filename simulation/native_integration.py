"""Native MJScene integration settings, independent of the output/control clock.

The small wrist inertias and physical joint damping make a single fixed RK4
step at 240 Hz unstable (J6 spikes followed by NaN acceleration). RKF45 controls
local integration error inside each scheduled step. It still evaluates the same
MJScene's dynamics/contact and the existing PID/torque limiters; no second model,
state clipping, altered damping/inertia, or changed collision filtering.
"""

RELATIVE_TOLERANCE = 1e-5
ABSOLUTE_TOLERANCE = 1e-6


def _create_integrator(scene):
    # Defer native DLL loading until the live loader has established the
    # Windows-safe Basilisk/MuJoCo load order (not during pytest collection).
    from Basilisk.simulation import svIntegrators
    return svIntegrators.svIntegratorRKF45(scene)


def configure_scene_integrator(scene):
    integrator = _create_integrator(scene)
    integrator.setRelativeTolerance(RELATIVE_TOLERANCE)
    integrator.setAbsoluteTolerance(ABSOLUTE_TOLERANCE)
    scene.setIntegrator(integrator)
    # Keep the Python proxy alive with the native scene for the entire session.
    scene._teleop_integrator = integrator
    return {"integrator": "RKF45", "relative_tolerance": RELATIVE_TOLERANCE,
            "absolute_tolerance": ABSOLUTE_TOLERANCE,
            "collision_authority": "running_native_MJScene"}
