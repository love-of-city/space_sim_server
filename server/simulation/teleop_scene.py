"""Saved scene validation and authoritative orbital initial conditions."""
from __future__ import annotations
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import numpy as np
from space_arm_platform.lighting import DEFAULT_SUNLIGHT_INTENSITY_SCALE, validate_sunlight_intensity_scale
from space_arm_platform.scene_targets import capture_target
from space_arm_platform.orbital_physics import OrbitalPhysics
from space_arm_platform.control_defaults import DEFAULT_OPERATING_JOINT_DEG, AUTO_PREPARE_TELEOP_PROFILE
from space_arm_platform.joint_limits import load_joint_limits
from space_arm_platform.control_defaults import MIN_DYNAMICS_STEP_S, MAX_DYNAMICS_STEP_S
PROJECT_ROOT = Path(__file__).resolve().parents[1]



def validate_dynamics_step(value: float) -> float:
    step = float(value)
    if not math.isfinite(step) or not MIN_DYNAMICS_STEP_S <= step <= MAX_DYNAMICS_STEP_S:
        raise ValueError(f"dynamics_step_s must be in [{MIN_DYNAMICS_STEP_S}, {MAX_DYNAMICS_STEP_S}]")
    return step

DEFAULT_EPHEMERIS_EPOCH_UTC = "2026 SEPTEMBER 02 00:00:00.000"

SUPPORTED_EPHEMERIS_CENTER = "Earth"

SUPPORTED_EPHEMERIS_FRAME = "J2000"

CELESTIAL_FIXED_FRAMES = ("IAU_EARTH", "IAU_SUN")

SUN_REFERENCE_DISTANCE_M = 149_597_870_700.0

EARTH_EQUATORIAL_RADIUS_M = 6_378_136.6

MINIMUM_PERIAPSIS_ALTITUDE_M = 120_000.0

DEFAULT_ORBIT = {
    "altitude_m": 500_000.0,
    "eccentricity": 0.0,
    "inclination_deg": 51.6,
    "raan_deg": 0.0,
    "argument_of_periapsis_deg": 0.0,
    "true_anomaly_deg": 180.0,
}


def load_scene_instance(path: Path | None, model_root: Path | None = None) -> dict[str, Any] | None:
    """Load and validate one reproducible scene-instance document."""

    if path is None:
        return None
    resolved = path.resolve()
    try:
        document = json.loads(resolved.read_text(encoding="utf-8-sig"))
    except (OSError, ValueError) as error:
        raise ValueError(f"unable to read scene instance {resolved}: {error}") from error
    if document.get("schema") != "space-arm-scene-instance/1":
        raise ValueError(f"unsupported scene instance schema: {document.get('schema')}")
    mode = document.setdefault("wrist_camera_mode", "rgb")
    if mode not in ("rgb", "fisheye", "monochrome", "monochrome_fisheye"):
        raise ValueError("wrist_camera_mode must be rgb, fisheye, monochrome or monochrome_fisheye")
    target = capture_target(document.get("template_id"))
    randomize_orbit_phase = document.get("randomize_orbit_phase", False)
    if not isinstance(randomize_orbit_phase, bool):
        raise ValueError("scene randomize_orbit_phase must be a boolean")
    document["randomize_orbit_phase"] = randomize_orbit_phase
    from space_arm_platform.overview_camera_records import validate_overview_camera
    validate_overview_camera(document)
    runtime = document.get("runtime")
    randomization = document.get("randomization")
    environment = document.get("environment", {})
    if not isinstance(runtime, dict) or not isinstance(randomization, dict):
        raise ValueError("scene instance requires runtime and randomization objects")
    if not isinstance(environment, dict):
        raise ValueError("scene instance environment must be an object")
    epoch = str(environment.get("ephemeris_epoch_utc", DEFAULT_EPHEMERIS_EPOCH_UTC)).strip()
    center = str(environment.get("ephemeris_center", SUPPORTED_EPHEMERIS_CENTER)).strip()
    frame = str(environment.get("ephemeris_frame", SUPPORTED_EPHEMERIS_FRAME)).strip()
    if not epoch:
        raise ValueError("scene environment.ephemeris_epoch_utc must not be empty")
    if center.casefold() != SUPPORTED_EPHEMERIS_CENTER.casefold():
        raise ValueError("scene environment.ephemeris_center must be Earth")
    if frame.casefold() != SUPPORTED_EPHEMERIS_FRAME.casefold():
        raise ValueError("scene environment.ephemeris_frame must be J2000")
    orbit = environment.get("orbit", DEFAULT_ORBIT)
    if not isinstance(orbit, dict):
        raise ValueError("scene environment.orbit must be an object")
    if randomize_orbit_phase and ("orbit" not in environment or "true_anomaly_deg" not in orbit):
        raise ValueError("randomized scene requires a saved environment.orbit.true_anomaly_deg")
    # Replay the stored angle, never resample from Seed during loading.
    normalized_orbit: dict[str, float] = {}
    for field, default in DEFAULT_ORBIT.items():
        try:
            value = float(orbit.get(field, default))
        except (TypeError, ValueError) as error:
            raise ValueError(f"scene environment.orbit.{field} must be a number") from error
        if not math.isfinite(value):
            raise ValueError(f"scene environment.orbit.{field} must be finite")
        normalized_orbit[field] = value
    eccentricity = normalized_orbit["eccentricity"]
    inclination_deg = normalized_orbit["inclination_deg"]
    altitude_m = normalized_orbit["altitude_m"]
    if altitude_m <= 0.0:
        raise ValueError("scene environment.orbit.altitude_m must be positive")
    if eccentricity < 0.0 or eccentricity >= 1.0:
        raise ValueError("scene environment.orbit.eccentricity must be in [0, 1)")
    if inclination_deg < 0.0 or inclination_deg > 180.0:
        raise ValueError("scene environment.orbit.inclination_deg must be in [0, 180]")
    periapsis_altitude_m = (
        (EARTH_EQUATORIAL_RADIUS_M + altitude_m) * (1.0 - eccentricity)
        - EARTH_EQUATORIAL_RADIUS_M
    )
    if periapsis_altitude_m < MINIMUM_PERIAPSIS_ALTITUDE_M:
        raise ValueError(
            "scene environment.orbit periapsis altitude must be at least "
            f"{MINIMUM_PERIAPSIS_ALTITUDE_M:.0f} m"
        )
    lighting = environment.get("lighting", {})
    if not isinstance(lighting, dict):
        raise ValueError("scene environment.lighting must be an object")
    sunlight_scale = validate_sunlight_intensity_scale(
        lighting.get("sunlight_intensity_scale", DEFAULT_SUNLIGHT_INTENSITY_SCALE)
    )
    document["environment"] = {
        "ephemeris_epoch_utc": epoch,
        "ephemeris_center": SUPPORTED_EPHEMERIS_CENTER,
        "ephemeris_frame": SUPPORTED_EPHEMERIS_FRAME,
        "orbit": normalized_orbit,
        "physics": (OrbitalPhysics.model_validate(environment["physics"])
                    if "physics" in environment else OrbitalPhysics.legacy()).model_dump(mode="json"),
        "lighting": {"sunlight_intensity_scale": sunlight_scale},
    }
    runtime_limits = {
        "simulation_rate": (0.0, 100.0),
        "capture_rate_hz": (0.0, 60.0),
        "ik_rate_hz": (0.0, 400.0),
    }
    for field, (minimum, maximum) in runtime_limits.items():
        try:
            value = float(runtime[field])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"scene runtime.{field} must be a number") from error
        if not math.isfinite(value) or value <= minimum or value > maximum:
            raise ValueError(
                f"scene runtime.{field} must be in ({minimum}, {maximum}]"
            )
        runtime[field] = value
    expected_lengths = {
        "target_position_m": 3,
        "target_orientation_wxyz": 4,
        "target_linear_velocity_m_s": 3,
        "target_angular_velocity_rad_s": 3,
        "arm_joint_position_rad": 8,
    }
    if "target_relative_velocity_m_s" in randomization:
        expected_lengths["target_relative_velocity_m_s"] = 3
    for field, length in expected_lengths.items():
        values = randomization.get(field)
        if not isinstance(values, list) or len(values) != length:
            raise ValueError(f"scene randomization.{field} must contain {length} numbers")
        try:
            converted = [float(value) for value in values]
        except (TypeError, ValueError) as error:
            raise ValueError(f"scene randomization.{field} must contain only numbers") from error
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f"scene randomization.{field} must contain finite numbers")
        randomization[field] = converted
    quaternion_norm = float(np.linalg.norm(randomization["target_orientation_wxyz"]))
    if quaternion_norm <= 1.0e-12:
        raise ValueError("scene target quaternion must have non-zero norm")
    if (randomization.get('capture_window_trial') or randomization.get('capture_visual_test', {}).get('enabled')
            or randomization.get('formation_hold', {}).get('enabled')):
        raise ValueError('Experimental autonomous capture/formation controllers are not enabled in this runtime')
    joints = np.asarray(randomization["arm_joint_position_rad"], dtype=float)
    from space_arm_platform.imported_scene_library import model_for_instance
    scene_model = model_for_instance(document) or (target.resolve_model(model_root) if model_root is not None else PROJECT_ROOT / target.runtime_model)
    from simulation.sarm_model_profile import load_model_profile
    scene_limits = load_joint_limits(scene_model, load_model_profile(scene_model).joint_names)
    if np.any(joints < np.array(scene_limits.lower)) or np.any(joints > np.array(scene_limits.upper)):
        raise ValueError("scene arm joint positions exceed the SARM joint limits")
    if document.get("arm_preparation_required", False):
        # Backward compatibility for old saved zero-start scenes. New scenes
        # initialize directly at their operating pose and set this flag false.
        if not np.allclose(joints[:6], 0., atol=1e-12, rtol=0):
            raise ValueError("legacy preparation scene must initialize six arm joints at zero")
        goal = np.deg2rad(np.asarray(document.get("operating_arm_joint_position_deg", DEFAULT_OPERATING_JOINT_DEG), dtype=float))
        if goal.shape != (6,) or not np.all(np.isfinite(goal)) or np.any(goal < np.asarray(scene_limits.lower[:6])) or np.any(goal > np.asarray(scene_limits.upper[:6])):
            raise ValueError("invalid saved operating joint angles")
    if document.get("randomization_profile") == AUTO_PREPARE_TELEOP_PROFILE:
        from simulation.preparation_contract import validate_preparation_plan
        if document.get("arm_preparation_required") is not True:
            raise ValueError("automatic preparation cannot bypass the ready gate")
        plan = document.get("arm_preparation_plan")
        validate_preparation_plan(plan, document["operating_arm_joint_position_deg"])
        if plan.get("model_sha256") != hashlib.sha256(scene_model.read_bytes()).hexdigest():
            raise ValueError("preparation model changed; recreate the scene and validate its path")
    if target.hinge_joint:
        angle = randomization.get("target_hinge_position_rad", 0.0)
        if isinstance(angle, bool) or not isinstance(angle, (int, float)) or not math.isfinite(angle) or angle != 0.0:
            raise ValueError("target_hinge_position_rad must be 0 at startup; only the CAD zero pose has validated clearance")
        randomization["target_hinge_position_rad"] = 0.0
    return document

def create_ephemeris_interface(gravity_factory: Any, epoch_utc: str) -> Any:
    """Use one clock/frame for gravity and rendering, with physical planet spin."""

    ephemeris = gravity_factory.createSpiceInterface(
        time=epoch_utc,
        spicePlanetFrames=list(CELESTIAL_FIXED_FRAMES),
        epochInMsg=True,
    )
    ephemeris.referenceBase = SUPPORTED_EPHEMERIS_FRAME
    ephemeris.zeroBase = SUPPORTED_EPHEMERIS_CENTER
    return ephemeris

def register_celestial_bodies(bridge: Any, earth: Any, sun: Any) -> None:
    """Share the gravity-bound body readers with the renderer; no backdrop Earth."""

    # Render the same Earth/Sun state messages used by native gravity.
    # The bridge subtracts the moving spacecraft origin for BOTH bodies.
    bridge.add_celestial_bodies(
        [earth, sun],
        visual_overrides={
            "sun": {
                "visual_role": "star",
                "luminous": True,
                "drives_directional_light": True,
                "light_color_rgb": (1.0, 0.8876306, 0.8572326),
                # Experimental solar illuminance baseline at reference distance; UE applies 1/r^2.
                "light_illuminance_lux_at_reference_distance": 132821.5,
                "light_reference_distance_m": SUN_REFERENCE_DISTANCE_M,
            }
        },
    )

def apply_orbital_initial_state(
    scene: Any,
    native: Any,
    earth: Any,
    orbit: dict[str, float],
    randomized: dict[str, Any],
    initial_joints: np.ndarray,
    orbit_origin: Any | None = None,
    local_stepper: Any | None = None,
) -> dict[str, Any]:
    """Apply one saved orbital phase to the authoritative MJScene bodies.

    The render bridge reads these same bodies. Local grasp offsets, joint
    states and the legacy common drift are retained, while both free bodies
    receive the orbital position AND velocity for the selected phase.
    With the local dynamics backend, ``orbit_origin`` (the Basilisk point that
    carries the orbit) starts on the same orbital state.
    """
    from Basilisk.utilities import orbitalMotion

    elements = orbitalMotion.ClassicElements()
    elements.a = float(earth.radEquator) + float(orbit["altitude_m"])
    elements.e = float(orbit["eccentricity"])
    elements.i = math.radians(float(orbit["inclination_deg"]))
    elements.Omega = math.radians(float(orbit["raan_deg"]))
    elements.omega = math.radians(float(orbit["argument_of_periapsis_deg"]))
    elements.f = math.radians(float(orbit["true_anomaly_deg"]))
    orbital_position, orbital_velocity = orbitalMotion.elem2rv(float(earth.mu), elements)
    orbital_position = np.asarray(orbital_position, dtype=float)
    orbital_velocity = np.asarray(orbital_velocity, dtype=float)
    common_velocity = np.asarray(native.COMMON_VELOCITY, dtype=float)
    bus_velocity = orbital_velocity + common_velocity
    if orbit_origin is not None:
        orbit_origin.set_state(orbital_position, orbital_velocity)
    bus = scene.getBody("cubesat_bus")
    target = scene.getBody("capture_target")
    bus.setPosition(orbital_position)
    bus.setVelocity(bus_velocity)
    if 'target_relative_velocity_m_s' in randomized:
        bus.setAttitudeRate(np.zeros(3))
    from space_arm_platform.task_box import initialize_free_plugs
    initialize_free_plugs(scene, native.MODEL_PATH, orbital_position, bus_velocity)
    target.setPosition(orbital_position + np.asarray(randomized["target_position_m"], dtype=float))
    # Both free bodies use the same inertial frame.  The randomized
    # target velocity is only a local offset, not its full orbital speed.
    target_velocity = (bus_velocity + np.asarray(randomized["target_relative_velocity_m_s"], dtype=float)
                       if "target_relative_velocity_m_s" in randomized else
                       orbital_velocity + np.asarray(randomized["target_linear_velocity_m_s"], dtype=float))
    target.setVelocity(target_velocity)
    if "target_relative_velocity_m_s" in randomized:
        print(json.dumps({"type": "target_relative_initial_state",
                          "position_m": randomized["target_position_m"],
                          "relative_velocity_m_s": (target_velocity-bus_velocity).tolist()}, sort_keys=True), flush=True)
    target.setAttitude(
        native._quaternion_to_mrp(
            np.asarray(randomized["target_orientation_wxyz"], dtype=float)
        )
    )
    target_spin = np.asarray(randomized["target_angular_velocity_rad_s"], dtype=float)
    if float(np.linalg.norm(target_spin)) > 1.0e-12:
        # Older scene files may still contain the pre-fix spin value.
        # Normalize it so they cannot reintroduce the MJScene startup NaN.
        target_spin = np.zeros(3)
    target.setAttitudeRate(target_spin)
    for (body_name, joint_name), position in zip(native.JOINTS, initial_joints, strict=True):
        joint = scene.getBody(body_name).getScalarJoint(joint_name)
        joint.setPosition(float(position))
        joint.setVelocity(0.0)

    if native.MODEL_PATH.name == 'sarm_dual_capture26.xml':
        from simulation.components.capture26 import INITIAL_JOINTS
        for body_name, joint_name, value in INITIAL_JOINTS:
            joint = scene.getBody(body_name).getScalarJoint(joint_name)
            joint.setPosition(value)
            joint.setVelocity(0.0)

    # Extra target DOFs never enter the SARM arm/IK/actuator arrays.
    if "target_hinge_position_rad" in randomized:
        hinge = scene.getBody("satellite_outer_panel").getScalarJoint("outer_panel_hinge")
        hinge.setPosition(float(randomized["target_hinge_position_rad"]))
        hinge.setVelocity(0.0)

    # Whole-spacecraft COM depends on the arm pose: assign it before measuring.
    if 'capture_arm_joint_position_rad' in randomized:
        for i, value in enumerate(randomized['capture_arm_joint_position_rad'], 1):
            scene.getBody(f'bottom_link{i}').getScalarJoint(f'bottom_joint{i}').setPosition(float(value))
    if 'preheld_capture_joint_position' in randomized:
        from simulation.components.preheld_capture import validate_preheld
        from simulation.capture_arm_target import INITIAL_JOINTS
        values = validate_preheld(native.MODEL_PATH, randomized['preheld_capture_joint_position'])
        for (body_name, joint_name, _), value in zip(INITIAL_JOINTS, values, strict=True):
            joint = scene.getBody(body_name).getScalarJoint(joint_name)
            joint.setPosition(float(value))
            joint.setVelocity(0.)

    orbit_state = {
        "semi_major_axis_m": float(elements.a),
        "initial_position_m": orbital_position.tolist(),
        "initial_velocity_m_s": bus_velocity.tolist(),
        "initial_altitude_m": float(np.linalg.norm(orbital_position) - float(earth.radEquator)),
        "initial_speed_m_s": float(np.linalg.norm(bus_velocity)),
    }
    formation = randomized.get('coorbital_initialization', {})
    if formation.get('enabled'):
        if float(orbit['eccentricity']) != 0 or not math.isfinite(float(formation['separation_m'])) or float(formation['separation_m']) <= 0:
            raise ValueError('coorbital initialization requires a circular orbit and positive finite separation')
        from Basilisk.utilities import RigidBodyKinematics as rbk
        from simulation.coorbital_initialization import reference
        import xml.etree.ElementTree as ET
        point = formation.get('reference_point', 'body_origin')
        if point not in ('body_origin', 'assembly_com'):
            raise ValueError('coorbital reference_point must be body_origin or assembly_com')
        com_args = {}
        if point == 'assembly_com':
            if local_stepper is None:
                raise ValueError('assembly_com initialization requires the authoritative local backend')
            properties = local_stepper.initial_assembly_properties(
                native.MODEL_PATH, ['cubesat_bus', 'capture_target'])
            com_args = dict(service_com_body_m=properties['cubesat_bus']['com_body_m'],
                            target_com_body_m=properties['capture_target']['com_body_m'])
            orbit_state['assembly_com_properties'] = properties
        ref = reference(orbital_position, orbital_velocity, formation['separation_m'],
                        service_body_rotation_wxyz=formation.get('service_body_rotation_wxyz'), **com_args)
        rs, rt, omega = ref['service_rotation'], ref['target_rotation'], ref['omega']
        bus.setPosition(ref['service_position'])
        bus.setAttitude(rbk.C2MRP(rs.T)); bus.setVelocity(ref['service_velocity'])
        bus.setAttitudeRate(rs.T @ omega)
        target.setPosition(ref['target_position']); target.setVelocity(ref['target_velocity'])
        target.setAttitude(rbk.C2MRP(rt.T)); target.setAttitudeRate(rt.T @ omega)
        # Free task-box plugs must follow the new bus initial transform too.
        root = ET.parse(native.MODEL_PATH).getroot()
        for element in root.findall('./worldbody/body'):
            name = element.get('name')
            if name in ('cubesat_bus','capture_target'): continue
            if element.find('freejoint') is None and element.find('joint[@type="free"]') is None: continue
            local = np.fromstring(element.get('pos','0 0 0'),sep=' ')
            quat = np.fromstring(element.get('quat','1 0 0 0'),sep=' ')
            body = scene.getBody(name); offset = rs @ local
            body.setPosition(ref['service_position']+offset)
            body.setVelocity(ref['service_velocity']+np.cross(omega,offset))
            rotation = rs @ np.asarray(rbk.EP2C(quat)).T
            body.setAttitude(rbk.C2MRP(rotation.T)); body.setAttitudeRate(rotation.T @ omega)
        orbit_state['initial_position_m'] = ref['service_position'].tolist()
        orbit_state['initial_velocity_m_s'] = ref['service_velocity'].tolist()
        orbit_state['initial_altitude_m'] = float(np.linalg.norm(ref['service_position'])-earth.radEquator)
        orbit_state['initial_speed_m_s'] = float(np.linalg.norm(ref['service_velocity']))
        orbit_state['coorbital_initialization'] = formation
        if point == 'assembly_com':
            orbit_state['initial_com_states'] = {
                name: dict(position_m=ref[prefix+'_com_position'].tolist(),
                           velocity_m_s=ref[prefix+'_com_velocity'].tolist())
                for name, prefix in [('cubesat_bus', 'service'), ('capture_target', 'target')]}
        if 'grip_frame_standoff_m' in formation:
            from simulation.capture_arm_target import CaptureArmTarget
            arm = CaptureArmTarget(native.MODEL_PATH)
            p, rotation = arm.kinematics.forward(randomized['capture_arm_joint_position_rad'])
            interface = rs.T @ (ref['target_position']-ref['service_position']
                                + rt @ np.array([.1755, .05475, .0420787001299893]))
            gap = float(np.linalg.norm(interface-p))
            expected_gap = float(formation['grip_frame_standoff_m'])
            if not math.isfinite(expected_gap) or expected_gap <= 0 or abs(gap-expected_gap) > .001:
                raise ValueError(f'initial capture standoff mismatch: {gap:.6f} m versus {expected_gap} m')
            orbit_state['initial_grip_to_interface_m'] = gap
    return orbit_state
