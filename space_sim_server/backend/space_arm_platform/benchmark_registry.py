"""Explicit execution families; importing this registry does not load Basilisk."""
from dataclasses import dataclass
import importlib

from .benchmark_attitude import AttitudeContext, AttitudeObservation, AttitudeTask, WheelTorqueAction
from .benchmark_attitude_score import AttitudeScorer
from .benchmark_contract import ForceAction, MethodContext, NavigationObservation, WaypointTask
from .benchmark_scoring import WaypointScorer
from .benchmark_orbit import CircularOrbitObjective, OrbitMaintenanceObjective, OrbitContext, OrbitObservation, OrbitTask
from .benchmark_orbit_score import OrbitScorer
from .benchmark_navigation import GNSSMeasurements, NavigationContext, NavigationEstimateAction, NavigationTask
from .benchmark_navigation_score import NavigationScorer
from .benchmark_navigation_recovery import NavigationRecoveryTask, NavigationRecoveryContext, NavigationRecoveryObservation
from .benchmark_navigation_recovery_score import NavigationRecoveryScorer
from .benchmark_relative_flight import RelativeFlightContext, RelativeFlightTask
from .benchmark_relative_flight_score import RelativeFlightScorer
from .benchmark_flyaround import FlyAroundContext, FlyAroundTask
from .benchmark_flyaround_score import FlyAroundScorer
from .benchmark_formation import FormationAction, FormationContext, FormationObservation, FormationTask
from .benchmark_formation_score import FormationScorer
from .benchmark_pointing import PointingContext, PointingObservation, PointingTask
from .benchmark_pointing_score import PointingScorer
from .benchmark_vision import VisionPoseTask, VisionPoseContext, VisionPoseObservation, VisionPoseAction
from .benchmark_vision_score import VisionPoseScorer
from .benchmark_vision_motion import VisionMotionTask, VisionMotionContext, VisionMotionObservation, VisionMotionAction
from .benchmark_vision_motion_score import VisionMotionScorer
from .benchmark_vision_approach import VisionApproachTask, VisionApproachContext, VisionApproachObservation, VisionApproachAction
from .benchmark_vision_approach_score import VisionApproachScorer
from .benchmark_wheel_recovery import WheelRecoveryTask, WheelRecoveryContext, WheelRecoveryObservation, WheelRecoveryAction
from .benchmark_wheel_recovery_score import WheelRecoveryScorer
from .benchmark_avoidance import AvoidanceTask, AvoidanceContext, AvoidanceObservation
from .benchmark_avoidance_score import AvoidanceScorer

Task = WaypointTask | AttitudeTask | OrbitTask | NavigationTask | NavigationRecoveryTask | RelativeFlightTask | FlyAroundTask | FormationTask | PointingTask | VisionPoseTask | VisionMotionTask | VisionApproachTask | WheelRecoveryTask | AvoidanceTask


@dataclass(frozen=True)
class FamilyImplementation:
    methods_module: str
    native_module: str
    environment_class: str
    method_ids: tuple[str, ...]


FAMILIES = {
    NavigationRecoveryTask: FamilyImplementation("simulation.benchmark_navigation_recovery_methods", "simulation.benchmark_native_navigation_recovery",
        "NativeNavigationRecoveryEnvironment", ("gnss-outage-recovery-v1", "freeze-during-outage-v1", "zero-recovery-estimate-v1")),
    AvoidanceTask: FamilyImplementation("simulation.benchmark_avoidance_methods", "simulation.benchmark_native_avoidance",
        "NativeAvoidanceEnvironment", ("catalog-avoidance-v1", "no-avoidance-thrust-v1", "unsafe-avoidance-burn-v1")),
    WheelRecoveryTask: FamilyImplementation("simulation.benchmark_wheel_recovery_methods", "simulation.benchmark_native_wheel_recovery",
        "NativeWheelRecoveryEnvironment", ("adaptive-wheel-recovery-v1", "unaware-wheel-recovery-v1", "no-recovery-torque-v1", "unsafe-recovery-spin-v1")),
    VisionApproachTask: FamilyImplementation("simulation.benchmark_vision_approach_methods", "simulation.benchmark_native_vision_approach",
        "NativeVisionApproachEnvironment", ("image-stereo-approach-v1", "no-visual-thrust-v1", "unsafe-visual-close-v1", "missing-visual-control-v1")),
    VisionMotionTask: FamilyImplementation("simulation.benchmark_vision_motion_methods", "simulation.benchmark_native_vision_motion",
        "NativeVisionMotionEnvironment", ("image-gyro-spin-v1", "zero-image-spin-v1", "missing-image-spin-v1")),
    VisionPoseTask: FamilyImplementation("simulation.benchmark_vision_methods", "simulation.benchmark_native_vision",
        "NativeVisionPoseEnvironment", ("image-fiducial-pose-v1", "missing-image-pose-v1", "zero-image-pose-v1")),
    WaypointTask: FamilyImplementation("simulation.benchmark_methods", "simulation.benchmark_native_flight",
        "NativeWaypointEnvironment", ("gnss-pd-v1", "no-thrust-v1", "unsafe-approach-v1")),
    AttitudeTask: FamilyImplementation("simulation.benchmark_attitude_methods", "simulation.benchmark_native_attitude",
        "NativeAttitudeEnvironment", ("attitude-wheel-pd-v1", "no-wheel-torque-v1", "unsafe-spin-v1", "attitude-rate-limited-pd-v1", "attitude-sequence-pd-v1")),
    OrbitTask: FamilyImplementation("simulation.benchmark_orbit_methods", "simulation.benchmark_native_orbit",
        "NativeOrbitEnvironment", ("gnss-orbit-maneuver-v1", "no-orbit-thrust-v1", "unsafe-orbit-burn-v1")),
    NavigationTask: FamilyImplementation("simulation.benchmark_navigation_methods", "simulation.benchmark_native_navigation",
        "NativeNavigationEnvironment", ("gnss-orbit-estimator-v1", "gnss-measurement-only-v1", "zero-navigation-estimate-v1")),
    RelativeFlightTask: FamilyImplementation("simulation.benchmark_relative_flight_methods", "simulation.benchmark_native_relative_flight",
        "NativeRelativeFlightEnvironment", ("gnss-relative-flight-v1", "no-relative-thrust-v1", "unsafe-relative-flight-v1")),
    FlyAroundTask: FamilyImplementation("simulation.benchmark_flyaround_methods", "simulation.benchmark_native_flyaround",
        "NativeFlyAroundEnvironment", ("gnss-flyaround-v1", "no-flyaround-thrust-v1", "unsafe-flyaround-v1")),
    FormationTask: FamilyImplementation("simulation.benchmark_formation_methods", "simulation.benchmark_native_formation",
        "NativeFormationEnvironment", ("gnss-formation-v1", "no-formation-thrust-v1", "unsafe-formation-v1", "one-sided-formation-v1")),
    PointingTask: FamilyImplementation("simulation.benchmark_pointing_methods", "simulation.benchmark_native_pointing",
        "NativePointingEnvironment", ("ephemeris-pointing-pd-v1", "frozen-pointing-pd-v1", "no-pointing-torque-v1", "unsafe-pointing-spin-v1")),
}


def method_ids(task: Task | None = None):
    if task is not None:
        methods = FAMILIES[type(task)].method_ids
        if isinstance(task, AttitudeTask) and task.source_id == "S04":
            preferred = "attitude-sequence-pd-v1"
            return (preferred, *(method for method in methods if method != preferred))
        if isinstance(task, AttitudeTask) and task.source_id in {"S01", "S03"}:
            preferred = "attitude-rate-limited-pd-v1"
            return (preferred, *(method for method in methods if method != preferred))
        return methods
    return tuple(name for family in FAMILIES.values() for name in family.method_ids)


def method_classes(task: Task):
    family = FAMILIES[type(task)]
    methods = importlib.import_module(family.methods_module).METHODS
    if tuple(methods) != family.method_ids:
        raise ValueError("method registration differs from its family implementation")
    return methods


def native_environment(task: Task, output):
    family = FAMILIES[type(task)]
    return getattr(importlib.import_module(family.native_module), family.environment_class)(task, output)


def method_input_types(task: Task):
    """Public method inputs, without importing a native simulation backend."""
    return {
        WaypointTask: (MethodContext, NavigationObservation),
        AttitudeTask: (AttitudeContext, AttitudeObservation),
        OrbitTask: (OrbitContext, OrbitObservation),
        NavigationTask: (NavigationContext, GNSSMeasurements),
        NavigationRecoveryTask: (NavigationRecoveryContext, NavigationRecoveryObservation),
        RelativeFlightTask: (RelativeFlightContext, NavigationObservation),
        FlyAroundTask: (FlyAroundContext, NavigationObservation),
        FormationTask: (FormationContext, FormationObservation),
        PointingTask: (PointingContext, PointingObservation),
        VisionPoseTask: (VisionPoseContext, VisionPoseObservation),
        VisionMotionTask: (VisionMotionContext, VisionMotionObservation),
        VisionApproachTask: (VisionApproachContext, VisionApproachObservation),
        WheelRecoveryTask: (WheelRecoveryContext, WheelRecoveryObservation),
        AvoidanceTask: (AvoidanceContext, AvoidanceObservation),
    }[type(task)]


def parse_action(task: Task, raw, observation):
    if isinstance(task, WheelRecoveryTask):
        action = WheelRecoveryAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_motor_torque_nm)
    elif isinstance(task, VisionApproachTask):
        action = VisionApproachAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, VisionMotionTask):
        action = VisionMotionAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, VisionPoseTask):
        action = VisionPoseAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, FormationTask):
        action = FormationAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, NavigationTask):
        action = NavigationEstimateAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, (WaypointTask, OrbitTask, FlyAroundTask, AvoidanceTask)):
        action = ForceAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_force_n)
    else:
        action = WheelTorqueAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_motor_torque_nm)
    return action


def make_scorer(task: Task, chaser_mass_kg):
    if isinstance(task, NavigationRecoveryTask):
        return NavigationRecoveryScorer(task, chaser_mass_kg)
    if isinstance(task, AvoidanceTask):
        return AvoidanceScorer(task, chaser_mass_kg)
    if isinstance(task, WheelRecoveryTask):
        return WheelRecoveryScorer(task)
    if isinstance(task, VisionApproachTask):
        return VisionApproachScorer(task, chaser_mass_kg)
    if isinstance(task, VisionMotionTask):
        return VisionMotionScorer(task, chaser_mass_kg)
    if isinstance(task, VisionPoseTask):
        return VisionPoseScorer(task, chaser_mass_kg)
    if isinstance(task, PointingTask):
        return PointingScorer(task)
    if isinstance(task, FormationTask):
        return FormationScorer(task, chaser_mass_kg)
    if isinstance(task, FlyAroundTask):
        return FlyAroundScorer(task, chaser_mass_kg)
    if isinstance(task, RelativeFlightTask):
        return RelativeFlightScorer(task, chaser_mass_kg)
    if isinstance(task, NavigationTask):
        return NavigationScorer(task, chaser_mass_kg)
    if isinstance(task, OrbitTask):
        return OrbitScorer(task, chaser_mass_kg)
    return WaypointScorer(task, chaser_mass_kg) if isinstance(task, WaypointTask) else AttitudeScorer(task)


def capability_matrix(catalogue, tasks):
    """Join all requirements to registered configurations, never to implied passes."""
    by_requirement = {requirement.id: [] for requirement in catalogue.tasks}
    seen = set()
    for task in tasks:
        task.validate_catalogue(catalogue)
        if task.configuration_id in seen or type(task) not in FAMILIES:
            raise ValueError("duplicate configuration or unregistered execution family")
        seen.add(task.configuration_id)
        by_requirement[task.requirement_id].append({"id": task.configuration_id, "task_fingerprint": task.fingerprint(),
            "backend": task.backend, "vision_mode": task.vision_mode, "target_cooperation": task.target_cooperation,
            "spacecraft_count": task.spacecraft_count, "controlled_spacecraft": task.controlled_spacecraft,
            "control_architecture": task.control_architecture})
    rows = [{**requirement.model_dump(mode="json"),
             "execution_registration": "registered" if by_requirement[requirement.id] else "not_registered",
             "configurations": sorted(by_requirement[requirement.id], key=lambda item: item["id"])}
            for requirement in catalogue.tasks]
    registered = sum(bool(row["configurations"]) for row in rows)
    return {"schema_version": "spacesim-autonomy-capability-matrix/1", "source_id": catalogue.source.id,
            "scope": "execution_registration_not_runtime_acceptance", "requirements": len(rows),
            "registered_requirements": registered, "unregistered_requirements": len(rows)-registered,
            "configuration_count": len(seen), "tasks": rows}


def configuration_title(catalogue, task):
    title = catalogue.task(task.requirement_id).title
    if isinstance(task, AvoidanceTask):
        title += "（单星与被动碎片，外部目录观测）"
    if isinstance(task, WheelRecoveryTask):
        title += "（单飞轮力矩衰减与姿态恢复）"
    if isinstance(task, RelativeFlightTask) and task.source_id == 'D40':
        title += "（GNSS，150 m 接近中止）"
    if isinstance(task, RelativeFlightTask) and task.source_id == 'D05':
        title += "（合作目标，7 km 至 150 m）"
    if isinstance(task, FormationTask) and task.source_id == 'D19':
        title += "（双星实际共同机动，600 m 至 150 m）"
    if isinstance(task, VisionPoseTask):
        title += "（已知被动标志物，真实图像）"
    if isinstance(task, PointingTask):
        title += "（"+{"earth_center": "地心", "sun": "太阳", "inertial_direction": "惯性方向", "ground_site": "地面固定点"}[task.objective.kind]+"）"
    if isinstance(task, OrbitTask) and isinstance(task.objective, CircularOrbitObjective):
        title += f"（目标圆轨道高度 {task.objective.altitude_m/1000:g} km）"
    if isinstance(task, OrbitTask) and isinstance(task.objective, OrbitMaintenanceObjective):
        title += f"（{task.objective.altitude_m/1000:g} km，J2 连续维持）"
    return title
