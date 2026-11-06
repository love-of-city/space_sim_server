"""Explicit execution families; importing this registry does not load Basilisk."""
from dataclasses import dataclass
import importlib

from .benchmark_attitude import AttitudeTask, WheelTorqueAction
from .benchmark_attitude_score import AttitudeScorer
from .benchmark_contract import ForceAction, WaypointTask
from .benchmark_scoring import WaypointScorer
from .benchmark_orbit import CircularOrbitObjective, OrbitTask
from .benchmark_orbit_score import OrbitScorer
from .benchmark_navigation import NavigationEstimateAction, NavigationTask
from .benchmark_navigation_score import NavigationScorer
from .benchmark_relative_flight import RelativeFlightTask
from .benchmark_relative_flight_score import RelativeFlightScorer

Task = WaypointTask | AttitudeTask | OrbitTask | NavigationTask | RelativeFlightTask


@dataclass(frozen=True)
class FamilyImplementation:
    methods_module: str
    native_module: str
    environment_class: str
    method_ids: tuple[str, ...]


FAMILIES = {
    WaypointTask: FamilyImplementation("simulation.benchmark_methods", "simulation.benchmark_native_flight",
        "NativeWaypointEnvironment", ("gnss-pd-v1", "no-thrust-v1", "unsafe-approach-v1")),
    AttitudeTask: FamilyImplementation("simulation.benchmark_attitude_methods", "simulation.benchmark_native_attitude",
        "NativeAttitudeEnvironment", ("attitude-wheel-pd-v1", "no-wheel-torque-v1", "unsafe-spin-v1", "attitude-rate-limited-pd-v1")),
    OrbitTask: FamilyImplementation("simulation.benchmark_orbit_methods", "simulation.benchmark_native_orbit",
        "NativeOrbitEnvironment", ("gnss-orbit-maneuver-v1", "no-orbit-thrust-v1", "unsafe-orbit-burn-v1")),
    NavigationTask: FamilyImplementation("simulation.benchmark_navigation_methods", "simulation.benchmark_native_navigation",
        "NativeNavigationEnvironment", ("gnss-orbit-estimator-v1", "gnss-measurement-only-v1", "zero-navigation-estimate-v1")),
    RelativeFlightTask: FamilyImplementation("simulation.benchmark_relative_flight_methods", "simulation.benchmark_native_relative_flight",
        "NativeRelativeFlightEnvironment", ("gnss-relative-flight-v1", "no-relative-thrust-v1", "unsafe-relative-flight-v1")),
}


def method_ids(task: Task | None = None):
    if task is not None:
        methods = FAMILIES[type(task)].method_ids
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


def parse_action(task: Task, raw, observation):
    if isinstance(task, NavigationTask):
        action = NavigationEstimateAction.model_validate(raw)
        action.validate_for(observation, task)
    elif isinstance(task, (WaypointTask, OrbitTask)):
        action = ForceAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_force_n)
    else:
        action = WheelTorqueAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_motor_torque_nm)
    return action


def make_scorer(task: Task, chaser_mass_kg):
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
    if isinstance(task, OrbitTask) and isinstance(task.objective, CircularOrbitObjective):
        title += f"（目标圆轨道高度 {task.objective.altitude_m/1000:g} km）"
    return title
