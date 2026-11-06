"""Explicit execution families; importing this registry does not load Basilisk."""
from dataclasses import dataclass
import importlib

from .benchmark_attitude import AttitudeTask, WheelTorqueAction
from .benchmark_attitude_score import AttitudeScorer
from .benchmark_contract import ForceAction, WaypointTask
from .benchmark_scoring import WaypointScorer

Task = WaypointTask | AttitudeTask


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
        "NativeAttitudeEnvironment", ("attitude-wheel-pd-v1", "no-wheel-torque-v1", "unsafe-spin-v1")),
}


def method_ids(task: Task | None = None):
    if task is not None:
        return FAMILIES[type(task)].method_ids
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
    if isinstance(task, WaypointTask):
        action = ForceAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_force_n)
    else:
        action = WheelTorqueAction.model_validate(raw)
        action.validate_for(observation, task.safety.maximum_motor_torque_nm)
    return action


def make_scorer(task: Task, chaser_mass_kg):
    return WaypointScorer(task, chaser_mass_kg) if isinstance(task, WaypointTask) else AttitudeScorer(task)
