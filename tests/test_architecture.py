from __future__ import annotations

from dataclasses import dataclass

import pytest

from simulation.architecture import (
    ControlOutput,
    EnvironmentState,
    JointActuatorCommand,
    JointState,
    RigidBodyState,
    SceneState,
    SimulationOrchestrator,
    WrenchCommand,
)


class FakeScene:
    def __init__(self) -> None:
        self.state = SceneState(
            sim_time_s=0.0,
            bodies={
                "cubesat_bus": RigidBodyState(
                    "cubesat_bus", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
                )
            },
            joints={"arm_joint_0": JointState("arm_joint_0")},
        )
        self.controls: list[ControlOutput] = []
        self.events: list[str] = []

    def initialize(self, state: SceneState) -> None:
        self.events.append("scene.initialize")
        self.state = state

    def apply_control(self, control: ControlOutput) -> None:
        self.events.append("scene.apply_control")
        self.controls.append(control)

    def step(self, dt_s: float) -> None:
        self.events.append("scene.step")
        self.state = self.state.at_time(self.state.sim_time_s + dt_s)

    def read_state(self) -> SceneState:
        self.events.append("scene.read_state")
        return self.state


class FakePublisher:
    def __init__(self, events: list[str]) -> None:
        self.events = events
        self.states: list[SceneState] = []

    def publish(self, state: SceneState) -> None:
        self.events.append("publisher.publish")
        self.states.append(state)


@dataclass
class FakeModule:
    name: str
    events: list[str]
    updates: int = 0

    def initialize(self, state: SceneState) -> None:
        self.events.append(f"{self.name}.initialize")

    def update(
        self, state: SceneState, environment: EnvironmentState, dt_s: float
    ) -> ControlOutput:
        self.events.append(f"{self.name}.update")
        self.updates += 1
        return ControlOutput(
            external_wrenches={"cubesat_bus": WrenchCommand(force_n=(1.0, 0.0, 0.0))},
            joint_commands={
                "arm_joint_0": JointActuatorCommand(position_rad=state.sim_time_s + dt_s)
            },
        )


def test_orchestrator_runs_the_canonical_cycle_in_order() -> None:
    scene = FakeScene()
    module = FakeModule("attitude_controller", scene.events)
    publisher = FakePublisher(scene.events)
    orchestrator = SimulationOrchestrator(scene, publishers=[publisher], modules=[module])

    initial = scene.state
    orchestrator.initialize(initial)
    result = orchestrator.step(0.1)

    assert result.previous_state.sim_time_s == 0.0
    assert result.state.sim_time_s == pytest.approx(0.1)
    assert result.environment.sim_time_s == 0.0
    assert result.control.joint_commands["arm_joint_0"].position_rad == pytest.approx(0.1)
    assert scene.controls == [result.control]
    assert module.updates == 1
    assert [event for event in scene.events if event in {
        "publisher.publish", "attitude_controller.update", "scene.apply_control", "scene.step"
    }] == [
        "publisher.publish",
        "publisher.publish",
        "attitude_controller.update",
        "scene.apply_control",
        "scene.step",
        "publisher.publish",
    ]
    assert [state.sim_time_s for state in publisher.states] == [0.0, 0.0, 0.1]


def test_control_outputs_add_efforts_but_reject_conflicting_joint_owners() -> None:
    combined = ControlOutput.combine(
        [
            ControlOutput(
                external_wrenches={"bus": WrenchCommand(force_n=(1.0, 0.0, 0.0))},
                reaction_wheel_torques_nm={"rw_x": 0.2},
                thruster_thrust_n={"thruster_a": 0.5},
            ),
            ControlOutput(
                external_wrenches={"bus": WrenchCommand(force_n=(0.0, 2.0, 0.0))},
                reaction_wheel_torques_nm={"rw_x": -0.1},
                thruster_thrust_n={"thruster_a": 0.25},
            ),
        ]
    )

    assert combined.external_wrenches["bus"].force_n == (1.0, 2.0, 0.0)
    assert combined.reaction_wheel_torques_nm["rw_x"] == pytest.approx(0.1)
    assert combined.thruster_thrust_n["thruster_a"] == pytest.approx(0.75)

    with pytest.raises(ValueError, match="multiple Basilisk modules command joint"):
        ControlOutput.combine(
            [
                ControlOutput(joint_commands={"joint": JointActuatorCommand(position_rad=1.0)}),
                ControlOutput(joint_commands={"joint": JointActuatorCommand(position_rad=2.0)}),
            ]
        )


def test_module_can_be_added_after_initialization() -> None:
    scene = FakeScene()
    orchestrator = SimulationOrchestrator(scene)
    orchestrator.initialize(scene.state)
    module = FakeModule("gravity", scene.events)

    orchestrator.add_module(module)
    assert "gravity.initialize" in scene.events
    orchestrator.step(0.01)
    assert module.updates == 1


def test_invalid_timestep_and_uninitialized_step_fail_loudly() -> None:
    scene = FakeScene()
    orchestrator = SimulationOrchestrator(scene)
    with pytest.raises(RuntimeError, match="initialize"):
        orchestrator.step(0.01)

    orchestrator.initialize(scene.state)
    with pytest.raises(ValueError, match="positive"):
        orchestrator.step(0.0)

class FakeBasiliskSimulation:
    def __init__(self) -> None:
        self.attachments: list[tuple[str, object, int]] = []

    def AddModelToTask(self, task_name: str, model: object, priority: int) -> None:
        self.attachments.append((task_name, model, priority))


def test_native_basilisk_module_registry_attaches_in_registration_order() -> None:
    from simulation.architecture import BasiliskModuleRegistry

    simulation = FakeBasiliskSimulation()
    registry = BasiliskModuleRegistry()
    ephemeris = object()
    controller = object()
    registry.register("spice", ephemeris, task_name="environmentTask", priority=200)
    registry.register("attitude", controller, task_name="controlTask", priority=-100)
    registry.attach(simulation)

    assert simulation.attachments == [
        ("environmentTask", ephemeris, 200),
        ("controlTask", controller, -100),
    ]
    assert registry.keep_alive() == (ephemeris, controller)


def test_scene_state_serializes_the_stable_state_boundary() -> None:
    state = SceneState(
        sim_time_s=2.5,
        bodies={"z_body": RigidBodyState("z_body", (1, 2, 3), (4, 5, 6))},
        joints={"joint": JointState("joint", position_rad=0.25, velocity_rad_s=0.5)},
        metadata={"source": "mjscene"},
    )

    message = state.to_message()
    assert message["protocol"] == "space-sim-state/1"
    assert message["type"] == "state"
    assert message["bodies"][0]["body_id"] == "z_body"
    assert message["joints"][0]["position_rad"] == pytest.approx(0.25)
    assert message["metadata"] == {"source": "mjscene"}
