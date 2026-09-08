#!/usr/bin/env python3
"""Run and assess the SARM satellite + arm grasp scene."""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco as mj
import numpy as np
from Basilisk.architecture import messaging, sysModel
from Basilisk.simulation import MJJointPIDController, mujoco, saturationSingleActuator
from Basilisk.utilities import SimulationBaseClass, macros

ROOT_DIR = Path(__file__).resolve().parents[2]
MODEL_PATH = ROOT_DIR / "platform" / "sarm_platform.xml"
MESH_DIR = ROOT_DIR / "meshes"

TIME_STEP = 0.002  # [s]
STOP_TIME = 10.0  # [s]
COMMON_VELOCITY = np.array([0.01, -0.004, 0.002])  # [m/s]
# MJScene 2.11.1 is unstable during startup when the second free body has
# both a non-identity attitude and a non-zero angular rate.  Apply target spin
# only after a dedicated, validated runtime path is available.
TARGET_SPIN = np.zeros(3)  # [rad/s], target body frame.
TARGET_POS = np.array([0.38754456, -0.00109359, 0.42397138])  # [m]
TARGET_QUAT = np.array(
    [0.99967109, 0.02433362, -0.00809494, 0.00024763]
)  # [-], MuJoCo w-x-y-z order.

JOINTS = (
    ("link1", "joint1"),
    ("link2", "joint2"),
    ("link3", "joint3"),
    ("link4", "joint4"),
    ("link5", "joint5"),
    ("link6", "joint6"),
    ("finger1", "joint_finger1"),
    ("finger2", "joint_finger2"),
)

ACTUATORS = (
    "sarm_joint1_motor", "sarm_joint2_motor", "sarm_joint3_motor",
    "sarm_joint4_motor", "sarm_joint5_motor", "sarm_joint6_motor",
    "sarm_joint_finger1_motor", "sarm_joint_finger2_motor",
)

# The last two joints are sliders: Kp is N/m, not Nm/rad.  The old
# 0.2 N/m could not overcome 0.01 N friction even over the full 0.0375 m stroke.
# Keep the existing 0.05 N per-finger force cap below unchanged.
KP = np.array([8.0, 8.0, 8.0, 5.0, 4.0, 2.0, 40.0, 40.0])
KD = np.array([1.0, 1.0, 1.0, 0.7, 0.5, 0.25, 0.05, 0.05])
TORQUE_LIMITS = np.array([1.5, 1.5, 1.5, 1.0, 1.0, 0.35, 0.05, 0.05])

PREGRASP = np.array([0.0, -0.1790243, 0.2159404, -0.0368382, 0.0, 0.0, 0.01875, 0.01875])
ALIGNED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01875, 0.01875])
CLOSED = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
WITHDRAW = np.array([0.0, -0.10, 0.12, -0.02, 0.0, 0.0, 0.0, 0.0])


@dataclass(frozen=True)
class GraspMetrics:
    """Numerical acceptance metrics for one maneuver."""

    max_linear_momentum_error: float
    max_com_angular_momentum_error: float
    max_origin_angular_momentum_error: float
    max_com_uniform_motion_error: float
    initial_mechanical_energy: float
    final_mechanical_energy: float
    max_mechanical_energy_change: float
    first_contact_time: float | None
    bilateral_contact_duration: float
    withdrawal_bilateral_coverage: float
    fixed_jaw_peak_normal_force: float
    moving_jaw_peak_normal_force: float
    target_withdrawal_distance: float
    grasp_site_drift: float
    final_relative_linear_speed: float
    final_relative_angular_speed: float
    invalid_contact_count: int
    max_abs_actuator_command: tuple[float, ...]


@dataclass(frozen=True)
class SimulationRecord:
    """Recorded Basilisk generalized state and actuator histories."""

    times: np.ndarray
    qpos: np.ndarray
    qvel: np.ndarray
    commands: np.ndarray


class JointTrajectoryPublisher(sysModel.SysModel):
    """Publish synchronized quintic joint position and velocity references."""

    def __init__(self) -> None:
        super().__init__()
        self.ModelTag = "sarmJointTrajectory"
        self.positionOutMsgs = [messaging.ScalarJointStateMsg() for _ in JOINTS]
        self.velocityOutMsgs = [messaging.ScalarJointStateMsg() for _ in JOINTS]

    @staticmethod
    def _segment(
        time: float, start: float, stop: float, q0: np.ndarray, q1: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Evaluate one zero-end-velocity quintic segment."""
        if time <= start:
            return q0.copy(), np.zeros_like(q0)
        if time >= stop:
            return q1.copy(), np.zeros_like(q0)
        duration = stop - start  # [s]
        phase = (time - start) / duration
        blend = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
        blend_rate = (
            30.0 * phase**2 - 60.0 * phase**3 + 30.0 * phase**4
        ) / duration  # [1/s]
        delta = q1 - q0
        return q0 + blend * delta, blend_rate * delta

    @classmethod
    def reference(cls, time: float) -> tuple[np.ndarray, np.ndarray]:
        """Return the task reference at simulation time ``time``."""
        if time < 1.0:
            return PREGRASP.copy(), np.zeros(len(JOINTS))
        if time < 4.0:
            return cls._segment(time, 1.0, 4.0, PREGRASP, ALIGNED)
        if time < 4.5:
            return ALIGNED.copy(), np.zeros(len(JOINTS))
        if time < 6.0:
            return cls._segment(time, 4.5, 6.0, ALIGNED, CLOSED)
        if time < 7.0:
            return CLOSED.copy(), np.zeros(len(JOINTS))
        if time < 9.0:
            return cls._segment(time, 7.0, 9.0, CLOSED, WITHDRAW)
        return WITHDRAW.copy(), np.zeros(len(JOINTS))

    def Reset(self, CurrentSimNanos: int) -> None:
        """Publish the initial trajectory sample."""
        self.UpdateState(CurrentSimNanos)

    def UpdateState(self, CurrentSimNanos: int) -> None:
        """Publish desired joint position and velocity messages."""
        time = CurrentSimNanos * macros.NANO2SEC  # [s]
        position, velocity = self.reference(time)
        for index in range(len(JOINTS)):
            self.positionOutMsgs[index].write(
                messaging.ScalarJointStateMsgPayload(state=float(position[index])),
                CurrentSimNanos,
                self.moduleID,
            )
            self.velocityOutMsgs[index].write(
                messaging.ScalarJointStateMsgPayload(state=float(velocity[index])),
                CurrentSimNanos,
                self.moduleID,
            )


def _quaternion_to_mrp(quaternion: np.ndarray) -> np.ndarray:
    """Convert a normalized w-x-y-z quaternion to principal MRPs."""
    quaternion = quaternion / np.linalg.norm(quaternion)
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    return quaternion[1:] / (1.0 + quaternion[0])


def _load_scene() -> mujoco.MJScene:
    """Load the grasp MJCF and its mesh assets through the MuJoCo VFS."""
    # The exported meshes use uppercase .STL; include them on case-sensitive hosts too.
    mesh_files = [
        str(path.resolve()) for path in sorted(MESH_DIR.iterdir())
        if path.is_file() and path.suffix.casefold() in {".stl", ".obj"}
    ]
    return mujoco.MJScene.fromFile(str(MODEL_PATH), files=mesh_files)


def _build_simulation(*, attitude_control_enabled: bool | None = None) -> tuple[Any, Any, list[Any], list[Any]]:
    """Create the Basilisk scene, controller chain, and recorders."""
    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("graspProcess")
    process.addTask(simulation.CreateNewTask("graspTask", macros.sec2nano(TIME_STEP)))

    scene = _load_scene()
    scene.ModelTag = "sarmSatelliteGraspScene"
    simulation.AddModelToTask("graspTask", scene)

    trajectory = JointTrajectoryPublisher()
    scene.AddModelToDynamicsTask(trajectory, 9000)
    dynamics_models: list[Any] = [trajectory]
    command_recorders = []
    for index, ((body_name, joint_name), actuator_name) in enumerate(
        zip(JOINTS, ACTUATORS, strict=True)
    ):
        joint = scene.getBody(body_name).getScalarJoint(joint_name)
        controller = MJJointPIDController.JointPIDController()
        controller.ModelTag = f"{joint_name}PID"
        controller.setProportionalGain(float(KP[index]))
        controller.setDerivativeGain(float(KD[index]))
        controller.setIntegralGain(0.0)
        controller.desiredPosInMsg.subscribeTo(trajectory.positionOutMsgs[index])
        controller.desiredVelInMsg.subscribeTo(trajectory.velocityOutMsgs[index])
        controller.measuredPosInMsg.subscribeTo(joint.stateOutMsg)
        controller.measuredVelInMsg.subscribeTo(joint.stateDotOutMsg)
        scene.AddModelToDynamicsTask(controller, 8000 - index)
        dynamics_models.append(controller)

        limiter = saturationSingleActuator.SaturationSingleActuator()
        limiter.ModelTag = f"{joint_name}TorqueLimiter"
        limiter.setMinInput(float(-TORQUE_LIMITS[index]))
        limiter.setMaxInput(float(TORQUE_LIMITS[index]))
        limiter.actuatorInMsg.subscribeTo(controller.outputOutMsg)
        scene.AddModelToDynamicsTask(limiter, 7000 - index)
        dynamics_models.append(limiter)
        scene.getSingleActuator(actuator_name).actuatorInMsg.subscribeTo(
            limiter.actuatorOutMsg
        )
        command_recorders.append(limiter.actuatorOutMsg.recorder())
        simulation.AddModelToTask("graspTask", command_recorders[-1])

    # The server-side BSK chain drives physical MJCF rotors, not a second hub.
    repository_root = Path(__file__).resolve().parents[4]
    if str(repository_root) not in sys.path:
        sys.path.insert(0, str(repository_root))
    from simulation.attitude_control import AttitudeControl
    simulation.attitude_control = AttitudeControl(
        simulation, process, scene, MODEL_PATH, enabled=attitude_control_enabled
    )

    state_recorder = scene.stateOutMsg.recorder()
    simulation.AddModelToTask("graspTask", state_recorder)
    return simulation, scene, dynamics_models, [state_recorder, *command_recorders]


def _initialize_state(simulation: Any, scene: Any) -> None:
    """Initialize free bodies and arm joints after Basilisk registration."""
    simulation.InitializeSimulation()
    bus = scene.getBody("cubesat_bus")
    target = scene.getBody("capture_target")
    bus.setVelocity(COMMON_VELOCITY)  # [m/s]
    target.setPosition(TARGET_POS)  # [m]
    target.setVelocity(COMMON_VELOCITY)  # [m/s]
    target.setAttitude(_quaternion_to_mrp(TARGET_QUAT))
    target.setAttitudeRate(TARGET_SPIN)  # [rad/s]
    for (body_name, joint_name), position in zip(JOINTS, PREGRASP, strict=True):
        joint = scene.getBody(body_name).getScalarJoint(joint_name)
        joint.setPosition(float(position))  # [rad]
        joint.setVelocity(0.0)  # [rad/s]


def _site_velocity(
    model: mj.MjModel, data: mj.MjData, site_id: int
) -> tuple[np.ndarray, np.ndarray]:
    """Return world-frame linear and angular velocity of a site."""
    jacobian_position = np.zeros((3, model.nv))
    jacobian_rotation = np.zeros((3, model.nv))
    mj.mj_jacSite(model, data, jacobian_position, jacobian_rotation, site_id)
    return jacobian_position @ data.qvel, jacobian_rotation @ data.qvel


def _analyze(
    times: np.ndarray,
    qpos: np.ndarray,
    qvel: np.ndarray,
    commands: np.ndarray,
) -> GraspMetrics:
    """Reconstruct native MuJoCo states and compute grasp/momentum metrics."""
    model = mj.MjModel.from_xml_path(str(MODEL_PATH))
    data = mj.MjData(model)
    total_mass = mj.mj_getTotalmass(model)  # [kg]
    handle_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "capture_target_handle")
    target_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "capture_target")
    gripper_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "sarm_ee")
    target_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "capture_target_grasp")
    fixed_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, "link6")
    moving_body_id = mj.mj_name2id(
        model, mj.mjtObj.mjOBJ_BODY, "finger1"
    )

    linear_momentum = []
    com_angular_momentum = []
    origin_angular_momentum = []
    system_com = []
    mechanical_energy = []
    target_positions = []
    relative_positions = []
    relative_linear_speeds = []
    relative_angular_speeds = []
    fixed_force = np.zeros(len(times))  # [N]
    moving_force = np.zeros(len(times))  # [N]
    invalid_contact_count = 0

    for sample in range(len(times)):
        data.qpos[:] = qpos[sample]
        data.qvel[:] = qvel[sample]
        mj.mj_forward(model, data)
        mj.mj_subtreeVel(model, data)
        mj.mj_energyPos(model, data)
        mj.mj_energyVel(model, data)

        momentum = total_mass * data.subtree_linvel[0]  # [kg*m/s]
        angular_com = data.subtree_angmom[0].copy()  # [kg*m^2/s]
        angular_origin = angular_com + np.cross(data.subtree_com[0], momentum)
        linear_momentum.append(momentum)
        com_angular_momentum.append(angular_com)
        origin_angular_momentum.append(angular_origin)
        system_com.append(data.subtree_com[0].copy())
        mechanical_energy.append(float(np.sum(data.energy)))  # [J]
        target_positions.append(data.xpos[target_body_id].copy())

        delta = data.site_xpos[target_site_id] - data.site_xpos[gripper_site_id]
        relative_positions.append(delta)
        target_v, target_w = _site_velocity(model, data, target_site_id)
        gripper_v, gripper_w = _site_velocity(model, data, gripper_site_id)
        relative_linear_speeds.append(np.linalg.norm(target_v - gripper_v))
        relative_angular_speeds.append(np.linalg.norm(target_w - gripper_w))

        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom1, geom2 = int(contact.geom1), int(contact.geom2)
            if handle_id not in (geom1, geom2):
                invalid_contact_count += 1
                continue
            other_geom = geom2 if geom1 == handle_id else geom1
            other_body = int(model.geom_bodyid[other_geom])
            contact_force = np.zeros(6)
            mj.mj_contactForce(model, data, contact_index, contact_force)
            normal_force = max(0.0, float(contact_force[0]))  # [N]
            if other_body == fixed_body_id:
                fixed_force[sample] += normal_force
            elif other_body == moving_body_id:
                moving_force[sample] += normal_force
            else:
                invalid_contact_count += 1

    linear_momentum = np.asarray(linear_momentum)
    com_angular_momentum = np.asarray(com_angular_momentum)
    origin_angular_momentum = np.asarray(origin_angular_momentum)
    system_com = np.asarray(system_com)
    mechanical_energy = np.asarray(mechanical_energy)
    target_positions = np.asarray(target_positions)
    relative_positions = np.asarray(relative_positions)
    bilateral = (fixed_force > 0.01) & (moving_force > 0.01)
    any_gripper_contact = (fixed_force > 0.0) | (moving_force > 0.0)

    contact_indices = np.flatnonzero(any_gripper_contact)
    first_contact_time = (
        None if contact_indices.size == 0 else float(times[contact_indices[0]])
    )
    longest_bilateral = 0
    current_bilateral = 0
    for present in bilateral:
        current_bilateral = current_bilateral + 1 if present else 0
        longest_bilateral = max(longest_bilateral, current_bilateral)
    bilateral_duration = longest_bilateral * TIME_STEP  # [s]

    withdrawal = (times >= 7.0) & (times <= 10.0)
    withdrawal_coverage = float(np.mean(bilateral[withdrawal]))
    start_withdrawal = int(np.searchsorted(times, 7.0))
    grasp_baseline = relative_positions[start_withdrawal]
    grasp_drift = np.max(
        np.linalg.norm(relative_positions[withdrawal] - grasp_baseline, axis=1)
    )

    elapsed = times - times[0]
    predicted_com = system_com[0] + elapsed[:, None] * (linear_momentum[0] / total_mass)
    return GraspMetrics(
        max_linear_momentum_error=float(
            np.max(np.linalg.norm(linear_momentum - linear_momentum[0], axis=1))
        ),
        max_com_angular_momentum_error=float(
            np.max(
                np.linalg.norm(com_angular_momentum - com_angular_momentum[0], axis=1)
            )
        ),
        max_origin_angular_momentum_error=float(
            np.max(
                np.linalg.norm(
                    origin_angular_momentum - origin_angular_momentum[0], axis=1
                )
            )
        ),
        max_com_uniform_motion_error=float(
            np.max(np.linalg.norm(system_com - predicted_com, axis=1))
        ),
        initial_mechanical_energy=float(mechanical_energy[0]),
        final_mechanical_energy=float(mechanical_energy[-1]),
        max_mechanical_energy_change=float(
            np.max(np.abs(mechanical_energy - mechanical_energy[0]))
        ),
        first_contact_time=first_contact_time,
        bilateral_contact_duration=float(bilateral_duration),
        withdrawal_bilateral_coverage=withdrawal_coverage,
        fixed_jaw_peak_normal_force=float(np.max(fixed_force)),
        moving_jaw_peak_normal_force=float(np.max(moving_force)),
        target_withdrawal_distance=float(
            np.linalg.norm(target_positions[-1] - target_positions[start_withdrawal])
        ),
        grasp_site_drift=float(grasp_drift),
        final_relative_linear_speed=float(relative_linear_speeds[-1]),
        final_relative_angular_speed=float(relative_angular_speeds[-1]),
        invalid_contact_count=invalid_contact_count,
        max_abs_actuator_command=tuple(np.max(np.abs(commands), axis=0)),
    )


def acceptance_failures(metrics: GraspMetrics) -> list[str]:
    """Return all failed task-level acceptance conditions."""
    checks = (
        (
            metrics.first_contact_time is not None,
            "the gripper never contacts the target",
        ),
        (
            metrics.first_contact_time is None or metrics.first_contact_time >= 4.5,
            "contact occurs before gripper closure starts",
        ),
        (
            metrics.invalid_contact_count == 0,
            "target or spacecraft has invalid contacts",
        ),
        (metrics.fixed_jaw_peak_normal_force > 0.01, "fixed jaw force is too low"),
        (metrics.moving_jaw_peak_normal_force > 0.01, "moving jaw force is too low"),
        (metrics.bilateral_contact_duration >= 0.5, "bilateral contact is too short"),
        (
            metrics.withdrawal_bilateral_coverage >= 0.90,
            "withdrawal bilateral-contact coverage is below 90%",
        ),
        (
            metrics.target_withdrawal_distance >= 0.008,
            "target withdrawal is below 8 mm",
        ),
        (metrics.grasp_site_drift <= 0.003, "grasp-site drift exceeds 3 mm"),
        (
            metrics.final_relative_linear_speed <= 0.005,
            "final relative linear speed exceeds 0.005 m/s",
        ),
        (
            metrics.final_relative_angular_speed <= 0.05,
            "final relative angular speed exceeds 0.05 rad/s",
        ),
        (
            metrics.max_linear_momentum_error <= 5e-5,
            "linear momentum error exceeds 5e-5 kg*m/s",
        ),
        (
            metrics.max_com_angular_momentum_error <= 1e-5,
            "COM angular momentum error exceeds 1e-5 kg*m^2/s",
        ),
        (
            metrics.max_origin_angular_momentum_error <= 1e-5,
            "origin angular momentum error exceeds 1e-5 kg*m^2/s",
        ),
        (
            metrics.max_com_uniform_motion_error <= 5e-5,
            "system COM uniform-motion error exceeds 5e-5 m",
        ),
    )
    return [message for passed, message in checks if not passed]


def _run_recorded() -> SimulationRecord:
    """Run the maneuver and return its recorded generalized state history."""
    simulation, scene, _dynamics_models, recorders = _build_simulation()
    _initialize_state(simulation, scene)
    simulation.ConfigureStopTime(macros.sec2nano(STOP_TIME))
    simulation.ExecuteSimulation()

    state_recorder, *command_recorders = recorders
    times = np.asarray(state_recorder.times()) * macros.NANO2SEC  # [s]
    qpos = np.asarray(state_recorder.qpos).reshape(len(times), -1)
    qvel = np.asarray(state_recorder.qvel).reshape(len(times), -1)
    commands = np.column_stack(
        [np.asarray(recorder.input).reshape(-1) for recorder in command_recorders]
    )  # [N*m]
    return SimulationRecord(times, qpos, qvel, commands)


def _task_phase(time: float) -> str:
    """Return a concise visualization label for the maneuver phase."""
    if time < 1.0:
        return "PREGRASP"
    if time < 4.0:
        return "APPROACH"
    if time < 4.5:
        return "SETTLE"
    if time < 6.0:
        return "CLOSE"
    if time < 7.0:
        return "HOLD"
    if time < 9.0:
        return "WITHDRAW"
    return "VERIFY"


def render_video(
    record: SimulationRecord,
    output_path: Path,
    fps: int = 30,
    width: int = 960,
    height: int = 720,
    contact_debug: bool = False,
) -> None:
    """Render recorded Basilisk states to an H.264 MP4 with MuJoCo."""
    try:
        import imageio_ffmpeg
        from PIL import Image, ImageDraw, ImageFont
    except ImportError as error:
        raise RuntimeError(
            "video export requires imageio-ffmpeg and pillow; run with "
            "--with imageio-ffmpeg --with pillow"
        ) from error

    output_path = output_path.expanduser().resolve()
    if output_path.suffix.lower() != ".mp4":
        raise ValueError("video output path must use the .mp4 extension")
    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = mj.MjModel.from_xml_path(str(MODEL_PATH))
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    data = mj.MjData(model)
    renderer = mj.Renderer(model, height=height, width=width)
    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    camera.distance = 0.72  # [m]
    camera.azimuth = 132.0  # [deg]
    camera.elevation = -18.0  # [deg]
    scene_option = mj.MjvOption()
    scene_option.flags[mj.mjtVisFlag.mjVIS_CONTACTPOINT] = contact_debug
    scene_option.flags[mj.mjtVisFlag.mjVIS_CONTACTFORCE] = contact_debug
    handle_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "capture_target_handle")
    gripper_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "sarm_ee")
    target_site_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, "capture_target_grasp")

    writer = imageio_ffmpeg.write_frames(
        str(output_path),
        (width, height),
        fps=fps,
        codec="libx264",
        pix_fmt_out="yuv420p",
        output_params=["-movflags", "+faststart"],
    )
    writer.send(None)
    font = ImageFont.load_default(size=18)
    frame_times = np.arange(0.0, STOP_TIME + 0.5 / fps, 1.0 / fps)
    try:
        for frame_time in frame_times:
            sample = min(
                int(np.searchsorted(record.times, frame_time)), len(record.times) - 1
            )
            data.qpos[:] = record.qpos[sample]
            data.qvel[:] = record.qvel[sample]
            mj.mj_forward(model, data)
            camera.lookat[:] = 0.5 * (
                data.site_xpos[gripper_site_id] + data.site_xpos[target_site_id]
            )
            renderer.update_scene(data, camera=camera, scene_option=scene_option)
            frame = Image.fromarray(renderer.render())
            draw = ImageDraw.Draw(frame)
            has_handle_contact = any(
                handle_id in (int(contact.geom1), int(contact.geom2))
                for contact in data.contact
            )
            contact_label = "CONTACT" if has_handle_contact else "FREE"
            label = (
                f"{frame_time:05.2f} s   {_task_phase(frame_time)}   {contact_label}"
            )
            draw.rounded_rectangle((18, 16, 380, 50), radius=4, fill=(12, 16, 22, 210))
            draw.text((30, 24), label, font=font, fill=(245, 247, 250))
            writer.send(np.asarray(frame))
    finally:
        writer.close()
        renderer.close()


def run(
    video_path: Path | None = None,
    fps: int = 30,
    contact_debug: bool = False,
) -> GraspMetrics:
    """Run the maneuver, optionally render it, and return grasp metrics."""
    record = _run_recorded()
    metrics = _analyze(record.times, record.qpos, record.qvel, record.commands)
    if video_path is not None:
        render_video(record, video_path, fps=fps, contact_debug=contact_debug)
    return metrics


def _parse_args() -> argparse.Namespace:
    """Parse the scenario command-line interface."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-assert",
        action="store_true",
        help="report metrics without returning failure for unmet grasp criteria",
    )
    parser.add_argument(
        "--video",
        type=Path,
        help="render the recorded maneuver to an H.264 .mp4 file",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="video frame rate; defaults to 30",
    )
    parser.add_argument(
        "--contact-debug",
        action="store_true",
        help="draw MuJoCo contact-force glyphs; these can obscure the handle",
    )
    return parser.parse_args()


def main() -> int:
    """Run the maneuver, print JSON metrics, and enforce acceptance criteria."""
    args = _parse_args()
    if args.fps <= 0:
        raise ValueError("video frame rate must be greater than zero")
    metrics = run(
        video_path=args.video,
        fps=args.fps,
        contact_debug=args.contact_debug,
    )
    print(json.dumps(asdict(metrics), indent=2))
    failures = acceptance_failures(metrics)
    if failures:
        print("Acceptance failures:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
    return 0 if args.no_assert or not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
