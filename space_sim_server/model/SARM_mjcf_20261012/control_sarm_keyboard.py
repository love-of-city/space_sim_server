#!/usr/bin/env python3
"""Keyboard Cartesian control for SARM_convex.xml.

The six arm position actuators are commanded through damped-least-squares IK.
Keyboard input changes the target pose of ``ee_site`` in the world frame and
the common finger tendon actuator opens or closes the gripper.
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import Iterable, Optional

import mujoco
import numpy as np


DEFAULT_XML = Path(__file__).resolve().with_name("SARM_convex.xml")
JOINT_NAMES = tuple("joint{}".format(index) for index in range(1, 7))
ACTUATOR_NAMES = tuple("joint{}_position".format(index) for index in range(1, 7))
INITIAL_JOINT_DEGREES = np.array([121.367,-100.364,68.627,31.737,-90.000,-60])



def named_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    object_id = mujoco.mj_name2id(model, object_type, name)
    if object_id < 0:
        raise ValueError("模型中找不到 {!r}".format(name))
    return object_id


def axis_angle_matrix(axis: Iterable[float], angle: float) -> np.ndarray:
    axis_array = np.asarray(axis, dtype=float)
    axis_array /= np.linalg.norm(axis_array)
    x, y, z = axis_array
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def orientation_error(target: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Return the world-frame rotation vector taking current to target."""
    relative = target @ current.T
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    vector = np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ]
    )
    if angle < 1e-7:
        return 0.5 * vector
    sine = math.sin(angle)
    if abs(sine) < 1e-7:
        # This branch is only expected near 180 degrees. The diagonal provides
        # a stable axis when the skew-symmetric part becomes very small.
        axis = np.sqrt(np.maximum((np.diag(relative) + 1.0) * 0.5, 0.0))
        signs = np.sign(vector)
        signs[signs == 0.0] = 1.0
        axis *= signs
        norm = np.linalg.norm(axis)
        return angle * axis / norm if norm > 1e-9 else np.zeros(3)
    return angle * vector / (2.0 * sine)


class CartesianKeyboardController:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        translation_speed: float,
        rotation_speed: float,
        gripper_speed: float,
        damping: float,
        kinematic_control: bool,
        grasp_translation_speed: Optional[float] = None,
        translation_acceleration: float = 0.5,
        grasp_translation_acceleration: float = 0.1,
        grasp_rotation_speed: Optional[float] = None,
        rotation_acceleration: float = math.radians(180.0),
        grasp_rotation_acceleration: float = math.radians(30.0),
        grasp_joint_speed: float = 0.8,
        plug_gripper_kp: Optional[float] = None,
        plug_gripper_kv: Optional[float] = None,
    ) -> None:
        self.model = model
        self.data = data
        self.ik_data = mujoco.MjData(model)
        self.translation_speed = translation_speed
        self.rotation_speed = rotation_speed
        self.gripper_speed = gripper_speed
        self.damping = damping
        self.kinematic_control = kinematic_control
        self.grasp_translation_speed = (
            translation_speed
            if grasp_translation_speed is None
            else grasp_translation_speed
        )
        self.translation_acceleration = translation_acceleration
        self.grasp_translation_acceleration = grasp_translation_acceleration
        self.grasp_rotation_speed = (
            rotation_speed if grasp_rotation_speed is None else grasp_rotation_speed
        )
        self.rotation_acceleration = rotation_acceleration
        self.grasp_rotation_acceleration = grasp_rotation_acceleration
        self.grasp_joint_speed = grasp_joint_speed
        self.plug_gripper_kp = plug_gripper_kp
        self.plug_gripper_kv = plug_gripper_kv
        self.translation_velocity = np.zeros(3)
        self.rotation_velocity = np.zeros(3)
        self.lock = threading.Lock()
        self.pressed_keys = set()

        self.site_id = named_id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        self.joint_ids = np.array(
            [named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINT_NAMES]
        )
        self.qpos_addresses = model.jnt_qposadr[self.joint_ids]
        self.dof_addresses = model.jnt_dofadr[self.joint_ids]
        self.actuator_ids = np.array(
            [
                named_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in ACTUATOR_NAMES
            ]
        )
        self.gripper_actuator_id = named_id(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, "ee_fingers_position"
        )
        self.default_gripper_kp = float(
            model.actuator_gainprm[self.gripper_actuator_id, 0]
        )
        self.default_gripper_kv = float(
            -model.actuator_biasprm[self.gripper_actuator_id, 2]
        )
        self.finger_body_ids = {
            named_id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            for name in ("ee_finger1", "ee_finger2")
        }
        self.plug_body_ids = {
            body_id for name in (
                "aviation_plug", "guide_plug", "cylindrical_plug", "fastener", "bolt"
            ) if (body_id := mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, name)) >= 0
        }
        self.plug_contact_held = False
        self.contact_scan_counter = 0

        self.jacobian_position = np.zeros((3, model.nv))
        self.jacobian_rotation = np.zeros((3, model.nv))
        self.home_position = np.zeros(3)
        self.home_rotation = np.eye(3)
        self.target_position = np.zeros(3)
        self.target_rotation = np.eye(3)
        self.gripper_target = 0.0
        self.home_gripper_target = 0.0
        self.last_arm_command = np.zeros(6)

    def initialize(self) -> None:
        mujoco.mj_resetData(self.model, self.data)
        initial_radians = np.deg2rad(INITIAL_JOINT_DEGREES)
        self.data.qpos[self.qpos_addresses] = initial_radians
        self.data.ctrl[self.actuator_ids] = initial_radians

        gripper_range = self.model.actuator_ctrlrange[self.gripper_actuator_id]
        self.gripper_target = float(np.clip(0.0, gripper_range[0], gripper_range[1]))
        self.data.ctrl[self.gripper_actuator_id] = self.gripper_target
        mujoco.mj_forward(self.model, self.data)

        self.ik_data.qpos[:] = self.data.qpos
        self.ik_data.qvel[:] = 0.0
        mujoco.mj_kinematics(self.model, self.ik_data)
        mujoco.mj_comPos(self.model, self.ik_data)

        self.home_position = self.data.site_xpos[self.site_id].copy()
        self.home_rotation = self.data.site_xmat[self.site_id].reshape(3, 3).copy()
        self.target_position = self.home_position.copy()
        self.target_rotation = self.home_rotation.copy()
        self.last_arm_command = initial_radians.copy()
        self.translation_velocity[:] = 0.0
        self.rotation_velocity[:] = 0.0
        self.plug_contact_held = False
        self.contact_scan_counter = 0

    def key_event(self, keycode: int, action: int) -> None:
        if keycode is None:
            self.release_all_keys()
            return
        try:
            key = chr(keycode).upper()
        except (TypeError, ValueError, OverflowError):
            return

        controlled_keys = set("WSADQEIKJLUOZX")
        with self.lock:
            if key in controlled_keys:
                if action == 0:  # GLFW_RELEASE
                    self.pressed_keys.discard(key)
                elif action == 1:  # GLFW_PRESS; repeats are not needed.
                    self.pressed_keys.add(key)
            if action != 1:
                return
            if key == "H":
                self.target_position = self.home_position.copy()
                self.target_rotation = self.home_rotation.copy()
                self.gripper_target = self.home_gripper_target
                self.translation_velocity[:] = 0.0
                self.rotation_velocity[:] = 0.0
                self.plug_contact_held = False
            elif key == "P":
                position = self.data.site_xpos[self.site_id]
                joints = np.rad2deg(self.data.qpos[self.qpos_addresses])
                print(
                    "ee=[{:.4f}, {:.4f}, {:.4f}] m, joints={} deg, gripper={:.2f}".format(
                        *position, np.round(joints, 2).tolist(), self.gripper_target
                    )
                )

    def release_all_keys(self) -> None:
        with self.lock:
            self.pressed_keys.clear()

    def advance_targets(self, elapsed: float) -> None:
        """Move targets continuously using real elapsed time while keys are held."""
        translations = {
            "W": np.array([1.0, 0.0, 0.0]),
            "S": np.array([-1.0, 0.0, 0.0]),
            "A": np.array([0.0, 1.0, 0.0]),
            "D": np.array([0.0, -1.0, 0.0]),
            "Q": np.array([0.0, 0.0, 1.0]),
            "E": np.array([0.0, 0.0, -1.0]),
        }
        rotations = {
            "I": np.array([1.0, 0.0, 0.0]),
            "K": np.array([-1.0, 0.0, 0.0]),
            "J": np.array([0.0, 1.0, 0.0]),
            "L": np.array([0.0, -1.0, 0.0]),
            "U": np.array([0.0, 0.0, 1.0]),
            "O": np.array([0.0, 0.0, -1.0]),
        }

        with self.lock:
            grasping = self.gripper_target <= 0.2
            # Keyboard intent is measured in wall time. The old grasp branch
            # used at most one 1-ms physics step here; when a contact-heavy
            # scene ran at 0.1-0.3x real time, a held key moved the target
            # only 0.1-0.3x as far and its acceleration ramp was equally slow.
            # Physical arm motion is still limited per simulation step below.
            motion_elapsed = min(max(elapsed, 0.0), 0.05)
            translation = sum(
                (translations[key] for key in self.pressed_keys if key in translations),
                np.zeros(3),
            )
            norm = np.linalg.norm(translation)
            # A grasped object needs a much gentler Cartesian command than an
            # empty gripper.  Ramp velocity instead of applying an instantaneous
            # step, which otherwise produces tangential contact slip and can
            # lever the object out of the fingertip recess.
            speed_limit = (
                self.grasp_translation_speed if grasping else self.translation_speed
            )
            acceleration_limit = (
                self.grasp_translation_acceleration if grasping
                else self.translation_acceleration
            )
            desired_velocity = (
                speed_limit * translation / norm if norm > 0.0 else np.zeros(3)
            )
            if norm == 0.0 and np.linalg.norm(self.translation_velocity) > 0.0:
                # On release, hold the measured EE position.  Letting the
                # target coast through its acceleration ramp made the arm
                # travel another several millimetres after the key was up.
                self.translation_velocity[:] = 0.0
                self.target_position[:] = self.data.site_xpos[self.site_id]
            else:
                velocity_delta = desired_velocity - self.translation_velocity
                velocity_delta_norm = np.linalg.norm(velocity_delta)
                max_velocity_delta = acceleration_limit * motion_elapsed
                if velocity_delta_norm > max_velocity_delta > 0.0:
                    velocity_delta *= max_velocity_delta / velocity_delta_norm
                self.translation_velocity += velocity_delta
            self.target_position += self.translation_velocity * motion_elapsed
            # Do not let a lagging IK/physics loop accumulate a large target
            # backlog.  Otherwise the arm keeps moving after key release and
            # a grasped free body swings before the controller catches up.
            current_position = self.data.site_xpos[self.site_id]
            pose_error = self.target_position - current_position
            distance = np.linalg.norm(pose_error)
            max_backlog = 0.004 if grasping else 0.006
            if distance > max_backlog:
                self.target_position = (
                    current_position + pose_error * (max_backlog / distance)
                )

            rotation = sum(
                (rotations[key] for key in self.pressed_keys if key in rotations),
                np.zeros(3),
            )
            norm = np.linalg.norm(rotation)
            # The UI target follows wall time so held keys remain responsive
            # in a contact-heavy scene. The low grasp speed, acceleration
            # ramp, 3-degree target cap, and per-step joint-speed cap keep the
            # actual contact motion bounded when physics runs slowly.
            rotation_elapsed = motion_elapsed
            rotation_speed = self.grasp_rotation_speed if grasping else self.rotation_speed
            angular_acceleration = (
                self.grasp_rotation_acceleration if grasping
                else self.rotation_acceleration
            )
            desired_rotation_velocity = (
                rotation_speed * rotation / norm if norm > 0 else np.zeros(3)
            )
            if norm == 0.0 and np.linalg.norm(self.rotation_velocity) > 0.0:
                self.rotation_velocity[:] = 0.0
                self.target_rotation[:] = self.data.site_xmat[
                    self.site_id].reshape(3, 3)
            else:
                angular_delta = desired_rotation_velocity - self.rotation_velocity
                angular_delta_norm = np.linalg.norm(angular_delta)
                max_angular_delta = angular_acceleration * rotation_elapsed
                if angular_delta_norm > max_angular_delta > 0:
                    angular_delta *= max_angular_delta / angular_delta_norm
                self.rotation_velocity += angular_delta
            angular_speed = np.linalg.norm(self.rotation_velocity)
            if angular_speed > 0:
                increment = axis_angle_matrix(
                    self.rotation_velocity / angular_speed,
                    angular_speed * rotation_elapsed,
                )
                self.target_rotation = increment @ self.target_rotation
            if grasping:
                current_rotation = self.data.site_xmat[self.site_id].reshape(3, 3)
                pose_error = orientation_error(self.target_rotation, current_rotation)
                error_angle = np.linalg.norm(pose_error)
                max_error = math.radians(3.0)
                if error_angle > max_error:
                    self.target_rotation = (
                        axis_angle_matrix(pose_error / error_angle, max_error)
                        @ current_rotation
                    )

            gripper_direction = float("Z" in self.pressed_keys) - float(
                "X" in self.pressed_keys
            )
            if gripper_direction:
                self._change_gripper(
                    gripper_direction * self.gripper_speed
                    * min(elapsed, self.model.opt.timestep)
                )

    def _change_gripper(self, delta: float) -> None:
        control_range = self.model.actuator_ctrlrange[self.gripper_actuator_id]
        self.gripper_target = float(
            np.clip(self.gripper_target + delta, control_range[0], control_range[1])
        )

    def update(self, elapsed: Optional[float] = None) -> None:
        with self.lock:
            target_position = self.target_position.copy()
            target_rotation = self.target_rotation.copy()
            gripper_target = self.gripper_target

        if not self.kinematic_control:
            # This is a position-actuated arm, not a kinematic follower.  The
            # private IK state must stay near the *measured* joints; otherwise
            # it silently integrates commands that the physical arm has not
            # reached and later snaps back when contacts resist the motion.
            measured = self.data.qpos[self.qpos_addresses]
            reference = self.ik_data.qpos[self.qpos_addresses]
            self.ik_data.qpos[self.qpos_addresses] = measured + np.clip(
                reference - measured, -0.02, 0.02)
            mujoco.mj_kinematics(self.model, self.ik_data)
            mujoco.mj_comPos(self.model, self.ik_data)

        if self.plug_gripper_kp is not None:
            self.contact_scan_counter += 1
            if self.contact_scan_counter % 10 == 0:
                ee_position = self.data.site_xpos[self.site_id]
                near_plug = any(
                    np.linalg.norm(self.data.xpos[body] - ee_position) < 0.06
                    for body in self.plug_body_ids
                )
                if near_plug:
                    # Select the firmer gain before the fingertips first touch.
                    self.plug_contact_held = True
                elif gripper_target > 0.5:
                    self.plug_contact_held = False
                elif not self.plug_contact_held:
                    for contact in self.data.contact:
                        body1 = self.model.geom_bodyid[contact.geom1]
                        body2 = self.model.geom_bodyid[contact.geom2]
                        if ((body1 in self.finger_body_ids and body2 in self.plug_body_ids)
                                or (body2 in self.finger_body_ids and body1 in self.plug_body_ids)):
                            self.plug_contact_held = True
                            break
            stiffness = (
                self.plug_gripper_kp if self.plug_contact_held
                else self.default_gripper_kp
            )
            damping = (
                (self.plug_gripper_kv if self.plug_gripper_kv is not None
                 else 0.1 * stiffness)
                if self.plug_contact_held else self.default_gripper_kv
            )
            if (self.model.actuator_gainprm[self.gripper_actuator_id, 0] != stiffness
                    or -self.model.actuator_biasprm[self.gripper_actuator_id, 2] != damping):
                self.model.actuator_gainprm[self.gripper_actuator_id, 0] = stiffness
                self.model.actuator_biasprm[self.gripper_actuator_id, 1] = -stiffness
                self.model.actuator_biasprm[self.gripper_actuator_id, 2] = -damping

        mujoco.mj_jacSite(
            self.model,
            self.ik_data,
            self.jacobian_position,
            self.jacobian_rotation,
            self.site_id,
        )
        current_rotation = self.ik_data.site_xmat[self.site_id].reshape(3, 3)
        error = np.concatenate(
            (
                target_position - self.ik_data.site_xpos[self.site_id],
                orientation_error(target_rotation, current_rotation),
            )
        )
        jacobian = np.vstack(
            (self.jacobian_position, self.jacobian_rotation)
        )[:, self.dof_addresses]

        regularized = jacobian @ jacobian.T + (self.damping ** 2) * np.eye(6)
        joint_delta = jacobian.T @ np.linalg.solve(regularized, error)
        delta_norm = np.linalg.norm(joint_delta)
        if delta_norm > 0.08:
            joint_delta *= 0.08 / delta_norm
        if self.kinematic_control:
            # qpos and qvel must describe the same motion. The old 0.08-rad
            # per-step jump paired with qvel clipped to 4 rad/s could inject a
            # large unmodelled tangential impulse during wrist rotation.
            max_joint_speed = (
                self.grasp_joint_speed if gripper_target <= 0.2 else 4.0
            )
            max_joint_delta = max_joint_speed * self.model.opt.timestep
            largest_delta = np.max(np.abs(joint_delta))
            if largest_delta > max_joint_delta:
                joint_delta *= max_joint_delta / largest_delta

        command = self.ik_data.qpos[self.qpos_addresses] + joint_delta
        if not self.kinematic_control:
            # Bound the servo's pose error to ~15 Nm at kp=500.  This still
            # drives through contact but avoids a large corrective impulse
            # after an IK or rendering stall.
            measured = self.data.qpos[self.qpos_addresses]
            command = measured + np.clip(command - measured, -0.03, 0.03)
        for index, (joint_id, actuator_id) in enumerate(
            zip(self.joint_ids, self.actuator_ids)
        ):
            if self.model.jnt_limited[joint_id]:
                command[index] = np.clip(
                    command[index], *self.model.jnt_range[joint_id]
                )
            if self.model.actuator_ctrllimited[actuator_id]:
                command[index] = np.clip(
                    command[index], *self.model.actuator_ctrlrange[actuator_id]
                )

        self.ik_data.qpos[self.qpos_addresses] = command
        mujoco.mj_kinematics(self.model, self.ik_data)
        mujoco.mj_comPos(self.model, self.ik_data)
        self.data.ctrl[self.actuator_ids] = command
        self.data.ctrl[self.gripper_actuator_id] = gripper_target
        if self.kinematic_control:
            # Direct qpos tracking is responsive, but contacts also need a
            # physically meaningful qvel. Without it, tangential friction sees
            # a stationary gripper and a grasped free object is left behind.
            # qpos is imposed once per MuJoCo step, so the matching velocity
            # must use simulation dt rather than wall-clock dt. Using wall time
            # makes the contact velocity much smaller than the actual qpos jump
            # when this contact-heavy model runs slower than real time, causing
            # a grasped object to lag behind and slip out.
            command_velocity = (
                command - self.last_arm_command
            ) / self.model.opt.timestep
            command_velocity = np.clip(command_velocity, -4.0, 4.0)
            self.data.qpos[self.qpos_addresses] = command
            self.data.qvel[self.dof_addresses] = command_velocity
            self.last_arm_command = command.copy()


class MinimalMujocoViewer:
    """Small GLFW viewer with no built-in MuJoCo keyboard shortcuts."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        key_callback,
    ) -> None:
        from mujoco.glfw import glfw

        self.glfw = glfw
        self.model = model
        self.data = data
        self.key_callback = key_callback
        self.button_left = False
        self.button_middle = False
        self.button_right = False
        self.last_x = 0.0
        self.last_y = 0.0
        self._camera_move_needs_scene = None

        if not glfw.init():
            raise RuntimeError("无法初始化 GLFW；请确认当前环境支持图形窗口")
        self.window = glfw.create_window(960, 540, "SARM keyboard control", None, None)
        if not self.window:
            glfw.terminate()
            raise RuntimeError("无法创建 GLFW 窗口")
        glfw.make_context_current(self.window)
        # Rendering must not throttle the 1 ms physics loop through VSync.
        glfw.swap_interval(0)

        self.camera = mujoco.MjvCamera()
        self.option = mujoco.MjvOption()
        # Thousands of transparent convex collision pieces need not enter the
        # visual scene.  In software OpenGL they can make one frame slower
        # than many physics steps and make the keyboard seem unresponsive.
        self.option.geomgroup[3] = 0
        self.scene = mujoco.MjvScene(model, maxgeom=max(10000, model.ngeom * 2))
        self.context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        self.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 0
        self.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = 0
        mujoco.mjv_defaultCamera(self.camera)
        mujoco.mjv_defaultOption(self.option)

        glfw.set_key_callback(self.window, self._on_key)
        glfw.set_window_focus_callback(self.window, self._on_focus)
        glfw.set_cursor_pos_callback(self.window, self._on_mouse_move)
        glfw.set_mouse_button_callback(self.window, self._on_mouse_button)
        glfw.set_scroll_callback(self.window, self._on_scroll)

    def _on_key(self, window, key, scancode, action, mods) -> None:
        del scancode, mods
        if key == self.glfw.KEY_ESCAPE and action == self.glfw.PRESS:
            self.glfw.set_window_should_close(window, True)
            return
        self.key_callback(key, action)

    def _on_focus(self, window, focused) -> None:
        del window
        if not focused:
            self.key_callback(None, 0)

    def _on_mouse_button(self, window, button, action, mods) -> None:
        del button, action, mods
        self.button_left = (
            self.glfw.get_mouse_button(window, self.glfw.MOUSE_BUTTON_LEFT)
            == self.glfw.PRESS
        )
        self.button_middle = (
            self.glfw.get_mouse_button(window, self.glfw.MOUSE_BUTTON_MIDDLE)
            == self.glfw.PRESS
        )
        self.button_right = (
            self.glfw.get_mouse_button(window, self.glfw.MOUSE_BUTTON_RIGHT)
            == self.glfw.PRESS
        )
        self.last_x, self.last_y = self.glfw.get_cursor_pos(window)

    def _move_camera(self, action, delta_x, delta_y) -> None:
        """Support both MuJoCo camera APIs (with and without ``scene``)."""
        if self._camera_move_needs_scene is None:
            try:
                mujoco.mjv_moveCamera(
                    self.model, action, delta_x, delta_y, self.camera
                )
            except TypeError:
                self._camera_move_needs_scene = True
            else:
                self._camera_move_needs_scene = False
                return
        if self._camera_move_needs_scene:
            mujoco.mjv_moveCamera(
                self.model, action, delta_x, delta_y, self.scene, self.camera
            )
        else:
            mujoco.mjv_moveCamera(
                self.model, action, delta_x, delta_y, self.camera
            )

    def _on_mouse_move(self, window, x_position, y_position) -> None:
        if not (self.button_left or self.button_middle or self.button_right):
            return
        delta_x = x_position - self.last_x
        delta_y = y_position - self.last_y
        self.last_x = x_position
        self.last_y = y_position

        _, height = self.glfw.get_window_size(window)
        shift = (
            self.glfw.get_key(window, self.glfw.KEY_LEFT_SHIFT) == self.glfw.PRESS
            or self.glfw.get_key(window, self.glfw.KEY_RIGHT_SHIFT) == self.glfw.PRESS
        )
        if self.button_right:
            camera_action = (
                mujoco.mjtMouse.mjMOUSE_MOVE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_MOVE_V
            )
        elif self.button_left:
            camera_action = (
                mujoco.mjtMouse.mjMOUSE_ROTATE_H
                if shift
                else mujoco.mjtMouse.mjMOUSE_ROTATE_V
            )
        else:
            camera_action = mujoco.mjtMouse.mjMOUSE_ZOOM
        self._move_camera(
            camera_action, delta_x / max(height, 1), delta_y / max(height, 1)
        )

    def _on_scroll(self, window, x_offset, y_offset) -> None:
        del window, x_offset
        self._move_camera(mujoco.mjtMouse.mjMOUSE_ZOOM, 0.0, -0.05 * y_offset)

    def is_running(self) -> bool:
        return not self.glfw.window_should_close(self.window)

    def render(self) -> None:
        width, height = self.glfw.get_framebuffer_size(self.window)
        viewport = mujoco.MjrRect(0, 0, width, height)
        mujoco.mjv_updateScene(
            self.model,
            self.data,
            self.option,
            None,
            self.camera,
            mujoco.mjtCatBit.mjCAT_ALL,
            self.scene,
        )
        mujoco.mjr_render(viewport, self.scene, self.context)
        self.glfw.swap_buffers(self.window)
        self.glfw.poll_events()

    def close(self) -> None:
        self.context.free()
        self.glfw.destroy_window(self.window)
        self.glfw.terminate()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()


def print_controls(bolt_mode: bool = False, fastener_mode: bool = False) -> None:
    print(
        """
键盘控制（世界坐标系）：
  W / S : 末端 +X / -X
  A / D : 末端 +Y / -Y
  Q / E : 末端 +Z / -Z
  I / K : 绕世界 X 轴正转 / 反转
  J / L : 绕世界 Y 轴正转 / 反转
  U / O : 绕世界 Z 轴正转 / 反转
  Z / X : 夹爪张开 / 闭合
  H     : 目标位姿和夹爪恢复初始值
  P     : 打印当前末端位置、关节角和夹爪控制量
  Esc   : 关闭窗口

鼠标视角：左键旋转、右键平移、中键或滚轮缩放。
""".strip()
    )
    if bolt_mode:
        print("螺栓模式：先用双指夹住螺栓并移到装配孔附近；T 启动接触式"
              "自动对孔、入牙相位校正和轴向预紧，之后按住 O 旋入、U 旋出；再按 T 退出。"
              "辅助仅控制机械臂，不改写螺栓位姿。")
    if fastener_mode:
        print("紧固件模式：先用双指夹住紧固件上端；T 启用辅助，G 一键旋紧，"
              "V 一键旋松，B 一键沿铰链圆弧拨出并拨回，N 中止，T 退出。"
              "旋拧与拨杆时末端位置和姿态同步调整；只驱动机械臂，不写入零件位姿。")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="键盘控制 SARM 末端位姿及夹爪")
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML)
    parser.add_argument(
        "--bolt-mode", action="store_true",
        help="使用当前 SARM_convex.xml 的 SDF 螺纹、动力学机械臂及 T/O/U 旋拧辅助",
    )
    parser.add_argument(
        "--fastener-mode", action="store_true",
        help="启用紧固件 SDF 螺纹和 T/G/V/B 一键旋拧、圆弧拨杆辅助",
    )
    parser.add_argument(
        "--translation-speed", type=float, default=0.04, help="按住按键时的平移速度（m/s）"
    )
    parser.add_argument(
        "--grasp-translation-speed",
        type=float,
        default=0.02,
        help="夹爪闭合时的键盘目标平移速度上限（真实时间 m/s）",
    )
    parser.add_argument(
        "--translation-acceleration",
        type=float,
        default=0.5,
        help="平移指令的加速度上限（m/s^2）",
    )
    parser.add_argument(
        "--grasp-translation-acceleration",
        type=float,
        default=0.1,
        help="夹爪闭合时的平移加速度上限（m/s^2）",
    )
    parser.add_argument(
        "--rotation-speed", type=float, default=45.0, help="按住按键时的旋转速度（度/s）"
    )
    parser.add_argument(
        "--grasp-rotation-speed", type=float, default=8.0,
        help="夹爪闭合时的键盘目标旋转速度上限（真实时间度/s）",
    )
    parser.add_argument(
        "--grasp-rotation-acceleration", type=float, default=30.0,
        help="夹爪闭合时的角加速度上限（仿真度/s^2）",
    )
    parser.add_argument(
        "--grasp-joint-speed", type=float, default=0.8,
        help="夹爪闭合时的关节速度上限（仿真rad/s）",
    )
    parser.add_argument(
        "--gripper-speed", type=float, default=4.0, help="夹爪控制量每秒变化值"
    )
    parser.add_argument(
        "--gripper-kp", type=float, default=80.0,
        help="检测到夹持插头等自由物体后采用的夹爪刚度（默认 80；抽屉仍用 XML 值）",
    )
    parser.add_argument(
        "--gripper-kv", type=float,
        help="夹持自由物体时的夹爪速度阻尼；默认与刚度保持原有 1:10 比例",
    )
    parser.add_argument("--ik-damping", type=float, default=0.03)
    parser.add_argument(
        "--dynamic-control",
        action="store_true",
        help="使用 XML 中的动力学位置执行器；默认采用响应更快的运动学位置跟踪",
    )
    parser.add_argument("--render-fps", type=float, default=30.0)
    parser.add_argument("--headless", action="store_true", help="不打开窗口，仅用于加载测试")
    parser.add_argument("--duration", type=float, default=1.0, help="无窗口测试时长（秒）")
    args = parser.parse_args()
    if args.bolt_mode and args.fastener_mode:
        parser.error("--bolt-mode 和 --fastener-mode 不能同时启用")
    if args.bolt_mode or args.fastener_mode:
        args.dynamic_control = True
        args.gripper_kp = max(args.gripper_kp, 200.0)
    if (
        args.translation_speed <= 0
        or args.grasp_translation_speed <= 0
        or args.translation_acceleration <= 0
        or args.grasp_translation_acceleration <= 0
        or args.rotation_speed <= 0
        or args.grasp_rotation_speed <= 0
        or args.grasp_rotation_acceleration <= 0
        or args.grasp_joint_speed <= 0
    ):
        parser.error("平移和旋转速度必须大于 0")
    if (args.gripper_speed <= 0 or args.gripper_kp <= 0
            or (args.gripper_kv is not None and args.gripper_kv <= 0)
            or args.ik_damping <= 0):
        parser.error("夹爪速度和 IK 阻尼必须大于 0")
    if args.render_fps <= 0 or args.duration <= 0:
        parser.error("渲染帧率和测试时长必须大于 0")
    return args


def main() -> None:
    args = parse_args()
    xml_path = args.xml.expanduser().resolve()
    if not xml_path.is_file():
        raise FileNotFoundError("找不到模型文件: {}".format(xml_path))

    if (args.bolt_mode or args.fastener_mode) and (
            "sarm.sdf.two_start_thread" not in xml_path.read_text(encoding="utf-8")):
        raise ValueError("螺纹专用模式需要包含 SDF 螺纹的 MJCF")

    if "sarm.sdf.two_start_thread" in xml_path.read_text(encoding="utf-8"):
        library = Path(__file__).resolve().parent / "libtwo_start_thread.so"
        if not library.is_file():
            raise FileNotFoundError(
                "请先运行 build_thread_sdf.py 编译双线螺纹 SDF: {}".format(library)
            )
        mujoco.mj_loadPluginLibrary(str(library))

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    controller = CartesianKeyboardController(
        model=model,
        data=data,
        translation_speed=args.translation_speed,
        rotation_speed=math.radians(args.rotation_speed),
        gripper_speed=args.gripper_speed,
        damping=args.ik_damping,
        kinematic_control=not args.dynamic_control,
        grasp_translation_speed=args.grasp_translation_speed,
        translation_acceleration=args.translation_acceleration,
        grasp_translation_acceleration=args.grasp_translation_acceleration,
        grasp_rotation_speed=math.radians(args.grasp_rotation_speed),
        grasp_rotation_acceleration=math.radians(args.grasp_rotation_acceleration),
        grasp_joint_speed=args.grasp_joint_speed,
        plug_gripper_kp=args.gripper_kp,
        plug_gripper_kv=args.gripper_kv,
    )
    controller.initialize()
    bolt_assist = None
    fastener_assist = None
    if args.bolt_mode:
        from bolt_screw_assist import BoltScrewAssist, prewind_wrist
        if prewind_wrist(model, data, controller):
            print("已把第 6 关节预绕到等价姿态，为旋拧留出一整圈行程")
        bolt_assist = BoltScrewAssist(model, data, controller)
    if args.fastener_mode:
        from fastener_locking_assist import FastenerLockingAssist
        fastener_assist = FastenerLockingAssist(model, data, controller)
    controller.gripper_target = 1.0
    controller.home_gripper_target = 1.0
    data.ctrl[controller.gripper_actuator_id] = 1.0
    # Both modes start with the *physical* four-bar gripper open.  Merely
    # setting ctrl=1 would still show closed fingers in the first GUI frame.
    for _ in range(1000):
        controller.update()
        mujoco.mj_step(model, data)
    print("初始夹爪已张开：腱长 {:.3f}".format(
        data.ten_length[model.tendon("ee_active_links_tendon").id]))
    print("已加载 {}，初始关节角（度）: {}".format(xml_path.name, INITIAL_JOINT_DEGREES.tolist()))

    if args.headless:
        end_time = data.time + args.duration
        while data.time < end_time:
            controller.update()
            mujoco.mj_step(model, data)
        print(
            "无窗口测试完成: time={:.3f}s, joints={} deg".format(
                data.time,
                np.round(np.rad2deg(data.qpos[controller.qpos_addresses]), 3).tolist(),
            )
        )
        return

    print_controls(args.bolt_mode, args.fastener_mode)

    def keyboard_event(keycode, action):
        if fastener_assist is not None and action == 1:
            if keycode == ord("T"):
                fastener_assist.toggle()
                return
            if keycode in (ord("G"), ord("V"), ord("B")):
                fastener_assist.command(chr(keycode))
                return
            if keycode == ord("N"):
                fastener_assist.cancel()
                return
            if keycode == ord("H"):
                fastener_assist.cancel()
        if bolt_assist is not None and keycode == ord("T") and action == 1:
            bolt_assist.toggle()
            return
        if bolt_assist is not None and keycode == ord("H") and action == 1:
            if bolt_assist.mode != "off":
                bolt_assist.toggle()
        controller.key_event(keycode, action)
        if bolt_assist is not None and keycode == ord("P") and action == 1:
            bolt_assist.report()
        if fastener_assist is not None and keycode == ord("P") and action == 1:
            fastener_assist.report()

    with MinimalMujocoViewer(model, data, keyboard_event) as viewer:
        viewer.camera.lookat[:] = data.site_xpos[controller.site_id]
        viewer.camera.distance = 1.5
        viewer.camera.azimuth = 135.0
        viewer.camera.elevation = -25.0
        if args.fastener_mode:
            viewer.camera.lookat[:] = [0.438, 0.565, 0.105]
            viewer.camera.distance = 0.65
        last_wall_time = time.perf_counter()
        next_render_time = last_wall_time
        render_period = 1.0 / args.render_fps
        steps_since_render = 0
        min_steps_per_render = max(1, int(0.02 / model.opt.timestep))
        while viewer.is_running():
            step_start = time.perf_counter()
            elapsed = min(step_start - last_wall_time, 0.05)
            last_wall_time = step_start
            if fastener_assist is not None and fastener_assist.is_moving:
                fastener_assist.prepare_step()
            elif bolt_assist is not None and bolt_assist.mode != "off":
                bolt_assist.prepare_step(controller.pressed_keys)
            else:
                controller.advance_targets(elapsed)
            controller.update(elapsed)
            if bolt_assist is not None:
                bolt_assist.apply_after_controller()
            mujoco.mj_step(model, data)
            steps_since_render += 1
            now = time.perf_counter()
            if now >= next_render_time and steps_since_render >= min_steps_per_render:
                viewer.render()
                steps_since_render = 0
                # Start the interval *after* the potentially expensive render.
                # Scheduling from the pre-render timestamp causes a runaway
                # render-every-physics-step loop when rendering is slow.
                next_render_time = time.perf_counter() + render_period
            remaining = model.opt.timestep - (time.perf_counter() - step_start)
            if remaining > 0:
                time.sleep(remaining)


if __name__ == "__main__":
    main()
