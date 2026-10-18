"""Live render/observation transport for scripted tasks, using production protocols."""

from __future__ import annotations
import time
import numpy as np

from simulation.control_client import SimulationControlClient
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
from simulation.wrench_sensors import WRIST_SENSOR
from space_arm_platform.protocol import CONTROL_PROTOCOL
from space_arm_platform.sampling import RENDER_HZ, DEFAULT_CAPTURE_HZ, WRENCH_HZ, DYNAMICS_HZ, SCRIPTED_TASK_HZ


class ScriptedCaptureTransport:
    def __init__(
        self,
        ctx,
        task,
        profile,
        command,
        model_hash,
        scene_instance,
        *,
        control_port,
        render_port,
        catalog,
        state_reader=None,
        disturbances=None,
        render_model_path=None,
    ):
        from simulation.components.render import RenderComponent
        from simulation.components.observation import ObservationComponent

        self.ctx, self.task, self.profile = ctx, task, profile
        self.command = command
        self.model_hash = model_hash
        self.scene_instance = scene_instance
        self.state_reader = state_reader
        self.disturbances = disturbances
        self.sequence = 0
        self.task_start_ns = None
        self.task_elapsed_s = 0.0
        self.client = SimulationControlClient(
            "127.0.0.1", control_port, "sarm-scripted-task"
        )
        self.render = None
        self.client.start()
        try:
            self.client.wait_until_ready(timeout=30.0)
            self.render = RenderComponent(
                model_path=render_model_path or ctx.config.model_path,
                catalog=catalog,
                host="127.0.0.1",
                port=render_port,
                render_rate_hz=RENDER_HZ,
                capture_rate_hz=DEFAULT_CAPTURE_HZ,
                capture_state_provider=self.client.capture_state,
                sunlight_intensity_scale=scene_instance.get("environment", {})
                .get("lighting", {})
                .get("sunlight_intensity_scale", 1.0),
                camera_display_names={
                    "spacecraft_overview": "Spacecraft Overview",
                    "sarm_wrist_cam": "SARM Wrist Camera",
                },
            )
            ctx.install([self.render])
            from simulation.wrench_sampling import install_wrench_sampler
            self.wrench_sampler = install_wrench_sampler(ctx, self._sensor_sample if disturbances else None)
            self.observation = ObservationComponent(self.render.bridge, self.snapshot)
            ctx.install([self.observation])
        except BaseException:
            if self.render is not None and self.render.bridge is not None:
                self.render.bridge.close()
            self.client.close()
            raise
        self._addresses = None

    @property
    def enabled(self):
        return bool(self.client.capture_state()["capture_episode_id"])

    def can_advance(self):
        return (
            self.client.has_observation_capacity()
            and self.render.bridge.publisher.has_frame_capacity()
        )

    def flush(self):
        for observation in self.observation.model.drain():
            self.client.send_observation(observation)
        self.client.complete_reset()

    def snapshot(self, frame, stamp):
        scene, stepper = self.ctx.scene, self.ctx.simulation.local_stepper
        if self._addresses is None:
            joints = [
                scene.getBody(b).getScalarJoint(j) for b, j in self.profile.joints
            ]
            self._addresses = (
                [j.getQposAdr() for j in joints],
                [j.getQvelAdr() for j in joints],
            )
            if self.state_reader is None:
                self.drawer = scene.getBody("drawer_base").getScalarJoint(
                    "drawer_base_slide"
                )
                self.buckle = scene.getBody("drawer_buckle").getScalarJoint(
                    "drawer_buckle_joint"
                )
        qa, va = self._addresses
        q, v = stepper.qpos[qa], stepper.qvel[va]
        p, r = self.task.chain.forward(q[:6])
        bridge = self.render.bridge
        payload = dict(
            protocol=CONTROL_PROTOCOL,
            type="observation",
            simulation_id="sarm-scripted-task",
            reset_generation=self.client.reset_generation,
            render_session_id=bridge.session_id,
            **bridge.last_capture_state,
            step_id=str(frame + 1),
            render_frame_id=str(frame),
            sim_time_ns=str(stamp),
            wall_time_ns=str(time.time_ns()),
            applied_action_sequence=str(self.sequence),
            scene_instance_id=self.scene_instance.get("instance_id"),
            scene_seed=self.scene_instance.get("seed"),
            scene_template_id=self.scene_instance.get("template_id"),
            observation_source="authoritative_render_snapshot",
            orbital_dynamics=stepper.telemetry()["orbital_environment"],
            joint_position_rad=q.tolist(),
            joint_velocity_rad_s=v.tolist(),
            target_joint_position_rad=self.command.tolist(),
            arm_joint_position_rad=q[:6].tolist(),
            arm_joint_velocity_rad_s=v[:6].tolist(),
            target_arm_joint_position_rad=self.command[:6].tolist(),
            model_revision="2026-10-12",
            model_sha256=self.model_hash,
            mujoco_runtime_version=3007000,
            gripper_type="four_bar_tendon",
            gripper_drive_position_rad=float(q[6]),
            gripper_drive_velocity_rad_s=float(v[6]),
            target_gripper_drive_position_rad=float(self.command[6]),
            end_effector_position_body_m=p.tolist(),
            end_effector_orientation_body_wxyz=matrix_to_quaternion_wxyz(r).tolist(),
            end_effector_twist_body=(self.task.chain.jacobian(q[:6]) @ v[:6]).tolist(),
            jacobian_rank=int(np.linalg.matrix_rank(self.task.chain.jacobian(q[:6]))),
            ik_mode="scripted_pose_path",
            dynamics_rate_hz=DYNAMICS_HZ,
            scripted_task_rate_hz=SCRIPTED_TASK_HZ,
            command_stale=False,
            wrist_wrench=WRIST_SENSOR.observation(stepper, stamp),
            wrist_wrench_rate_hz=WRENCH_HZ,
            wrist_wrench_samples=self.wrench_sampler.drain(stamp),
            **self.ctx.simulation.attitude_control.telemetry(),
            scripted_task=dict(
                name=getattr(self.task, "name", "drawer_lift"),
                stage=self.task.stage,
                result=self.task.result,
                reason=self.task.reason,
                elapsed_sim_seconds=self.task_elapsed_s,
                start_sim_time_ns=None
                if self.task_start_ns is None
                else str(self.task_start_ns),
                **self._task_metrics(q, v, stamp),
                command_source="scripted_joint_reference",
                task_parameters=vars(self.task.config),
            ),
        )

        if getattr(self, "disturbances", None) is not None:
            d = self.disturbances
            measured_q, measured_v, source_stamp = d.observe(q, v, stamp)
            if any((d.options.joint_observation_noise.enabled, d.options.joint_bias.enabled, d.options.observation_delay.enabled)):
                measured_p, measured_r = self.task.chain.forward(measured_q[:6])
                payload.update(
                    observation_source="authoritative_render_with_measured_proprioception",
                    ground_truth=dict(joint_position_rad=q.tolist(), joint_velocity_rad_s=v.tolist(),
                                      end_effector_position_body_m=p.tolist()),
                    joint_measurement_time_ns=str(source_stamp),
                    joint_measurement_age_ns=str(int(stamp)-source_stamp),
                    joint_position_rad=measured_q.tolist(), joint_velocity_rad_s=measured_v.tolist(),
                    arm_joint_position_rad=measured_q[:6].tolist(), arm_joint_velocity_rad_s=measured_v[:6].tolist(),
                    gripper_drive_position_rad=float(measured_q[6]), gripper_drive_velocity_rad_s=float(measured_v[6]),
                    end_effector_position_body_m=measured_p.tolist(),
                    end_effector_orientation_body_wxyz=matrix_to_quaternion_wxyz(measured_r).tolist(),
                    end_effector_twist_body=(self.task.chain.jacobian(measured_q[:6]) @ measured_v[:6]).tolist(),
                    jacobian_rank=int(np.linalg.matrix_rank(self.task.chain.jacobian(measured_q[:6]))),
                )
            payload["wrist_wrench"] = d.wrench(payload["wrist_wrench"])
        return payload

    def _sensor_sample(self, reading):
        stepper, scene = self.ctx.simulation.local_stepper, self.ctx.scene
        if not hasattr(self, "_sensor_addresses"):
            joints = [scene.getBody(b).getScalarJoint(j) for b, j in self.profile.joints]
            self._sensor_addresses = ([j.getQposAdr() for j in joints], [j.getQvelAdr() for j in joints])
        qa, va = self._sensor_addresses
        self.disturbances.ingest(stepper.qpos[qa], stepper.qvel[va], int(reading["interval_end_time_ns"]))
        return self.disturbances.wrench(reading)

    def _task_metrics(self, q, v, stamp):
        reader = getattr(self, "state_reader", None)
        if reader is not None:
            sample = reader.sample(
                self.task_elapsed_s,
                q,
                v,
                WRIST_SENSOR.observation(self.ctx.simulation.local_stepper, stamp),
            )
            return reader.metrics(sample)
        stepper = self.ctx.simulation.local_stepper
        return dict(
            drawer_position_m=float(stepper.qpos[self.drawer.getQposAdr()]),
            drawer_velocity_m_s=float(stepper.qvel[self.drawer.getQvelAdr()]),
            buckle_rad=float(stepper.qpos[self.buckle.getQposAdr()]),
        )

    def close(self):
        self.render.bridge.close()
        self.client.close()
