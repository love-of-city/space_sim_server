"""Capture12 adaptation of zmh's authoritative paired recording transport."""
import time
import numpy as np
from .capture_transport import ScriptedCaptureTransport
from .capture_cameras import CAMERAS, configure_capture_cameras
from simulation.capture_arm_target import INITIAL_JOINTS
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz, quaternion_wxyz_to_matrix
from simulation.wrench_sensors import WrenchSensor
from space_arm_platform.protocol import CONTROL_PROTOCOL

CAPTURE_SENSOR = WrenchSensor('capture_wrist_ft','capture_wrist_ft_site',
    'capture_wrist_ft_force','capture_wrist_ft_torque','bottom_link6','gripper')


def bolt_seat_observation(stepper, socket):
    """Read actual bolt/owner poses; preserve common spacecraft motion."""
    owner = stepper.body_detail(socket['owner_body'])
    bolt = stepper.body_detail('bolt')
    owner_rotation = quaternion_wxyz_to_matrix(owner[3:7])
    bolt_rotation = quaternion_wxyz_to_matrix(bolt[3:7])
    position = owner_rotation.T @ (bolt[:3] - owner[:3])
    rotation = owner_rotation.T @ bolt_rotation
    seat_position = np.asarray(socket['position_body_m'])
    expected_axis = np.asarray(socket['plug_rotation_body'])[:, 2]
    return dict(relative_position_owner_m=position.tolist(),
                seat_position_error_m=float(np.linalg.norm(position-seat_position)),
                seat_axis_error_rad=float(np.arccos(np.clip(expected_axis@rotation[:, 2], -1., 1.))))


class CaptureGraspTransport(ScriptedCaptureTransport):
    def __init__(self, ctx, task, command, model_hash, scene_instance, **kwargs):
        self.feedback = {}
        profile = scene_instance.get('render_profile', {})
        super().__init__(ctx,task,None,command,model_hash,scene_instance,
                         wrench_sensor=CAPTURE_SENSOR,
                         visual_attachments=profile.get('visual_attachments', []),
                         vision_capture=profile.get('vision_capture'), **kwargs)
        bridge = self.render.bridge
        gp, _ = task.chain.forward(task.reference[:6])
        self.original_cameras = configure_capture_cameras(bridge, gp)

    def payload_retention_snapshot(self, stepper):
        bodies = self.plug_retention.telemetry() if self.plug_retention else {}
        sockets = self.retention_configuration.get('sockets', {}).get('bolt', [])
        if 'bolt' in bodies and sockets:
            bodies['bolt'].update(bolt_seat_observation(stepper, sockets[0]))
        return {**self.retention_configuration, 'bodies': bodies}

    def snapshot(self, frame, stamp):
        scene, stepper = self.ctx.scene, self.ctx.simulation.local_stepper
        if self._addresses is None:
            joints=[scene.getBody(b).getScalarJoint(j) for b,j,_ in INITIAL_JOINTS]
            self._addresses=([j.getQposAdr() for j in joints],[j.getQvelAdr() for j in joints])
        qa,va=self._addresses
        q,v=stepper.qpos[qa],stepper.qvel[va]
        p,r=self.task.chain.forward(q[:6]);bridge=self.render.bridge
        return dict(protocol=CONTROL_PROTOCOL,type='observation',simulation_id='capture-target-grasp',
            reset_generation=self.client.reset_generation,render_session_id=bridge.session_id,
            **bridge.last_capture_state,step_id=str(frame+1),render_frame_id=str(frame),
            sim_time_ns=str(stamp),wall_time_ns=str(time.time_ns()),applied_action_sequence=str(self.sequence),
            scene_instance_id=self.scene_instance.get('instance_id'),scene_seed=self.scene_instance.get('seed'),
            scene_template_id='sarm-dual-capture26',observation_source='authoritative_render_snapshot',
            joint_position_rad=q.tolist(),joint_velocity_rad_s=v.tolist(),target_joint_position_rad=self.command.tolist(),
            joint_position_si=q.tolist(),joint_velocity_si_s=v.tolist(),target_joint_position_si=self.command.tolist(),
            arm_joint_position_rad=q[:6].tolist(),arm_joint_velocity_rad_s=v[:6].tolist(),
            target_arm_joint_position_rad=self.command[:6].tolist(),gripper_type='capture26_four_jaw',
            model_revision='capture26-state-expert/1',model_sha256=self.model_hash,mujoco_runtime_version=3007000,
            end_effector_position_body_m=p.tolist(),end_effector_orientation_body_wxyz=matrix_to_quaternion_wxyz(r).tolist(),
            end_effector_twist_body=(self.task.chain.jacobian(q[:6])@v[:6]).tolist(),
            jacobian_rank=int(np.linalg.matrix_rank(self.task.chain.jacobian(q[:6]))),
            ik_mode='capture_state_expert',command_stale=False,dynamics_rate_hz=400,scripted_task_rate_hz=200,
            wrist_wrench=CAPTURE_SENSOR.observation(stepper,stamp),wrist_wrench_rate_hz=200,
            wrist_wrench_samples=self.wrench_sampler.drain(stamp),
            orbital_dynamics=stepper.telemetry()['orbital_environment'],
            service_gyro=self.ctx.extra['capture_environment']['gyro'].observation(),
            plug_retention=self.payload_retention_snapshot(stepper),
            atmospheric_drag=[c.telemetry() for c in self.ctx.extra['capture_environment']['drag']],
            **self.ctx.simulation.attitude_control.telemetry(),
            scripted_task=dict(**self.task.metrics(),elapsed_sim_seconds=self.task_elapsed_s,
                start_sim_time_ns=None if self.task_start_ns is None else str(self.task_start_ns),
                feedback=self.feedback,command_source='scripted_joint_reference'))
