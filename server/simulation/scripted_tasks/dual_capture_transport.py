"""Dual-arm snapshots over the same reliable frame/capture transport."""
import time
from .capture_transport import ScriptedCaptureTransport
from simulation.sarm_model_profile import DUAL_ARMS
from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
from space_arm_platform.protocol import CONTROL_PROTOCOL
from space_arm_platform.sampling import DYNAMICS_HZ, SCRIPTED_TASK_HZ, WRENCH_HZ


class DualCaptureTransport(ScriptedCaptureTransport):
    def __init__(self,runtime,**kwargs):
        self.runtime=runtime
        record=runtime.instance['collection_config']['dual_floating']
        super().__init__(runtime.ctx,runtime.task,None,runtime.task.commands,
            record['model_sha256'],runtime.instance,render_model_path=runtime.model,**kwargs)
        try:
            objects={body.object_id:body for body in self.render.bridge._objects}
            for side in DUAL_ARMS:
                for name in ('link1','link2','link3','link4','link5','wrist_ft_tool','ee_finger1','ee_finger2'):
                    body=objects[f'teleop/{side}_{name}']
                    if not any(g.asset_path for g in body.geometries):
                        raise ValueError(f'Missing rendered arm mesh: {body.object_id}')
        except BaseException:
            self.close()
            raise

    def snapshot(self,frame,stamp):
        runtime=self.runtime;stepper=runtime.stepper;bridge=self.render.bridge
        arms={}
        for side,description in DUAL_ARMS.items():
            qa,va=runtime.addresses[side]
            q,v=stepper.qpos[qa],stepper.qvel[va]
            chain=runtime.task.tasks[side].chain
            p,r=chain.forward(q[:6])
            arms[side]=dict(joint_position_rad=q.tolist(),joint_velocity_rad_s=v.tolist(),
                target_joint_position_rad=runtime.task.commands[side].tolist(),
                end_effector_position_body_m=p.tolist(),end_effector_orientation_body_wxyz=matrix_to_quaternion_wxyz(r).tolist(),
                end_effector_twist_body=(chain.jacobian(q[:6])@v[:6]).tolist(),
                wrist_wrench=description.wrench_sensor.observation(stepper,stamp),wrist_wrench_rate_hz=WRENCH_HZ,
                wrist_wrench_samples=self.arm_wrench_samplers[side].drain(stamp))
        samples=runtime.samples(stamp,self.task_elapsed_s)
        return dict(protocol=CONTROL_PROTOCOL,type='observation',observation_layout='dual-arm/1',
            simulation_id='dual-scripted-task',reset_generation=self.client.reset_generation,
            render_session_id=bridge.session_id,**bridge.last_capture_state,
            step_id=str(frame+1),render_frame_id=str(frame),sim_time_ns=str(stamp),wall_time_ns=str(time.time_ns()),
            applied_action_sequence=str(self.sequence),scene_instance_id=self.scene_instance.get('instance_id'),
            scene_seed=self.scene_instance['seed'],scene_template_id=self.scene_instance['template_id'],
            observation_source='authoritative_render_snapshot',model_revision='dual-service-v2-normalized-1',
            model_sha256=self.model_hash,mujoco_runtime_version=3007000,gripper_type='four_bar_tendon',arms=arms,
            orbital_dynamics=stepper.telemetry()['orbital_environment'],dynamics_rate_hz=DYNAMICS_HZ,
            scripted_task_rate_hz=SCRIPTED_TASK_HZ,
            scripted_task=dict(**runtime.task.metrics(samples),elapsed_sim_seconds=self.task_elapsed_s,
                start_sim_time_ns=None if self.task_start_ns is None else str(self.task_start_ns),
                command_source='scripted_joint_reference',right_recheck=runtime.right_recheck))
