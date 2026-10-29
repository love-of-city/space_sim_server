"""Continue a parked capture26 with the same navigation and physical runtime."""
from pathlib import Path
from types import SimpleNamespace
import json
import os
import time

import numpy as np


class OperationsArmClient:
    """Enable arm motion after the receiving scene satisfies its readiness rules."""
    def __init__(self, client):
        self.client = client
        self.enabled = False

    def latest_action(self):
        action, stale = self.client.latest_action()
        if not self.enabled:
            action = dict(action, deadman=False, arm_preparation=None, allow_reference_recovery=False)
        return action, stale or not self.enabled


class OperationsControlBridge:
    """Reuse the existing two operator arm controllers; never reset the scene."""
    def __init__(self, args, model_path, profile, reference):
        from simulation.teleop_grasp_unreal import CartesianTeleopTarget, load_joint_limits
        from simulation.serial_chain_kinematics import SerialChainKinematics
        self.args = args
        self.client = args.operation_client
        self.arm_client = OperationsArmClient(self.client)
        self.kinematics = SerialChainKinematics.from_mjcf(model_path,
            base_body='cubesat_bus', joint_names=profile.joint_names[:6], tool_site='sarm_ee')
        class ParkedOperatingTarget(CartesianTeleopTarget):
            def update(self, sim_seconds):
                if not self.client.enabled:
                    self.velocity[:] = 0.
                    self.last_sim_seconds = sim_seconds
                    return self.cached_reference(sim_seconds)
                return super().update(sim_seconds)

        self.target = ParkedOperatingTarget(np.asarray(reference), self.arm_client, self.kinematics,
            joint_limits=load_joint_limits(model_path, profile.joint_names), ik_mode=args.ik_mode)
        self.failed = ''
        self.last_reset = False

    def install(self, ctx, profile):
        from simulation.components.teleop_ik import TeleopIkComponent
        # These are joint encoders for arm control, not spacecraft truth used
        # by navigation. Bulk joint states remain available at 400 Hz.
        self.positions = [ctx.ports.joint_state(n).addSubscriber() for n in profile.joint_names]
        self.rates = [ctx.ports.joint_rate(n).addSubscriber() for n in profile.joint_names]
        self.target.bind_joint_state_provider(lambda: np.array([r().state for r in self.positions[:6]]))
        self.target.bind_joint_velocity_provider(lambda: np.array([r().state for r in self.rates[:6]]))
        ctx.install([TeleopIkComponent(self.target, ik_rate_hz=self.args.ik_rate)])

    def fail(self, reason):
        self.failed = self.failed or reason
        self.arm_client.enabled = False

    def publish(self, core, nav, gyros, capture, gate, now, bridge, guidance, capacity):
        from space_arm_platform.protocol import CONTROL_PROTOCOL
        from simulation.serial_chain_kinematics import matrix_to_quaternion_wxyz
        if not self.client.connected:
            self.fail('operator communication lost')
        if self.client.take_reset() is not None:
            raise RuntimeError('continuous capture26 scene cannot reset physical state; start a new mission')
        if not self.client.has_observation_capacity():
            self.fail('operator telemetry acknowledgement interrupted')
            return  # No pause of physics or unbounded enqueue on this path.
        q, v = (np.array([r().state for r in readers]) for readers in (self.positions, self.rates))
        p, r = self.kinematics.forward(q[:6])
        from simulation.capture26_runtime import demo_telemetry
        data = dict(protocol=CONTROL_PROTOCOL, type='observation', simulation_id='sarm-teleop',
            reset_generation=self.client.reset_generation,
            render_session_id=bridge.session_id, render_frame_id=str(bridge.last_published_frame_id),
            **bridge.last_capture_state,
            sim_time_ns=str(now), wall_time_ns=str(time.time_ns()), step_id=str(now//100_000_000),
            scene_instance_id=self.args.scene_instance['instance_id'], scene_seed=self.args.seed,
            scene_template_id='sarm-dual-capture26', model_revision='2026-10-12',
            model_sha256=core.model_xml_sha256, gripper_type='four_bar_tendon',
            observation_source='authoritative_render_snapshot', applied_action_sequence=self.target.applied_sequence,
            joint_position_rad=q.tolist(), joint_velocity_rad_s=v.tolist(),
            target_joint_position_rad=self.target.position.tolist(),
            arm_joint_position_rad=q[:6].tolist(), arm_joint_velocity_rad_s=v[:6].tolist(),
            target_arm_joint_position_rad=self.target.position[:6].tolist(),
            gripper_drive_position_rad=float(q[6]), gripper_drive_velocity_rad_s=float(v[6]),
            target_gripper_drive_position_rad=float(self.target.position[6]),
            end_effector_position_body_m=p.tolist(),
            end_effector_orientation_body_wxyz=matrix_to_quaternion_wxyz(r).tolist(),
            end_effector_twist_body=(self.kinematics.jacobian(q[:6])@v[:6]).tolist(),
            capture_arm=capture.telemetry(), service_gyro=gyros[0].observation(),
            target_gyro=gyros[1].observation(),
            capture26_navigation=dict(phase=guidance['phase'],
                **demo_telemetry(guidance), optical_depth_m=guidance.get('optical_depth_m'),
                gyro_health=nav.gyro_health(now), visual=gate.status(now, time.time_ns()),
                gnss_healthy=guidance.get('gnss_healthy', False),
                platform_stable_duration_s=guidance.get('platform_stable_duration_s', 0.),
                platform_stable=guidance.get('platform_stable', False), released=guidance.get('released', False),
                service_angular_rate_rad_s=guidance.get('service_angular_rate_rad_s'),
                service_angular_rate_stable=guidance.get('service_angular_rate_stable', False),
                processing_real_time_factor=capacity, estimated_com_position_n_m=nav.state['p'].tolist(),
                estimated_rotation_n_from_body=nav.state['r'].tolist()),
            rendezvous_handoff=dict(source_run_id=self.args.scene_instance.get('mission', {}).get('source_run_id'),
                epoch_s=nav.base['time_ns']*1e-9, model_id='sarm-dual-capture26',
                navigation_source='dual pose ESKF continuation'))
        self.client.send_observation(data)


def run_scene(cli, instance):
    from simulation.capture26_handoff import validate
    from simulation.capture26_mission import MODEL_ID
    from simulation.control_client import SimulationControlClient
    from space_arm_platform.rendezvous_config import FlightOptions

    root = Path(__file__).resolve().parents[1]
    incoming = validate(instance['rendezvous_handoff'], root)
    if instance['template_id'] != MODEL_ID or incoming['phase'] != 'NearParked':
        raise ValueError('capture26 operations require a matching accepted NearParked handoff')
    if (cli.disable_attitude_control or cli.dynamics_backend not in (None, 'local')
            or cli.local_substeps not in (None, 1)):
        raise ValueError('capture26 continuation requires fixed 400 Hz dynamics and attitude control')
    if getattr(cli, 'model_path', None):
        raise ValueError('capture26 handoff cannot change its physical model')
    output = root/'run/capture26-operations'/instance['instance_id']
    output.mkdir(parents=True, exist_ok=False)
    handoff_file = output/'incoming-handoff.json'
    handoff_file.write_text(json.dumps(incoming)+'\n', encoding='utf-8')
    options = FlightOptions.from_dict(incoming['mission_options'])
    client = SimulationControlClient(cli.control_host, cli.control_port, 'sarm-teleop', allow_scene_reset=False)
    client.start()
    try:
        client.wait_until_ready()
        args = SimpleNamespace(output=output, adapter=cli.adapter_root, duration=cli.duration or 86400.,
            depth=incoming['control']['approach']['state']['depth'], seed=options.seed,
            gnss_delay=(options.differential_gnss or {}).get('delay_s', 0.),
            ue=True, catalog=cli.catalog, render_port=cli.render_port, render_rate=10,
            capture_port=getattr(cli, 'capture_port', 8767), realtime=True, profile=False,
            simulation_rate=float(instance['runtime']['simulation_rate']),
            save_all_images=False, pointing_error_deg=[0., 0., 0.], approach=False,
            fault='', fault_after=2., fault_after_ready=False, mission_stage='operations',
            scenario='operations', flight_options=options, handoff=handoff_file,
            mission_id=incoming['mission_id'], external_vision=True, state_port=0,
            operation_client=client, scene_instance=instance, ik_mode=cli.ik_mode,
            ik_rate=int(instance['runtime']['ik_rate_hz']))
        os.environ['SPACE_SIM_LOCAL_PUBLISH_STRIDE'] = '8'
        from simulation.capture26_runtime import run
        return run(args)
    finally:
        client.close()
