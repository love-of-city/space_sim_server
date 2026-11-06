"""Dual SARM assembly on the shared Basilisk clock and LocalMujocoStepper."""
from pathlib import Path
import math
import numpy as np
from simulation.sarm_model_profile import DUAL_ARMS
from .dual_floating_capture import DualFloatingTask
from .native_state import FloatingState


class DualFloatingRuntime:
    def __init__(self, model, scene_instance):
        from .dual_floating_grasps import validate_dual_selection
        validate_dual_selection(model,scene_instance)
        from scenario_spacecraft_arm_grasp_unreal import load_native_grasp_module
        native=load_native_grasp_module(Path(__file__).resolve().parents[2]/'model/SARM/platform')
        native.MODEL_PATH=Path(model)
        self.native = native
        from Basilisk.utilities import SimulationBaseClass, macros
        from simulation.assembly import AssemblyContext,SessionConfig,Slot
        from simulation.components.clock import ClockComponent
        from simulation.components.orbit import OrbitComponent
        from simulation.components.orbital_gravity import OrbitalGravityComponent
        from simulation.orbital_configuration import resolve_orbital_physics
        from simulation.teleop_grasp_unreal import DEFAULT_EPHEMERIS_EPOCH_UTC
        from simulation.joint_reference_publisher import HeldJointReferencePublisher
        from simulation.gripper_control import tendon_reference_publisher
        from simulation.local_mujoco_stepper import LocalMujocoStepper,servo_specs
        self.model,self.instance=Path(model),scene_instance
        self.task=DualFloatingTask(model,scene_instance)
        self.sim=SimulationBaseClass.SimBaseClass()
        process=self.sim.CreateNewProcess('graspProcess')
        process.addTask(self.sim.CreateNewTask('graspTask',macros.sec2nano(.0025)))
        process.addTask(self.sim.CreateNewTask('localScenePublication',10**18))
        self.scene=native._load_scene(kinematics_only=True)
        self.sim.AddModelToTask('localScenePublication',self.scene)
        self.ctx=AssemblyContext(self.sim,self.scene,config=SessionConfig(model_path=self.model,dynamics_backend='local',publish_stride=8))
        self.orbit=OrbitComponent(epoch_utc=scene_instance.get('environment',{}).get('ephemeris_epoch_utc',DEFAULT_EPHEMERIS_EPOCH_UTC),physics=resolve_orbital_physics(scene_instance))
        self.ctx.install([ClockComponent(),self.orbit])
        servos=[];self.references={}
        for side,arm in DUAL_ARMS.items():
            publisher=HeldJointReferencePublisher(lambda t,side=side:(self.task.commands[side],self.task.velocities[side]),8)
            self.references[side]=publisher
            self.ctx.add(publisher,Slot.COMMAND,component=side+'_reference')
            servos+=servo_specs(arm.arm_actuators,arm.joint_names[:6],np.full(6,500.),np.full(6,50.),np.full(6,100.),publisher)
            tendon=tendon_reference_publisher(self.scene,publisher,arm.gripper_actuator)
            self.ctx.add(tendon,Slot.ACTUATOR,component=side+'_gripper')
        self.stepper=LocalMujocoStepper(self.scene,self.model,servos,substeps=4,publish_every=8,origin=self.orbit.origin)
        self.sim.local_stepper=self.stepper
        self.ctx.ports.set_stepper(self.stepper)
        self.ctx.add(self.stepper,Slot.PHYSICS,component='dynamics_core')
        self.ctx.install([OrbitalGravityComponent(mode='bsk')])
        self.readers={s:FloatingState(self.task.tasks[s],self.scene,self.stepper) for s in DUAL_ARMS}
        self.right_recheck=None

    def initialize(self):
        from Basilisk.utilities import orbitalMotion
        from simulation.teleop_grasp_unreal import DEFAULT_ORBIT
        native = self.native
        self.sim.InitializeSimulation()
        self.addresses={}
        for side,arm in DUAL_ARMS.items():
            joints=[self.scene.getBody(b).getScalarJoint(j) for b,j in arm.joints]
            self.addresses[side]=([j.getQposAdr() for j in joints],[j.getQvelAdr() for j in joints])
        saved=self.instance.get('environment',{}).get('orbit',DEFAULT_ORBIT)
        oe=orbitalMotion.ClassicElements();oe.a=self.orbit.earth.radEquator+saved['altitude_m'];oe.e=saved['eccentricity']
        oe.i=math.radians(saved['inclination_deg']);oe.Omega=math.radians(saved['raan_deg'])
        oe.omega=math.radians(saved['argument_of_periapsis_deg']);oe.f=math.radians(saved['true_anomaly_deg'])
        r,v=map(np.asarray,orbitalMotion.elem2rv(self.orbit.earth.mu,oe))
        self.orbit.origin.set_state(r,v)
        record=self.instance['collection_config']['dual_floating']
        self.scene.getBody('cubesat_bus').setPosition(r)
        self.scene.getBody('cubesat_bus').setVelocity(v)
        for side,arm in DUAL_ARMS.items():
            for body_name,joint_name in arm.joints:
                joint=self.scene.getBody(body_name).getScalarJoint(joint_name)
                joint.setPosition(record['initial_joint_positions'][joint_name][0]);joint.setVelocity(0.)
        for obj in record['instances']:
            body=self.scene.getBody(obj['instance_id'])
            body.setPosition(r+np.array(obj['position_body_m']));body.setVelocity(v)
            body.setAttitude(native._quaternion_to_mrp(np.array(obj['orientation_wxyz'])))
            body.setAttitudeRate(np.zeros(3))
        self.sim.ConfigureStopTime(0);self.sim.ExecuteSimulation()

    def samples(self,stamp_ns,elapsed_s):
        result={}
        for side,arm in DUAL_ARMS.items():
            qa,va=self.addresses[side]
            wrench=arm.wrench_sensor.observation(self.stepper,stamp_ns)
            if wrench is None:raise ValueError(f'Missing {side} wrist wrench')
            result[side]=self.readers[side].sample(elapsed_s,self.stepper.qpos[qa].copy(),self.stepper.qvel[va].copy(),wrench)
        return result

    def recheck_right(self,samples):
        from .drawer_initialization import NativePoseProbe
        from .floating_grasps import requested_candidates,evaluate_grasps
        from .dual_floating_grasps import choose_grasp
        task=self.task.tasks['right'];sample=samples['right']
        probe=NativePoseProbe(self.model,arm=DUAL_ARMS['right'])
        try:
            probe.use_measured_qpos(self.stepper.qpos.copy(),sample.joints)
            probe.held_finger_contacts={finger:'object_left' for finger in DUAL_ARMS['left'].finger_bodies}
            options,candidates=requested_candidates(task.name,self.instance)
            feasible,report=evaluate_grasps(probe,task,sample.joints,sample.plug_position_body_m,sample.plug_rotation_body,
                self.instance['collection_config']['dual_floating']['instances'],{},candidates)
            self.right_recheck=dict(sim_time_s=sample.time_s,**report)
            if not feasible:return False
            chosen=choose_grasp(feasible,options,self.instance['seed'],'dual-grasp-right-recheck-v1')
            task.grasp=chosen[0]
            self.right_recheck['selected']=chosen[0].snapshot()
            return True
        finally:probe.close()

    def close(self):
        self.stepper.close()
        if self.orbit.gravity_factory is not None:
            self.orbit.gravity_factory.unloadSpiceKernels()
