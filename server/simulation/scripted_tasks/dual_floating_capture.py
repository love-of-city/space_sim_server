"""Sequential two-arm coordinator; each hand retains the original physical gates."""
from dataclasses import replace
import numpy as np
from .dual_floating_grasps import make_arm_task
from .dual_floating_model import DUAL_TASK


class DualFloatingTask:
    version='dual-floating-capture/1'
    name=DUAL_TASK

    def __init__(self, model, scene):
        self.tasks={side:make_arm_task(model,scene,side) for side in ('left','right')}
        arms=scene['collection_config']['dual_floating']['arms']
        self.commands={s:np.array(arms[s]['initial_joint_position_rad']) for s in self.tasks}
        self.velocities={s:np.zeros(8) for s in self.tasks}
        self.stage='left';self.result=None;self.reason=''
        self.events=[];self.right_start_s=None;self.last_time=None

    def step(self, samples, *, right_preflight=None):
        stamp=samples['left'].time_s
        if samples['right'].time_s!=stamp:raise ValueError('Both arms require the same physics timestamp')
        if self.result is not None:return self.commands,self.velocities
        if self.last_time is not None and not 0 < stamp-self.last_time <= .005000001:
            self.result,self.reason='failed','invalid_observation_timing'
            return self.commands,self.velocities
        self.last_time=stamp
        if self.stage=='left':
            right=self.tasks['right']
            right.check_parked(samples['right'])
            if right.result=='failed':
                self.result,self.reason='failed','right:'+right.reason
                return self.commands,self.velocities
        left=self.tasks['left']
        self.commands['left'],self.velocities['left']=left.step(samples['left'],complete=False,continue_holding=self.stage=='right')
        if left.result=='failed':
            self.result,self.reason='failed','left:'+left.reason
            return self.commands,self.velocities
        if self.stage=='left' and left.completion_ready:
            # Re-check right reachability using the actual left held pose before motion.
            if right_preflight is None:raise ValueError('Right-arm start requires measured-state preflight')
            if not right_preflight(samples):
                self.result,self.reason='failed','right_recheck_rejected'
                return self.commands,self.velocities
            self.stage='right';self.right_start_s=stamp
            self.events.append(dict(stage='right',sim_time_s=stamp))
        if self.stage=='right':
            right=self.tasks['right']
            sample=replace(samples['right'],time_s=stamp-self.right_start_s)
            self.commands['right'],self.velocities['right']=right.step(sample)
            if right.result=='failed':self.result,self.reason='failed','right:'+right.reason
            elif right.result=='success':
                self.result,self.reason='success','both_objects_retained_retracted_and_settled'
                self.events.append(dict(stage='success',sim_time_s=stamp))
        return self.commands,self.velocities

    def metrics(self,samples):
        return dict(name=self.name,task_version=self.version,stage=self.stage,result=self.result,reason=self.reason,
            arms={side:dict(stage=task.stage,result=task.result,reason=task.reason,**task.metrics(samples[side]),
                           task_parameters=task.config.snapshot()) for side,task in self.tasks.items()})
