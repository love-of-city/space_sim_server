"""Ordered physical predicates. Never read controller stages or success flags."""
import math
import numpy as np
from .fixed_trajectory import TASKS


class PhysicalScorer:
    version='space-arm-physical-score/1'

    def __init__(self, task, final_tool_position, *, hole=None, hole_rotation=None, tip=.095, flange=.025, keyed=False):
        if task not in TASKS:raise ValueError('Unsupported evaluation task')
        self.task=task;self.final=np.array(final_tool_position,dtype=float)
        self.hole=None if hole is None else np.array(hole,dtype=float)
        self.hole_rotation=np.eye(3) if hole_rotation is None else np.array(hole_rotation,dtype=float)
        self.tip,self.flange,self.keyed=tip,flange,keyed
        if task!='drawer_cycle' and (self.hole is None or self.hole.shape!=(3,)):raise ValueError('Plug scorer requires its fixed interface pose')
        self.initial=None;self.last=None;self.hold=None;self.overload=None
        self.milestones=[False]*3;self.success=False;self.error=None;self.safety_failure=None
        self.max_force=0.;self.max_torque=0.;self.samples=0;self.final_metrics={}
        self.tracking_sum=0.;self.tracking_count=0;self.peak_displacement=0.

    def update(self, row):
        try:
            t=float(row['time_s']);q=np.asarray(row['joint_position'],dtype=float);p=np.asarray(row['tool_position_m'],dtype=float)
            wrench=np.asarray(row['wrench'],dtype=float);contacts=np.asarray(row['contacts'])
            if q.shape!=(8,) or p.shape!=(3,) or wrench.shape!=(6,) or contacts.shape!=(2,) or np.any(contacts<0):raise ValueError('Invalid sample dimensions')
            # At t=0 the native sensor has not executed its first dynamics step.
            bootstrap=self.samples==0 and t==0 and row['wrench_valid'] is False and np.all(wrench==0)
            if not all(np.isfinite(a).all() for a in [q,p,wrench,contacts]) or not math.isfinite(t) or t<0 or (row['wrench_valid'] is not True and not bootstrap):raise ValueError('Invalid measurement')
            if self.last is not None and not 0 < t-self.last <= .005001:raise ValueError('Nonmonotonic or missing 200Hz evidence')
            if self.last is None and t>.000001:raise ValueError('Missing initial evidence')
            force=float(np.linalg.norm(wrench[:3]));torque=float(np.linalg.norm(wrench[3:]))
            self.max_force=max(self.max_force,force);self.max_torque=max(self.max_torque,torque)
            limit=(80.,8.,.1) if self.task=='drawer_cycle' else (40.,4.,.15)
            if force>limit[0] or torque>limit[1]:
                if self.overload is None:self.overload=t
                if t-self.overload>=limit[2]-1e-9:self.safety_failure='wrist_overload'
            else:self.overload=None
            if self.initial is None:self.initial=dict(row)
            if 'reference_position_m' in row:
                ref=np.asarray(row['reference_position_m'],dtype=float)
                if ref.shape!=(3,) or not np.isfinite(ref).all():raise ValueError('Invalid reference pose')
                self.tracking_sum+=float(np.sum((p-ref)**2));self.tracking_count+=1
            if self.task=='drawer_cycle':
                vals=np.array([row[k] for k in ['drawer_m','drawer_velocity_m_s','buckle_rad']],dtype=float)
                if not np.isfinite(vals).all():raise ValueError('Invalid drawer state')
                disp=vals[0]-float(self.initial['drawer_m']);self.peak_displacement=max(self.peak_displacement,float(disp))
                closed=abs(disp)<=.002 and abs(vals[1])<.003 and abs(vals[2]-float(self.initial['buckle_rad']))<=.025
                if disp>=.075 and abs(vals[1])<.003 and np.all(contacts>0):self.milestones[0]=True
                if self.milestones[0] and closed and q[6]<.95:self.milestones[1]=True
                if self.milestones[1] and closed and q[6]>.95 and np.all(contacts==0):self.milestones[2]=True
                ready=self.milestones[2] and closed and q[6]>.95 and np.all(contacts==0) and np.linalg.norm(p-self.final)<.003
                self.final_metrics=dict(drawer_displacement_m=float(disp),buckle_error_rad=float(vals[2]-float(self.initial['buckle_rad'])))
            else:
                pos=np.asarray(row['plug_position_m'],dtype=float);rot=np.asarray(row['plug_rotation'],dtype=float)
                vel=np.asarray(row['plug_velocity_m_s'],dtype=float);omega=np.asarray(row['plug_angular_velocity_rad_s'],dtype=float)
                if pos.shape!=(3,) or rot.shape!=(3,3) or vel.shape!=(3,) or omega.shape!=(3,) or not all(np.isfinite(a).all() for a in [pos,rot,vel,omega]):raise ValueError('Invalid plug state')
                if not np.allclose(rot@rot.T,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(rot),1,atol=1e-5):raise ValueError('Invalid plug rotation')
                axis=rot[:,2];tip=pos+axis*self.tip;flange=pos+axis*self.flange
                if abs(axis[2])<1e-9:radial=1e9
                else:radial=float(np.linalg.norm((tip+axis*((self.hole[2]-tip[2])/axis[2])-self.hole)[:2]))
                angle=float(np.arccos(np.clip(-axis[2],-1,1)));depth=float(self.hole[2]-tip[2]);gap=float(flange[2]-self.hole[2])
                expected=self.hole_rotation@np.diag([1.,-1.,-1.])
                orient=float(np.arccos(np.clip((np.trace(expected@rot.T)-1)/2,-1,1)))
                lift=float(pos[2]-np.array(self.initial['plug_position_m'])[2]);self.peak_displacement=max(self.peak_displacement,lift)
                seated=radial<.0004 and angle<.01 and depth>self.tip-self.flange-.002 and abs(gap)<.001 and (not self.keyed or orient<.01)
                if lift>=.04 and np.all(contacts>0):self.milestones[0]=True
                if self.milestones[0] and seated:self.milestones[1]=True
                if self.milestones[1] and seated and q[6]>=.62 and np.all(contacts==0):self.milestones[2]=True
                ready=self.milestones[2] and seated and q[6]>=.62 and np.all(contacts==0) and np.linalg.norm(vel)<.002 and np.linalg.norm(omega)<.03 and np.linalg.norm(p[:2]-self.hole[:2])>=.09
                self.final_metrics=dict(radial_error_m=radial,axis_error_rad=angle,orientation_error_rad=orient,insertion_depth_m=depth,seat_gap_m=gap)
            if ready:
                if self.hold is None:self.hold=t
            else:self.hold=None
            self.success=bool(self.hold is not None and t-self.hold>=2.-1e-9 and not self.safety_failure)
            self.last=t;self.samples+=1
        except (KeyError,ValueError,TypeError,IndexError) as exc:
            self.error=str(exc);self.success=False

    def report(self):
        valid=self.samples>=2 and self.error is None
        status='unscorable' if not valid else 'success' if self.success else 'task_failure'
        return dict(schema=self.version,task=self.task,status=status,success=status=='success',
                    reason=self.error or self.safety_failure or ('physical_predicates_satisfied' if self.success else 'physical_predicates_not_satisfied'),
                    progress=(sum(self.milestones)+int(self.success))/4 if valid else None,
                    milestones=dict(zip(['opened_or_lifted','returned_or_inserted','released'],self.milestones)),
                    final_hold_s=0. if self.hold is None or self.last is None else self.last-self.hold,
                    max_wrist_force_n=self.max_force,max_wrist_torque_nm=self.max_torque,
                    peak_object_displacement_m=self.peak_displacement,samples=self.samples,
                    tcp_tracking_rmse_m=math.sqrt(self.tracking_sum/self.tracking_count) if self.tracking_count else None,
                    **self.final_metrics)


def aggregate(results):
    from collections import Counter
    allowed={'success','task_failure','initialization_rejected','infrastructure_error','unscorable'}
    if any(x['status'] not in allowed for x in results):raise ValueError('Unknown evaluation outcome')
    counts=Counter(x['status'] for x in results);n=len(results);k=counts['success'];valid=k+counts['task_failure']
    if n:
        z=1.959963984540054;den=1+z*z/n;center=(k/n+z*z/(2*n))/den
        radius=z*math.sqrt((k/n)*(1-k/n)/n+z*z/(4*n*n))/den;ci=[max(0.,center-radius),min(1.,center+radius)]
    else:ci=None
    progress=[x['progress'] for x in results if x['status'] in ('success','task_failure') and x.get('progress') is not None]
    return dict(attempts=n,counts={x:counts[x] for x in sorted(allowed)},success_rate_all_attempts=k/n if n else None,
                success_rate_scorable=k/valid if valid else None,wilson95_all_attempts=ci,
                mean_progress_scorable=float(np.mean(progress)) if progress else None)
