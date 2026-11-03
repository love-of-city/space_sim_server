"""Ordered physical predicates. Never read controller stages or success flags."""
import math
import numpy as np
from .fixed_trajectory import TASKS
from simulation.policy_task_success import DrawerCycleSuccess, EVALUATION_PERIOD_NS


class PhysicalScorer:
    version='space-arm-physical-score/1'

    def __init__(self, task, final_tool_position, *, hole=None, hole_rotation=None, tip=.095, flange=.025, keyed=False, drawer_spec=None, moving_hole=False,
                 pickup=None, pickup_rotation=None, moving_pickup=False):
        if task not in TASKS:raise ValueError('Unsupported evaluation task')
        self.task=task;self.final=np.array(final_tool_position,dtype=float)
        self.hole=None if hole is None else np.array(hole,dtype=float)
        self.hole_rotation=np.eye(3) if hole_rotation is None else np.array(hole_rotation,dtype=float)
        self.tip,self.flange,self.keyed=tip,flange,keyed
        self.moving_hole = bool(moving_hole)
        self.pickup = None if pickup is None else np.array(pickup, dtype=float)
        self.pickup_rotation = None if pickup_rotation is None else np.array(pickup_rotation, dtype=float)
        self.moving_pickup = bool(moving_pickup)
        self.initial_pickup_clearance = None
        self.drawer_monitor=None
        self.drawer_state={}
        if task=='drawer_cycle':
            self.version='space-arm-physical-score/2'
            if drawer_spec is None:raise ValueError('Drawer scoring v2 requires a frozen drawer_spec; regenerate from complete raw observations')
            self.drawer_monitor=DrawerCycleSuccess.from_spec(drawer_spec)
            if self.drawer_monitor.sample_period_ns!=EVALUATION_PERIOD_NS:
                raise ValueError('Drawer evaluation requires a 200 Hz drawer_spec')
            if self.drawer_monitor.mode!='benchmark':
                raise ValueError('Drawer evaluation requires benchmark task criteria, not calibrated expert-pose checks')
        if task!='drawer_cycle':
            self._validate_pose(self.hole, self.hole_rotation, 'interface')
            if (self.pickup is None) != (self.pickup_rotation is None):
                raise ValueError('Plug scorer requires both pickup position and rotation')
            if self.pickup is None:
                # Preserve old fixed horizontal recordings exactly. A moving or
                # rotated donor cannot be reconstructed from the receiver alone.
                if self.moving_pickup or self.moving_hole or not np.allclose(self.hole_rotation[:, 2], [0., 0., 1.]):
                    raise ValueError('Non-horizontal or moving interfaces require an explicit pickup pose')
            else:
                self._validate_pose(self.pickup, self.pickup_rotation, 'pickup')
                self.version='space-arm-physical-score/3'
        self.initial=None;self.last=None;self.hold=None;self.overload=None
        self.milestones=[False]*3;self.success=False;self.error=None;self.safety_failure=None
        self.max_force=0.;self.max_torque=0.;self.samples=0;self.final_metrics={}
        self.tracking_sum=0.;self.tracking_count=0;self.peak_displacement=0.

    @staticmethod
    def _validate_pose(position, rotation, label):
        if (position is None or position.shape != (3,) or rotation.shape != (3, 3)
                or not np.isfinite(position).all() or not np.isfinite(rotation).all()
                or not np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)):
            raise ValueError(f'Invalid {label} pose')

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
            limit=(40.,4.,.1) if self.task=='drawer_cycle' else (40.,4.,.15)
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
                velocity=None
                if 'joint_velocity' in row:
                    velocity=np.asarray(row['joint_velocity'],dtype=float)
                    if velocity.shape!=(8,) or not np.isfinite(velocity).all():raise ValueError('Invalid drawer joint velocity evidence')
                sample=dict(sim_time_ns=round(t*1e9),drawer_position_m=row['drawer_m'],
                            drawer_velocity_m_s=row['drawer_velocity_m_s'],buckle_rad=row['buckle_rad'],
                            gripper_drive_position_rad=float(q[6]),finger_drawer_contacts=row['contacts'],
                            end_effector_position_body_m=p.tolist(),end_effector_rotation_body=row.get('tool_rotation'),
                            arm_joint_velocity_rad_s=None if velocity is None else velocity[:6].tolist())
                if self.drawer_monitor.moving_mount:
                    sample.update(drawer_origin_body_m=row['drawer_origin_body_m'],
                                  drawer_rotation_body=row['drawer_rotation_body'])
                if self.samples==0:self.drawer_monitor.reset(sample)
                self.drawer_state=self.drawer_monitor.update(sample)
                if not self.drawer_state['success_available']:
                    raise ValueError(self.drawer_state['success_unavailable_reason'])
                self.milestones=[self.drawer_state[k] for k in ['open_verified','seated_before_release_verified','release_verified']]
                self.peak_displacement=self.drawer_state['peak_drawer_displacement_m']
                self.success=bool(self.drawer_state['success'] and not self.safety_failure)
                self.final_metrics=dict(drawer_displacement_m=float(row['drawer_m']-self.initial['drawer_m']),
                                        buckle_error_rad=float(row['buckle_rad']-self.initial['buckle_rad']))
            else:
                if self.moving_hole:
                    self.hole = np.asarray(row['hole_position_m'], dtype=float)
                    self.hole_rotation = np.asarray(row['hole_rotation'], dtype=float)
                    self._validate_pose(self.hole, self.hole_rotation, 'measured interface')
                if self.moving_pickup:
                    self.pickup = np.asarray(row['pickup_position_m'], dtype=float)
                    self.pickup_rotation = np.asarray(row['pickup_rotation'], dtype=float)
                    self._validate_pose(self.pickup, self.pickup_rotation, 'measured pickup')
                pos=np.asarray(row['plug_position_m'],dtype=float);rot=np.asarray(row['plug_rotation'],dtype=float)
                vel=np.asarray(row['plug_velocity_m_s'],dtype=float);omega=np.asarray(row['plug_angular_velocity_rad_s'],dtype=float)
                if pos.shape!=(3,) or rot.shape!=(3,3) or vel.shape!=(3,) or omega.shape!=(3,) or not all(np.isfinite(a).all() for a in [pos,rot,vel,omega]):raise ValueError('Invalid plug state')
                if not np.allclose(rot@rot.T,np.eye(3),atol=1e-5) or not np.isclose(np.linalg.det(rot),1,atol=1e-5):raise ValueError('Invalid plug rotation')
                axis=rot[:,2];tip=pos+axis*self.tip;flange=pos+axis*self.flange
                normal = self.hole_rotation[:, 2]
                axial = float(axis @ normal)
                if abs(axial)<1e-9:radial=1e9
                else:radial=float(np.linalg.norm(tip+axis*((self.hole-tip) @ normal/axial)-self.hole))
                angle=float(np.arccos(np.clip(-axial,-1,1)));depth=float((self.hole-tip) @ normal);gap=float((flange-self.hole) @ normal)
                expected=self.hole_rotation@np.diag([1.,-1.,-1.])
                orient=float(np.arccos(np.clip((np.trace(expected@rot.T)-1)/2,-1,1)))
                if self.pickup is None:
                    lift=float(pos[2]-np.array(self.initial['plug_position_m'])[2])
                else:
                    clearance=float((pos-self.pickup) @ self.pickup_rotation[:, 2])
                    if self.initial_pickup_clearance is None:self.initial_pickup_clearance=clearance
                    lift=clearance-self.initial_pickup_clearance
                self.peak_displacement=max(self.peak_displacement,lift)
                seated=radial<.0004 and angle<.01 and depth>self.tip-self.flange-.002 and abs(gap)<.001 and (not self.keyed or orient<.01)
                if lift>=.04 and np.all(contacts>0):self.milestones[0]=True
                if self.milestones[0] and seated:self.milestones[1]=True
                if self.milestones[1] and seated and q[6]>=.62 and np.all(contacts==0):self.milestones[2]=True
                retreat=float(np.linalg.norm(self.hole_rotation[:, :2].T @ (p-self.hole)))
                ready=self.milestones[2] and seated and q[6]>=.62 and np.all(contacts==0) and np.linalg.norm(vel)<.002 and np.linalg.norm(omega)<.03 and retreat>=.09
                self.final_metrics=dict(radial_error_m=radial,axis_error_rad=angle,orientation_error_rad=orient,insertion_depth_m=depth,seat_gap_m=gap)
            if self.task!='drawer_cycle':
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
        extra={}
        final_hold=0. if self.hold is None or self.last is None else self.last-self.hold
        if self.drawer_monitor is not None:
            state=self.drawer_state or self.drawer_monitor.status()
            extra={key:state[key] for key in ['grasp_verified','max_grasp_hold_s','max_open_hold_s','max_seat_hold_s',
                                             'hover_position_error_m','jaw_alignment_error_rad','criteria_mode','retreat_distance_m']}
            extra['criteria_version']=state['task_version']
            final_hold=state['verification_stable_s']
        return dict(schema=self.version,task=self.task,status=status,success=status=='success',
                    reason=self.error or self.safety_failure or ('physical_predicates_satisfied' if self.success else 'physical_predicates_not_satisfied'),
                    progress=(sum(self.milestones)+int(self.success))/4 if valid else None,
                    milestones=dict(zip(['opened_or_lifted','returned_or_inserted','released'],self.milestones)),
                    final_hold_s=final_hold,
                    max_wrist_force_n=self.max_force,max_wrist_torque_nm=self.max_torque,
                    peak_object_displacement_m=self.peak_displacement,samples=self.samples,
                    tcp_tracking_rmse_m=math.sqrt(self.tracking_sum/self.tracking_count) if self.tracking_count else None,
                    **self.final_metrics,**extra)


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
