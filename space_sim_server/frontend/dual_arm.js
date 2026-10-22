export function armObservation(raw, arm) {
  if (arm !== 'capture') return raw;
  const cap=raw.capture_arm || {};
  return {...raw, ...cap, arm_joint_position_rad:cap.joint_position_rad || [],
    arm_joint_limits_rad:cap.arm_joint_limits_rad || [],
    wrist_wrench:null, motion_speed_diagnostics:null, gripper_type:'capture_four_L',
    gripper_velocity_m_s:[], joint_velocity_rad_s:cap.joint_velocity_rad_s || [],
    end_effector_position_body_m:cap.end_effector_position_body_m || [],
    end_effector_twist_body:cap.end_effector_twist_body || [],
    jacobian_rank:null, ik_mode:null, ik_status:cap.controller_status,
    reference_governor_state:null, reference_limited_joints:[], tracking_scale:null,
    ik_control_rate_hz:400,
    ik_elbow_preference:null, ik_limit_reasons:[], ik_damping:null, ik_solve_time_ms:null};
}

// A second subscriber to the same renderer. It has no camera-command path:
// subscribing, reconnecting and removing it cannot reset the main orbit.
export class MainCompanion {
  constructor({create,show,status=()=>{}}) {Object.assign(this,{create,show,status});this.stream=null;this.id=null;this.live=false;}
  update(selected,main) {
    const wanted=selected?.endsWith('__teleop_camera_capture_wrist_cam') && main;
    if(!wanted){this.dispose();return;}
    if(this.stream && this.id===main)return;
    this.dispose();this.id=main;this.show(true);this.status('主视口连接中');this.stream=this.create(main);
    const stream=this.stream;
    stream.addEventListener?.('videoInitialized',()=>{if(this.stream===stream){this.live=true;this.status('主视口 · 中键观察 / 双击切换');}});
    for(const name of ['webRtcDisconnected','webRtcFailed','subscribeFailed']) stream.addEventListener?.(name,()=>{
      if(this.stream===stream){this.live=false;this.status('主视口无信号');}
    });
  }
  dispose(){const stream=this.stream;this.stream=null;this.live=false;this.id=null;stream?.disconnect();this.show(false);}
}
