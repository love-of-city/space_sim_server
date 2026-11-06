import {element} from '../ui.js';

export const benchmarkTerminal = new Set(['complete','stopped','interrupted','failed']);
export const benchmarkLabels = {
  starting:'正在启动',running:'运行中',stopping:'正在停止',complete:'评测结束',stopped:'已停止',interrupted:'已中断',failed:'运行失败',
  success:'成功',pending:'待运行',cancelled:'已取消',safety_violation:'违反安全限制',time_limit:'达到时限',
  method_error:'方法失败',infrastructure_error:'基础设施失败',invalid_evidence:'证据无效',
  invalid_initialization:'初态无效',task_failure:'任务未达标',
};
export const methodLabels = {
  'catalog-avoidance-v1':'外部目录与 GNSS 自主避碰','external-catalog-avoidance-v1':'目录避碰 · 独立进程示例',
  'no-avoidance-thrust-v1':'零避碰推力（负对照）','unsafe-avoidance-burn-v1':'持续避碰推力（负对照）',
  'adaptive-wheel-recovery-v1':'飞轮异常诊断与自适应控制','external-wheel-recovery-v1':'飞轮异常恢复 · 独立进程示例',
  'unaware-wheel-recovery-v1':'不诊断飞轮异常（对照）','no-recovery-torque-v1':'零恢复力矩（负对照）',
  'unsafe-recovery-spin-v1':'异常期间持续满力矩（负对照）',
  'image-stereo-approach-v1':'双目图像闭环接近','external-image-stereo-approach-v1':'图像闭环接近 · 独立进程示例',
  'no-visual-thrust-v1':'图像观测与零推力（负对照）','unsafe-visual-close-v1':'盲目满推力接近（负对照）',
  'missing-visual-control-v1':'缺失图像估计与零推力（负对照）',
  'image-gyro-spin-v1':'图像与自身陀螺角速度估计','external-image-gyro-spin-v1':'图像角速度估计 · 独立进程示例',
  'zero-image-spin-v1':'零目标角速度（负对照）','missing-image-spin-v1':'始终缺失运动估计（负对照）',
  'image-fiducial-pose-v1':'图像标志物位姿估计','external-image-fiducial-pose-v1':'图像位姿估计 · 独立进程示例',
  'missing-image-pose-v1':'始终失锁（负对照）','zero-image-pose-v1':'零位姿（负对照）',
  'ephemeris-pointing-pd-v1':'星历与遥测指向','frozen-pointing-pd-v1':'保持初始参考方向（对照）',
  'no-pointing-torque-v1':'零指向力矩（负对照）','unsafe-pointing-spin-v1':'持续满力矩（负对照）',
  'external-ephemeris-pointing-v1':'星历与遥测指向 · 独立进程示例',
  'gnss-pd-v1':'GNSS PD','no-thrust-v1':'零推力（负对照）','unsafe-approach-v1':'持续接近（负对照）',
  'attitude-wheel-pd-v1':'姿态 PD','attitude-rate-limited-pd-v1':'角速度受限姿态 PD',
  'attitude-sequence-pd-v1':'连续姿态切换','external-attitude-sequence-v1':'连续姿态切换 · 独立进程示例',
  'no-wheel-torque-v1':'零飞轮力矩（负对照）','unsafe-spin-v1':'持续满力矩（负对照）',
  'external-attitude-pd-v1':'姿态 PD · 独立进程示例','external-gnss-pd-v1':'GNSS PD · 独立进程示例',
  'gnss-orbit-maneuver-v1':'GNSS 轨道控制','no-orbit-thrust-v1':'零轨控推力（负对照）',
  'unsafe-orbit-burn-v1':'持续轨控推力（负对照）','external-orbit-maneuver-v1':'轨道控制 · 独立进程示例',
  'gnss-orbit-estimator-v1':'GNSS 轨道状态估计','gnss-measurement-only-v1':'直接使用 GNSS 测量',
  'zero-navigation-estimate-v1':'零状态估计（负对照）','external-gnss-estimator-v1':'GNSS 状态估计 · 独立进程示例',
  'gnss-relative-flight-v1':'GNSS 相对飞行','no-relative-thrust-v1':'零相对飞行推力（负对照）',
  'unsafe-relative-flight-v1':'横向偏离（负对照）','external-relative-flight-v1':'相对飞行 · 独立进程示例',
  'gnss-flyaround-v1':'GNSS 安全绕飞','no-flyaround-thrust-v1':'零绕飞推力（负对照）',
  'unsafe-flyaround-v1':'绕飞法向偏离（负对照）','external-flyaround-v1':'安全绕飞 · 独立进程示例',
  'gnss-formation-v1':'GNSS 编队协同控制','no-formation-thrust-v1':'全星零推力（负对照）',
  'unsafe-formation-v1':'成员危险接近（负对照）','external-formation-v1':'编队协同控制 · 独立进程示例',
  'one-sided-formation-v1':'仅一星施力（协同交会负对照）',
};
export function benchmarkMetric(score){
  if(score?.schema_version==='spacesim-autonomy-debris-avoidance-score/1'){
    const range=Number.isFinite(score.minimum_range_m)?`${score.minimum_range_m.toFixed(2)} m`:'—';
    const coast=Number.isFinite(score.final_coast_s)?`${score.final_coast_s.toFixed(2)} s`:'—';
    return `最小碎片间距 ${range}；末段无推力滑行 ${coast}`;
  }
  if(score?.schema_version==='spacesim-autonomy-wheel-recovery-score/1'){
    const diagnosis=Number.isFinite(score.diagnosis_latency_s)?`${score.diagnosis_latency_s.toFixed(2)} s`:'未确认';
    const angle=Number.isFinite(score.final_attitude_error_rad)?`${(score.final_attitude_error_rad*180/Math.PI).toFixed(4)}°`:'—';
    return `异常诊断延迟 ${diagnosis}；姿态误差 ${angle}`;
  }
  if(score?.schema_version==='spacesim-autonomy-visual-approach-score/1'){
    return Number.isFinite(score.position_error_m)&&Number.isFinite(score.optical_relative_speed_m_s)
      ?`位置误差 ${score.position_error_m.toFixed(4)} m；相对速度 ${score.optical_relative_speed_m_s.toFixed(4)} m/s；连续保持 ${score.dwell_s.toFixed(2)} s`:'—';
  }
  if(score?.schema_version==='spacesim-autonomy-rendered-spin-score/1'){
    const available=`有效角速度估计 ${score.available_spin_count}/${score.expected_spin_estimates}`;
    return Number.isFinite(score.angular_velocity_rmse_rad_s)
      ?`${available}；角速度 RMSE ${score.angular_velocity_rmse_rad_s.toFixed(5)} rad/s`:available;
  }
  if(score?.schema_version==='spacesim-autonomy-rendered-pose-score/1'){
    const available=`有效估计 ${score.available_pose_count}/${score.expected_estimates}`;
    return Number.isFinite(score.position_rmse_m)&&Number.isFinite(score.orientation_rmse_rad)
      ?`${available}；位置 RMSE ${(score.position_rmse_m*1000).toFixed(2)} mm；姿态 RMSE ${(score.orientation_rmse_rad*180/Math.PI).toFixed(3)}°`:available;
  }
  if(Number.isFinite(score?.final_boresight_error_rad))return `指向误差 ${(score.final_boresight_error_rad*180/Math.PI).toFixed(4)}°；跟踪时窗超限 ${score.failed_tracking_samples}/${score.tracking_samples} 帧`;
  if(Number.isInteger(score?.target_count)&&Array.isArray(score.completed_targets)&&Number.isFinite(score.final_attitude_error_rad))return `已完成姿态 ${score.completed_targets.length}/${score.target_count}；当前姿态误差 ${(score.final_attitude_error_rad*180/Math.PI).toFixed(4)}°`;
  if(Number.isFinite(score?.cooperative_range_reduction_m))return `双星交会缩短 ${score.cooperative_range_reduction_m.toFixed(2)} m；共同机动 ${score.simultaneous_maneuver_s.toFixed(2)} s；逐星推力积分 ${score.per_spacecraft_thrust_impulse_n_s.map(value=>value.toFixed(2)).join(' / ')} N·s`;
  if(Number.isFinite(score?.formation_position_error_m))return `成员最大位置误差 ${score.formation_position_error_m.toFixed(4)} m；最小星间距离 ${score.minimum_pair_distance_m.toFixed(2)} m；连续保持 ${score.longest_dwell_s.toFixed(2)} s`;
  if(Number.isFinite(score?.final_attitude_error_rad))return `${(score.final_attitude_error_rad*180/Math.PI).toFixed(4)}°`;
  if(Number.isFinite(score?.position_rmse_m))return `位置 RMSE ${score.position_rmse_m.toFixed(4)} m；速度 RMSE ${score.velocity_rmse_m_s.toFixed(4)} m/s`;
  if(Number.isFinite(score?.signed_revolutions)&&Number.isFinite(score.maximum_radial_error_m))return `绕行 ${score.signed_revolutions.toFixed(4)} 圈；最大径向偏差 ${score.maximum_radial_error_m.toFixed(4)} m；平面偏差 ${score.maximum_plane_error_m.toFixed(4)} m`;
  if(score?.path_requirement==='full_duration_relative_hold'&&Number.isFinite(score.maximum_position_error_m))return `最大位置误差 ${score.maximum_position_error_m.toFixed(4)} m；连续保持 ${score.longest_dwell_s.toFixed(2)} s`;
  if(score?.path_requirement==='axis_approach_corridor'&&Number.isFinite(score.maximum_cross_track_m))return `位置误差 ${score.final_position_error_m.toFixed(4)} m；最大横向偏差 ${score.maximum_cross_track_m.toFixed(4)} m`;
  if(score?.path_requirement==='abort_and_retreat'){
    const reversal=Number.isFinite(score.reversal_confirmed_s)?`${score.reversal_confirmed_s.toFixed(2)} s`:'未确认';
    const progress=Number.isFinite(score.retreat_progress_m)?`${score.retreat_progress_m.toFixed(2)} m`:'—';
    return `向外运动确认 ${reversal}；撤离距离 ${progress}`;
  }
  if(score?.path_requirement==='far_to_proximity_rendezvous'){
    const reduction=Number.isFinite(score.range_reduction_m)?`${score.range_reduction_m.toFixed(2)} m`:'—';
    const distance=Number.isFinite(score.final_range_m)?`${score.final_range_m.toFixed(2)} m`:'—';
    return `交会缩短距离 ${reduction}；当前距离 ${distance}`;
  }
  if(Number.isFinite(score?.final_position_error_m))return `${score.final_position_error_m.toFixed(4)} m`;
  if(score?.path_requirement==='continuous_orbit_maintenance'&&Number.isFinite(score.maximum_maintenance_axis_error_m))return `维持时窗最大半长轴误差 ${score.maximum_maintenance_axis_error_m.toFixed(2)} m；超限 ${score.failed_maintenance_samples}/${score.maintenance_samples} 帧`;
  if(Number.isFinite(score?.final_orbit?.impulse_vector_error_m_s))return `Δv 误差 ${score.final_orbit.impulse_vector_error_m_s.toFixed(4)} m/s`;
  if(Number.isFinite(score?.final_orbit?.semimajor_axis_error_m))return `半长轴误差 ${score.final_orbit.semimajor_axis_error_m.toFixed(2)} m；e=${score.final_orbit.eccentricity.toExponential(2)}`;
  return '—';
}
export function benchmarkRate(summary){
  const rate=summary.success_rate_all_planned;
  return rate==null?'—':`${(rate*100).toFixed(1)}%（${summary.successes}/${summary.planned}）`;
}

export function renderBenchmarkJob(job,{api,notice,refresh,title=job.title||job.configuration,detail=false}){
  const card=element('article','panel job');
  const head=element('div','job-head'),heading=element('div');
  const split={train:'训练集',validation:'验证集',test:'冻结测试集'}[job.request.split]||job.request.split;
  heading.append(element('h3','',title),element('p','muted',`${methodLabels[job.method]||job.method} · ${split} · ${job.request.selection==='single'?'单次评测':'完整冻结划分'}`));
  head.append(heading,element('span','badge',benchmarkLabels[job.status]||job.status));card.append(head);
  const summary=job.summary,metrics=element('div','metrics');
  for(const [caption,value] of [['已结束样本',`${summary.finished}/${summary.planned}`],['成功率（全部计划样本）',benchmarkRate(summary)]]){
    const item=element('div');item.append(element('span','',caption),element('strong','',value));metrics.append(item);
  }
  card.append(metrics);
  const interval=summary.wilson_95_interval;
  card.append(element('p','panel-body muted',interval?`Wilson 95% 区间：${(interval[0]*100).toFixed(2)}%–${(interval[1]*100).toFixed(2)}%。失败与取消样本均保留。`:'批次尚未完成；未运行样本仍在计划分母中，暂不报告置信区间。'));
  if(job.error)card.append(element('p','panel-body error',job.error));
  const scroll=element('div','table-scroll'),table=element('table','platform-table'),thead=element('thead'),header=element('tr');
  for(const name of ['种子','结果','仿真时长','误差指标',...(detail?['原始记录']:[])])header.append(element('th','',name));
  thead.append(header);table.append(thead);const body=element('tbody');
  for(const trial of summary.trials){
    const row=element('tr'),outcome=element('td','',benchmarkLabels[trial.outcome]||trial.outcome);
    if(trial.detail)outcome.append(element('small','muted',trial.detail==='operator requested episode stop'?'用户停止了该回合':trial.detail));
    if(trial.physics_score?.safety_violations?.length){
      const labels={approach_corridor:'超出接近走廊',wrong_approach_side:'接近方向错误',relative_speed:'相对速度超限',
        minimum_range:'小于最小安全距离',maximum_range:'超出距离范围',applied_force:'实际推力超限',
        minimum_pair_distance:'星间距离小于安全下限',maximum_pair_distance:'星间距离超出上限',pair_relative_speed:'星间相对速度超限'};
      outcome.append(element('small','error',trial.physics_score.safety_violations.map(name=>labels[name]||name).join('、')));
    }
    if(trial.physics_score?.quality_violations?.length){
      const labels={position_rmse:'位置 RMSE',velocity_rmse:'速度 RMSE',maximum_position_error:'最大位置误差',maximum_velocity_error:'最大速度误差',
        closest_approach_outside_encounter_window:'最近接近不在规定会遇时窗内',outward_pass_not_confirmed:'尚未确认安全远离碎片',
        final_unpowered_coast_not_confirmed:'末段无推力滑行不足',no_actual_avoidance_maneuver:'未验证实际避碰机动',
        rendezvous_range_reduction:'远距交会的距离缩减不足',rendezvous_terminal_range:'尚未进入规定近距范围',
        orientation_rmse:'姿态 RMSE',maximum_orientation_error:'最大姿态误差',pose_availability:'有效位姿估计不足',
        false_motor_loss_diagnosis:'正常阶段误报电机异常',motor_loss_diagnosis_deadline:'电机异常诊断超时',motor_loss_not_observed_under_load:'未验证带载电机衰减',
        angular_velocity_rmse:'角速度 RMSE',maximum_angular_velocity_error:'最大角速度误差',spin_availability:'有效角速度估计不足',
        continuous_hold_envelope:'全程保持范围',continuous_orbit_band:'轨道维持时窗超限',approach_progress:'接近距离',flyaround_radius:'绕飞半径偏差',
        abort_reversal_deadline:'未及时确认持续向外运动',abort_reversal_not_confirmed:'尚未确认持续向外运动',abort_retreat_progress:'撤离距离不足',
        flyaround_plane:'绕飞平面偏差',flyaround_reverse_travel:'绕飞反向行程',continuous_formation_envelope:'成员未全程保持编队',
        both_spacecraft_active_impulse:'至少一星实际机动不足',simultaneous_cooperative_maneuver:'持续共同机动不足',cooperative_range_reduction:'协同交会距离缩减不足',
        boresight_error:'跟踪时窗指向超差',direction_rate_error:'跟踪方向变化率超差',
        ground_target_below_elevation_limit:'地面目标低于可见仰角',sun_occulted_by_earth:'太阳被地球遮挡'};
      outcome.append(element('small','error',trial.physics_score.quality_violations.map(name=>labels[name]||name).join('、')));
    }
    row.append(element('td','',String(trial.seed)),outcome,
      element('td','',trial.physics_score?.final_time_s==null?'—':`${trial.physics_score.final_time_s.toFixed(2)} s`),
      element('td','',benchmarkMetric(trial.physics_score)));
    if(detail){
      const links=element('td','actions');
      if(trial.result_sha256&&trial.outcome!=='invalid_evidence'){
        for(const [name,label] of [['result','评分'],['physics','物理日志'],['observations','观测'],['actions','动作'],['context','初始输入'],['manifest','版本清单'],['runtime_final','环境变化记录']]){
          if(!trial.artifacts?.includes(name))continue;
          const link=element('a','',label);link.href=`/api/benchmarks/${job.id}/trials/${trial.trial_id}/artifacts/${name}`;links.append(link);
        }
      }
      row.append(links);
    }
    body.append(row);
  }
  table.append(body);scroll.append(table);card.append(scroll);
  const footer=element('div','panel-tail');
  const control=(text,action)=>{
    const button=element('button','button',text);button.type='button';
    button.onclick=async()=>{button.disabled=true;try{const result=await api(`/api/benchmarks/${job.id}/${action}`,{method:'POST'});
      if(action==='rescore')notice(`复算完成：${result.trials.filter(row=>row.matches_recorded_physics_score).length} 个回合与原评分一致。`,false);
      await refresh();}catch(error){notice(error.message);}finally{button.disabled=false;}};
    footer.append(button);return button;
  };
  if(!benchmarkTerminal.has(job.status))control(job.status==='stopping'?'正在停止…':'停止批次','stop').disabled=job.status==='stopping';
  if(job.can_resume)control('继续未启动样本','resume');
  if(benchmarkTerminal.has(job.status)){
    control('独立复算','rescore');
    for(const format of ['json','csv']){const link=element('a','button',`导出 ${format.toUpperCase()}`);link.href=`/api/benchmarks/${job.id}/export?format=${format}`;footer.append(link);}
  }
  if(!detail){const link=element('a','button','查看评测数据');link.href=`/datasets?benchmark=${job.id}`;link.dataset.route='';footer.append(link);}
  card.append(footer,element('p','panel-body muted',`${job.owner.username} · ${new Date(job.started_at*1000).toLocaleString()} · ${job.id}`));
  return card;
}
