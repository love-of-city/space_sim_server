import {element} from '../ui.js';

export const benchmarkTerminal = new Set(['complete','stopped','interrupted','failed']);
export const benchmarkLabels = {
  starting:'正在启动',running:'运行中',stopping:'正在停止',complete:'评测结束',stopped:'已停止',interrupted:'已中断',failed:'运行失败',
  success:'成功',pending:'待运行',cancelled:'已取消',safety_violation:'违反安全限制',time_limit:'达到时限',
  method_error:'方法失败',infrastructure_error:'基础设施失败',invalid_evidence:'证据无效',
  invalid_initialization:'初态无效',task_failure:'任务未达标',
};
export const methodLabels = {
  'gnss-pd-v1':'GNSS PD','no-thrust-v1':'零推力（负对照）','unsafe-approach-v1':'持续接近（负对照）',
  'attitude-wheel-pd-v1':'姿态 PD','attitude-rate-limited-pd-v1':'角速度受限姿态 PD',
  'no-wheel-torque-v1':'零飞轮力矩（负对照）','unsafe-spin-v1':'持续满力矩（负对照）',
  'external-attitude-pd-v1':'姿态 PD · 独立进程示例','external-gnss-pd-v1':'GNSS PD · 独立进程示例',
  'gnss-orbit-maneuver-v1':'GNSS 轨道机动','no-orbit-thrust-v1':'零轨控推力（负对照）',
  'unsafe-orbit-burn-v1':'持续轨控推力（负对照）','external-orbit-maneuver-v1':'轨道机动 · 独立进程示例',
  'gnss-orbit-estimator-v1':'GNSS 轨道状态估计','gnss-measurement-only-v1':'直接使用 GNSS 测量',
  'zero-navigation-estimate-v1':'零状态估计（负对照）','external-gnss-estimator-v1':'GNSS 状态估计 · 独立进程示例',
  'gnss-relative-flight-v1':'GNSS 相对飞行','no-relative-thrust-v1':'零相对飞行推力（负对照）',
  'unsafe-relative-flight-v1':'横向偏离（负对照）','external-relative-flight-v1':'相对飞行 · 独立进程示例',
};
export function benchmarkMetric(score){
  if(Number.isFinite(score?.final_attitude_error_rad))return `${(score.final_attitude_error_rad*180/Math.PI).toFixed(4)}°`;
  if(Number.isFinite(score?.position_rmse_m))return `位置 RMSE ${score.position_rmse_m.toFixed(4)} m；速度 RMSE ${score.velocity_rmse_m_s.toFixed(4)} m/s`;
  if(Number.isFinite(score?.final_position_error_m))return `${score.final_position_error_m.toFixed(4)} m`;
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
    if(trial.physics_score?.safety_violations?.length)outcome.append(element('small','error',trial.physics_score.safety_violations.join('、')));
    if(trial.physics_score?.quality_violations?.length){
      const labels={position_rmse:'位置 RMSE',velocity_rmse:'速度 RMSE',maximum_position_error:'最大位置误差',maximum_velocity_error:'最大速度误差',
        continuous_hold_envelope:'全程保持范围',approach_progress:'接近距离'};
      outcome.append(element('small','error',trial.physics_score.quality_violations.map(name=>labels[name]||name).join('、')));
    }
    row.append(element('td','',String(trial.seed)),outcome,
      element('td','',trial.physics_score?.final_time_s==null?'—':`${trial.physics_score.final_time_s.toFixed(2)} s`),
      element('td','',benchmarkMetric(trial.physics_score)));
    if(detail){
      const links=element('td','actions');
      if(trial.result_sha256&&trial.outcome!=='invalid_evidence'){
        for(const [name,label] of [['result','评分'],['physics','物理日志'],['observations','观测'],['actions','动作'],['manifest','版本清单']]){
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
