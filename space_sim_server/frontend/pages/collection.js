import template from './collection.html?raw';
import {mountView} from '../ui.js';
export function createCollectionPage(root,{api}) {
const $=mountView(root,template);
const taskLabels = new Map();
const terminal = new Set(["complete", "failed", "cancelled", "interrupted", "attempt_limit"]);
const names = {finalizing:"封装视频与数据", validating:"检查任务与数据完整性", seat:"确认就位",running:"运行中", starting:"启动中", queued:"等待启动", stopping:"正在收尾", complete:"已完成", failed:"失败", cancelled:"已停止", interrupted:"已中断", attempt_limit:"达到尝试上限", preflight:"初始状态检查", recording:"录制中", accepted:"校验通过", initialization_rejected:"初始状态未通过", validation_failed:"数据校验失败", recording_or_task_failed:"任务或录制失败", writer_ready:"录制器就绪", process_started:"启动场景", warming_renderer:"相机预热", completed:"录制完成", task_progress:"执行任务", settle:"初始停稳", raise:"抬高夹爪", above_pickup:"跟踪取件位置", above_drawer:"移动到抽屉上方", align:"调整抓取姿态", open:"张开夹爪", descend:"下降", grip:"夹持", lift:"提起 / 抽出", transfer:"搬运到目标孔", above_hole:"移动到孔口上方", align_hole:"对准插孔", insert:"插入", return:"放回", release:"松爪", retract:"撤离", verify:"最终检查"};
const label = (value) => names[value] || value || "等待进度";
const element = (tag, text, cls) => { const el=document.createElement(tag); if(text!==undefined) el.textContent=text; if(cls) el.className=cls; return el; };
function notice(message=""){ $("notice").textContent=message; $("notice").hidden=!message; }
const duration = (seconds) => `${Math.floor(seconds/60)} 分 ${Math.floor(seconds%60)} 秒`;
function renderJob(job) {
  const card=element("article",undefined,"panel job");
  const head=element("div",undefined,"job-head"), title=element("div");
  title.append(element("h3",taskLabels.get(job.config.task) || job.config.task),element("div",job.id,"muted job-id"));
  head.append(title,element("span",label(job.status),`badge ${job.status}`)); card.append(head);
  const metrics=element("div",undefined,"metrics");
  for(const [value,caption] of [[`${job.success_count} / ${job.config.count}`,"成功记录"],[job.config.workers,"并发实例"],[duration(job.elapsed_seconds),"实际耗时"]]){
    const item=element("div");item.append(element("strong",String(value)),element("span",caption));metrics.append(item);
  }
  card.append(metrics);
  const bar=element("progress");bar.max=job.config.count;bar.value=job.success_count;bar.setAttribute("aria-label","成功采集进度");card.append(bar);
  if(job.error)card.append(element("p",job.error,"error"));
  const workers=element("div",undefined,"workers");
  for(const worker of (job.workers || [])){
    const box=element("div",undefined,"worker");
    box.append(element("h4",`实例 ${worker.index+1} · ${label(worker.status)}`));
    const progress=worker.progress || {}, task=progress.task || {};
    box.append(element("p",`阶段：${label(terminal.has(job.status) ? worker.phase || worker.status : worker.phase === "validating" ? "validating" : progress.stage === "finalizing" ? "finalizing" : task.stage || progress.stage || worker.phase || worker.collector_status)}`));
    box.append(element("p",`已验收 ${worker.accepted_episodes.length} / ${worker.count} · 已尝试 ${(worker.attempts || []).length} / ${worker.max_attempts}`));
    box.append(element("p",`种子 ${worker.seed}–${worker.seed+worker.max_attempts-1}`));
    const sync=progress.sync || {}; if(sync.dataset_frame_count!==undefined) box.append(element("p",`已录制 ${sync.dataset_frame_count} 帧`));
    if(worker.error) box.append(element("p",worker.error,"error"));
    if(worker.forced_stop) box.append(element("p","正常收尾超时，进程已强制清理；未完成记录不会计入成功。"));
    const attempts=worker.attempts || [];
    for(const attempt of attempts){
      if(attempt.end_to_end_wall_seconds!==undefined) box.append(element("p",`尝试 ${attempt.attempt} · ${attempt.status==='accepted'?'启动至验收':'本次耗时'} ${duration(attempt.end_to_end_wall_seconds)}`));
      if(attempt.episode_id && !worker.accepted_episodes.includes(attempt.episode_id)){
        const link=element("a",`查看尝试 ${attempt.attempt} · ${label(attempt.status)}`);link.href=`/datasets?episode=${encodeURIComponent(attempt.episode_id)}`;link.dataset.route="";box.append(link);
      }
    }
    if(worker.log_tail){const detail=element("details",undefined,"error");detail.dataset.detailKey=job.id+":"+worker.index;detail.append(element("summary","运行日志"),element("pre",worker.log_tail));box.append(detail);}
    workers.append(box);
  }
  card.append(workers);
  const links=element("div",undefined,"episodes");
  for(const [i,id] of job.accepted_episodes.entries()) {const link=element("a",`成功记录 ${i+1} · 查看曲线与视频`);link.href=`/datasets?episode=${encodeURIComponent(id)}`;link.dataset.route="";links.append(link);}card.append(links);
  const footer=element("div",undefined,"panel-tail");footer.append(element("span",`${job.owner.username} · ${new Date(job.started_at*1000).toLocaleString()} · 初始偏移 ±${job.config.range_deg}°`,"muted"));
  const datasetLink=element("a","查看该批次数据","button text-button");datasetLink.href=`/datasets?batch=${encodeURIComponent(job.id)}`;datasetLink.dataset.route="";footer.append(datasetLink);
  if(!terminal.has(job.status)){
    const stop=element("button",job.status==="stopping"?"正在收尾…":"停止该批次","button");stop.dataset.focusKey=job.id;stop.disabled=job.status==="stopping";
    stop.onclick=async()=>{stop.disabled=true;try{await api(`/api/collections/${encodeURIComponent(job.id)}/stop`,{method:"POST"});await refresh();}catch(error){notice(error.message);stop.disabled=false;}};
    footer.append(stop);
  }card.append(footer); return card;
}

let active=false, timer=null, epoch=0, refreshing=false, submitting=false, previousJobs='', catalog=null;
function historyRow(job) {
  const details=element('details',undefined,'panel'); details.dataset.history=job.id;
  const summary=element('summary',undefined,'history-row'), info=element('span');
  info.append(element('strong',taskLabels.get(job.config.task)||job.config.task),element('small',    `${new Date(job.started_at*1000).toLocaleString()} · ${job.success_count} / ${job.config.count} 条成功 · ${job.owner.username}`));
  summary.append(info,element('span',label(job.status),`badge ${job.status}`));details.append(summary);
  details.addEventListener('toggle',()=>{if(details.open&&!details.dataset.loaded){details.dataset.loaded='true';const body=renderJob(job);body.classList.remove('panel');details.append(body);}});
  return details;
}
async function refresh(){
  if(!active||refreshing)return;refreshing=true;const version=epoch;
  try{
    const {jobs}=await api('/api/collections'); if(!active||version!==epoch)return;
    const snapshot=JSON.stringify(jobs);
    if(snapshot!==previousJobs){
      const opened=new Set([...root.querySelectorAll('details[open][data-detail-key]')].map(node=>node.dataset.detailKey));
      const historyOpened=new Set([...root.querySelectorAll('details[open][data-history]')].map(node=>node.dataset.history));
      const focused=document.activeElement?.dataset.focusKey;
      const current=jobs.filter(job=>!terminal.has(job.status)), history=jobs.filter(job=>terminal.has(job.status));
      $('jobs').replaceChildren(...(current.length?current.map(renderJob):[element('div','当前没有运行中的采集批次。点击“新建采集”开始。','panel empty')]));
      $('history').replaceChildren(...(history.length?history.map(historyRow):[element('div','尚无历史批次','panel empty')]));
      root.querySelectorAll('details[data-history]').forEach(node=>{node.open=historyOpened.has(node.dataset.history);});
      root.querySelectorAll('details[data-detail-key]').forEach(node=>{node.open=opened.has(node.dataset.detailKey);});
      if(focused)[...root.querySelectorAll('[data-focus-key]')].find(node=>node.dataset.focusKey===focused)?.focus({preventScroll:true});
      $('history-count').textContent=history.length+' 个批次';previousJobs=snapshot;
    }
    $('submit').disabled=submitting||!catalog||jobs.some(job=>!terminal.has(job.status));
    $('refresh-state').textContent='更新于 '+new Date().toLocaleTimeString();
  }catch(error){if(active&&version===epoch)notice(error.message);}finally{refreshing=false;}
}
function specs(data){
  const rates=data.rates,camera=data.cameras;
  const rows=[['初始姿态 J1–J6',data.baseline_joint_deg.map(value=>value+'°').join('，')],['动力学 / IK',rates.dynamics+' Hz / '+rates.ik+' Hz'],['状态、动作、双相机',rates.main+' Hz · '+camera.width+' × '+camera.height],['六维力',rates.wrench+' Hz · 噪声'+(data.wrist_wrench_noise?'开启':'关闭')],['光照随机化',data.lighting_randomized?'开启':'关闭'],['运动与观测扰动',data.disturbances_enabled?'开启':'关闭'],['训练状态与动作','6 个关节 + 夹爪 0–1']];
  $('specifications').replaceChildren(...rows.flatMap(([key,value])=>[element('dt',key),element('dd',value)]));
}
function describeTask(){ $('task-description').textContent=catalog?.tasks.find(task=>task.id===$('task').value)?.description||''; }
$('task').onchange=describeTask;
$('new-collection').onclick=()=>{$('setup').hidden=false;$('task').focus();};
$('close-setup').onclick=$('cancel-setup').onclick=()=>{$('setup').hidden=true;};
$('collection-form').onsubmit=async event=>{
  event.preventDefault();if(submitting)return;const version=epoch;notice();submitting=true;$('submit').disabled=true;
  const values=Object.fromEntries(new FormData(event.currentTarget));for(const key of ['count','workers','max_attempts','seed','range_deg'])values[key]=Number(values[key]);
  try{await api('/api/collections',{method:'POST',body:JSON.stringify(values)});if(active&&version===epoch)$('setup').hidden=true;}
  catch(error){if(active&&version===epoch)notice(error.message);}
  finally{submitting=false;if(active&&version===epoch)await refresh();}
};
return {
  async activate(){
    active=true;const version=++epoch;
    try{
      if(!catalog){const data=await api('/api/collections/catalog');if(!active||version!==epoch)return;catalog=data;
        for(const task of data.tasks)taskLabels.set(task.id,task.label);
        $('task').replaceChildren(...data.tasks.map(task=>{const option=element('option',task.label);option.value=task.id;return option;}));
        describeTask();
        [...$('workers').options].forEach(option=>{option.disabled=Number(option.value)>data.max_workers;});specs(data);
      }
    }catch(error){if(active&&version===epoch)notice(error.message);}
    if(active&&version===epoch){await refresh();if(active&&version===epoch){clearInterval(timer);timer=setInterval(refresh,2000);}}
  },
  deactivate(){active=false;epoch++;clearInterval(timer);timer=null;},
  reset(){this.deactivate();catalog=null;previousJobs='';taskLabels.clear();$('jobs').replaceChildren();$('history').replaceChildren();$('setup').hidden=true;$('collection-form').reset();notice();},
};
}
