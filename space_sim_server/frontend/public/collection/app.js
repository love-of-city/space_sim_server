const $ = (id) => document.getElementById(id);
const taskLabels = new Map();
const terminal = new Set(["complete", "failed", "cancelled", "interrupted", "attempt_limit"]);
const names = {finalizing:"封装视频与数据", validating:"检查任务与数据完整性", seat:"确认抽屉归位",running:"运行中", starting:"启动中", queued:"等待启动", stopping:"正在收尾", complete:"已完成", failed:"失败", cancelled:"已停止", interrupted:"已中断", attempt_limit:"达到尝试上限", preflight:"初始状态检查", recording:"录制中", accepted:"校验通过", initialization_rejected:"初始状态未通过", validation_failed:"数据校验失败", recording_or_task_failed:"任务或录制失败", writer_ready:"录制器就绪", process_started:"启动场景", warming_renderer:"相机预热", completed:"录制完成", task_progress:"执行任务", settle:"初始停稳", raise:"抬高夹爪", above_pickup:"移动到抽屉上方", above_drawer:"移动到抽屉上方", align:"调整抓取姿态", open:"张开夹爪", descend:"下降", grip:"夹持", lift:"抽出", return:"放回", release:"松爪", retract:"撤离", verify:"最终检查"};
const label = (value) => names[value] || value || "等待进度";
const element = (tag, text, cls) => { const el=document.createElement(tag); if(text!==undefined) el.textContent=text; if(cls) el.className=cls; return el; };
function notice(message=""){ $("notice").textContent=message; $("notice").hidden=!message; }
async function api(path, options={}) {
  const response=await fetch(path, {...options, headers:{"Content-Type":"application/json", ...options.headers}});
  if(response.status===401){ location.href="/"; throw new Error("请先登录平台"); }
  const body=await response.json();
  if(!response.ok) throw new Error(typeof body.detail === "string" ? body.detail : JSON.stringify(body.detail || body));
  return body;
}
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
  for(const worker of job.workers){
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
      if(attempt.episode_id && !worker.accepted_episodes.includes(attempt.episode_id)){
        const link=element("a",`查看尝试 ${attempt.attempt} · ${label(attempt.status)} ↗`);link.href=`/datasets?episode=${encodeURIComponent(attempt.episode_id)}`;link.target="_blank";link.rel="noopener";box.append(link);
      }
    }
    if(worker.log_tail){const detail=element("details",undefined,"error");detail.append(element("summary","运行日志"),element("pre",worker.log_tail));box.append(detail);}
    workers.append(box);
  }
  card.append(workers);
  const links=element("div",undefined,"episodes");
  for(const [i,id] of job.accepted_episodes.entries()) {const link=element("a",`成功记录 ${i+1} · 查看曲线与视频 ↗`);link.href=`/datasets?episode=${encodeURIComponent(id)}`;link.target="_blank";link.rel="noopener";links.append(link);}card.append(links);
  const footer=element("div",undefined,"footer");footer.append(element("span",`${job.owner.username} · ${new Date(job.started_at*1000).toLocaleString()} · 初始偏移 ±${job.config.range_deg}°`,"muted"));
  if(!terminal.has(job.status)){
    const stop=element("button",job.status==="stopping"?"正在收尾…":"停止该批次");stop.disabled=job.status==="stopping";
    stop.onclick=async()=>{stop.disabled=true;try{await api(`/api/collections/${encodeURIComponent(job.id)}/stop`,{method:"POST"});await refresh();}catch(error){notice(error.message);stop.disabled=false;}};
    footer.append(stop);
  }card.append(footer); return card;
}
let refreshing=false, submitting=false, previousJobs="";
async function refresh(){
  if(refreshing) return;refreshing=true;
  try{
    const {jobs}=await api("/api/collections");
    const snapshot=JSON.stringify(jobs);
    if(snapshot!==previousJobs) $("jobs").replaceChildren(...(jobs.length ? jobs.map(renderJob) : [element("div","还没有采集批次。设置左侧参数，开始第一批自动采集。","panel empty")]));
    previousJobs=snapshot;
    $("submit").disabled=submitting || jobs.some(job=>!terminal.has(job.status));
    $("refresh-state").textContent=`更新于 ${new Date().toLocaleTimeString()}`;
  }catch(error){notice(error.message);}finally{refreshing=false;}
}
$("collection-form").onsubmit=async(event)=>{
  event.preventDefault();notice();submitting=true;$("submit").disabled=true;
  const values=Object.fromEntries(new FormData(event.currentTarget));
  for(const key of ["count","workers","max_attempts","seed","range_deg"])values[key]=Number(values[key]);
  try{await api("/api/collections",{method:"POST",body:JSON.stringify(values)});}catch(error){notice(error.message);}finally{submitting=false;await refresh();}
};
try{
  const {user}=await api("/api/auth/me");$("account").textContent=`${user.username} · ${user.role==="admin"?"管理员":"操作员"}`;
  const catalog=await api("/api/collections/catalog");
  for(const task of catalog.tasks) taskLabels.set(task.id,task.label);
  $("task").replaceChildren(...catalog.tasks.map(task=>{const option=element("option",task.label);option.value=task.id;return option;}));
  await refresh();setInterval(refresh,2000);
}catch(error){notice(error.message);}
