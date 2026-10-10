import { rendezvousPlayerUrl } from './rendezvous_player.js';
const byId = (id) => document.getElementById(id);
const points = [];
function missionOptions() {
  return {seed: Number(byId('seed').value), gnss_position_sigma_m: Number(byId('gnssPosition').value),
    gnss_velocity_sigma_m_s: Number(byId('gnssVelocity').value), initial_position_sigma_m: Number(byId('initialPosition').value),
    orbit_acceleration_sigma_m_s2: Number(byId('orbitNoise').value), handoff_distance_m: Number(byId('holdDistance').value),
    drag_enabled: byId('dragEnabled').checked, srp_enabled: byId('srpEnabled').checked};
}
let currentRun = null;
let requesting = false;
let playerUrl = null;
const phases = {'Orbit raising':'升轨飞行', 'Team approach':'团队双星接近', '100 m handoff hold':'100 米交接保持', 'Abort':'已中止', 'Distant approach':'远距离接近', 'Transfer acquisition':'交会准备',
  'Transfer burn 1':'第一次变轨', 'Transfer coast':'滑行接近', 'Transfer turn':'姿态转向',
  'Transfer burn 2':'第二次变轨', 'Handoff hold':'交接保持', 'Near-field approach':'近距离接近',
  '20 m hold':'20 米保持', 'Close approach':'近程接近', '3 m hold':'3 米保持',
  'Final approach':'末段接近', 'Contact approach':'接触接近', 'Closing jaws':'捕获爪闭合',
  'Soft capture':'软捕获', 'Retracting':'回撤', 'Seating':'法兰贴合', 'Coupled':'硬锁与联合体运动'};

async function request(path, options = {}) {
  const response = await fetch(path, { cache: "no-store", ...options });
  if (response.status === 401) throw new Error("请先返回控制台登录");
  const data = await response.json();
  if (!response.ok) throw new Error(data.detail || `HTTP ${response.status}`);
  return data;
}

function draw() {
  const canvas = byId("chart");
  const context = canvas.getContext("2d");
  const { width, height } = canvas;
  context.clearRect(0, 0, width, height);
  context.strokeStyle = "#cbd5e1";
  context.beginPath();
  context.moveTo(46, 14); context.lineTo(46, height - 30); context.lineTo(width - 15, height - 30);
  context.stroke();
  if (points.length < 2) return;
  const maxTime = Math.max(1, ...points.map((point) => point.time));
  const maxRange = Math.max(1, ...points.map((point) => point.range));
  context.strokeStyle = "#225dba";
  context.lineWidth = 2;
  context.beginPath();
  points.forEach((point, index) => {
    const x = 46 + (point.time / maxTime) * (width - 65);
    const y = height - 30 - (point.range / maxRange) * (height - 48);
    if (index === 0) context.moveTo(x, y); else context.lineTo(x, y);
  });
  context.stroke();
}

async function refresh() {
  try {
    const data = await request("/api/rendezvous/state");
    const run = data.runtime || {};
    if (data.run_id !== currentRun) {
      currentRun = data.run_id;
      points.length = 0;
      draw();
    }
    const centerRange = data.state?.metadata?.center_distance_m ?? data.separation_m;
    if (data.state && Number.isFinite(centerRange)) {
      const last = points.at(-1);
      if (!last || data.sim_time_s > last.time) {
        points.push({ time: data.sim_time_s, range: centerRange });
        if (points.length > 1000) points.shift();
        draw();
      }
    }
    const demo = run.mode === 'demo';
    byId("connection").textContent = demo ? (run.running ? '演示播放中' : run.completed ? '演示完成' : run.error ? '演示异常' : '演示已停止') : data.connected ? "仿真运行中" : run.running ? "等待状态" : run.report ? '任务已结束' : "未连接";
    const team = (run.report?.scenario && run.report.scenario !== 'legacy') || run.scenario?.startsWith('team-');
    const phase = data.phase || (run.report?.basic_nominal_passed ? (team ? '100 m handoff hold' : 'Coupled') : "—");
    byId("phase").textContent = phases[phase] || phase;
    byId('playbackTitle').textContent = demo ? `15 秒演示 · ${phases[phase] || phase}` : '交会对接 · 三维画面';
    byId('playbackProgress').textContent = demo ? `${(run.elapsed_s || 0).toFixed(1)} / 15 秒` : '完整仿真';
    byId('progress').value = demo ? run.elapsed_s || 0 : 0;
    if (run.error) byId('message').textContent = run.error;
    const simTime = data.sim_time_s ?? run.report?.final_time_s;
    const separation = centerRange ?? run.report?.final_center_distance_m ?? run.report?.final_body_origin_distance_m;
    const latchTime = data.capture_time_s ?? run.report?.latch_time_s;
    byId("time").textContent = simTime == null ? "—" : `${simTime.toFixed(1)} s`;
    byId("range").textContent = separation == null ? "—" : `${separation.toFixed(3)} m`;
    byId('completionLabel').textContent = team ? '交接状态' : '硬锁时刻';
    byId("latch").textContent = team ? (run.report?.basic_nominal_passed ? '可交接' : '待验收') : latchTime == null ? "—" : `${latchTime.toFixed(3)} s`;
    byId("result").textContent = run.report ? `${demo ? '原任务验收' : '任务验收'} · ${run.report.basic_nominal_passed ? "通过" : "未通过"}` : "—";
    byId('figures').hidden = !run.report || team;
    byId('resultFiles').hidden = !run.report;
    byId('handoffLink').hidden = !team || !run.report?.basic_nominal_passed;
    byId('continue').hidden = !team || !run.report?.basic_nominal_passed;
    byId('continue').disabled = requesting || run.running;
    byId("output").textContent = run.output_dir || "—";
    byId("log").textContent = run.log_file || "—";
    for (const [element, name] of [
      ["missionFigure", "full_mission_diagnostics.png"],
      ["contactFigure", "rendezvous_contact.png"],
    ]) {
      const figure = byId(element);
      figure.hidden = !run.report || team;
      if (run.report && !team && figure.dataset.directory !== run.output_dir) {
        figure.src = `/api/rendezvous/figure/${name}?run=${encodeURIComponent(run.output_dir)}`;
        figure.dataset.directory = run.output_dir;
      }
    }
    byId("start").disabled = requesting || !run.configured || run.running;
    byId('demo').disabled = requesting || !run.demo_available || run.running;
    byId("stop").disabled = !run.running;
  } catch (error) {
    byId("message").textContent = error.message;
  }
}

for (const action of ["start", "stop", "demo"]) {
  byId(action).addEventListener("click", async () => {
    try {
      requesting = true;
      byId('demo').disabled = byId('start').disabled = true;
      byId("message").textContent = "正在处理…";
      await request(`/api/rendezvous/${action === 'demo' ? 'start' : action}`, {
        method: "POST",
        ...(action !== "stop" ? {
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ live: action === 'demo' || byId("live").checked, mode: action === 'demo' ? 'demo' : 'simulation', scenario: byId('scenario').value, options: missionOptions() }),
        } : {}),
      });
      // A cold UE launch can outlast the player's initial streamer retries.
      // Reconnect once the backend has started the renderer for this mission.
      if (action !== 'stop' && (action === 'demo' || byId('live').checked) && playerUrl) {
        byId('demoVideo').src = playerUrl;
      }
      byId("message").textContent = action === 'demo' ? '正在播放已验收任务的 15 秒演示' : action === "start" ? "任务已启动" : "任务已停止";
      requesting = false;
      await refresh();
    } catch (error) {
      requesting = false;
      byId("message").textContent = error.message;
    }
  });
}

request('/api/client-config').then((config) => {
  playerUrl = rendezvousPlayerUrl(window.location.href, config);
  byId('demoVideo').src = playerUrl;
}).catch((error) => { byId('message').textContent = `视频连接失败：${error.message}`; });
refresh();
byId('continue').addEventListener('click', async () => {
  requesting = true;
  byId('continue').disabled = true;
  byId('message').textContent = '正在将双星状态交接到机械臂场景…';
  try {
    await request('/api/rendezvous/handoff', {method: 'POST'});
    window.location.assign('/');
  } catch (error) {
    byId('message').textContent = error.message;
    requesting = false;
  }
});
setInterval(refresh, 100);
byId('scenario').addEventListener('change', () => {
  const legacy = byId('scenario').value === 'legacy';
  byId('missionOptions').hidden = legacy;
  byId('demo').hidden = !legacy;
  byId('missionDescription').textContent = legacy ? '历史通用模型：接近、捕获、硬锁与联合体控制' : '双星导航 → 推进与姿态控制 → 100 m 保持 → 近程交接';
});
