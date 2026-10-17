import { requestJson } from './api_client.js';

const phases = {idle: '平台空闲', starting: '正在启动任务', flight: '交会运行中', ready: '交会已完成 · 可接续',
  transitioning: '正在连续接入近距阶段',
  handoff: '正在接续遥操作', launching: '正在启动场景', starting_renderer: '正在启动 UE',
  starting_simulation: '正在启动仿真', running: '遥操作场景运行中', stopping: '正在停止任务',
  stopped: '任务已停止', failed: '任务未完成', completed: '任务已结束', unknown: '状态未知'};
const flightPhases = {'Orbit raising': '升轨飞行', 'Team approach': '双星接近', '100 m handoff hold': '100 m 外保持', Abort: '已中止',
  FarHandoff: '远距状态交接', GNSSApproach: '相对 GNSS 接近', VisualFusion: '视觉融合', VisualApproach: '视觉主导接近',
  PlatformHold: '平台驻留', ArmDeploy: '机械臂展开', ArmAlign: '机械臂视觉对准', FinalApproach: '末段接近',
  ContactProxy: '进入虚拟接触窗口', Grasp: '夹取代理', CaptureConfirm: '捕获确认代理', CaptureHold: '捕获保持（逻辑代理）', SafeHold: '安全保持 · 闭合已禁止'};
const fields = {missionSeed: 'seed', missionHold: 'handoff_distance_m', missionGnssPosition: 'gnss_position_sigma_m',
  missionGnssVelocity: 'gnss_velocity_sigma_m_s', missionInitialPosition: 'initial_position_sigma_m', missionOrbitNoise: 'orbit_acceleration_sigma_m_s2'};
const number = (value, digits, unit) => Number.isFinite(value) ? `${value.toFixed(digits)} ${unit}` : '—';

export function missionArtifactLinks(mission) {
  if (!mission.mission_id) return [];
  return (mission.stages || []).filter(stage => ['far', 'near'].includes(stage.phase)).flatMap(stage =>
    [['mission_report.json', '验收报告'], ['handoff.json', '交接状态'], ['team_telemetry.npz', '原始遥测']].map(([file, label]) => ({
      label: `${stage.phase === 'far' ? '远距' : '近距'}${label}`,
      href: `/api/rendezvous/artifact/${file}?mission_id=${encodeURIComponent(mission.mission_id)}&phase=${stage.phase}`,
    })));
}

export function missionPresentation(mission = {}, selectedMode = 'teleop') {
  const inFlight = mission.stage === 'flight' && mission.phase !== 'stopped';
  const flightView = inFlight || (!mission.active && selectedMode !== 'teleop');
  return {flightView, label: phases[mission.phase] || '平台空闲',
    canStart: !mission.active, canStop: (!!mission.active || mission.phase === 'ready') && !!mission.can_manage && mission.phase !== 'stopping',
    canHandoff: !!mission.can_handoff && !!mission.can_manage && !mission.active,
    showHandoff: !!mission.can_handoff};
}

export function createMissionPanel({$, state, apiRequest, refresh, message, configureScene, reconnect}) {
  let catalog = {modes: [], flight_defaults: {}};
  let snapshot = {};
  let observedId;
  let points = [];
  let available = false;
  let stateFailed = true;
  const intro = $('controlDescription').textContent;

  function configure() {
    state.missionMode = $('missionMode').value || 'teleop';
    const flight = state.missionMode !== 'teleop';
    const near = state.missionMode.startsWith('near-field');
    $('missionSettingsNote').textContent = near
      ? '自动接近、机械臂对准及逻辑捕获保持。主视窗使用演示补光；暗处视觉能力尚未验证。'
      : '使用团队双星模型，按同一任务推进远距交会、近距接近与操作。当前近距视觉仍为参数化观测，捕获结果需区分逻辑代理与真实接触。';
    $('nearFieldSettings').hidden = !near;
    $('orbitalFlightOptions').hidden = near;
    $('missionHoldField').hidden = near;
    $('nearFieldFailure').disabled = !!snapshot.active;
    const templateIds = catalog.modes.find(m => m.id === state.missionMode)?.template_ids;
    if (templateIds && !templateIds.includes($('sceneTemplate').value)) {
      $('sceneTemplate').value = templateIds[0];
      configureScene();
    }
    $('teleopSettings').hidden = flight;
    $('flightSettings').hidden = !flight || !!snapshot.active;
    $('sceneTemplate').disabled = !!snapshot.active || flight;
    for (const input of $('flightSettings').querySelectorAll('input')) input.disabled = !!snapshot.active;
    // A restored task may replace the mode on its first state response. Do not
    // accept a user selection before that response (including login recovery).
    $('missionMode').disabled = !!snapshot.active || !available || stateFailed;
    const view = missionPresentation(snapshot, state.missionMode);
    $('sceneCollisionWarning').hidden = view.flightView;
    $('sceneInstanceSummary').hidden = view.flightView;
    $('sceneDetails').hidden = view.flightView;
    document.body.classList.toggle('flight-mode', view.flightView);
    $('flightTelemetry').hidden = !view.flightView;
    $('controlHeading').textContent = view.flightView ? '双星交会任务' : '末端六维遥操作';
    $('controlDescription').textContent = view.flightView
      ? near ? '近距闭环演示：GNSS → 视觉融合与接管 → 驻留 → 机械臂对准 → 夹取与逻辑捕获保持。双目为参数化观测。' : '连续任务在原工作台推进，当前阶段与导航质量见遥测。C 进入自由视角，Home 返回主视角。' : intro;
    $('startScene').textContent = flight ? '启动连续任务' : '生成并启动场景';
    $('stopScene').textContent = '停止任务';
    const configured = flight ? snapshot.runtime?.configured !== false : snapshot.scene?.enabled !== false;
    $('startScene').disabled = !available || !configured || stateFailed || !view.canStart || !!state.sceneTransitionPending;
    $('stopScene').disabled = stateFailed || !view.canStop || !!state.sceneTransitionPending;
    $('continueMission').hidden = !view.showHandoff;
    $('continueMission').disabled = stateFailed || !view.canHandoff || !!state.sceneTransitionPending;
  }

  function draw() {
    const canvas = $('missionChart'), ctx = canvas.getContext('2d');
    const w = canvas.width, h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.strokeStyle = '#cbd5e1'; ctx.beginPath(); ctx.moveTo(60, 15); ctx.lineTo(60, h - 30); ctx.lineTo(w - 15, h - 30); ctx.stroke();
    if (!points.length) return;
    const start = points[0].t, end = points.at(-1).t, max = Math.max(1, ...points.map(p => p.r));
    ctx.fillStyle = '#526577'; ctx.font = '16px system-ui';
    ctx.fillText(`${max.toFixed(0)} m`, 0, 18); ctx.fillText(`${start.toFixed(0)} s`, 60, h - 5);
    ctx.fillText(`${end.toFixed(0)} s`, w - 95, h - 5);
    ctx.strokeStyle = '#225dba'; ctx.beginPath();
    points.forEach((p, i) => { const x = 60 + (p.t - start) / Math.max(1, end - start) * (w - 80), y = h - 30 - p.r / max * (h - 50);
      if (i) ctx.lineTo(x, y); else ctx.moveTo(x, y); });
    ctx.stroke();
  }

  function render(mission) {
    if (!mission) return;
    const previousPhase = snapshot.phase;
    snapshot = mission;
    state.mission = mission;
    stateFailed = false;
    const changed = observedId !== mission.mission_id;
    if (changed || mission.active) {
      if (catalog.modes.some(m => m.id === mission.mode)) $('missionMode').value = mission.mode;
    }
    if (changed) {
      observedId = mission.mission_id;
      points = [];
      $('nearFieldFailure').value = mission.options?.vision_loss_phase || '';
      for (const [id, key] of Object.entries(fields)) {
        if (mission.options?.[key] != null) $(id).value = String(mission.options[key]);
      }
      for (const [id, key] of [['missionDrag', 'drag_enabled'], ['missionSrp', 'srp_enabled']]) {
        if (mission.options?.[key] != null) $(id).checked = mission.options[key];
      }
    }
    configure();
    const view = missionPresentation(mission, state.missionMode);
    if (mission.phase !== previousPhase) {
      if (['flight', 'ready', 'failed'].includes(mission.phase)) message(mission.error || view.label);
      // A background cold start acknowledges before UE is ready. Reconnect
      // once the flight executor is live, instead of waiting for the watchdog.
      if (mission.phase === 'flight' && !state.streamLive) reconnect();
    }
    $('scenePhase').textContent = view.label;
    $('missionStatus').textContent = mission.error || (mission.mission_id
      ? `${!mission.active ? '保存任务：' : ''}${catalog.modes.find(m => m.id === mission.mode)?.label || mission.mode} · ${view.label}${mission.restored && !mission.active ? ' · 已恢复保存记录' : ''}` : '选择任务后启动');
    $('missionProvenance').textContent = mission.mission_id ? JSON.stringify({mission_id: mission.mission_id,
      creator: mission.created_by?.username || '历史任务', flight_output_dir: mission.flight_output_dir,
      source_run_id: mission.source_run_id, scene_instance_id: mission.scene_instance_id, episodes: mission.episodes,
      stages: mission.stages, options: mission.options}, null, 2) : '尚未创建任务';
    $('missionStageFiles').innerHTML = missionArtifactLinks(mission).map(link => `<a href="${link.href}" download>${link.label}</a>`).join('');
    const telemetry = mission.telemetry || {}, meta = telemetry.state?.metadata || {}, report = mission.report;
    const t = telemetry.sim_time_s ?? report?.final_time_s;
    const distance = meta.center_distance_m ?? telemetry.separation_m ?? report?.final_center_distance_m;
    $('missionConnection').textContent = telemetry.connected ? '遥测在线' : mission.phase === 'flight' ? '等待遥测' : '保存结果';
    $('missionPhase').textContent = flightPhases[telemetry.phase || report?.outcome] || (report?.basic_nominal_passed ? '100 m 外保持' : view.label);
    $('missionTime').textContent = number(t, 1, 's');
    $('missionRange').textContent = number(distance, 3, 'm');
    const nearMeta = meta.navigation_source ? meta : report?.final || {};
    $('missionNavigation').textContent = nearMeta.navigation_source || (meta.navigation_valid === true && meta.gnss?.length === 2
      ? '双星有效' : meta.navigation_valid === false ? '导航失效' : '—');
    $('nearPlatformDistance').textContent = number(nearMeta.platform_to_grasp_m, 4, 'm');
    $('nearTcpGap').textContent = number(nearMeta.tcp_gap_m, 4, 'm');
    $('nearCameraDepth').textContent = number(nearMeta.optical?.camera_depth_m, 4, 'm');
    $('nearAlignment').textContent = `${number(nearMeta.lateral_error_m, 4, 'm')} / ${number(nearMeta.alignment_error_deg, 3, '°')}`;
    $('nearQuality').textContent = number(nearMeta.position_sigma_m, 4, 'm');
    $('nearCapture').textContent = nearMeta.capture_state || '—';
    const bodies = telemetry.state?.bodies || [];
    const chaser = bodies.find(b => b.body_id === 'rendezvous/chaser'), target = bodies.find(b => b.body_id === 'rendezvous/target');
    $('missionAltitude').textContent = chaser ? number((Math.hypot(...chaser.position_m) - 6378136.6) / 1000, 3, 'km') : '—';
    $('missionSpeed').textContent = Number.isFinite(nearMeta.relative_speed_m_s) ? number(nearMeta.relative_speed_m_s, 4, 'm/s') : chaser && target ? number(Math.hypot(...chaser.velocity_m_s.map((v, i) => v - target.velocity_m_s[i])), 3, 'm/s') : '—';
    $('missionResult').textContent = report ? `任务验收 · ${report.basic_nominal_passed ? '通过' : '未通过'}` : '等待任务验收';
    $('missionFiles').hidden = !report;
    for (const link of $('missionFiles').querySelectorAll('a')) {
      let path = link.getAttribute('href').split('?')[0];
      if (/\/(input_)?handoff\.json$/.test(path)) {
        const near = mission.mode?.startsWith('near-field');
        path = path.replace(/\/(input_)?handoff\.json$/, near ? '/input_handoff.json' : '/handoff.json');
        link.textContent = near ? '输入交接状态' : '交接状态';
      }
      link.href = path + (mission.mission_id ? '?mission_id=' + encodeURIComponent(mission.mission_id) : '');
    }
    if (Number.isFinite(t) && Number.isFinite(distance) && (!points.length || t > points.at(-1).t)) {
      points.push({t, r: distance});
      if (points.length > 1200) points = points.filter((_, i) => i % 2 === 0);
    }
    draw();
  }

  async function act(path, body) {
    if (state.sceneTransitionPending) return;
    state.sceneTransitionPending = true;
    state.stateRequestSequence += 1;
    configure();
    try {
      const data = await requestJson(path, {method: 'POST', ...(body ? {headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)} : {})}, apiRequest);
      render(data);
      message(phases[data.phase] || '任务请求已接收');
      // An idle player may have exhausted its initial attempts before cold UE startup.
      reconnect();
    } catch (error) { message(error.message); }
    finally { state.sceneTransitionPending = false; configure(); await refresh(); }
  }

  $('missionMode').addEventListener('change', configure);
  $('continueMission').addEventListener('click', () => act('/api/mission/handoff'));
  return {
    async load() {
      catalog = await requestJson('/api/mission/catalog', {}, apiRequest);
      $('missionMode').replaceChildren(...catalog.modes.map(m => new Option(m.label, m.id)));
      if (!snapshot.mission_id) {
        for (const [id, key] of Object.entries(fields)) {
          if (catalog.flight_defaults?.[key] != null) $(id).value = String(catalog.flight_defaults[key]);
        }
      }
      available = true;
      configure();
    },
    render, configure,
    reset() { snapshot = {}; observedId = undefined; points = []; available = false; stateFailed = true; state.mission = null; configure(); },
    failed() { stateFailed = true; $('missionStatus').textContent = '任务状态连接中断，恢复后自动同步'; configure(); },
    async startFlight() {
      for (const input of $('flightSettings').querySelectorAll('input[type="number"]')) {
        if (!input.reportValidity()) return;
      }
      const near = state.missionMode.startsWith('near-field');
      const options = near ? {seed: Number($('missionSeed').value), vision_loss_phase: $('nearFieldFailure').value}
        : Object.fromEntries(Object.entries(fields).map(([id, key]) => [key, Number($(id).value)]));
      if (!near) { options.drag_enabled = $('missionDrag').checked; options.srp_enabled = $('missionSrp').checked; }
      const continuous = !near && catalog.continuous_default === true;
      if (continuous) options.differential_gnss = {...catalog.flight_defaults.differential_gnss,
        absolute_position_sigma_m: options.gnss_position_sigma_m, absolute_velocity_sigma_m_s: options.gnss_velocity_sigma_m_s};
      await act('/api/mission/start', {mode: state.missionMode, scene: {template_id: $('sceneTemplate').value}, options, continuous});
    },
  };
}
