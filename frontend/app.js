import {
  Config,
  Flags,
  OptionParameters,
  PixelStreaming,
  TextParameters,
} from "@epicgames-ps/lib-pixelstreamingfrontend-ue5.6";
import "./styles.css";
import { FreeCameraController } from "./free_camera.js";
import { GamepadInput, gamepadControlBlockReason } from "./gamepad_input.js";

const gamepadInput = new GamepadInput();

const state = {
  ws: null,
  connected: false,
  controlGranted: false,
  activeEpisode: null,
  sequence: 0,
  pressed: new Set(),
  estopped: false,
  operationActive: false,
  freeCameraMode: false,
  lastAckAt: 0,
  linearSpeed: 0.05,
  pixelStreaming: null,
  pixelConfig: null,
  streamConfig: null,
  selectedStreamerId: "BskRenderer",
  lastPresentedFrames: null,
  lastPresentedTimestamp: null,
  streamLive: false,
  streamReconnectTimer: null,
  pixelConnectPromise: null,
  scenePhase: "idle",
  sceneReady: false,
  sceneDefaults: { simulation_rate: 1, capture_rate_hz: 10, ik_rate_hz: 100 },
  currentUser: null,
  canManageScene: false,
  operationRequested: false,
  actionTimer: null,
  stateTimer: null,
  gamepadSnapshot: null,
};

const $ = (id) => document.getElementById(id);
const controlKeys = new Set(["KeyA", "KeyD", "KeyW", "KeyS", "KeyQ", "KeyE", "KeyR", "KeyF", "ShiftLeft", "ShiftRight", "Escape"]);

function setPixelStreamingInputEnabled(keyboardEnabled, mouseEnabled) {
  const stream = state.pixelStreaming;
  if (!stream?.config) return;
  stream.config.setFlagEnabled(Flags.KeyboardInput, keyboardEnabled);
  stream.config.setFlagEnabled(Flags.MouseInput, mouseEnabled);
}

const freeCamera = new FreeCameraController({
  element: $("pixelStream"),
  send: command => state.streamLive && state.pixelStreaming?.emitCommand(command) === true,
  onExit: () => setFreeCameraMode(false, "鼠标已解锁，已返回主视角"),
  onLockState: locked => {
    updateOperationUI();
    if (locked) setMessage(freeCamera.inputMode === "raw"
      ? "鼠标已锁定（原始输入）：固定增益观察，WASD/QE 移动；C/Home/Esc 返回"
      : "鼠标已锁定（兼容输入）：当前浏览器未确认原始输入支持，指针速度可能受系统设置影响");
  },
  onLockError: error => {
    updateOperationUI();
    setMessage(`鼠标未锁定：请点击视频画面重试；锁定成功前鼠标不会控制视角${error?.name ? `（${error.name}）` : ""}`);
  },
});

// Read-only mouse diagnostics; does not expose camera mutation or robot input.
window.__freeCameraDiagnostics = () => freeCamera.getDiagnostics();

function setFreeCameraMode(enabled, message = "") {
  if (state.freeCameraMode === enabled) return;
  if (enabled && state.selectedStreamerId !== (state.streamConfig?.pixel_streaming_streamer_id || "BskRenderer")) {
    setMessage("请先选择“主视口”，再按 C 进入全局自由视角；模型相机是固定安装视角");
    return;
  }
  // No SDK mouse/keyboard camera events: one explicit command owns the mode,
  // translation and relative look, including blur and stream replacement.
  setPixelStreamingInputEnabled(true, false);
  if (!freeCamera.setActive(enabled) && enabled) {
    setMessage("视频尚未就绪，暂时不能进入自由视角");
    return;
  }
  state.freeCameraMode = enabled;
  state.pressed.clear();
  if (enabled && state.operationActive) sendNeutralAction("keyboard");
  updateOperationUI();
  if (message) setMessage(message);
}

function setMessage(text) { $("message").textContent = text; }
function setOnline(id, online) { $(id).classList.toggle("online", online); }

const scenePhaseLabels = {
  idle: "平台空闲",
  launching: "正在创建实例",
  starting_renderer: "正在启动 UE",
  starting_simulation: "正在启动 BSK/MJScene",
  running: "场景运行中",
  completed: "任务已结束",
  failed: "场景启动失败",
  stopped: "场景已停止",
  unknown: "状态未知",
};

function showLogin(message = "") {
  state.currentUser = null;
  state.operationRequested = false;
  document.body.classList.add("auth-required");
  document.body.classList.remove("auth-pending");
  $("authGate").hidden = false;
  $("loginError").textContent = message;
  $("loginPassword").value = "";
  setTimeout(() => $("loginUsername").focus(), 0);
}

async function apiRequest(url, options = {}) {
  const response = await fetch(url, options);
  if (response.status === 401 && url !== "/api/auth/login") {
    shutdownAuthenticatedApp();
    showLogin("登录已过期，请重新登录");
  }
  return response;
}

function applyAuthenticatedUser(user) {
  state.currentUser = user;
  document.body.classList.remove("auth-pending", "auth-required");
  $("authGate").hidden = true;
  $("currentUsername").textContent = user.username;
  $("currentRole").textContent = user.role === "admin" ? "管理员" : "操作员";
  $("adminUsers").hidden = user.role !== "admin";
}

async function loadOperators() {
  if (state.currentUser?.role !== "admin") return;
  const response = await apiRequest("/api/users", { cache: "no-store" });
  if (!response.ok) return;
  const data = await response.json();
  const operators = (data.users || []).filter((user) => user.role === "operator");
  $("operatorList").replaceChildren(...operators.map((user) => {
    const row = document.createElement("div");
    row.className = "operator-row";
    row.innerHTML = `<span><strong>${user.username}</strong><small>操作员</small></span>`;
    const actions = document.createElement("span");
    actions.className = "operator-actions";
    const reset = document.createElement("button");
    reset.type = "button"; reset.textContent = "重置密码";
    reset.addEventListener("click", () => resetOperatorPassword(user));
    const remove = document.createElement("button");
    remove.type = "button"; remove.textContent = "删除"; remove.className = "danger-button";
    remove.addEventListener("click", () => deleteOperator(user));
    actions.append(reset, remove); row.append(actions); return row;
  }));
  if (!operators.length) $("operatorList").textContent = "尚未创建操作员";
}

async function resetOperatorPassword(user) {
  const password = prompt(`为操作员 ${user.username} 设置新密码（至少8位）`);
  if (!password) return;
  const response = await apiRequest(`/api/users/operators/${user.user_id}/reset-password`, {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ password }),
  });
  const data = await readApiResponse(response);
  setMessage(response.ok ? `已重置 ${user.username} 的密码` : (data.detail || "重置失败"));
}

async function deleteOperator(user) {
  if (!confirm(`确定删除操作员 ${user.username}？`)) return;
  const response = await apiRequest(`/api/users/operators/${user.user_id}`, { method: "DELETE" });
  const data = await readApiResponse(response);
  if (!response.ok) return setMessage(data.detail || "删除失败");
  setMessage(`已删除操作员 ${user.username}`);
  await loadOperators();
}

function shutdownAuthenticatedApp() {
  exitOperationMode("");
  if (state.actionTimer) clearInterval(state.actionTimer);
  if (state.stateTimer) clearInterval(state.stateTimer);
  state.actionTimer = null; state.stateTimer = null;
  if (state.ws) { state.ws.onclose = null; state.ws.close(); state.ws = null; }
  disposePixelStream();
  state.connected = false; state.controlGranted = false;
}

async function startAuthenticatedApp() {
  connect();
  await loadSceneCatalog();
  connectPixelStreaming();
  await refreshState();
  await loadOperators();
  updateOperationUI();
  if (!state.actionTimer) state.actionTimer = setInterval(sendAction, 33);
  if (!state.stateTimer) state.stateTimer = setInterval(refreshState, 1500);
}

async function initialize() {
  try {
    const response = await fetch("/api/auth/me", { cache: "no-store" });
    if (!response.ok) return showLogin();
    const data = await response.json();
    applyAuthenticatedUser(data.user);
    await startAuthenticatedApp();
  } catch (_) { showLogin("后端暂时不可用"); }
}

async function loadSceneCatalog() {
  try {
    const response = await apiRequest("/api/scenes/catalog", { cache: "no-store" });
    const catalog = await response.json();
    if (!response.ok) throw new Error(catalog.detail || `catalog ${response.status}`);
    const template = $("sceneTemplate");
    template.replaceChildren(...catalog.templates.map((item) => new Option(item.label, item.id)));
    const profile = $("randomizationProfile");
    profile.replaceChildren(...catalog.randomization_profiles.map((item) => new Option(item.label, item.id)));
    const defaults = catalog.defaults || {};
    template.value = defaults.template_id || template.value;
    profile.value = defaults.randomization_profile || profile.value;
    $("sceneDatasetCapture").checked = Boolean(defaults.dataset_capture);
    $("sceneRandomizeOrbitPhase").checked = defaults.randomize_orbit_phase === true;
    $("sceneSunlightIntensity").value = String(defaults.sunlight_intensity_scale ?? 1);
    state.sceneDefaults = {
      simulation_rate: Number(defaults.simulation_rate || 1),
      capture_rate_hz: Number(defaults.capture_rate_hz || 10),
      ik_rate_hz: Number(defaults.ik_rate_hz || 100),
    };
  } catch (error) {
    console.warn("Scene catalog is unavailable", error);
    setMessage("无法读取场景模板目录");
  }
}

function applySceneRuntime(runtime = {}) {
  const previousPhase = state.scenePhase;
  const phase = runtime.phase || "idle";
  const active = Boolean(runtime.active);
  const enabled = runtime.enabled !== false;
  const instance = runtime.instance || {};
  const ownerId = instance.created_by?.user_id;
  state.canManageScene = Boolean(state.currentUser) && (
    state.currentUser.role === "admin" || !ownerId || ownerId === state.currentUser.user_id
  );
  state.scenePhase = phase;
  state.sceneReady = phase === "running";
  if (!state.sceneReady && state.operationActive) {
    exitOperationMode("场景已结束，已退出操作模式");
  }
  $("scenePhase").textContent = enabled ? (scenePhaseLabels[phase] || phase) : "运行时未配置";
  $("scenePhase").classList.toggle("scene-running", state.sceneReady);
  $("scenePhase").classList.toggle("scene-failed", phase === "failed");
  $("startScene").disabled = active || !enabled;
  $("stopScene").disabled = !active || !state.canManageScene;
  ["sceneTemplate", "randomizationProfile", "sceneSeed", "sceneRandomizeOrbitPhase", "sceneDatasetCapture", "sceneSunlightIntensity"].forEach((id) => {
    $(id).disabled = active;
  });
  $("linearSpeed").disabled = !state.sceneReady;
  $("estop").disabled = !state.sceneReady;
  updateOperationUI();
  $("sceneInstanceId").textContent = instance.instance_id || "—";
  $("sceneInstanceSeed").textContent = instance.seed ?? "—";
  $("sceneInstanceTarget").textContent = !instance.instance_id ? "—" : instance.template_id === "sarm-ground-validation-self-collision-grasp" ? "地面验证星（粗碰撞体·内部碰撞）" : instance.template_id === "sarm-ground-validation-mesh-grasp" ? "地面验证星（高精度三角网格·实验）" : instance.template_id === "sarm-ground-validation-grasp" ? "地面验证星（粗碰撞盒·无内部碰撞）" : "原小方块目标";
  const instanceSunlight = instance.environment?.lighting?.sunlight_intensity_scale ?? 1;
  $("sceneInstanceSunlight").textContent = instance.instance_id ? `${instanceSunlight} 倍` : "—";
  const instanceOrbitPhase = instance.environment?.orbit?.true_anomaly_deg ?? 180;
  const randomizeOrbitPhase = instance.randomize_orbit_phase === true;
  $("sceneInstanceOrbitPhase").textContent = instance.instance_id ? `${Number(instanceOrbitPhase).toFixed(2)}°` : "—";
  if (active && instance.instance_id) {
    if (instance.template_id) $("sceneTemplate").value = instance.template_id;
    $("sceneSunlightIntensity").value = String(instanceSunlight);
    $("sceneRandomizeOrbitPhase").checked = randomizeOrbitPhase;
  }
  $("sceneParameters").textContent = instance.randomization
    ? JSON.stringify({ capture_target: instance.capture_target || {}, randomize_orbit_phase: randomizeOrbitPhase, environment: instance.environment || {}, randomization: instance.randomization }, null, 2)
    : "尚未生成实例";
  if (runtime.error) setMessage(`场景失败：${runtime.error}`);
  if (phase === "running" && previousPhase !== "running") {
    setMessage(`场景 ${instance.instance_id || ""} 已运行，可开始遥操作或采集`);
    // The waiting player may have subscribed milliseconds ago. Moving from
    // launcher startup to running is not a reason to destroy its RTC handshake.
    if (!state.pixelStreaming && !state.pixelConnectPromise) connectPixelStreaming();
  }
  updateEpisodeUI();
}

async function readApiResponse(response) {
  const text = await response.text();
  if (!text) return {};
  try { return JSON.parse(text); } catch (_) { return { detail: text }; }
}

async function startScene() {
  const sunlightText = $("sceneSunlightIntensity").value.trim();
  const sunlightScale = Number(sunlightText);
  if (sunlightText === "" || !Number.isFinite(sunlightScale) || sunlightScale < 0 || sunlightScale > 20000) {
    setMessage("太阳光照强度请输入 0～20,000 之间的数值（1 为默认倍率）");
    $("sceneSunlightIntensity").focus();
    return;
  }
  const seedText = $("sceneSeed").value.trim();
  const request = {
    template_id: $("sceneTemplate").value,
    randomization_profile: $("randomizationProfile").value,
    randomize_orbit_phase: $("sceneRandomizeOrbitPhase").checked,
    seed: seedText === "" ? null : Number(seedText),
    simulation_rate: state.sceneDefaults.simulation_rate,
    capture_rate_hz: state.sceneDefaults.capture_rate_hz,
    ik_rate_hz: state.sceneDefaults.ik_rate_hz,
    dataset_capture: $("sceneDatasetCapture").checked,
    sunlight_intensity_scale: sunlightScale,
  };
  $("startScene").disabled = true;
  setMessage("正在生成可复现场景实例…");
  try {
    const response = await apiRequest("/api/scenes/start", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(request),
    });
    const data = await readApiResponse(response);
    if (!response.ok) throw new Error(data.detail || `无法启动场景（HTTP ${response.status}）`);
    applySceneRuntime({ ...data, active: true, instance: data });
    setMessage(`已生成场景 ${data.instance_id}，Seed ${data.seed}，正在启动 UE`);
  } catch (error) {
    setMessage(error.message || "无法启动场景");
    await refreshState();
  }
}

async function stopScene() {
  $("stopScene").disabled = true;
  setMessage("正在停止场景…");
  try {
    const response = await apiRequest("/api/scenes/stop", { method: "POST" });
    const data = await readApiResponse(response);
    if (!response.ok) throw new Error(data.detail || `无法停止场景（HTTP ${response.status}）`);
    applySceneRuntime(data);
    setMessage("场景已停止，控制平台仍保持运行");
  } catch (error) {
    setMessage(error.message || "无法停止场景");
    await refreshState();
  }
}

function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const accessKey = new URLSearchParams(location.search).get("access_key");
  const operatorUrl = new URL(`${scheme}://${location.host}/ws/operator`);
  if (accessKey) operatorUrl.searchParams.set("access_key", accessKey);
  state.ws = new WebSocket(operatorUrl);
  state.ws.onopen = () => {
    state.connected = true;
    setOnline("backendDot", true);
    $("backendState").textContent = "后端在线";
    updateOperationUI();
    setMessage("操作链路已连接；场景运行后点击画面进入操作模式");
  };
  state.ws.onclose = () => {
    state.connected = false;
    state.controlGranted = false;
    setOnline("backendDot", false);
    $("backendState").textContent = "后端断开";
    $("controlAuthority").textContent = "只读";
    exitOperationMode("连接中断，已退出操作模式并自动归零");
    if (state.currentUser) setTimeout(connect, 1000);
  };
  state.ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "session") {
      state.controlGranted = message.control_granted;
      state.activeEpisode = message.active_episode;
      if (message.user) applyAuthenticatedUser(message.user);
      applyControlLimits(message.control_limits);
      $("controlAuthority").textContent = state.controlGranted ? "控制已授权" : "只读观察";
      if (!state.controlGranted && state.operationActive) {
        exitOperationMode("当前页面没有控制权，已退出操作模式");
      }
      updateOperationUI();
      updateEpisodeUI();
    } else if (message.type === "control_granted") {
      state.controlGranted = true;
      $("controlAuthority").textContent = "控制已授权";
      updateOperationUI();
      if (state.operationRequested) {
        state.operationRequested = false;
        completeEnterOperationMode();
      } else {
        setMessage("当前页面已成为活动操作页面");
      }
    } else if (message.type === "control_revoked") {
      state.controlGranted = false;
      state.operationRequested = false;
      $("controlAuthority").textContent = "其他页面正在操作";
      exitOperationMode("同一用户的其他页面已进入操作，本页面自动退出");
    } else if (message.type === "observation") {
      const obs = message.payload;
      setOnline("simDot", true);
      $("simState").textContent = "仿真在线";
      $("simTime").textContent = `${(Number(obs.sim_time_ns) / 1e9).toFixed(3)} s`;
      $("actionSequence").textContent = obs.applied_action_sequence;
      updateMotionOutputs(obs.end_effector_twist_body || [], obs.gripper_velocity_m_s?.[0] ?? obs.joint_velocity_rad_s?.[6] ?? 0);
      const position = obs.end_effector_position_body_m || [];
      $("toolPosition").textContent = position.length === 3 ? position.map((value) => Number(value).toFixed(3)).join(", ") : "—";
      $("jacobianRank").textContent = `${obs.jacobian_rank ?? "—"} / 6`;
      const attitude = obs.attitude_control;
      const wheels = obs.reaction_wheels;
      $("attitudeMode").textContent = !attitude ? "—" : !attitude.enabled ? "控制关闭" :
        !attitude.reference_initialized ? "初始化" : attitude.saturated ? "惯性保持 · 饱和" : "惯性保持";
      $("attitudeError").textContent = attitude ? `${(attitude.attitude_error_angle_rad * 180 / Math.PI).toFixed(3)} °` : "—";
      $("bodyRate").textContent = attitude ? attitude.angular_velocity_body_rad_s.map(v => Number(v).toFixed(4)).join(", ") + " rad/s" : "—";
      $("wheelSpeeds").textContent = wheels ? wheels.speed_rad_s.map(v => (v * 60 / (2 * Math.PI)).toFixed(1)).join(", ") + " rpm" : "—";
      $("wheelTorques").textContent = wheels ? wheels.applied_motor_torque_nm.map(v => Number(v).toFixed(3)).join(", ") + " N·m" : "—";
    } else if (message.type === "action_ack") {
      state.lastAckAt = performance.now();
      $("actionSequence").textContent = message.server_sequence;
    } else if (message.type === "action_rejected") {
      const reasons = { inactive_page: "当前页面不是活动操作页面", not_scene_owner: "当前用户不能操作这个场景" };
      state.operationRequested = false;
      setMessage(reasons[message.reason] || `动作被拒绝：${message.reason}`);
    }
  };
}

async function connectPixelStreaming() {
  // Reconnect callbacks, the 10-second startup watchdog, and manual reconnect
  // can otherwise overlap.  That creates two Pixel Streaming players for the
  // main viewport and makes the displayed stream flash while old participants
  // are being removed.
  if (state.pixelConnectPromise) return state.pixelConnectPromise;
  let pending;
  pending = (async () => {
    try {
    const accessKey = new URLSearchParams(location.search).get("access_key");
    const response = await apiRequest("/api/client-config", {
      cache: "no-store",
      headers: accessKey ? { "X-Space-Arm-Access-Key": accessKey } : {},
    });
    if (!response.ok) throw new Error(`config ${response.status}`);
    const config = await response.json();
    state.streamConfig = config;
    const selector = $("streamSelector");
    const previousStreamerId = state.selectedStreamerId;
    selector.replaceChildren();
    for (const item of config.pixel_streaming_streamers || [
      { id: config.pixel_streaming_streamer_id, label: "主视口" },
    ]) {
      const option = document.createElement("option");
      option.value = item.id;
      const labels = { WristCamera: "机械臂腕部相机", SpacecraftOverview: "卫星总览相机" };
      option.textContent = labels[item.label] || item.label || item.id;
      selector.append(option);
    }
    if ([...selector.options].some((option) => option.value === previousStreamerId)) {
      selector.value = previousStreamerId;
    }
    state.selectedStreamerId = selector.value || config.pixel_streaming_streamer_id;
    createPixelStream();
    } catch (error) {
      console.warn("Pixel Streaming configuration is unavailable", error);
      $("frameState").textContent = "WEBRTC OFFLINE";
      schedulePixelStreamingReconnect();
    }
  })().finally(() => {
    if (state.pixelConnectPromise === pending) state.pixelConnectPromise = null;
  });
  state.pixelConnectPromise = pending;
  return pending;
}

function signallingUrl() {
  if (state.streamConfig.pixel_streaming_signalling_url) {
    const url = new URL(state.streamConfig.pixel_streaming_signalling_url, location.href);
    if (state.streamConfig.pixel_streaming_access_token) {
      url.searchParams.set("token", state.streamConfig.pixel_streaming_access_token);
    }
    return url.toString();
  }
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  return `${scheme}//${location.hostname}:${state.streamConfig.pixel_streaming_player_port}`;
}

function resetWebRtcStats() {
  state.lastPresentedFrames = null;
  state.lastPresentedTimestamp = null;
  $("webrtcRtt").textContent = "—";
  $("webrtcBitrate").textContent = "—";
  $("webrtcLoss").textContent = "—";
  $("webrtcFps").textContent = "—";
  $("webrtcResolution").textContent = "—";
}

function updateWebRtcStats(aggregatedStats) {
  const pair = aggregatedStats?.getActiveCandidatePair?.();
  const video = aggregatedStats?.inboundVideoStats;
  if (!video) return;
  const packetsReceived = Number(video.packetsReceived || 0);
  const packetsLost = Number(video.packetsLost || 0);
  const totalPackets = packetsReceived + packetsLost;
  // WebRTC's inbound framesPerSecond is the receive/decode rate. It can be
  // much higher than what the browser actually presents because decoded frames
  // may be dropped by the compositor. Measure non-dropped video frames instead.
  const player = $("pixelStream").querySelector("video");
  const quality = player?.getVideoPlaybackQuality?.();
  const totalVideoFrames = Number(quality?.totalVideoFrames ?? player?.webkitDecodedFrameCount);
  const droppedVideoFrames = Number(quality?.droppedVideoFrames ?? player?.webkitDroppedFrameCount ?? 0);
  const presentedFrames = totalVideoFrames - droppedVideoFrames;
  const presentedTimestamp = performance.now();
  let fps = null;
  if (Number.isFinite(presentedFrames) && presentedFrames >= 0) {
    if (state.lastPresentedTimestamp != null
        && presentedTimestamp > state.lastPresentedTimestamp
        && presentedFrames >= state.lastPresentedFrames) {
      fps = (presentedFrames - state.lastPresentedFrames) * 1000
        / (presentedTimestamp - state.lastPresentedTimestamp);
    }
    state.lastPresentedTimestamp = presentedTimestamp;
    state.lastPresentedFrames = presentedFrames;
  }
  $("webrtcRtt").textContent = pair?.currentRoundTripTime != null
    ? `${(pair.currentRoundTripTime * 1000).toFixed(0)} ms` : "—";
  $("webrtcBitrate").textContent = video.bitrate != null
    ? `${(video.bitrate / 1000).toFixed(0)} kbps` : "—";
  $("webrtcLoss").textContent = totalPackets > 0
    ? `${(packetsLost * 100 / totalPackets).toFixed(1)}%` : "—";
  $("webrtcFps").textContent = fps == null ? "—" : fps.toFixed(0);
  $("webrtcResolution").textContent = video.frameWidth && video.frameHeight
    ? `${video.frameWidth}×${video.frameHeight}` : "—";
}

function disposePixelStream() {
  if (state.freeCameraMode) setFreeCameraMode(false);
  if (state.streamReconnectTimer != null) {
    clearTimeout(state.streamReconnectTimer);
    state.streamReconnectTimer = null;
  }
  const previousStream = state.pixelStreaming;
  try {
    setPixelStreamingInputEnabled(false, false);
  } catch (_) { /* best effort */ }
  // Invalidate ownership BEFORE disconnect(): the SDK can synchronously emit
  // webRtcDisconnected here. A retired player must never schedule a reconnect
  // or overwrite the new player's LIVE state/stats.
  state.pixelStreaming = null;
  state.pixelConfig = null;
  try {
    previousStream?.disconnect();
  } catch (_) { /* best effort */ }
  state.freeCameraMode = false;
  state.streamLive = false;
  $("pixelStream").replaceChildren();
  resetWebRtcStats();
}

function schedulePixelStreamingReconnect(delayMs = 1500) {
  if (state.streamReconnectTimer != null) clearTimeout(state.streamReconnectTimer);
  state.streamReconnectTimer = setTimeout(() => {
    state.streamReconnectTimer = null;
    connectPixelStreaming();
  }, delayMs);
}

function armPixelStreamingWatchdog(stream, delayMs = 60000) {
  if (state.streamReconnectTimer != null) clearTimeout(state.streamReconnectTimer);
  state.streamReconnectTimer = setTimeout(() => {
    state.streamReconnectTimer = null;
    if (state.pixelStreaming === stream && !state.streamLive) connectPixelStreaming();
  }, delayMs);
}

function createPixelStream() {
  disposePixelStream();
  $("frameState").textContent = "CONNECTING";
  $("frameState").style.color = "var(--danger)";
  $("previewEmpty").style.display = "grid";
  $("playStream").hidden = true;
  const config = new Config({
    initialSettings: {
      [TextParameters.SignallingServerUrl]: signallingUrl(),
      [OptionParameters.StreamerId]: state.selectedStreamerId,
      [Flags.AutoConnect]: true,
      [Flags.AutoPlayVideo]: true,
      [Flags.StartVideoMuted]: true,
      [Flags.WaitForStreamer]: true,
      [Flags.HoveringMouseMode]: false,
      // Retain SDK keyboard input for non-camera UE shortcuts only. Camera
      // keys use physical codes and explicit BskCameraInput commands.
      [Flags.KeyboardInput]: true,
      [Flags.MouseInput]: false,
      [Flags.TouchInput]: false,
      [Flags.GamepadInput]: false,
    },
  });
  const stream = new PixelStreaming(config, { videoElementParent: $("pixelStream") });
  state.pixelConfig = config;
  state.pixelStreaming = stream;
  window.__pixelStream = stream;
  stream.addEventListener("webRtcConnecting", () => {
    if (state.pixelStreaming !== stream) return;
    $("frameState").textContent = "CONNECTING";
    // Give negotiation its own deadline, independent of how long UE took to load.
    armPixelStreamingWatchdog(stream);
  });
  stream.addEventListener("streamerListMessage", () => {
    if (state.pixelStreaming !== stream || state.streamLive) return;
    // Healthy signalling / streamer discovery is progress, not a reason to
    // periodically replace a player while the renderer is coming online.
    armPixelStreamingWatchdog(stream);
  });
  stream.addEventListener("webRtcConnected", () => {
    if (state.pixelStreaming !== stream) return;
    // WebRTC connection is already the authoritative readiness signal for the
    // main viewport.  Waiting only for videoInitialized made the watchdog
    // tear down a healthy player every 10 seconds in Pixel Streaming 2, which
    // produced repeated PlayerN connections and visible main-view flashing.
    state.streamLive = true;
    if (state.streamReconnectTimer != null) {
      clearTimeout(state.streamReconnectTimer);
      state.streamReconnectTimer = null;
    }
    $("frameState").textContent = "MEDIA SETUP";
  });
  stream.addEventListener("videoInitialized", () => {
    if (state.pixelStreaming !== stream) return;
    state.streamLive = true;
    $("previewEmpty").style.display = "none";
    $("pixelStream").style.display = "block";
    $("frameState").textContent = "LIVE WEBRTC";
    $("frameState").style.color = "var(--accent)";
  });
  stream.addEventListener("playStreamRejected", () => {
    if (state.pixelStreaming !== stream) return;
    $("playStream").hidden = false;
    $("frameState").textContent = "CLICK TO PLAY";
  });
  stream.addEventListener("webRtcDisconnected", () => {
    if (state.pixelStreaming !== stream) return;
    if (state.freeCameraMode) setFreeCameraMode(false);
    state.streamLive = false;
    $("frameState").textContent = "DISCONNECTED";
    $("frameState").style.color = "var(--danger)";
    resetWebRtcStats();
    if (state.pixelStreaming === stream) schedulePixelStreamingReconnect();
  });
  stream.addEventListener("webRtcFailed", () => {
    if (state.pixelStreaming !== stream) return;
    if (state.freeCameraMode) setFreeCameraMode(false);
    state.streamLive = false;
    $("frameState").textContent = "WEBRTC FAILED";
    $("frameState").style.color = "var(--danger)";
    if (state.pixelStreaming === stream) schedulePixelStreamingReconnect();
  });
  stream.addEventListener("subscribeFailed", (event) => {
    if (state.pixelStreaming !== stream) return;
    if (state.freeCameraMode) setFreeCameraMode(false);
    state.streamLive = false;
    $("frameState").textContent = event?.message || "STREAM UNAVAILABLE";
    if (state.pixelStreaming === stream) schedulePixelStreamingReconnect(1000);
  });
  stream.addEventListener("statsReceived", (event) => {
    if (state.pixelStreaming !== stream) return;
    updateWebRtcStats(event?.data?.aggregatedStats);
  });
  // Bound a genuinely stalled setup, but never recycle players on an absolute
  // 10-second cycle: UE startup and public ICE negotiation can take much longer.
  armPixelStreamingWatchdog(stream);
}

function applyControlLimits(limits = {}) {
  const slider = $("linearSpeed");
  const minimum = Number(limits.linear_speed_min_m_s ?? slider.min);
  const maximum = Number(limits.linear_speed_max_m_s ?? slider.max);
  const fallback = Number(limits.linear_speed_default_m_s ?? slider.value);
  slider.min = String(minimum);
  slider.max = String(maximum);
  state.linearSpeed = Math.max(minimum, Math.min(maximum, Number(slider.value) || fallback));
  slider.value = String(state.linearSpeed);
  $("linearSpeedValue").textContent = `${state.linearSpeed.toFixed(2)} m/s`;
}

function keyboardAction() {
  const linear = [0, 0, 0];
  const angular = [0, 0, 0];
  const rotationMode = state.pressed.has("ShiftLeft") || state.pressed.has("ShiftRight");
  const value = (positive, negative) => Number(state.pressed.has(positive)) - Number(state.pressed.has(negative));
  if (rotationMode) {
    angular[0] = value("KeyQ", "KeyE");
    angular[1] = value("KeyW", "KeyS");
    angular[2] = value("KeyA", "KeyD");
  } else {
    linear[0] = value("KeyW", "KeyS");
    linear[1] = value("KeyA", "KeyD");
    linear[2] = value("KeyQ", "KeyE");
  }
  $("motionMode").textContent = rotationMode ? "旋转 RPY" : "平移 XYZ";
  return { linear, angular, grip: value("KeyR", "KeyF"), source: "keyboard" };
}

function gamepadAction() {
  const snapshot = gamepadInput.read();
  state.gamepadSnapshot = snapshot;
  const labels = {unsupported: "API 不可用", blocked: "读取被阻止", disconnected: "未发现设备",
    unmapped: "映射待适配", centering: "等待回中", connected: "已连接"};
  $("gamepadStatus").textContent = labels[snapshot.status];
  $("gamepadName").textContent = snapshot.device
    ? `${snapshot.device.id}（#${snapshot.device.index} / ${snapshot.device.mapping}）` : "—";
  $("gamepadAxes").textContent = snapshot.axes.length ? snapshot.axes.map(v => v.toFixed(2)).join(" / ") : "—";
  $("gamepadButtons").textContent = snapshot.buttons.map((v, i) => v > 0.01 ? `${i}:${v.toFixed(2)}` : "").filter(Boolean).join(" ") || "无";
  $("gamepadHint").textContent = snapshot.hint;
  $("gamepadGate").textContent = gamepadControlBlockReason(state);
  return snapshot.action;
}

function transmitAction(action, deadman) {
  if (!state.connected || !state.controlGranted || !state.ws || state.ws.readyState !== WebSocket.OPEN) {
    return false;
  }
  state.sequence += 1;
  state.ws.send(JSON.stringify({
    type: "operator_action",
    client_sequence: state.sequence,
    client_time_ns: String(BigInt(Date.now()) * 1000000n),
    deadman,
    end_effector_linear_speed_m_s: state.linearSpeed,
    end_effector_linear_velocity: action.linear,
    end_effector_angular_velocity: action.angular,
    gripper_velocity: action.grip,
    input_source: action.source,
  }));
  return true;
}

function sendNeutralAction(source = "keyboard") {
  return transmitAction(
    { linear: [0, 0, 0], angular: [0, 0, 0], grip: 0, source },
    false,
  );
}

function updateOperationUI() {
  const viewport = $("viewport");
  const hintTitle = $("operationHintTitle");
  const hintDetail = $("operationHintDetail");
  const available = state.sceneReady && state.connected && state.canManageScene && !state.estopped;
  viewport.classList.toggle("operation-active", state.operationActive);
  viewport.classList.toggle("operation-available", available && !state.operationActive);
  viewport.setAttribute("aria-pressed", String(state.operationActive));

  // Camera look depends on pointer-lock, not robot-control/scene permissions.
  // Keep its actual lock state visible even while simulation is disconnected.
  if (state.freeCameraMode) {
    hintTitle.textContent = freeCamera.locked ? "自由视角 · 鼠标已锁定" : "自由视角 · 鼠标未锁定";
    hintDetail.textContent = freeCamera.locked
      ? `${freeCamera.inputMode === "raw" ? "原始输入 · 固定增益" : "兼容输入 · 受系统指针设置影响"}；WASD / QE 移动；C / Home / Esc 返回`
      : "点击视频画面锁定鼠标后再转动；C / Home / Esc 返回";
  } else if (!state.sceneReady) {
    hintTitle.textContent = "等待场景运行";
    hintDetail.textContent = "场景运行后点击画面进入操作";
  } else if (!state.connected) {
    hintTitle.textContent = "操作链路未连接";
    hintDetail.textContent = "等待后端 WebSocket 恢复";
  } else if (!state.canManageScene) {
    hintTitle.textContent = "当前场景由其他操作员创建";
    hintDetail.textContent = "只有场景创建者或管理员可以操作";
  } else if (state.estopped) {
    hintTitle.textContent = "急停已锁存";
    hintDetail.textContent = "先在右侧点击恢复控制";
  } else if (!state.controlGranted) {
    hintTitle.textContent = "点击切换到当前操作页面";
    hintDetail.textContent = "同一用户的旧页面会自动退出操作";
  } else if (state.operationActive) {
    hintTitle.textContent = "操作模式已开启";
    hintDetail.textContent = "WASD / QE 控制，F/R 夹爪；按 C 进入自由视角，Esc 退出操作";
  } else {
    hintTitle.textContent = "点击画面进入操作";
    hintDetail.textContent = "C 进入自由视角；Home 返回主视角；点击画面后可操作机械臂";
  }

  const directStatus = $("directControl");
  if (state.estopped) {
    directStatus.textContent = "急停已锁存";
  } else if (state.freeCameraMode) {
    directStatus.textContent = freeCamera.locked
      ? "自由视角：鼠标已锁定，WASD/QE 移动（C/Home/Esc 返回）"
      : "自由视角：鼠标未锁定，请点击视频画面";
  } else if (state.operationActive) {
    directStatus.textContent = "操作模式：等待输入（C 自由视角，Esc 退出）";
  } else if (available) {
    directStatus.textContent = "点击实时画面进入操作模式";
  } else {
    directStatus.textContent = "等待场景和控制链路就绪";
  }
  directStatus.classList.toggle("operation-mode", state.operationActive);
  if (!state.operationActive) directStatus.classList.remove("active");
}

function enterOperationMode() {
  if (!state.sceneReady) return setMessage("场景尚未运行，暂时不能进入操作模式");
  if (!state.connected) return setMessage("操作链路尚未连接");
  if (!state.canManageScene) return setMessage("只能操作自己创建的场景");
  if (state.estopped) return setMessage("急停已锁存，请先在右侧点击恢复控制");
  if (!state.controlGranted) {
    state.operationRequested = true;
    state.ws.send(JSON.stringify({ type: "activate_control" }));
    return setMessage("正在切换到当前操作页面…");
  }
  completeEnterOperationMode();
}

function completeEnterOperationMode() {
  state.pressed.clear();
  state.operationActive = true;
  $("viewport").focus({ preventScroll: true });
  updateOperationUI();
  setMessage("已进入操作模式：键盘或手柄控制；按 C 进入自由视角");
}

function exitOperationMode(message = "已退出操作模式，点击实时画面可重新进入") {
  const wasActive = state.operationActive;
  const wasFreeCamera = state.freeCameraMode;
  state.pressed.clear();
  if (wasActive) sendNeutralAction("keyboard");
  state.operationActive = false;
  if (wasFreeCamera) setFreeCameraMode(false);
  highlightKeys();
  updateOperationUI();
  if (message) setMessage(message);
}

function sendAction() {
  // Poll diagnostics even outside operation mode; this never grants control.
  const gamepad = gamepadAction();
  if (!state.operationActive || state.freeCameraMode || !state.sceneReady || !state.canManageScene || state.estopped) return;
  if (!state.connected || !state.controlGranted || !state.ws || state.ws.readyState !== WebSocket.OPEN) return;
  const keyboard = keyboardAction();
  const gamepadActive = gamepad && [...gamepad.linear, ...gamepad.angular, gamepad.grip].some((value) => Math.abs(value) > 0.01);
  const action = gamepadActive ? gamepad : keyboard;
  const motionActive = [...action.linear, ...action.angular, action.grip].some((value) => Math.abs(value) > 0.01);
  transmitAction(action, motionActive);
  $("directControl").classList.toggle("active", motionActive);
  $("directControl").textContent = motionActive
    ? "操作模式：正在运动（Esc 退出）"
    : "操作模式：等待输入（Esc 退出）";
  $("inputSource").textContent = action.source === "gamepad" ? "手柄" : "键盘";
  highlightKeys();
  const elapsed = state.lastAckAt ? performance.now() - state.lastAckAt : 0;
  $("latency").textContent = state.lastAckAt ? `ACK ${elapsed.toFixed(0)} ms` : "等待 ACK";
}

function updateMotionOutputs(twist, gripperVelocity) {
  ["vx", "vy", "vz", "wx", "wy", "wz"].forEach((id, index) => {
    $(id).textContent = Number(twist[index] || 0).toFixed(3);
  });
  $("grip").textContent = Number(gripperVelocity || 0).toFixed(3);
}

function highlightKeys() {
  document.querySelectorAll("kbd").forEach((element) => {
    const label = element.textContent.replace("⇧", "");
    const labels = { A: "KeyA", D: "KeyD", W: "KeyW", S: "KeyS", Q: "KeyQ", E: "KeyE", R: "KeyR", F: "KeyF" };
    const requiresShift = element.textContent.startsWith("⇧");
    const shift = state.pressed.has("ShiftLeft") || state.pressed.has("ShiftRight");
    const modeMatches = ["R", "F"].includes(label) || (requiresShift ? shift : !shift);
    element.classList.toggle("active", state.pressed.has(labels[label]) && modeMatches);
  });
}

async function refreshState() {
  try {
    const response = await apiRequest("/api/state", { cache: "no-store" });
    const data = await response.json();
    setOnline("simDot", data.simulation.connected);
    $("simState").textContent = data.simulation.connected ? "仿真在线" : "仿真未连接";
    state.activeEpisode = data.active_episode;
    $("episodePath").textContent = data.episode_directory || "—";
    applySceneRuntime(data.scene_runtime || {});
  } catch (_) {
    setOnline("backendDot", false);
  }
}

function updateEpisodeUI() {
  const active = Boolean(state.activeEpisode);
  $("episodeBadge").textContent = active ? "● 正在采集" : "未采集";
  $("episodeBadge").classList.toggle("recording", active);
  $("startEpisode").disabled = active || !state.sceneReady || !state.canManageScene;
  $("successEpisode").disabled = !active;
  $("failureEpisode").disabled = !active;
}

async function startEpisode() {
  const response = await apiRequest("/api/episodes/start", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ instruction: $("instruction").value }),
  });
  const data = await response.json();
  if (!response.ok) return setMessage(data.detail || "无法开始采集");
  state.activeEpisode = data.episode_id;
  setMessage(`开始采集 ${data.episode_id}`);
  await refreshState();
}

async function stopEpisode(outcome) {
  const response = await apiRequest("/api/episodes/stop", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ outcome }),
  });
  const data = await response.json();
  if (!response.ok) return setMessage(data.detail || "无法结束采集");
  state.activeEpisode = null;
  setMessage(`采集已结束：${outcome}，共 ${data.step_count} 步`);
  await refreshState();
}

$("viewport").addEventListener("pointerdown", (event) => {
  if (event.button !== 0) return;
  if (event.target.closest("button, select, .viewport-toolbar, .webrtc-stats")) return;
  if (state.freeCameraMode) {
    freeCamera.requestPointerLock();
    return;
  }
  enterOperationMode();
}, true);

window.addEventListener("keydown", (event) => {
  const editing = ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable;
  if (editing) {
    // Typing into page forms must never reach UE keyboard shortcuts.
    event.stopImmediatePropagation();
    return;
  }
  if (event.code === "KeyC" || event.code === "Home") {
    event.preventDefault();
    event.stopImmediatePropagation(); // Do not toggle a second time in UE.
    if (event.repeat || !state.streamLive) return;
    if (event.code === "KeyC") {
      const enabled = !state.freeCameraMode;
      setFreeCameraMode(enabled, enabled
        ? "已进入全局自由视角：WASD/QE 移动，Shift 加速，鼠标观察；C/Home/Esc 返回"
        : "已返回主视角");
    } else if (state.freeCameraMode) setFreeCameraMode(false, "已返回主视角");
    return;
  }
  if (event.code === "Escape" && (state.operationActive || state.freeCameraMode)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    exitOperationMode();
    return;
  }
  if (freeCamera.handleKey(event, true)) return;
  if (!controlKeys.has(event.code)) return;
  event.preventDefault();
  event.stopImmediatePropagation();
  if (!state.freeCameraMode && state.operationActive && !event.repeat) state.pressed.add(event.code);
}, true);
window.addEventListener("keyup", (event) => {
  if (freeCamera.handleKey(event, false)) return;
  if (controlKeys.has(event.code) || ["KeyC", "Home"].includes(event.code)) {
    event.preventDefault();
    event.stopImmediatePropagation();
    state.pressed.delete(event.code);
  } else if (["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable) {
    event.stopImmediatePropagation();
  }
}, true);
window.addEventListener("keypress", (event) => {
  if (state.freeCameraMode || controlKeys.has(event.code) || event.code === "KeyC"
      || ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable) {
    event.stopImmediatePropagation();
  }
}, true);
window.addEventListener("blur", () => {
  if (state.operationActive || state.freeCameraMode) exitOperationMode("页面失去焦点，已停止输入并返回主视角");
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden && (state.operationActive || state.freeCameraMode)) {
    exitOperationMode("页面已隐藏，已停止输入并返回主视角");
  }
});
document.addEventListener("focusin", event => {
  if (state.freeCameraMode && (["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable)) {
    exitOperationMode("正在编辑表单，已停止相机输入");
  }
});
$("estop").addEventListener("click", () => {
  if (!state.estopped) {
    state.estopped = true;
    exitOperationMode("急停已锁存；点击“恢复控制”后，再点击画面进入操作");
  } else {
    state.estopped = false;
    state.pressed.clear();
    updateOperationUI();
    setMessage("急停已解除，点击实时画面重新进入操作模式");
  }
  $("estop").textContent = state.estopped ? "恢复控制" : "立即停止机械臂输出";
});
$("linearSpeed").addEventListener("input", (event) => {
  state.linearSpeed = Number(event.target.value);
  $("linearSpeedValue").textContent = `${state.linearSpeed.toFixed(2)} m/s`;
});
$("startScene").addEventListener("click", startScene);
$("stopScene").addEventListener("click", stopScene);
$("startEpisode").addEventListener("click", startEpisode);
$("successEpisode").addEventListener("click", () => stopEpisode("success"));
$("failureEpisode").addEventListener("click", () => stopEpisode("failure"));
$("streamSelector").addEventListener("change", (event) => {
  state.selectedStreamerId = event.target.value;
  createPixelStream();
});
$("reconnectStream").addEventListener("click", () => {
  connectPixelStreaming();
});
$("playStream").addEventListener("click", () => {
  state.pixelStreaming?.play();
  $("playStream").hidden = true;
});

$("loginForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  $("loginError").textContent = "正在登录…";
  const response = await fetch("/api/auth/login", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ username: $("loginUsername").value, password: $("loginPassword").value }),
  });
  const data = await readApiResponse(response);
  if (!response.ok) return showLogin(data.detail || "登录失败");
  applyAuthenticatedUser(data.user);
  await startAuthenticatedApp();
});
$("changePasswordButton").addEventListener("click", async () => {
  const currentPassword = prompt("请输入当前密码");
  if (!currentPassword) return;
  const newPassword = prompt("请输入新密码（至少8位）");
  if (!newPassword) return;
  const response = await apiRequest("/api/auth/change-password", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ current_password: currentPassword, new_password: newPassword }),
  });
  const data = await readApiResponse(response);
  if (!response.ok) return setMessage(data.detail || "修改密码失败");
  shutdownAuthenticatedApp();
  showLogin("密码已修改，请使用新密码重新登录");
});
$("logoutButton").addEventListener("click", async () => {
  await apiRequest("/api/auth/logout", { method: "POST" });
  shutdownAuthenticatedApp();
  showLogin("已退出登录");
});
$("createOperatorForm").addEventListener("submit", async (event) => {
  event.preventDefault();
  const response = await apiRequest("/api/users/operators", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ username: $("operatorUsername").value, password: $("operatorPassword").value }),
  });
  const data = await readApiResponse(response);
  if (!response.ok) return setMessage(data.detail || "新增操作员失败");
  event.target.reset(); setMessage(`已新增操作员 ${data.username}`); await loadOperators();
});

initialize();

// Read-only diagnostics for local-browser / remote-desktop troubleshooting.
window.__gamepadDiagnostics = () => ({
  secureContext: window.isSecureContext, pageVisible: !document.hidden,
  pageFocused: document.hasFocus(), snapshot: state.gamepadSnapshot
    ? JSON.parse(JSON.stringify(state.gamepadSnapshot)) : null,
  operationStatus: gamepadControlBlockReason(state),
});
