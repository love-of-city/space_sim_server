import { WorkLight } from "./work_light.js";
import { WristVisionOverlay } from "./wrist_vision.js";
import { PreviewOrbit } from "./preview_orbit.js";
import { armObservation, MainCompanion } from "./dual_arm.js";
import { formatWristWrench } from "./wrist_wrench.js";
import { createArmPreparation, AUTO_PREPARE_PROFILE, ZERO_START_PROFILE } from "./arm_preparation.js";
import { createInitialJointAngles } from "./initial_joint_angles.js";
import { createJointAnglePanel } from "./joint_angles.js";
import { formatMotionSpeedDiagnostics } from "./motion_diagnostics.js";
import {
  Config,
  Flags,
  OptionParameters,
  NumericParameters,
  PixelStreaming,
  TextParameters,
} from "@epicgames-ps/lib-pixelstreamingfrontend-ue5.6";
import "./styles.css";
import "./platform.css";
import {createPlatformShell} from './platform_shell.js';
import {createCollectionPage} from './pages/collection.js';
import {createDatasetPage} from './pages/datasets.js';
import {createUsersPage} from './pages/users.js';
import {askDialog, userLabel} from './ui.js';
import { FreeCameraController } from "./free_camera.js";
import { GamepadInput, gamepadControlBlockReason } from "./gamepad_input.js";
import { referenceProtectionLabel, dynamicsTimingLabel } from "./control_status.js";
import { createMissionPanel } from "./mission_panel.js";
import { resolvePreviewStreams } from "./preview_streams.js";
import { observePreviewAvailability } from "./preview_availability.js";

function clearWristWrench() {
  $("wristForce").textContent = "—";
  $("wristTorque").textContent = "—";
  $("wristWrenchDetail").textContent = "无实时测量";
}

function clearSimulationTelemetry(mission = null) {
  clearWristWrench();
  state.lastAckAt = 0;
  for (const id of ["simTime", "actionSequence", "toolPosition", "jacobianRank", "ikSolver",
    "referenceProtection", "dynamicsTiming", "attitudeMode", "attitudeError", "bodyRate",
    "wheelSpeeds", "wheelTorques", "vx", "vy", "vz", "wx", "wy", "wz", "grip",
    "relativeDistance", "relativeVelocity", "relativeAngularVelocity", "formationHold",
    "formationError", "formationVelocity", "captureToolStatus"]) {
    $(id).textContent = "—";
  }
  $("motionSpeedStatus").textContent = "等待当前场景的速度测量";
  $("motionSpeedDetail").textContent = "场景就绪后显示实测速度。";
  $("motionSpeedDiagnostic").className = "motion-speed-diagnostic";
  state.renderSessionId = "";
  state.visualProbe = false;
  state.hasCaptureArm = false;
  $("captureArmOption").disabled = true;
  $("captureToolControls").hidden = true;
  if (mission?.stage === "flight" && mission.telemetry?.connected) {
    const time = mission.telemetry.sim_time_s;
    if (Number.isFinite(time)) $("simTime").textContent = `${time.toFixed(3)} s`;
    const rate = mission.telemetry.state?.bodies?.find(body => body.body_id === "rendezvous/chaser")?.angular_velocity_rad_s;
    if (Array.isArray(rate) && rate.length === 3 && rate.every(Number.isFinite)) {
      $("bodyRate").textContent = rate.map(value => value.toFixed(4)).join(", ") + " rad/s";
    }
  }
}

const gamepadInput = new GamepadInput();

const state = {
  activePage: "operate",
  ws: null,
  connected: false,
  controlGranted: false,
  activeEpisode: null,
  sequence: 0,
  pressed: new Set(),
  estopped: false,
  operationActive: false,
  freeCameraMode: false,
  relativeView: null,
  lastAckAt: 0,
  linearSpeed: 0.05,
  pixelStreaming: null,
  pixelConfig: null,
  streamConfig: null,
  selectedStreamerId: "BskRenderer",
  lastReceivedFrames: null,
  lastReceivedTimestamp: null,
  lastPresentedFrames: null,
  lastPresentedTimestamp: null,
  lastQpSample: null,
  streamLive: false,
  streamReconnectTimer: null,
  pixelConnectPromise: null,
  scenePhase: "idle",
  sceneReady: false,
  resetPending: false,
  simulationResetting: false,
  simulationConnected: false,
  resetSupported: false,
  sceneDefaults: { simulation_rate: 1, capture_rate_hz: 50, ik_rate_hz: 100 },
  currentUser: null,
  canManageScene: false,
  operationRequested: false,
  actionTimer: null,
  stateTimer: null,
  stateRequestSequence: 0,
  sceneTransitionPending: false,
  missionMode: "teleop",
  mission: null,
  stateReadPending: false,
  gamepadSnapshot: null,
};

const $ = (id) => document.getElementById(id);
state.controlledArm = 'operating';
state.captureTool = {radial:0, axial:0};
const mainCompanion = new MainCompanion({
  status: text => { $('mainCompanionTitle').textContent=text; $('mainCompanionVideo').style.visibility=mainCompanion.live?'visible':'hidden'; },
  show: visible => { $('mainCompanion').hidden=!visible; if(!visible)$('mainCompanionVideo').replaceChildren(); },
  create: id => {
    const config = new Config({initialSettings:{
      [TextParameters.SignallingServerUrl]:signallingUrl(), [OptionParameters.StreamerId]:id,
      [Flags.AutoConnect]:true,[Flags.AutoPlayVideo]:true,[Flags.StartVideoMuted]:true,
      [Flags.WaitForStreamer]:true,[Flags.KeyboardInput]:false,[Flags.MouseInput]:false,
      [Flags.TouchInput]:false,[Flags.GamepadInput]:false,
    }});
    return new PixelStreaming(config,{videoElementParent:$('mainCompanionVideo')});
  },
});
const initialJoints = createInitialJointAngles({
  toggle: $("sceneCustomInitialJoints"), fields: $("sceneInitialJointFields"),
  presetButton: $("sceneInitialJointPreset"), help: $("sceneInitialJointsHelp"),
});
const armPreparation = createArmPreparation({
  fields: $("operatingJointFields"), startButton: $("prepareArm"), cancelButton: $("cancelPrepareArm"),
  defaultButton: $("operatingJointDefault"), status: $("armPreparationStatus"),
  getContext: () => state,
  send: request => transmitAction({linear:[0,0,0], angular:[0,0,0], grip:0, source:"keyboard", preparation:request}, !!request),
  cancelMotion: () => exitOperationMode(""),
  activateControl: () => state.ws?.send(JSON.stringify({type:"activate_control"})),
  message: setMessage, changed: () => { updateOperationUI(); updateEpisodeUI(); },
});
let initialJointCatalog = null;
function configureInitialJoints() {
  const zeroStart = [ZERO_START_PROFILE, "teleop-zero-prepare-v2"].includes($("randomizationProfile").value);
  $("legacyInitialJointSettings").hidden = zeroStart;
  if (zeroStart) $("sceneCustomInitialJoints").checked = false;
  armPreparation.configure(initialJointCatalog?.templates.find(item => item.id === $("sceneTemplate").value), $("randomizationProfile").value);
  initialJoints.configure(
    initialJointCatalog?.templates.find(item => item.id === $("sceneTemplate").value),
    initialJointCatalog?.initial_arm_presets_deg?.[$("randomizationProfile").value],
  );
}
function configureSceneSelection() {
  const template = initialJointCatalog?.templates.find(item => item.id === $("sceneTemplate").value);
  if (template && $("sceneCollisionWarning")) $("sceneCollisionWarning").textContent = template.description;
  const fallbackTemplate = template?.id === "sarm-task-box-free-plugs";
  const unsupported = template?.unsupported_randomization_profiles
    ?? (fallbackTemplate ? [AUTO_PREPARE_PROFILE] : []);
  if (unsupported.includes($("randomizationProfile").value)) {
    $("randomizationProfile").value = template?.default_randomization_profile
      ?? (fallbackTemplate ? ZERO_START_PROFILE : $("randomizationProfile").value);
  }
  configureInitialJoints();
}
const jointAngles = createJointAnglePanel($("jointAngleGrid"), $("jointAngleState"));
const missionPanel = createMissionPanel({$, state, apiRequest, refresh: refreshState,
  message: setMessage, configureScene: configureSceneSelection,
  reconnect: () => { if (!state.streamLive) connectPixelStreaming(); }});
const controlKeys = new Set(["KeyA", "KeyD", "KeyW", "KeyS", "KeyQ", "KeyE", "KeyR", "KeyF", "ShiftLeft", "ShiftRight", "Escape"]);

function setPixelStreamingInputEnabled(keyboardEnabled, mouseEnabled) {
  const stream = state.pixelStreaming;
  if (!stream?.config) return;
  stream.config.setFlagEnabled(Flags.KeyboardInput, keyboardEnabled);
  stream.config.setFlagEnabled(Flags.MouseInput, mouseEnabled);
}

const workLight = new WorkLight({send: command => state.streamLive && state.pixelStreaming?.emitCommand(command) === true});
function updateWorkLight() {
  const button = $("workLightToggle");
  button.disabled = !state.streamLive || !workLight.available;
  button.textContent = !state.streamLive ? "工作灯：未连接" : !workLight.available ? "工作灯：不可用" : workLight.enabled ? "工作灯：开（点击关闭）" : "工作灯：关（点击开启）";
  button.setAttribute("aria-pressed", String(workLight.available && workLight.enabled));
}
$("workLightToggle").addEventListener("click", event => {
  event.stopPropagation();
  if (workLight.toggle()) setMessage("已请求切换工作灯，等待渲染器确认；只改变预览补光。");
});
setInterval(() => { if (state.streamLive) workLight.query(); updateWorkLight(); }, 2000);

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
window.__relativeViewDiagnostics = () => state.relativeView ? structuredClone(state.relativeView) : null;

function requestRelativeView(mode = "query") {
  return state.pixelStreaming?.emitCommand({BskRelativeView: JSON.stringify({mode})}) === true;
}

function updateRelativeViewControl() {
  const main = state.selectedStreamerId === state.streamConfig?.pixel_streaming_streamer_id;
  const available = main && !!state.relativeView?.mode && state.mission?.stage === "flight" && state.streamLive;
  $("relativeViewSelector").disabled = !available;
  $("relativeViewSelector").title = available ? "选择双星取景方式" : "双星交会主视口就绪后可调整取景";
}

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
  platform.setUser(null);
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
    const error = await response.clone().json().catch(() => ({}));
    if (error.detail === "a valid stream access key is required" || !state.currentUser) return response;
    shutdownAuthenticatedApp();
    showLogin("登录已过期，请重新登录");
  }
  return response;
}

function applyAuthenticatedUser(user) {
  state.currentUser = user;
  document.body.classList.remove("auth-pending", "auth-required");
  $("authGate").hidden = true;
  $("currentUsername").textContent = userLabel(user);
  $("currentRole").textContent = user.role === "admin" ? "管理员" : "操作员";
  platform.setUser(user);
}

function shutdownAuthenticatedApp() {
  state.stateRequestSequence += 1;
  state.stateReadPending = false;
  missionPanel.reset();
  exitOperationMode("");
  Object.values(pages).forEach(page=>page.reset());
  document.querySelectorAll("dialog.platform-dialog").forEach(dialog=>dialog.close());
  if (state.actionTimer) clearInterval(state.actionTimer);
  if (state.stateTimer) clearInterval(state.stateTimer);
  state.actionTimer = null; state.stateTimer = null;
  if (state.ws) { state.ws.onclose = null; state.ws.close(); state.ws = null; }
  disposePixelStream();
  jointAngles.clear("已退出登录");
  clearSimulationTelemetry();
  state.connected = false; state.controlGranted = false;
}

async function startAuthenticatedApp() {
  // Keep the team shell and deep links; load mission settings after the scene catalog.
  await Promise.all([loadSceneCatalog().then(() => missionPanel.load()), refreshState(), platform.activateCurrent()]);
  if (!state.currentUser) return;
  updateOperationUI();
  if (!state.actionTimer) state.actionTimer = setInterval(sendAction, 33);
  if (!state.stateTimer) state.stateTimer = setInterval(() => { jointAngles.checkFreshness(); refreshState(); }, 500);
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
    initialJointCatalog = catalog;
    configureSceneSelection();
    $("sceneDatasetCapture").checked = Boolean(defaults.dataset_capture);
    $("sceneRandomizeOrbitPhase").checked = defaults.randomize_orbit_phase === true;
    $("sceneSunlightIntensity").value = String(defaults.sunlight_intensity_scale ?? 1);
    $("sceneWristCameraMode").value = defaults.wrist_camera_mode || "rgb";
    state.sceneDefaults = {
      simulation_rate: Number(defaults.simulation_rate || 1),
      capture_rate_hz: Number(defaults.capture_rate_hz || 50),
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
  state.sceneInstanceId = instance.instance_id || null;
  const currentObservation = state.simulationConnected &&
    (!state.sceneInstanceId || state.observationSceneId === state.sceneInstanceId);
  state.sceneReady = phase === "running" && currentObservation && !state.resetPending && !state.simulationResetting;
  if (!state.sceneReady) $("fastenerDemoStatus").hidden = true;
  if (!state.sceneReady) clearSimulationTelemetry(state.mission);
  if (!state.sceneReady && state.operationActive) {
    exitOperationMode("当前场景尚未就绪，已退出操作模式");
  }
  $("scenePhase").textContent = state.resetPending || state.simulationResetting ? "正在重置" : enabled ? (scenePhaseLabels[phase] || phase) : "运行时未配置";
  $("scenePhase").classList.toggle("scene-running", state.sceneReady);
  $("scenePhase").classList.toggle("scene-failed", phase === "failed");
  $("startScene").disabled = active || !enabled;
  $("stopScene").disabled = !active || !state.canManageScene;
  $("resetScene").disabled = !state.sceneReady || !state.simulationConnected
    || !state.resetSupported || !state.canManageScene || Boolean(state.activeEpisode);
  $("resetScene").textContent = state.resetPending || state.simulationResetting ? "正在复原…" : "一键复原初始位置";
  $("resetScene").title = state.activeEpisode ? "请先结束当前采集，再重置状态"
    : "恢复机械臂、夹爪、目标物和仿真时间到本次场景初态；不是规划机械臂返航轨迹";
  ["sceneTemplate", "randomizationProfile", "sceneSeed", "sceneRandomizeOrbitPhase", "sceneDatasetCapture", "sceneSunlightIntensity", "sceneWristCameraMode"].forEach((id) => {
    $(id).disabled = active;
  });
  $("linearSpeed").disabled = !state.sceneReady;
  $("estop").disabled = !state.sceneReady;
  updateOperationUI();
  $("sceneInstanceId").textContent = instance.instance_id || "—";
  $("sceneInstanceSeed").textContent = instance.seed ?? "—";
  $("sceneInstanceTarget").textContent = !instance.instance_id ? "—" : instance.template_id?.startsWith("sarm-task-box-20261012") ? "新版 SARM 完整任务盒 + 地面验证星" : instance.template_id === "sarm-ground-validation-self-collision-grasp" ? "地面验证星（粗碰撞体·内部碰撞）" : instance.template_id === "sarm-ground-validation-mesh-grasp" ? "地面验证星（高精度三角网格·实验）" : instance.template_id === "sarm-ground-validation-grasp" ? "地面验证星（粗碰撞盒·无内部碰撞）" : "原小方块目标";
  const instanceSunlight = instance.environment?.lighting?.sunlight_intensity_scale ?? 1;
  $("sceneInstanceSunlight").textContent = instance.instance_id ? `${instanceSunlight} 倍` : "—";
  const instanceOrbitPhase = instance.environment?.orbit?.true_anomaly_deg ?? 180;
  const randomizeOrbitPhase = instance.randomize_orbit_phase === true;
  $("sceneInstanceOrbitPhase").textContent = instance.rendezvous_handoff ? "继承交会终态"
    : instance.instance_id ? `${Number(instanceOrbitPhase).toFixed(2)}°` : "—";
  if (active && instance.instance_id) {
    if (instance.template_id) $("sceneTemplate").value = instance.template_id;
    if (instance.capture_target?.runtime_warning) $("sceneCollisionWarning").textContent = instance.capture_target.runtime_warning;
    if (instance.randomization_profile) $("randomizationProfile").value = instance.randomization_profile;
    $("sceneSunlightIntensity").value = String(instanceSunlight);
    $("sceneWristCameraMode").value = instance.wrist_camera_mode || "rgb";
    $("sceneRandomizeOrbitPhase").checked = randomizeOrbitPhase;
  }
  configureInitialJoints();
  initialJoints.setRuntime(active, instance);
  armPreparation.setRuntime(active ? instance : null, state.sceneReady);
  if (["failed", "stopping", "stopped"].includes(phase)) armPreparation.cancel("场景停止或失败，请显式重试");
  $("sceneParameters").textContent = instance.randomization
    ? JSON.stringify({ capture_target: instance.capture_target || {}, wrist_camera_mode: instance.wrist_camera_mode || "rgb", randomize_orbit_phase: randomizeOrbitPhase, environment: instance.environment || {}, randomization: instance.randomization }, null, 2)
    : "尚未生成实例";
  if (runtime.error) setMessage(`场景失败：${runtime.error}`);
  if (phase === "running" && previousPhase !== "running") {
    const readiness = instance.arm_preparation_required
      ? '等待机械臂准备实测到位后操作或采集' : '可进入操作或开始采集';
    setMessage(`场景 ${instance.instance_id || ""} ${instance.rendezvous_handoff ? '已接续交会终态' : '已运行'}，${readiness}`);
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
  if (state.sceneTransitionPending) return;
  if (state.missionMode && state.missionMode !== "teleop") return missionPanel.startFlight();
  let initialAngles;
  let operatingAngles;
  const operatingProfile = [ZERO_START_PROFILE, "teleop-zero-prepare-v2"].includes($("randomizationProfile").value);
  try { initialAngles = operatingProfile ? null : initialJoints.read(); operatingAngles = armPreparation.read(); }
  catch (error) { setMessage(error.message); return; }
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
    wrist_camera_mode: $("sceneWristCameraMode").value,
    initial_arm_joint_position_deg: initialAngles,
    ...(operatingProfile ? {operating_arm_joint_position_deg: operatingAngles} : {}),
  };
  $("startScene").disabled = true;
  setMessage("正在生成可复现场景实例…");
  state.sceneTransitionPending = true;
  state.stateRequestSequence += 1;
  try {
    const response = await apiRequest("/api/scenes/start", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(request),
    });
    const data = await readApiResponse(response);
    if (!response.ok) throw new Error(data.detail || `无法启动场景（HTTP ${response.status}）`);
    applySceneRuntime({ ...data, active: true, instance: data });
    armPreparation.armAutoStart?.(data.instance_id);
    setMessage(`已生成场景 ${data.instance_id}，Seed ${data.seed}，正在启动 UE`);
  } catch (error) {
    setMessage(error.message || "无法启动场景");
  } finally {
    state.sceneTransitionPending = false;
    await refreshState();
  }
}

async function resetScene() {
  if (state.resetPending || state.simulationResetting || !state.sceneReady) return;
  if (state.activeEpisode) return setMessage("请先结束当前采集，再复原初始位置");
  if (state.canManageScene === false || state.resetSupported === false || state.simulationConnected === false) {
    return setMessage("当前无法复原：请检查场景权限及仿真连接");
  }
  state.resetPending = true;
  state.sceneReady = false;
  state.operationRequested = false;
  exitOperationMode("正在重置场景，已停止操控输入");
  $("resetScene").disabled = true;
  $("resetScene").textContent = "正在复原…";
  updateEpisodeUI();
  try {
    const response = await apiRequest("/api/scenes/reset", { method: "POST" });
    const data = await readApiResponse(response);
    if (!response.ok) throw new Error(data.detail || "重置失败");
    setMessage(data.status === "completed"
      ? "已恢复本次场景的初始状态，点击画面可重新进入操作"
      : "重置已取消");
  } catch (error) {
    setMessage(error.message || "重置请求失败，请检查场景状态");
  } finally {
    state.resetPending = false;
    await refreshState();
  }
}

async function stopScene() {
  if (state.sceneTransitionPending) return;
  $("stopScene").disabled = true;
  setMessage("正在停止场景…");
  state.sceneTransitionPending = true;
  state.stateRequestSequence += 1;
  try {
    const response = await apiRequest("/api/scenes/stop", { method: "POST" });
    const data = await readApiResponse(response);
    if (!response.ok) throw new Error(data.detail || `无法停止场景（HTTP ${response.status}）`);
    applySceneRuntime(data);
    setMessage("场景已停止，控制平台仍保持运行");
  } catch (error) {
    setMessage(error.message || "无法停止场景");
  } finally {
    state.sceneTransitionPending = false;
    await refreshState();
  }
}

function connect() {
  if (!state.currentUser || (state.activePage && state.activePage !== "operate")) return;
  if (state.ws && [WebSocket.OPEN, WebSocket.CONNECTING].includes(state.ws.readyState)) return;
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
    clearSimulationTelemetry();
    state.connected = false;
    state.controlGranted = false;
    setOnline("backendDot", false);
    $("backendState").textContent = "后端断开";
    jointAngles.clear("连接中断，等待新遥测");
    $("motionSpeedStatus").textContent = "连接中断，速度诊断已失效";
    $("motionSpeedDetail").textContent = "等待新的实测遥测，不沿用断线前的速度与原因。";
    $("motionSpeedDiagnostic").className = "motion-speed-diagnostic";
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
      if (message.reason === "scene_reset") jointAngles.clear("场景重置中");
      state.operationRequested = false;
      $("controlAuthority").textContent = "其他页面正在操作";
      exitOperationMode(message.reason === "scene_reset" ? "场景正在重置，已撤销操控；完成后请重新点击画面" : "同一用户的其他页面已进入操作，本页面自动退出");
    } else if (message.type === "observation") {
      const raw = message.payload;
      if (state.mission?.stage === "flight") return;
      if (state.sceneInstanceId && raw.scene_instance_id !== state.sceneInstanceId) return;
      state.renderSessionId = raw.render_session_id;
      const relative = raw.target_relative_state;
      $('relativeDistance').textContent = relative ? `${Math.hypot(...relative.position_inertial_m).toFixed(4)} m` : '—';
      const formation = raw.formation_hold;
      $('formationHold').textContent = formation?.enabled ? '开启 · 限幅推力/力矩' : '关闭';
      $('formationError').textContent = formation?.enabled ? formation.position_error_m.map(e=>(Math.hypot(...e)*1000).toFixed(2)).join(' / ')+' mm（服务 / 目标）' : '—';
      const rotatingVelocity=formation?.enabled ? formation.relative_velocity_rotating_m_s : relative?.velocity_rotating_m_s;
      $('formationVelocity').textContent = rotatingVelocity ? (Math.hypot(...rotatingVelocity)*1000).toFixed(3)+' mm/s' : '—';
      $('relativeVelocity').textContent = relative ? relative.velocity_inertial_m_s.map(x=>(x*1000).toFixed(3)).join(', ')+' mm/s' : '—';
      $('relativeAngularVelocity').textContent = relative ? relative.angular_velocity_inertial_rad_s.map(x=>x.toFixed(6)).join(', ')+' rad/s' : '—';
      state.hasCaptureArm = !!raw.capture_arm;
      state.visualProbe = raw.capture_arm?.visual_probe_enabled===true;
      $('captureArmOption').disabled = !state.hasCaptureArm;
      if (!state.hasCaptureArm && state.controlledArm === 'capture') {
        exitOperationMode('当前场景没有抓取臂');state.controlledArm='operating';$('controlledArm').value='operating';
      }
      $('captureToolControls').hidden = state.controlledArm !== 'capture';
      if(raw.capture_arm) $('captureToolStatus').textContent = `轴向 ${(raw.capture_arm.axial_position_m*1000).toFixed(1)} mm · 径向 ${(raw.capture_arm.radial_position_m*1000).toFixed(1)} mm · ${raw.capture_arm.controller_status}`;
      const obs = armObservation(raw,state.controlledArm);
      const demo = obs.fastener_demo;
      $("fastenerDemoStatus").hidden = !demo;
      if (demo) {
        const phase = { settle: "松开等待", tighten: "正在拧紧", hold_tight: "压紧保持", loosen: "正在拧松", hold_loose: "松开保持" }[demo.phase] || demo.phase;
        $("fastenerDemoStatus").textContent = `防脱演示 · ${phase} · 行程 ${Number(demo.travel_mm).toFixed(2)} mm · 转角 ${Number(demo.angle_deg).toFixed(1)}°`;
      }
      jointAngles.update(obs);
      armPreparation.update(raw.arm_preparation, raw);
      setOnline("simDot", true);
      $("simState").textContent = "仿真在线";
      $("simTime").textContent = `${(Number(obs.sim_time_ns) / 1e9).toFixed(3)} s`;
      $("actionSequence").textContent = obs.applied_action_sequence;
      updateMotionOutputs(obs.end_effector_twist_body || [], obs.gripper_type === 'four_bar_tendon' ? obs.gripper_drive_velocity_rad_s : obs.gripper_velocity_m_s?.[0] ?? obs.joint_velocity_rad_s?.[6]);
      $("grip").title = obs.gripper_type === 'four_bar_tendon' ? '四连杆驱动角速度 rad/s' : '手指滑动速度 m/s';
      const speedDiagnostic = formatMotionSpeedDiagnostics(obs.motion_speed_diagnostics);
      $("motionSpeedStatus").textContent = speedDiagnostic.title;
      $("motionSpeedDetail").textContent = speedDiagnostic.detail;
      $("motionSpeedDiagnostic").className = speedDiagnostic.warning ? "motion-speed-diagnostic warning" : "motion-speed-diagnostic";
      const position = obs.end_effector_position_body_m || [];
      $("toolPosition").textContent = position.length === 3 ? position.map((value) => Number(value).toFixed(3)).join(", ") : "—";
      $("jacobianRank").textContent = `${obs.jacobian_rank ?? "—"} / 6`;
      $("ikSolver").textContent = obs.ik_mode
        ? `${obs.ik_mode}${obs.ik_status ? ` / ${obs.ik_status}` : ""} · λ=${Number(obs.ik_damping ?? 0).toFixed(4)} · scale=${Number(obs.ik_velocity_scale ?? 1).toFixed(2)} · σmin=${Number(obs.ik_minimum_singular_value ?? 0).toExponential(1)} · ${Number(obs.ik_solve_time_ms ?? 0).toFixed(2)} ms${obs.ik_elbow_preference?.enabled ? ` · 构型偏好=${obs.ik_elbow_preference.status} · 肘高=${Number(obs.ik_elbow_preference.measured_height_m ?? 0).toFixed(3)}m${obs.ik_elbow_preference.wrist_enabled ? ` · J4−J6=${Number(obs.ik_elbow_preference.measured_wrist_drop_m ?? 0).toFixed(3)}m` : ""}${obs.ik_elbow_preference.joint3_enabled ? ` · J3负角偏好=${(Number(obs.ik_elbow_preference.measured_joint3_rad ?? 0) * 180 / Math.PI).toFixed(1)}°` : ""}` : ""}`
        : "—";
      const wrench = formatWristWrench(obs.wrist_wrench);
      $("wristForce").textContent = wrench.force;
      $("wristTorque").textContent = wrench.torque;
      $("wristWrenchDetail").textContent = wrench.detail;
      const attitude = obs.attitude_control;
      $("referenceProtection").textContent = referenceProtectionLabel(obs);
      if(state.controlledArm==='capture') {
        $('ikSolver').textContent=`抓取臂阻尼 IK · ${obs.ik_status || '等待遥测'}`;
        $('referenceProtection').textContent='关节限速 0.25 rad/s · 参考超前限制 0.05 rad · 松键保持实测位置';
      }
      $("dynamicsTiming").textContent = dynamicsTimingLabel(obs);
      const wheels = obs.reaction_wheels;
      $("attitudeMode").textContent = !attitude ? "—" : !attitude.enabled ? "控制关闭" :
        !attitude.reference_initialized ? "初始化" : (formation?.enabled || raw.coorbital_initialization ? "轨道定向保持" : "惯性保持") + (attitude.saturated ? " · 饱和" : "");
      $("attitudeError").textContent = attitude ? `${(attitude.attitude_error_angle_rad * 180 / Math.PI).toFixed(3)} °` : "—";
      $("bodyRate").textContent = attitude ? attitude.angular_velocity_body_rad_s.map(v => Number(v).toFixed(4)).join(", ") + " rad/s" : "—";
      $("wheelSpeeds").textContent = wheels ? wheels.speed_rad_s.map(v => (v * 60 / (2 * Math.PI)).toFixed(1)).join(", ") + " rpm" : "—";
      $("wheelTorques").textContent = wheels ? wheels.applied_motor_torque_nm.map(v => Number(v).toFixed(3)).join(", ") + " N·m" : "—";
    } else if (message.type === "action_ack") {
      state.lastAckAt = performance.now();
      $("actionSequence").textContent = message.server_sequence;
    } else if (message.type === "action_rejected") {
      const reasons = { scene_resetting: "场景正在重置，请稍候", inactive_page: "当前页面不是活动操作页面", not_scene_owner: "当前用户不能操作这个场景" };
      state.operationRequested = false;
      setMessage(reasons[message.reason] || `动作被拒绝：${message.reason}`);
    }
  };
}

async function connectPixelStreaming() {
  if (!state.currentUser || (state.activePage && state.activePage !== "operate")) return;
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
    if (!state.currentUser || (state.activePage && state.activePage !== "operate")) return;
    if (response.status === 401) {
      disposePixelStream();
      state.streamConfig = null;
      $("previewEmpty").style.display = "grid";
      $("previewEmptyTitle").textContent = "访问链接缺少密钥或密钥无效";
      $("previewEmptyHint").textContent = "账号已登录。请从服务器访问窗口复制完整链接，包含 access_key，再在本机浏览器打开。";
      $("frameState").textContent = "ACCESS KEY REQUIRED";
      setMessage("登录成功，但视频和控制需要有效访问密钥。这不是密码错误，请使用包含 access_key 的完整访问链接。");
      return;
    }
    if (!response.ok) throw new Error(`config ${response.status}`);
    const config = await response.json();
    state.streamConfig = config;
    configureVideoFps();
    updatePreviewStreams(null);
    createPixelStream();
    } catch (error) {
      console.warn("Pixel Streaming configuration is unavailable", error);
      $("frameState").textContent = "WEBRTC OFFLINE";
    if (state.currentUser) schedulePixelStreamingReconnect();
    }
  })().finally(() => {
    if (state.pixelConnectPromise === pending) state.pixelConnectPromise = null;
  });
  state.pixelConnectPromise = pending;
  return pending;
}

function updatePreviewStreams(availableIds) {
  const config = state.streamConfig;
  const configured = config.pixel_streaming_streamers || [{id: config.pixel_streaming_streamer_id, label: "主视口"}];
  const resolved = resolvePreviewStreams(configured, availableIds, state.selectedStreamerId);
  const selector = $("streamSelector");
  selector.replaceChildren();
  for (const item of resolved.streams) {
    const option = document.createElement("option");
    const labels = {WristCamera: "机械臂腕部相机", SpacecraftOverview: "卫星总览相机"};
    option.value = item.id;
    option.textContent = item.id.endsWith("__teleop_camera_capture_wrist_cam")
      ? "抓取臂末端相机" : labels[item.label] || item.label || item.id;
    option.disabled = item.disabled;
    selector.append(option);
  }
  selector.value = resolved.selectedId;
  state.selectedStreamerId = resolved.selectedId;
}

function signallingUrl() {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  const url = new URL(state.streamConfig.pixel_streaming_signalling_url
    || `${scheme}//${location.hostname}:${state.streamConfig.pixel_streaming_player_port}`, location.href);
  if (state.streamConfig.pixel_streaming_access_token) {
    url.searchParams.set("token", state.streamConfig.pixel_streaming_access_token);
  }
  return url.toString();
}

function resetWebRtcStats() {
  state.playbackSample = null;
  state.playbackHealth = "unknown";
  state.playbackAlert = false;
  state.keyframeRecoveryAttempts = 0;
  state.lastKeyframeRequestAt = null;
  $("webrtcDecodeFps").textContent = "—";
  $("webrtcPlayback").textContent = "等待视频";
  state.lastPacketSample = null;
  state.videoDiagnostics = [];
  state.lastReceivedFrames = null;
  state.lastReceivedTimestamp = null;
  $("webrtcReceiveFps").textContent = "—";
  state.lastPresentedFrames = null;
  state.lastPresentedTimestamp = null;
  state.lastQpSample = null;
  $("webrtcQp").textContent = "—";
  $("webrtcRtt").textContent = "—";
  $("webrtcBitrate").textContent = "—";
  $("webrtcLoss").textContent = "—";
  $("webrtcFps").textContent = "—";
  $("webrtcResolution").textContent = "—";
}

function videoFpsOptions() {
  const maximum = Number(state.streamConfig?.pixel_streaming_fps ?? 90);
  return [...new Set([30, 60, 90, 120, maximum])]
    .filter(value => Number.isFinite(value) && value > 0 && value <= maximum)
    .sort((left, right) => left - right);
}

function configureVideoFps() {
  const options = videoFpsOptions();
  // Most browser/RDP displays present 60 Hz; keep higher server-supported
  // choices available without decoding/rendering discarded frames by default.
  if (!options.includes(state.requestedVideoFps)) state.requestedVideoFps = options.filter(fps => fps <= 60).at(-1) ?? options[0];
  const selector = $("videoFpsSelector");
  selector.replaceChildren();
  for (const fps of options) {
    const option = document.createElement("option");
    option.value = String(fps);
    option.textContent = `${fps} FPS`;
    selector.append(option);
  }
  selector.value = String(state.requestedVideoFps);
}

function selectVideoFps(value) {
  const fps = Number(value);
  if (!videoFpsOptions().includes(fps)) return false;
  state.requestedVideoFps = fps;
  state.pixelConfig?.setNumericSetting(NumericParameters.WebRTCFPS, fps);
  return true;
}

function samplePacketLoss(video) {
  const sample = {
    received: video.packetsReceived, lost: video.packetsLost, timestamp: video.timestamp,
    id: video.id, ssrc: video.ssrc,
  };
  const previous = state.lastPacketSample;
  if (![sample.received, sample.lost, sample.timestamp].every(Number.isFinite)) {
    state.lastPacketSample = null;
    return null;
  }
  state.lastPacketSample = sample;
  if (!previous || sample.id !== previous.id || sample.ssrc !== previous.ssrc
      || sample.timestamp <= previous.timestamp || sample.received < previous.received) return null;
  const lost = Math.max(0, sample.lost - previous.lost);
  const total = sample.received - previous.received + lost;
  return total > 0 ? lost * 100 / total : null;
}

// qpSum is cumulative across decoded frames. Difference consecutive samples so
// a long clear period cannot conceal a recent compression-quality drop.
function sampleAverageQp(video) {
  if (!video || !Number.isFinite(video.qpSum) || video.qpSum < 0
      || !Number.isFinite(video.framesDecoded) || video.framesDecoded < 0
      || !Number.isFinite(video.timestamp)) {
    state.lastQpSample = null;
    return null;
  }
  const sample = {
    qpSum: video.qpSum, framesDecoded: video.framesDecoded, timestamp: video.timestamp,
    id: video.id, ssrc: video.ssrc, trackIdentifier: video.trackIdentifier, codecId: video.codecId,
  };
  const previous = state.lastQpSample;
  state.lastQpSample = sample;
  if (!previous || ["id", "ssrc", "trackIdentifier", "codecId"].some(key => sample[key] !== previous[key])
      || sample.timestamp <= previous.timestamp || sample.framesDecoded <= previous.framesDecoded
      || sample.qpSum < previous.qpSum) return null;
  const average = (sample.qpSum - previous.qpSum) / (sample.framesDecoded - previous.framesDecoded);
  return Number.isFinite(average) ? average : null;
}

function samplePlayback(video, player, now) {
  const sample = {
    id: video.id, ssrc: video.ssrc, timestamp: video.timestamp,
    decoded: video.framesDecoded, received: video.framesReceived, currentTime: player?.currentTime,
  };
  const previous = state.playbackSample;
  state.playbackSample = sample;
  const validInterval = previous && Number.isFinite(sample.timestamp) && Number.isFinite(previous.timestamp)
    && sample.timestamp > previous.timestamp && sample.id === previous.id && sample.ssrc === previous.ssrc;
  let decodedFps = null;
  if (validInterval && Number.isFinite(sample.decoded) && Number.isFinite(previous.decoded)
      && sample.decoded >= previous.decoded) {
    decodedFps = (sample.decoded - previous.decoded) * 1000 / (sample.timestamp - previous.timestamp);
  }
  if (!validInterval || decodedFps == null || document.hidden) {
    state.lastDecodeProgressAt = now;
    state.lastMediaProgressAt = now;
    state.keyframeRecoveryAttempts = 0;
    state.lastKeyframeRequestAt = null;
  }
  if (decodedFps > 0) state.lastDecodeProgressAt = now;
  const hasMediaClock = Number.isFinite(sample.currentTime) && Number.isFinite(previous?.currentTime);
  if (!hasMediaClock || sample.currentTime !== previous.currentTime) state.lastMediaProgressAt = now;
  const receiving = validInterval && Number.isFinite(sample.received) && Number.isFinite(previous.received)
    && sample.received > previous.received;
  let health = "waiting";
  if (document.hidden) health = "hidden";
  else if (player?.error) health = "media-error";
  else if (player?.paused === true) health = "paused";
  else if (receiving && decodedFps === 0 && now - state.lastDecodeProgressAt >= 5000) health = "decode-stalled";
  else if (decodedFps > 0 && hasMediaClock && now - state.lastMediaProgressAt >= 5000) health = "playback-stalled";
  else if (decodedFps > 0) health = "playing";
  if (health === "playing") {
    state.keyframeRecoveryAttempts = 0;
    state.lastKeyframeRequestAt = null;
  }
  if (health === "decode-stalled" && state.streamLive && state.keyframeRecoveryAttempts < 2
      && (state.lastKeyframeRequestAt == null || now - state.lastKeyframeRequestAt >= 15000)) {
    state.lastKeyframeRequestAt = now;
    state.keyframeRecoveryAttempts++;
    try { state.pixelStreaming?.requestIframe?.(); } catch (_) { }
  }
  state.playbackHealth = health;
  return { decodedFps, health };
}

function updateWebRtcStats(aggregatedStats) {
  const pair = aggregatedStats?.getActiveCandidatePair?.();
  const video = aggregatedStats?.inboundVideoStats;
  const averageQp = sampleAverageQp(video);
  $("webrtcQp").textContent = averageQp == null ? "—" : averageQp.toFixed(1);
  if (!video) return;
  const packetLoss = samplePacketLoss(video);
  // Received frames and displayed frames are different counters. A 60 Hz client
  // can receive 90 FPS without presenting 90 unique frames on its screen.
  const receivedFrames = Number(video.framesReceived);
  const receivedTimestamp = Number(video.timestamp);
  let receivedFps = null;
  if (Number.isFinite(receivedFrames) && Number.isFinite(receivedTimestamp)) {
    if (state.lastReceivedTimestamp != null && receivedTimestamp > state.lastReceivedTimestamp
        && receivedFrames >= state.lastReceivedFrames) {
      receivedFps = (receivedFrames - state.lastReceivedFrames) * 1000
        / (receivedTimestamp - state.lastReceivedTimestamp);
    }
    state.lastReceivedFrames = receivedFrames;
    state.lastReceivedTimestamp = receivedTimestamp;
  }
  $("webrtcReceiveFps").textContent = receivedFps == null ? "—" : receivedFps.toFixed(0);
  // WebRTC's inbound framesPerSecond is the receive/decode rate. It can be
  // much higher than what the browser actually presents because decoded frames
  // may be dropped by the compositor. Measure non-dropped video frames instead.
  const player = $("pixelStream").querySelector("video");
  const quality = player?.getVideoPlaybackQuality?.();
  const totalVideoFrames = Number(quality?.totalVideoFrames ?? player?.webkitDecodedFrameCount);
  const droppedVideoFrames = Number(quality?.droppedVideoFrames ?? player?.webkitDroppedFrameCount ?? 0);
  const presentedFrames = totalVideoFrames - droppedVideoFrames;
  const presentedTimestamp = performance.now();
  const playback = samplePlayback(video, player, presentedTimestamp);
  $("webrtcDecodeFps").textContent = playback.decodedFps == null ? "—" : playback.decodedFps.toFixed(0);
  const playbackLabels = {
    waiting: "等待新帧", hidden: "页面在后台", playing: "播放中", paused: "已暂停，请播放",
    "decode-stalled": "接收正常，解码停滞", "playback-stalled": "解码正常，播放停滞",
    "media-error": "播放器错误，请重连",
  };
  $("webrtcPlayback").textContent = playbackLabels[playback.health];
  const playbackFault = ["paused", "decode-stalled", "playback-stalled", "media-error"].includes(playback.health);
  if (playbackFault) {
    $("playStream").textContent = playback.health === "paused" ? "继续播放实时画面" : "重新连接恢复画面";
    $("playStream").hidden = false;
    $("frameState").textContent = "VIDEO PAUSED / STALLED";
    state.playbackAlert = true;
  } else if (playback.health === "playing" && state.playbackAlert) {
    $("playStream").hidden = true;
    if (state.streamLive) $("frameState").textContent = "LIVE WEBRTC";
    state.playbackAlert = false;
  }
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
    ? `${Number(video.bitrate).toFixed(0)} kbps` : "—";
  $("webrtcLoss").textContent = packetLoss == null ? "—" : `${packetLoss.toFixed(1)}%`;
  $("webrtcFps").textContent = fps == null ? "—" : fps.toFixed(0);
  $("webrtcResolution").textContent = video.frameWidth && video.frameHeight
    ? `${video.frameWidth}×${video.frameHeight}` : "—";
  const local = aggregatedStats.localCandidates?.find(candidate => candidate.id === pair?.localCandidateId);
  const remote = aggregatedStats.remoteCandidates?.find(candidate => candidate.id === pair?.remoteCandidateId);
  state.videoDiagnostics ??= [];
  state.videoDiagnostics.push({
    time: new Date().toISOString(), targetFps: state.requestedVideoFps ?? null,
    receivedFps, decodedFps: playback.decodedFps, displayedFps: fps, bitrateKbps: video.bitrate ?? null,
    playbackHealth: playback.health, paused: player?.paused ?? null, readyState: player?.readyState ?? null,
    mediaTime: player?.currentTime ?? null, mediaErrorCode: player?.error?.code ?? null,
    visibility: document.hidden ? "hidden" : "visible", keyFramesDecoded: video.keyFramesDecoded ?? null,
    pliCount: video.pliCount ?? null, decoderImplementation: video.decoderImplementation ?? null,
    keyframeRecoveryAttempts: state.keyframeRecoveryAttempts,
    rttMs: pair?.currentRoundTripTime == null ? null : pair.currentRoundTripTime * 1000,
    lossPercent: packetLoss, averageQp, jitterMs: video.jitter == null ? null : video.jitter * 1000,
    localCandidateType: local?.candidateType ?? null, remoteCandidateType: remote?.candidateType ?? null,
    protocol: local?.protocol ?? null, relayProtocol: local?.relayProtocol ?? null,
    width: video.frameWidth ?? null, height: video.frameHeight ?? null,
  });
  if (state.videoDiagnostics.length > 60) state.videoDiagnostics.shift();
}

function disposePixelStream() {
  state.previewAvailability?.close();
  state.previewAvailability = null;
  state.previewAvailableIds = null;
  mainCompanion.dispose();
  if (state.freeCameraMode) setFreeCameraMode(false);
  if (state.streamMediaTimer != null) {
    clearTimeout(state.streamMediaTimer);
    state.streamMediaTimer = null;
  }
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

function showReceivedVideo(stream) {
  if (state.pixelStreaming !== stream) return false;
  const video = $("pixelStream").querySelector("video");
  const frames = video?.getVideoPlaybackQuality?.().totalVideoFrames;
  if (!video || video.readyState < 2 || !video.videoWidth || video.paused === true || frames === 0) return false;
  $("previewEmpty").style.display = "none";
  $("pixelStream").style.display = "block";
  $("frameState").textContent = "LIVE WEBRTC";
  $("frameState").style.color = "var(--accent)";
  return true;
}

function monitorPixelStream(stream) {
  let previousTime;
  let progressedAt = Date.now();
  const check = () => {
    if (state.pixelStreaming !== stream) return;
    const video = $("pixelStream").querySelector("video");
    const mediaTime = video?.currentTime;
    if (!state.streamLive || document.hidden || video?.paused === true
        || (Number.isFinite(mediaTime) && mediaTime !== previousTime)) {
      progressedAt = Date.now();
    }
    previousTime = mediaTime;
    // ICE can remain connected after a renderer restart while the media/data
    // channels have stopped. A historic frame count is not current video.
    // Allow slow cold asset loading without repeatedly interrupting negotiation.
    if (state.streamLive && Date.now() - progressedAt >= 60000) {
      state.streamLive = false;
      if (state.freeCameraMode) setFreeCameraMode(false);
      $("frameState").textContent = "VIDEO RECOVERING";
      $("previewEmpty").style.display = "grid";
      $("previewEmptyTitle").textContent = "视频暂时中断，正在重新连接";
      $("previewEmptyHint").textContent = "仿真继续运行；收到新画面后自动恢复。";
      updateRelativeViewControl();
      schedulePixelStreamingReconnect(0);
      state.streamMediaTimer = null;
      return;
    }
    state.streamMediaTimer = setTimeout(check, 5000);
  };
  state.streamMediaTimer = setTimeout(check, 5000);
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
  mainCompanion.update(state.selectedStreamerId,state.streamConfig?.pixel_streaming_streamer_id);
  $("frameState").textContent = "CONNECTING";
  $("frameState").style.color = "var(--danger)";
  $("previewEmpty").style.display = "grid";
  $("previewEmptyTitle").textContent = "等待实时画面";
  $("previewEmptyHint").textContent = "UE 与 BSK/MJScene 就绪后，WebRTC 视频将在此显示";
  $("playStream").hidden = true;
  const config = new Config({
    initialSettings: {
      [TextParameters.SignallingServerUrl]: signallingUrl(),
      [OptionParameters.StreamerId]: state.selectedStreamerId,
      [NumericParameters.WebRTCFPS]: state.requestedVideoFps ?? state.streamConfig?.pixel_streaming_fps ?? 90,
      // Match the UE launch setting; nullish fallback must preserve an explicit 0.
      [NumericParameters.MinQuality]: state.streamConfig?.pixel_streaming_encoder_min_quality ?? 60,
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
  state.relativeView = null;
  updateRelativeViewControl();
  state.pixelConfig = config;
  state.pixelStreaming = stream;
  const applyAvailableStreams = available => {
    if (state.pixelStreaming !== stream) return;
    const previous = state.selectedStreamerId;
    updatePreviewStreams(available);
    if (previous !== state.selectedStreamerId && available.includes(state.selectedStreamerId)) {
      config.setOptionSettingValue(OptionParameters.StreamerId, state.selectedStreamerId);
    }
  };
  state.previewAvailability = observePreviewAvailability(signallingUrl(), available => {
    if (state.pixelStreaming !== stream) return;
    state.previewAvailableIds = available;
    applyAvailableStreams(available);
  });
  let lastRelativeQuery = 0;
  stream.addResponseEventListener("relative-view", response => {
    if (state.pixelStreaming !== stream) return;
    let data;
    try { data = JSON.parse(response); } catch { return; }
    if (data?.type !== "BskRelativeView" || !["", "overview", "overview-fixed", "chaser", "target", "free"].includes(data.mode)) return;
    state.relativeView = data;
    if (["overview", "overview-fixed", "chaser", "target"].includes(data.mode)) $("relativeViewSelector").value = data.mode;
    updateRelativeViewControl();
  });
  stream.addEventListener("dataChannelOpen", () => {
    if (state.pixelStreaming === stream) requestRelativeView();
  });
  stream.addEventListener("dataChannelClose", () => {
    if (state.pixelStreaming !== stream) return;
    state.streamLive = false;
    if (state.freeCameraMode) setFreeCameraMode(false);
    $("frameState").textContent = "CHANNEL RECONNECTING";
    updateRelativeViewControl();
    schedulePixelStreamingReconnect();
  });
  workLight.reset();
  updateWorkLight();
  stream.addResponseEventListener("work-light", response => {
    if (state.pixelStreaming !== stream) return;
    workLight.receive(response);
    updateWorkLight();
  });
  stream.addResponseEventListener("preview-orbit", response => {
    if(state.pixelStreaming !== stream)return;
    try {
      const reply=JSON.parse(response);
      if(reply.type==='bsk_preview_orbit' && Number.isFinite(reply.yaw) && Number.isFinite(reply.pitch))
        setMessage(reply.yaw===0 && reply.pitch===0 ? '已恢复当前视角'
          : `展示视角偏转 ${reply.yaw.toFixed(1)}° / ${reply.pitch.toFixed(1)}°；Home 恢复，卫星姿态不变`);
    } catch (_) { /* Ignore unrelated renderer responses. */ }
  });
  stream.addEventListener("initialSettings", () => {
    queueMicrotask(() => {
      if (state.pixelStreaming === stream && state.requestedVideoFps != null) {
        config.setNumericSetting(NumericParameters.WebRTCFPS, state.requestedVideoFps);
      }
    });
  });
  window.__pixelStream = stream;
  stream.addEventListener("webRtcConnecting", () => {
    if (state.pixelStreaming !== stream) return;
    $("frameState").textContent = "CONNECTING";
    // Give negotiation its own deadline, independent of how long UE took to load.
    armPixelStreamingWatchdog(stream);
  });
  stream.addEventListener("streamerListMessage", (event) => {
    if (state.pixelStreaming !== stream) return;
    const available = event?.data?.messageStreamerList?.ids;
    if (Array.isArray(available)) {
      applyAvailableStreams(state.previewAvailableIds ?? available);
    }
    if (state.streamLive) return;
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
    previewOrbit.select(state.selectedStreamerId);
    previewOrbit.restore();
    requestRelativeView();
    // Media setup can precede the first decoded frame during a cold asset
    // load. Keep the waiting view until actual video arrives.
    if (!showReceivedVideo(stream)) {
      $("frameState").textContent = "WAITING FOR FRAMES";
      $("previewEmptyTitle").textContent = "等待视频首帧";
      $("previewEmptyHint").textContent = "首次载入模型时可能较慢，收到实际画面后会自动显示。";
      $("pixelStream").querySelector("video")?.requestVideoFrameCallback?.(() => showReceivedVideo(stream));
    }
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
    if (state.streamLive) showReceivedVideo(stream);
    // Discover manifests arriving after WebRTC and camera changes made by
    // another viewer. Reuse the SDK cadence instead of adding a timer.
    if (state.streamLive && Date.now() - lastRelativeQuery >= 2000) {
      lastRelativeQuery = Date.now();
      requestRelativeView();
    }
  });
  // Bound a genuinely stalled setup, but never recycle players on an absolute
  // 10-second cycle: UE startup and public ICE negotiation can take much longer.
  armPixelStreamingWatchdog(stream);
  monitorPixelStream(stream);
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

function transmitAction(action, deadman, allowReferenceRecovery = false) {
  if (!state.connected || !state.controlGranted || !state.ws || state.ws.readyState !== WebSocket.OPEN) {
    return false;
  }
  state.sequence += 1;
  state.ws.send(JSON.stringify({
    type: "operator_action",
    client_sequence: state.sequence,
    client_time_ns: String(BigInt(Date.now()) * 1000000n),
    deadman,
    active_arm: state.controlledArm,
    capture_radial_velocity: deadman && state.controlledArm==='capture' ? state.captureTool.radial || action.grip || 0 : 0,
    capture_axial_velocity: deadman && state.controlledArm==='capture' ? state.captureTool.axial : 0,
    ...(action.preparation ? {arm_preparation: action.preparation} : {}),
    allow_reference_recovery: allowReferenceRecovery && !deadman,
    end_effector_linear_speed_m_s: state.linearSpeed,
    end_effector_linear_velocity: action.linear,
    end_effector_angular_velocity: action.angular,
    angular_control_frame: action.angularFrame || "spacecraft_body",
    joint6_velocity: action.joint6 || 0,
    gripper_velocity: state.controlledArm==='capture' ? 0 : action.grip,
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
  updateRelativeViewControl();
  const viewport = $("viewport");
  const hintTitle = $("operationHintTitle");
  const hintDetail = $("operationHintDetail");
  const available = state.sceneReady && state.connected && state.canManageScene && !state.estopped && armPreparation.ready();
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
    const flight = state.mission?.stage === "flight";
    hintTitle.textContent = flight ? state.mission.active ? "交会自动控制" : "交会任务已结束" : "等待场景运行";
    hintDetail.textContent = flight ? "C 查看自由视角；可在右侧查看结果或接续遥操作" : "场景运行后点击画面进入操作";
  } else if (!state.connected) {
    hintTitle.textContent = "操作链路未连接";
    hintDetail.textContent = "等待后端 WebSocket 恢复";
  } else if (!state.canManageScene) {
    hintTitle.textContent = "当前场景由其他操作员创建";
    hintDetail.textContent = "只有场景创建者或管理员可以操作";
  } else if (state.estopped) {
    hintTitle.textContent = "急停已锁存";
    hintDetail.textContent = "先在右侧点击恢复控制";
  } else if (!armPreparation.ready()) {
    hintTitle.textContent = "请先到达操作姿态";
    hintDetail.textContent = "等待准备进度；中止后先复原场景再显式重试，实测稳定到位后才能操作与采集";
  } else if (!state.controlGranted) {
    hintTitle.textContent = "点击切换到当前操作页面";
    hintDetail.textContent = "同一用户的旧页面会自动退出操作";
  } else if (state.operationActive) {
    hintTitle.textContent = "操作模式已开启";
    hintDetail.textContent = "WASD / QE 控制，F/R 夹爪；按 C 进入自由视角，Esc 退出操作";
  } else {
    hintTitle.textContent = "点击画面进入操作";
    hintDetail.textContent = "中键拖动观察，Home 复位；左键进入机械臂操作";
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
  if (state.activePage && state.activePage !== "operate") return;
  if (!armPreparation.ready()) return setMessage("请等待机械臂实测稳定到位；准备已中止时，请先复原场景再显式重试");
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
  if (state.activePage && state.activePage !== "operate") return;
  if (!armPreparation.ready()) return setMessage("机械臂尚未准备到位");
  state.pressed.clear();
  state.operationActive = true;
  $("viewport").focus({ preventScroll: true });
  updateOperationUI();
  setMessage("已进入操作模式：键盘或手柄控制；按 C 进入自由视角");
}

function exitOperationMode(message = "已退出操作模式，点击实时画面可重新进入") {
  state.captureTool = {radial:0,axial:0};
  armPreparation.cancel("已停止准备运动");
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
  if (state.activePage && state.activePage !== "operate") return;
  // Poll diagnostics even outside operation mode; this never grants control.
  const gamepad = gamepadAction();
  if (armPreparation.tick()) return;
  if (state.operationActive && !armPreparation.ready()) { exitOperationMode("准备状态未就绪或遥测过期，已停止操作"); return; }
  if (!state.operationActive || state.freeCameraMode || !state.sceneReady || !state.canManageScene || state.estopped) return;
  if (!state.connected || !state.controlGranted || !state.ws || state.ws.readyState !== WebSocket.OPEN) return;
  const keyboard = keyboardAction();
  const gamepadActive = gamepad && [...gamepad.linear, ...gamepad.angular, gamepad.joint6 || 0, gamepad.grip].some((value) => Math.abs(value) > 0.01);
  const action = gamepadActive ? gamepad : keyboard;
  const motionActive = [...action.linear, ...action.angular, action.joint6 || 0, action.grip, state.captureTool?.radial || 0, state.captureTool?.axial || 0].some((value) => Math.abs(value) > 0.01);
  transmitAction(action, motionActive, !motionActive);
  $("directControl").classList.toggle("active", motionActive);
  $("directControl").textContent = motionActive
    ? "操作模式：正在运动（Esc 退出）"
    : "操作模式：松键可卸除卡滞目标；Esc 停止并退出";
  $("inputSource").textContent = action.source === "gamepad" ? "手柄" : "键盘";
  highlightKeys();
  const elapsed = state.lastAckAt ? performance.now() - state.lastAckAt : 0;
  $("latency").textContent = state.lastAckAt ? `ACK ${elapsed.toFixed(0)} ms` : "等待 ACK";
}

function updateMotionOutputs(twist, gripperVelocity) {
  ["vx", "vy", "vz", "wx", "wy", "wz"].forEach((id, index) => {
    $(id).textContent = Number(twist[index] || 0).toFixed(3);
  });
  $("grip").textContent = Number.isFinite(gripperVelocity) ? gripperVelocity.toFixed(3) : '—';
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
  if (state.sceneTransitionPending) return;
  if (state.stateReadPending && state.stateReadPending === state.stateRequestSequence) return;
  const sequence = ++state.stateRequestSequence;
  state.stateReadPending = sequence;
  try {
    const response = await apiRequest("/api/state", { cache: "no-store", signal: globalThis.AbortSignal?.timeout(10000) });
    const data = await response.json();
    if (response.ok === false) throw new Error(data.detail || "无法读取任务状态");
    if (sequence !== state.stateRequestSequence) return;
    if (!response.ok) throw new Error(`state ${response.status}`);
    if (state.activePage !== 'operate') {
      setOnline('backendDot', true);
      $('backendState').textContent = '平台已连接';
    }
    state.simulationConnected = data.simulation.connected;
    state.observationSceneId = data.simulation.latest_observation?.scene_instance_id || null;
    state.simulationResetting = data.simulation.resetting === true;
    state.resetSupported = data.simulation.reset_supported === true;
    setOnline("simDot", data.simulation.connected);
    $("simState").textContent = data.simulation.connected ? "仿真在线" : "仿真未连接";
    state.activeEpisode = data.active_episode;
    $("episodePath").textContent = data.episode_directory || "—";
    applySceneRuntime(data.scene_runtime || {});
    if (data.mission) {
      setOnline("backendDot", true);
      $("backendState").textContent = "后端在线";
      missionPanel.render(data.mission);
      const flightOnline = data.mission.stage === "flight" && data.mission.telemetry?.connected;
      setOnline("simDot", data.simulation.connected || flightOnline);
      $("simState").textContent = flightOnline ? "交会仿真在线" : data.simulation.connected ? "仿真在线" : "仿真未连接";
      updateOperationUI();
    }
    if (!state.simulationConnected || state.simulationResetting || !state.sceneReady) {
      clearSimulationTelemetry(data.mission);
      jointAngles.clear(state.simulationResetting ? "场景重置中" : "等待运行场景的遥测");
    }
    if (data.simulation.reset_error) setMessage(data.simulation.reset_error);
    if (data.active_episode && data.capture_sync?.dataset_error) {
      setMessage(`LeRobot 采集已出现错误，请结束本次采集：${data.capture_sync.dataset_error}`);
    } else if (data.capture_sync?.finalizing) {
      setMessage("正在等待末帧、编码视频并封口 LeRobot v3 数据集，请勿关闭服务");
    }
  } catch (_) {
    if (sequence !== state.stateRequestSequence) return;
    clearSimulationTelemetry();
    jointAngles.clear("连接中断，等待新遥测");
    setOnline("backendDot", false);
    if (state.activePage !== 'operate') $('backendState').textContent = '平台连接异常';
    if (state.mission) missionPanel.failed();
  } finally {
    if (state.stateReadPending === sequence) state.stateReadPending = false;
  }
}

function updateEpisodeUI() {
  const active = Boolean(state.activeEpisode);
  $("episodeBadge").textContent = active ? "● 正在采集" : "未采集";
  $("episodeBadge").classList.toggle("recording", active);
  $("startEpisode").disabled = active || !state.sceneReady || !state.connected || !state.canManageScene || !armPreparation.ready();
  $("successEpisode").disabled = !active;
  $("failureEpisode").disabled = !active;
}

async function startEpisode() {
  if (!state.sceneReady || !state.connected || !state.canManageScene || !armPreparation.ready()) {
    return setMessage("场景或机械臂尚未准备到位，不能开始采集");
  }
  const response = await apiRequest("/api/episodes/start", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ instruction: $("instruction").value }),
  });
  const data = await response.json();
  if (!response.ok) return setMessage(data.detail || "无法开始采集");
  state.activeEpisode = data.episode_id;
  setMessage(`开始 LeRobot v3 采集 ${data.episode_id}（${data.dataset_fps} FPS）`);
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
  setMessage(data.dataset_status === "complete"
    ? `LeRobot v3 数据集已完成：${data.dataset_frame_count} 帧，${data.episode_id}/${data.dataset_path}`
    : `采集未生成有效数据集（${data.dataset_status}）：${data.dataset_error || "没有完整样本"}`);
  await refreshState();
}

const previewOrbit = new PreviewOrbit({
  storage: window.sessionStorage,
  available: () => state.streamLive && !state.freeCameraMode,
  send: command => state.pixelStreaming?.emitCommand(command) === true,
  stopRobot: () => exitOperationMode("中键拖动观察；松开保持，Home 恢复当前视角"),
});
// Presentation offsets are separate from the body-fixed camera transforms.
$("viewport").addEventListener("pointerdown", event => {
  if(event.button!==1 || event.target.closest('button, select, .viewport-toolbar, .webrtc-stats, .main-companion'))return;
  event.preventDefault();event.stopImmediatePropagation();
  if(previewOrbit.begin(event.button,event.clientX,event.clientY))
    $("viewport").setPointerCapture(event.pointerId);
  else setMessage("请先连接视频；C 自由视角模式下请先按 Home 返回");
},true);
$("viewport").addEventListener("pointermove", event => {
  if(!previewOrbit.dragging)return;
  if(!(event.buttons&4)){previewOrbit.end();return;}
  event.preventDefault();event.stopImmediatePropagation();
  previewOrbit.move(event.clientX,event.clientY);
},true);
for(const name of ['pointerup','pointercancel','lostpointercapture'])
  $("viewport").addEventListener(name,()=>previewOrbit.end(),true);
$("viewport").addEventListener('auxclick',event=>{if(event.button===1)event.preventDefault();});
window.addEventListener('blur',()=>previewOrbit.end());
document.addEventListener('visibilitychange',()=>{if(document.hidden)previewOrbit.end();});
$("streamSelector").addEventListener('change',()=>previewOrbit.end());
$("viewport").title = '中键拖动当前视角，Home 复位；相机安装位姿不变';

$("viewport").addEventListener("pointerdown", (event) => {
  if (event.button !== 0) return;
  if (event.target.closest("button, select, .viewport-toolbar, .webrtc-stats, .main-companion")) return;
  if (state.freeCameraMode) {
    freeCamera.requestPointerLock();
    return;
  }
  enterOperationMode();
}, true);

window.addEventListener("keydown", (event) => {
  if (state.activePage && state.activePage !== "operate") return;
  if (event.code === "Escape") armPreparation.cancel();
  // The camera selector retains focus after selection. C is a viewport command,
  // not text entry in that selector; keep every actual editing field protected.
  const cameraShortcut = event.code === "KeyC" && event.target?.id === "streamSelector";
  const editing = !cameraShortcut && (["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable);
  if (editing) {
    // Typing into page forms must never reach UE keyboard shortcuts.
    event.stopImmediatePropagation();
    return;
  }
  if (event.code === "KeyC" || event.code === "Home") {
    event.preventDefault();
    event.stopImmediatePropagation(); // Do not toggle a second time in UE.
    if (event.repeat || !state.streamLive) return;
    if(event.code === 'Home') previewOrbit.reset();
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
  if (state.activePage && state.activePage !== "operate") return;
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
  if (state.activePage && state.activePage !== "operate") return;
  if (state.freeCameraMode || controlKeys.has(event.code) || event.code === "KeyC"
      || ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable) {
    event.stopImmediatePropagation();
  }
}, true);
window.addEventListener("blur", () => {
  armPreparation.cancel("页面失焦，已中止准备");
  if (state.operationActive || state.freeCameraMode) exitOperationMode("页面失去焦点，已停止输入并返回主视角");
});
document.addEventListener("visibilitychange", () => {
  if (document.hidden) armPreparation.cancel("页面隐藏，已中止准备");
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
$("sceneTemplate").addEventListener("change", configureSceneSelection);
$("randomizationProfile").addEventListener("change", configureSceneSelection);
$("stopScene").addEventListener("click", stopScene);
$("resetScene").addEventListener("click", resetScene);
$("startEpisode").addEventListener("click", startEpisode);
$("successEpisode").addEventListener("click", () => stopEpisode("success"));
$("failureEpisode").addEventListener("click", () => stopEpisode("failure"));
$("streamSelector").addEventListener("change", (event) => {
  state.selectedStreamerId = event.target.value;
  createPixelStream();
  $("viewport").focus({ preventScroll: true });
});
$("relativeViewSelector").addEventListener("change", event => {
  if (state.freeCameraMode) setFreeCameraMode(false);
  if (!requestRelativeView(event.target.value)) setMessage("视频控制通道尚未就绪，请稍后重试视角切换。");
  else setMessage(event.target.value === "overview"
    ? "总览保持双星真实位置和模型尺寸；标记用于辨认，自动取景时请以距离和比例尺判断尺度。"
    : event.target.value === "overview-fixed"
    ? "已固定总览尺度，两星会随实际接近在画面上靠拢。切回自动取景可重新放大；相机仍跟随双星中点。"
    : "近景用于查看单颗卫星。双星间距请以质心距离为准，切回总览可同时定位两颗卫星。");
});
$('controlledArm').addEventListener('change', event => {
  exitOperationMode('已切换控制臂；点击画面后控制所选机械臂');
  state.controlledArm = event.target.value;
  $('captureToolControls').hidden = state.controlledArm !== 'capture';
  jointAngles.clear('等待所选机械臂遥测');
});
// Image-derived display only. No sendAction, IK or renderer command is passed in.
const wristVisionOverlay = new WristVisionOverlay({
  viewport:$('viewport'),canvas:$('wristVisionCanvas'),panel:$('wristVisionPanel'),
  video:()=>$('pixelStream').querySelector('video'),request:apiRequest,
  getContext:()=>({selected:state.selectedStreamerId.endsWith('__teleop_camera_capture_wrist_cam'),
    visible:state.activePage==='operate' && !document.hidden,
    ready:state.streamLive && state.simulationConnected && state.sceneReady && !state.resetPending && !state.simulationResetting,
    orbitShifted:Math.abs(previewOrbit.yaw)>1e-6 || Math.abs(previewOrbit.pitch)>1e-6,
    session:state.renderSessionId||'',visualProbe:state.visualProbe===true}),
});
for (const button of document.querySelectorAll('[data-capture-axis]')) {
  button.addEventListener('pointerdown', event => {
    if(event.button!==0 || state.controlledArm!=='capture') return;
    if(!state.operationActive) {setMessage('请先点击实时画面进入操作，再按住抓夹按钮');return;}
    event.preventDefault();button.setPointerCapture(event.pointerId);
    state.captureTool[button.dataset.captureAxis]=Number(button.dataset.rate);
    sendAction();
  });
  for(const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,()=>{
    state.captureTool[button.dataset.captureAxis]=0;sendAction();
  });
}
// Share the main viewport's angle cache, but never send orbit on subscription.
const companionOrbit = new PreviewOrbit({
  storage: window.sessionStorage,
  available:()=>mainCompanion.live,
  send:command=>mainCompanion.stream?.emitCommand(command)===true,
  stopRobot:()=>exitOperationMode('中键调整主视口小窗；抓取相机保持原观察角度'),
});
companionOrbit.views=previewOrbit.views;
$('mainCompanion').addEventListener('pointerdown',event=>{
  if(event.button!==1)return;
  event.preventDefault();event.stopPropagation();
  companionOrbit.end();
  companionOrbit.key=state.streamConfig.pixel_streaming_streamer_id;
  [companionOrbit.yaw,companionOrbit.pitch]=previewOrbit.views.get(companionOrbit.key)||[0,0];
  if(companionOrbit.begin(1,event.clientX,event.clientY))$('mainCompanion').setPointerCapture(event.pointerId);
});
$('mainCompanion').addEventListener('pointermove',event=>{
  if(!(event.buttons&4)){companionOrbit.end();return;}
  companionOrbit.move(event.clientX,event.clientY);
  if(companionOrbit.dragging)previewOrbit.views.set(companionOrbit.key,[companionOrbit.yaw,companionOrbit.pitch]);
});
for(const event of ['pointerup','pointercancel','lostpointercapture'])$('mainCompanion').addEventListener(event,()=>companionOrbit.end());
window.addEventListener('blur',()=>companionOrbit.end());
$('mainCompanion').addEventListener('dblclick',()=>{
  exitOperationMode('已返回主视口');state.selectedStreamerId=state.streamConfig.pixel_streaming_streamer_id;
  $('streamSelector').value=state.selectedStreamerId;createPixelStream();
});
// Never pass companion input to robot controls in the parent viewport.
for(const name of ['pointerdown','pointermove','click','auxclick']) $('mainCompanion').addEventListener(name,event=>event.stopPropagation());
$("reconnectStream").addEventListener("click", () => {
  connectPixelStreaming();
});
$("videoFpsSelector").addEventListener("change", (event) => {
  if (selectVideoFps(event.target.value)) {
    setMessage(`视频目标 ${state.requestedVideoFps} FPS；实际帧率取决于网络、渲染和屏幕刷新率。此设置会影响共享 UE 实例。`);
  }
});
$("copyVideoDiagnostics").addEventListener("click", async () => {
  if (!state.videoDiagnostics?.length) {
    setMessage("视频连接后等待几秒，再复制诊断。");
    return;
  }
  try {
    await navigator.clipboard.writeText(JSON.stringify({ samples: state.videoDiagnostics }, null, 2));
    setMessage("已复制最近 60 次视频采样（不含账号、令牌和 IP 地址），可用于排查卡顿。");
  } catch (_) {
    setMessage("浏览器未允许复制，请允许此 HTTPS 网站访问剪贴板后重试。");
  }
});
$("playStream").addEventListener("click", () => {
  if (["decode-stalled", "playback-stalled", "media-error"].includes(state.playbackHealth)) {
    connectPixelStreaming();
    return;
  }
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
  const values=await askDialog({title:'修改密码',message:'修改后需要重新登录。',fields:[{name:'current_password',label:'当前密码',type:'password',autocomplete:'current-password'},{name:'new_password',label:'新密码',type:'password',minLength:8,autocomplete:'new-password'}],confirmLabel:'修改密码'});
  if(!values||!state.currentUser)return;
  try{await platformApi('/api/auth/change-password',{method:'POST',body:JSON.stringify(values)});shutdownAuthenticatedApp();showLogin('密码已修改，请重新登录');}
  catch(error){await askDialog({title:'修改失败',message:error.message,confirmLabel:'知道了'});}
});
$("logoutButton").addEventListener("click", async () => {
  await apiRequest("/api/auth/logout", { method: "POST" });
  shutdownAuthenticatedApp();
  showLogin("已退出登录");
});

async function platformApi(url,options={}) {
 const response=await apiRequest(url,{...options,cache:'no-store',headers:{...(options.body?{'Content-Type':'application/json'}:{}),...options.headers}});
 const data=await readApiResponse(response);
 if(!response.ok){const detail=data.detail;throw new Error(typeof detail==='string'?detail:Array.isArray(detail)?detail.map(item=>item.msg).join('；'):'请求失败，请稍后重试');}
 return data;
}
function leaveWorkbench(){
 exitOperationMode('已切换页面，遥操作输入已停止');state.operationRequested=false;
 if(state.ws){state.ws.onopen=null;state.ws.onclose=null;state.ws.onmessage=null;state.ws.close();state.ws=null;}
 state.connected=false;state.controlGranted=false;disposePixelStream();
 setOnline('backendDot',true);$('backendState').textContent='平台已连接';
}
const platform=createPlatformShell({onNavigate:async(page,params)=>{
 const previous=state.activePage;
 if(previous==='operate'&&page!=='operate')leaveWorkbench();
 Object.values(pages).forEach(view=>view.deactivate());state.activePage=page;
 document.querySelectorAll('[data-platform-page]').forEach(view=>{view.hidden=view.dataset.platformPage!==page;});
 $('workbenchFooter').hidden=page!=='operate';
 if(page==='operate'){
  await connectPixelStreaming();
  if(state.currentUser&&state.activePage==='operate'){if(state.streamConfig)connect();await refreshState();}
 }else await pages[page].activate(params);
}});
const pages={
 collection:createCollectionPage($('collectionPage'),{api:platformApi}),
 datasets:createDatasetPage($('datasetsPage'),{api:platformApi,navigate:(path)=>platform.navigate(path)}),
 users:createUsersPage($('usersPage'),{api:platformApi,getUser:()=>state.currentUser,onUserChange:applyAuthenticatedUser}),
};

initialize();

// Read-only diagnostics for local-browser / remote-desktop troubleshooting.
window.__gamepadDiagnostics = () => ({
  secureContext: window.isSecureContext, pageVisible: !document.hidden,
  pageFocused: document.hasFocus(), snapshot: state.gamepadSnapshot
    ? JSON.parse(JSON.stringify(state.gamepadSnapshot)) : null,
  operationStatus: gamepadControlBlockReason(state),
});
