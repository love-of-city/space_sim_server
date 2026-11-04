import { createApiClient } from "./api_client.js";
import { createVideoDiagnostics } from "./video_diagnostics.js";
import { createPixelStreamSession } from "./pixel_stream_session.js";
import { createSceneSession } from "./scene_session.js";
import { createOperatorSession } from "./operator_session.js";
import { WorkLight } from "./work_light.js";
import { groupSceneTemplates, libraryFamilyId, resolveSceneTask } from './library_templates.js';
import { createStereoVisionPanel } from "./stereo_vision.js";
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
import {createTaskCatalog} from './task_catalog.js';
import {createCollectionPage} from './pages/collection.js';
import {createDatasetPage} from './pages/datasets.js';
import {createUsersPage} from './pages/users.js';
import {askDialog, userLabel} from './ui.js';
import { FreeCameraController } from "./free_camera.js";
import { GamepadInput, gamepadControlBlockReason } from "./gamepad_input.js";
import { referenceProtectionLabel, dynamicsTimingLabel } from "./control_status.js";
import { createMissionPanel } from "./mission_panel.js";

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
    "relativeDistance", "relativeVelocity", "relativeAngularVelocity", "captureToolStatus"]) {
    $(id).textContent = "—";
  }
  $("motionSpeedStatus").textContent = "等待当前场景的速度测量";
  $("motionSpeedDetail").textContent = "场景就绪后显示实测速度。";
  $("motionSpeedDiagnostic").className = "motion-speed-diagnostic";
  state.renderSessionId = "";
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
let visibleSceneTemplates = [];
function selectedSceneTemplate() {
  return resolveSceneTask(initialJointCatalog?.templates || [], $('sceneTemplate').value, $('sceneTaskProfile').value);
}
function configureLibraryTasks(preferredId) {
  const family = visibleSceneTemplates.find(item => item.id === $('sceneTemplate').value);
  const selector = $('sceneTaskProfile');
  $('sceneTaskProfileLabel').hidden = !family?.library_family;
  if (!family?.library_family) return;
  const previous = preferredId || selector.value;
  if (selector.dataset.familyId !== family.id) {
    selector.replaceChildren(...family.tasks.map(task => new Option(task.label.split(' / ').at(-1), task.id)));
    selector.dataset.familyId = family.id;
  }
  selector.value = family.tasks.some(task => task.id === previous) ? previous : family.tasks[0].id;
}
function configureInitialJoints() {
  const zeroStart = [ZERO_START_PROFILE, "teleop-zero-prepare-v2"].includes($("randomizationProfile").value);
  $("legacyInitialJointSettings").hidden = zeroStart;
  if (zeroStart) $("sceneCustomInitialJoints").checked = false;
  armPreparation.configure(selectedSceneTemplate(), $("randomizationProfile").value);
  initialJoints.configure(
    selectedSceneTemplate(),
    initialJointCatalog?.initial_arm_presets_deg?.[$("randomizationProfile").value],
  );
}
function configureSceneSelection() {
  configureLibraryTasks();
  const template = selectedSceneTemplate();
  if (template?.library_scene) {
    $("randomizationProfile").value = template.default_randomization_profile;
    $("sceneRandomizeOrbitPhase").checked = false;
  }
  const overviewSupported = ["sarm-task-box-20261012", "sarm-task-box-20261012-sdf"].includes($("sceneTemplate").value);
  $("sceneRandomizeOverviewCamera").disabled = !overviewSupported;
  if (!overviewSupported) $("sceneRandomizeOverviewCamera").checked = false;
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

function applyAuthenticatedUser(user) {
  state.currentUser = user;
  document.body.classList.remove("auth-pending", "auth-required");
  $("authGate").hidden = true;
  $("currentUsername").textContent = userLabel(user);
  $("currentRole").textContent = user.role === "admin" ? "管理员" : "操作员";
  platform.setUser(user);
}

function shutdownAuthenticatedApp() {
  stereoVisionPanel.setOpen(false);
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
    visibleSceneTemplates = groupSceneTemplates(catalog.templates);
    template.replaceChildren(...visibleSceneTemplates.map((item) => new Option(item.label, item.id)));
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
    $("sceneRandomizeOverviewCamera").checked = defaults.randomize_overview_camera === true;
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

function applySceneRuntime(runtime = {}, mission = state.mission) {
  const previousPhase = state.scenePhase;
  // The arm executor retains its last instance after stopping. During flight
  // it is not the current task's scene, even on the first state poll.
  const flight = mission?.stage === "flight";
  const phase = flight ? "idle" : runtime.phase || "idle";
  const active = !flight && Boolean(runtime.active);
  const enabled = runtime.enabled !== false;
  const instance = flight ? {} : runtime.instance || {};
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
  $('sceneTaskProfile').disabled = active || Boolean(mission?.active);
  $("resetScene").disabled = !state.sceneReady || !state.simulationConnected
    || !state.resetSupported || !state.canManageScene || Boolean(state.activeEpisode);
  $("resetScene").textContent = state.resetPending || state.simulationResetting ? "正在复原…" : "一键复原初始位置";
  $("resetScene").title = state.activeEpisode ? "请先结束当前采集，再重置状态"
    : "恢复机械臂、夹爪、目标物和仿真时间到本次场景初态；不是规划机械臂返航轨迹";
  ["sceneTemplate", "randomizationProfile", "sceneSeed", "sceneRandomizeOrbitPhase", "sceneDatasetCapture", "sceneSunlightIntensity", "sceneWristCameraMode", "sceneRandomizeOverviewCamera"].forEach((id) => {
    $(id).disabled = active;
  });
  if (!["sarm-task-box-20261012", "sarm-task-box-20261012-sdf"].includes($("sceneTemplate").value)) $("sceneRandomizeOverviewCamera").disabled = true;
  $("linearSpeed").disabled = !state.sceneReady;
  $("estop").disabled = !state.sceneReady;
  updateOperationUI();
  $("sceneInstanceId").textContent = instance.instance_id || "—";
  $("sceneInstanceSeed").textContent = instance.seed ?? "—";
  $("sceneInstanceTarget").textContent = !instance.instance_id ? "—" : instance.template_id === "sarm-dual-capture26" ? "双臂 SARM + 地面验证星" : instance.template_id?.startsWith("sarm-task-box-20261012") ? "新版 SARM 完整任务盒 + 地面验证星" : instance.template_id === "sarm-ground-validation-self-collision-grasp" ? "地面验证星（粗碰撞体·内部碰撞）" : instance.template_id === "sarm-ground-validation-mesh-grasp" ? "地面验证星（高精度三角网格·实验）" : instance.template_id === "sarm-ground-validation-grasp" ? "地面验证星（粗碰撞盒·无内部碰撞）" : "原小方块目标";
  const instanceSunlight = instance.environment?.lighting?.sunlight_intensity_scale ?? 1;
  $("sceneInstanceSunlight").textContent = instance.instance_id ? `${instanceSunlight} 倍` : "—";
  const instanceOrbitPhase = instance.environment?.orbit?.true_anomaly_deg ?? 180;
  const randomizeOrbitPhase = instance.randomize_orbit_phase === true;
  $("sceneInstanceOrbitPhase").textContent = instance.rendezvous_handoff ? "继承交会终态"
    : instance.instance_id ? `${Number(instanceOrbitPhase).toFixed(2)}°` : "—";
  $("sceneInstanceOverviewCamera").textContent = !instance.instance_id ? "—"
    : instance.overview_camera ? `${instance.overview_camera.preset_id} · ${instance.overview_camera.label}` : "原始固定机位";
  if (active && instance.instance_id) {
    if (instance.template_id) {
      const taskId = instance.library_scene?.id;
      $("sceneTemplate").value = libraryFamilyId(taskId) || instance.template_id;
      configureLibraryTasks(taskId);
    }
    if (instance.capture_target?.runtime_warning) $("sceneCollisionWarning").textContent = instance.capture_target.runtime_warning;
    if (instance.randomization_profile) $("randomizationProfile").value = instance.randomization_profile;
    $("sceneSunlightIntensity").value = String(instanceSunlight);
    $("sceneWristCameraMode").value = instance.wrist_camera_mode || "rgb";
    $("sceneRandomizeOverviewCamera").checked = instance.randomize_overview_camera === true;
    $("sceneRandomizeOrbitPhase").checked = randomizeOrbitPhase;
  }
  configureInitialJoints();
  initialJoints.setRuntime(active, instance);
  armPreparation.setRuntime(active ? instance : null, state.sceneReady);
  if (["failed", "stopping", "stopped"].includes(phase)) armPreparation.cancel("场景停止或失败，请显式重试");
  $("sceneParameters").textContent = instance.randomization
    ? JSON.stringify({ capture_target: instance.capture_target || {}, wrist_camera_mode: instance.wrist_camera_mode || "rgb", randomize_overview_camera: instance.randomize_overview_camera === true, overview_camera: instance.overview_camera || null, randomize_orbit_phase: randomizeOrbitPhase, environment: instance.environment || {}, randomization: instance.randomization }, null, 2)
    : "尚未生成实例";
  if (!flight && runtime.error) setMessage(`场景失败：${runtime.error}`);
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

const { apiRequest, readApiResponse, platformApi } = createApiClient({
  state, shutdownAuthenticatedApp, showLogin,
});
const missionPanel = createMissionPanel({$, state, platformApi, refresh: () => refreshState(),
  message: setMessage, configureScene: configureSceneSelection,
  reconnect: () => { if (!state.streamLive) connectPixelStreaming(); }});
const previewOrbit = new PreviewOrbit({
  storage: window.sessionStorage,
  available: () => state.streamLive && !state.freeCameraMode,
  send: command => state.pixelStreaming?.emitCommand(command) === true,
  stopRobot: () => exitOperationMode("中键拖动观察；松开保持，Home 恢复当前视角"),
});
const diagnostics = createVideoDiagnostics({ state, $, NumericParameters });
const { selectVideoFps } = diagnostics;
const { connectPixelStreaming, disposePixelStream, createPixelStream, signallingUrl } = createPixelStreamSession({
  state, $, apiRequest, setFreeCameraMode, setPixelStreamingInputEnabled, setMessage, diagnostics,
  mainCompanion, workLight, updateWorkLight, requestRelativeView, updateRelativeViewControl, previewOrbit,
  Config, Flags, OptionParameters, NumericParameters, PixelStreaming, TextParameters,
});
const { startScene, resetScene, stopScene, refreshState } = createSceneSession({
  state, $, ZERO_START_PROFILE, initialJoints, armPreparation, apiRequest, readApiResponse,
  applySceneRuntime, setMessage, exitOperationMode, updateEpisodeUI, clearWristWrench, jointAngles, setOnline,
  missionPanel, selectedSceneTemplate, getInitialJointCatalog: () => initialJointCatalog, clearSimulationTelemetry, updateOperationUI,
});
const { connect } = createOperatorSession({
  state, $, clearWristWrench, setOnline, jointAngles, updateOperationUI, updateEpisodeUI,
  setMessage, exitOperationMode, applyAuthenticatedUser, applyControlLimits, completeEnterOperationMode,
  armPreparation, updateMotionOutputs, formatMotionSpeedDiagnostics, formatWristWrench,
  referenceProtectionLabel, dynamicsTimingLabel, clearSimulationTelemetry,
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
$("sceneTaskProfile").addEventListener("change", configureSceneSelection);
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
const stereoVisionPanel = createStereoVisionPanel({
  button: $('stereoVisionToggle'), panel: $('stereoVisionPanel'), status: $('stereoVisionStatus'),
  images: {left: $('stereoVisionLeft'), right: $('stereoVisionRight')}, request: apiRequest,
  visible: () => state.currentUser && state.activePage === 'operate' && !document.hidden
    && state.sceneReady && !state.resetPending && !state.simulationResetting,
});
window.addEventListener('pagehide', () => stereoVisionPanel.setOpen(false));
const wristVisionOverlay = new WristVisionOverlay({
  viewport:$('viewport'),canvas:$('wristVisionCanvas'),panel:$('wristVisionPanel'),
  video:()=>$('pixelStream').querySelector('video'),request:apiRequest,
  getContext:()=>({selected:state.selectedStreamerId.endsWith('__teleop_camera_capture_wrist_cam'),
    visible:state.activePage==='operate' && !document.hidden,
    ready:state.streamLive && state.simulationConnected && state.sceneReady && !state.resetPending && !state.simulationResetting,
    orbitShifted:Math.abs(previewOrbit.yaw)>1e-6 || Math.abs(previewOrbit.pitch)>1e-6,
    session:state.renderSessionId||''}),
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
const taskCatalog=createTaskCatalog(platformApi);
const pages={
 collection:createCollectionPage($('collectionPage'),{api:platformApi,taskCatalog}),
 datasets:createDatasetPage($('datasetsPage'),{api:platformApi,taskCatalog,navigate:(path)=>platform.navigate(path)}),
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
