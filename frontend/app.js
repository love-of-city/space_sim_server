const state = {
  ws: null,
  connected: false,
  controlGranted: false,
  activeEpisode: null,
  sequence: 0,
  pressed: new Set(),
  estopped: false,
  lastAckAt: 0,
  linearSpeed: 0.05,
};

const $ = (id) => document.getElementById(id);
const controlKeys = new Set(["KeyA", "KeyD", "KeyW", "KeyS", "KeyQ", "KeyE", "KeyR", "KeyF", "ShiftLeft", "ShiftRight", "Escape"]);

function setMessage(text) { $("message").textContent = text; }
function setOnline(id, online) { $(id).classList.toggle("online", online); }

function connect() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  state.ws = new WebSocket(`${scheme}://${location.host}/ws/operator`);
  state.ws.onopen = () => {
    state.connected = true;
    setOnline("backendDot", true);
    $("backendState").textContent = "后端在线";
    setMessage("操作链路已连接");
  };
  state.ws.onclose = () => {
    state.connected = false;
    state.controlGranted = false;
    setOnline("backendDot", false);
    $("backendState").textContent = "后端断开";
    $("controlAuthority").textContent = "只读";
    setMessage("连接中断，动作已自动归零");
    setTimeout(connect, 1000);
  };
  state.ws.onmessage = (event) => {
    const message = JSON.parse(event.data);
    if (message.type === "session") {
      state.controlGranted = message.control_granted;
      state.activeEpisode = message.active_episode;
      applyControlLimits(message.control_limits);
      $("controlAuthority").textContent = state.controlGranted ? "控制已授权" : "只读观察";
      updateEpisodeUI();
    } else if (message.type === "control_granted") {
      state.controlGranted = true;
      $("controlAuthority").textContent = "控制已授权";
      setMessage("旧页面已断开，控制权已转移到当前页面");
    } else if (message.type === "observation") {
      const obs = message.payload;
      setOnline("simDot", true);
      $("simState").textContent = "仿真在线";
      $("simTime").textContent = `${(Number(obs.sim_time_ns) / 1e9).toFixed(3)} s`;
      $("actionSequence").textContent = obs.applied_action_sequence;
      updateMotionOutputs(obs.end_effector_twist_body || [], obs.joint_velocity_rad_s?.[5] || 0);
      const position = obs.end_effector_position_body_m || [];
      $("toolPosition").textContent = position.length === 3 ? position.map((value) => Number(value).toFixed(3)).join(", ") : "—";
      $("jacobianRank").textContent = `${obs.jacobian_rank ?? "—"} / 5`;
    } else if (message.type === "action_ack") {
      state.lastAckAt = performance.now();
      $("actionSequence").textContent = message.server_sequence;
    } else if (message.type === "action_rejected") {
      setMessage(`动作被拒绝：${message.reason}`);
    }
  };
}

async function connectPixelStreaming() {
  try {
    const response = await fetch("/api/client-config", { cache: "no-store" });
    if (!response.ok) throw new Error(`config ${response.status}`);
    const config = await response.json();
    const protocol = location.protocol === "https:" ? "https:" : "http:";
    const playerUrl = new URL(`${protocol}//${location.hostname}:${config.pixel_streaming_player_port}/player.html`);
    playerUrl.searchParams.set("StreamerId", config.pixel_streaming_streamer_id);
    playerUrl.searchParams.set("AutoConnect", "true");
    playerUrl.searchParams.set("AutoPlayVideo", "true");
    playerUrl.searchParams.set("HoveringMouse", "false");
    const stream = $("pixelStream");
    stream.src = playerUrl.toString();
    stream.onload = () => {
      stream.style.display = "block";
      $("previewEmpty").style.display = "none";
      // iframe 加载只表示官方播放器就绪；实际 WebRTC 连接状态由播放器自行维护。
      $("frameState").textContent = "WEBRTC PLAYER";
      $("frameState").style.color = "var(--accent)";
    };
  } catch (_) {
    $("frameState").textContent = "WEBRTC OFFLINE";
    setTimeout(connectPixelStreaming, 1500);
  }
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
    angular[2] = value("KeyD", "KeyA");
  } else {
    linear[0] = value("KeyW", "KeyS");
    linear[1] = value("KeyD", "KeyA");
    linear[2] = value("KeyQ", "KeyE");
  }
  $("motionMode").textContent = rotationMode ? "旋转 RPY" : "平移 XYZ";
  return { linear, angular, grip: value("KeyR", "KeyF"), source: "keyboard" };
}

function gamepadAction() {
  const gamepad = [...navigator.getGamepads()].find(Boolean);
  if (!gamepad) return null;
  const dz = (value) => Math.abs(value || 0) < 0.12 ? 0 : -(value || 0);
  const buttons = gamepad.buttons;
  return {
    linear: [dz(gamepad.axes[1]), dz(gamepad.axes[0]), (buttons[7]?.value || 0) - (buttons[6]?.value || 0)],
    angular: [(buttons[1]?.value || 0) - (buttons[0]?.value || 0), dz(gamepad.axes[3]), dz(gamepad.axes[2])],
    grip: (buttons[3]?.value || 0) - (buttons[2]?.value || 0),
    source: "gamepad",
  };
}

function sendAction() {
  if (!state.connected || !state.controlGranted || state.ws.readyState !== WebSocket.OPEN) return;
  const gamepad = gamepadAction();
  const keyboard = keyboardAction();
  const gamepadActive = gamepad && [...gamepad.linear, ...gamepad.angular, gamepad.grip].some((value) => Math.abs(value) > 0.01);
  let action = gamepadActive ? gamepad : keyboard;
  let motionActive = [...action.linear, ...action.angular, action.grip].some((value) => Math.abs(value) > 0.01);
  if (state.estopped) {
    action = { linear: [0, 0, 0], angular: [0, 0, 0], grip: 0, source: action.source };
    motionActive = false;
  }
  state.sequence += 1;
  state.ws.send(JSON.stringify({
    type: "operator_action",
    client_sequence: state.sequence,
    client_time_ns: String(BigInt(Date.now()) * 1000000n),
    deadman: motionActive,
    end_effector_linear_speed_m_s: state.linearSpeed,
    end_effector_linear_velocity: action.linear,
    end_effector_angular_velocity: action.angular,
    gripper_velocity: action.grip,
    input_source: action.source,
  }));
  $("directControl").classList.toggle("active", motionActive);
  $("directControl").textContent = state.estopped ? "急停已锁存" : motionActive ? "直接控制：正在运动" : "直接控制：等待输入";
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
    const response = await fetch("/api/state", { cache: "no-store" });
    const data = await response.json();
    setOnline("simDot", data.simulation.connected);
    $("simState").textContent = data.simulation.connected ? "仿真在线" : "仿真未连接";
    state.activeEpisode = data.active_episode;
    $("episodePath").textContent = data.episode_directory || "—";
    updateEpisodeUI();
  } catch (_) {
    setOnline("backendDot", false);
  }
}

function updateEpisodeUI() {
  const active = Boolean(state.activeEpisode);
  $("episodeBadge").textContent = active ? "● 正在采集" : "未采集";
  $("episodeBadge").classList.toggle("recording", active);
  $("startEpisode").disabled = active;
  $("successEpisode").disabled = !active;
  $("failureEpisode").disabled = !active;
}

async function startEpisode() {
  const response = await fetch("/api/episodes/start", {
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
  const response = await fetch("/api/episodes/stop", {
    method: "POST", headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ outcome }),
  });
  const data = await response.json();
  if (!response.ok) return setMessage(data.detail || "无法结束采集");
  state.activeEpisode = null;
  setMessage(`采集已结束：${outcome}，共 ${data.step_count} 步`);
  await refreshState();
}

window.addEventListener("keydown", (event) => {
  const editing = ["INPUT", "TEXTAREA", "SELECT"].includes(event.target?.tagName) || event.target?.isContentEditable;
  if (editing && event.code !== "Escape") return;
  if (!controlKeys.has(event.code) || event.repeat) return;
  event.preventDefault();
  if (event.code === "Escape") {
    state.estopped = true;
    state.pressed.clear();
    $("estop").textContent = "恢复控制";
    setMessage("急停已锁存；点击“恢复控制”后才能继续操作");
    return;
  }
  state.pressed.add(event.code);
});
window.addEventListener("keyup", (event) => { state.pressed.delete(event.code); });
window.addEventListener("blur", () => state.pressed.clear());
document.addEventListener("visibilitychange", () => { if (document.hidden) state.pressed.clear(); });
$("estop").addEventListener("click", () => {
  state.estopped = !state.estopped;
  state.pressed.clear();
  $("estop").textContent = state.estopped ? "恢复控制" : "立即停止输出（Esc）";
  setMessage(state.estopped ? "急停已锁存；点击“恢复控制”后才能继续操作" : "控制已恢复，按键可以直接运动");
});
$("linearSpeed").addEventListener("input", (event) => {
  state.linearSpeed = Number(event.target.value);
  $("linearSpeedValue").textContent = `${state.linearSpeed.toFixed(2)} m/s`;
});
$("startEpisode").addEventListener("click", startEpisode);
$("successEpisode").addEventListener("click", () => stopEpisode("success"));
$("failureEpisode").addEventListener("click", () => stopEpisode("failure"));

connect();
connectPixelStreaming();
refreshState();
setInterval(sendAction, 33);
setInterval(refreshState, 1500);
