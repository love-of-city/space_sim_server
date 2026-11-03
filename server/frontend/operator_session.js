// Authenticated operator WebSocket lifecycle, control handoff and live telemetry.
export function createOperatorSession({ state, $, clearWristWrench, setOnline, jointAngles,
  updateOperationUI, updateEpisodeUI, setMessage, exitOperationMode, applyAuthenticatedUser,
  applyControlLimits, completeEnterOperationMode, armPreparation, updateMotionOutputs,
  formatMotionSpeedDiagnostics, formatWristWrench, referenceProtectionLabel, dynamicsTimingLabel,
  location = globalThis.location, WebSocket = globalThis.WebSocket,
  performance = globalThis.performance, setTimeout = globalThis.setTimeout,
}) {
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
      clearWristWrench();
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
        handleObservation(message.payload);
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

  function handleObservation(obs) {
    const demo = obs.fastener_demo;
    $("fastenerDemoStatus").hidden = !demo;
    if (demo) {
      const phase = { settle: "松开等待", tighten: "正在拧紧", hold_tight: "压紧保持", loosen: "正在拧松", hold_loose: "松开保持" }[demo.phase] || demo.phase;
      $("fastenerDemoStatus").textContent = `防脱演示 · ${phase} · 行程 ${Number(demo.travel_mm).toFixed(2)} mm · 转角 ${Number(demo.angle_deg).toFixed(1)}°`;
    }
    jointAngles.update(obs);
    armPreparation.update(obs.arm_preparation, obs);
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
    $("dynamicsTiming").textContent = dynamicsTimingLabel(obs);
    const wheels = obs.reaction_wheels;
    $("attitudeMode").textContent = !attitude ? "—" : !attitude.enabled ? "控制关闭" :
      !attitude.reference_initialized ? "初始化" : attitude.saturated ? "惯性保持 · 饱和" : "惯性保持";
    $("attitudeError").textContent = attitude ? `${(attitude.attitude_error_angle_rad * 180 / Math.PI).toFixed(3)} °` : "—";
    $("bodyRate").textContent = attitude ? attitude.angular_velocity_body_rad_s.map(v => Number(v).toFixed(4)).join(", ") + " rad/s" : "—";
    $("wheelSpeeds").textContent = wheels ? wheels.speed_rad_s.map(v => (v * 60 / (2 * Math.PI)).toFixed(1)).join(", ") + " rpm" : "—";
    $("wheelTorques").textContent = wheels ? wheels.applied_motor_torque_nm.map(v => Number(v).toFixed(3)).join(", ") + " N·m" : "—";
  }

  return { connect, handleObservation };

}
