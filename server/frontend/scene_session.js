// Scene start/reset/stop and stale-response rejection share one transition state.
export function createSceneSession({ state, $, ZERO_START_PROFILE, initialJoints, armPreparation,
  apiRequest, readApiResponse, applySceneRuntime, setMessage, exitOperationMode, updateEpisodeUI,
  clearWristWrench, jointAngles, setOnline, refreshState: refreshOverride,
  missionPanel, selectedSceneTemplate = () => null, getInitialJointCatalog = () => null,
  clearSimulationTelemetry = clearWristWrench, updateOperationUI = () => {},
}) {
  const refresh = refreshOverride || refreshState;
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
      template_id: selectedSceneTemplate()?.id || $("sceneTemplate").value,
      randomization_profile: $("randomizationProfile").value,
      randomize_orbit_phase: $("sceneRandomizeOrbitPhase").checked,
      seed: seedText === "" ? null : Number(seedText),
      simulation_rate: state.sceneDefaults.simulation_rate,
      capture_rate_hz: state.sceneDefaults.capture_rate_hz,
      ik_rate_hz: state.sceneDefaults.ik_rate_hz,
      dataset_capture: $("sceneDatasetCapture").checked,
      sunlight_intensity_scale: sunlightScale,
      wrist_camera_mode: $("sceneWristCameraMode").value,
      randomize_overview_camera: $("sceneRandomizeOverviewCamera").checked,
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
      if (getInitialJointCatalog()?.templates.find(item => item.id === request.template_id)?.library_kind === "rendezvous") {
        setMessage("场景库交会任务已启动，正在连接系统交会画面");
        await refresh();
        return;
      }
      applySceneRuntime({ ...data, active: true, instance: data });
      armPreparation.armAutoStart?.(data.instance_id);
      setMessage(`已生成场景 ${data.instance_id}，Seed ${data.seed}，正在启动 UE`);
    } catch (error) {
      setMessage(error.message || "无法启动场景");
    } finally {
      state.sceneTransitionPending = false;
      await refresh();
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
      await refresh();
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
      await refresh();
    }
  }

  async function refreshState() {
    if (state.sceneTransitionPending) return;
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
      applySceneRuntime(data.scene_runtime || {}, data.mission);
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

  return { startScene, resetScene, stopScene, refreshState };

}
