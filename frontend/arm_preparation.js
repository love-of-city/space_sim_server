import { parseInitialJointAngles } from './initial_joint_angles.js';

export const DEFAULT_OPERATING_DEG = [0, -67.6, -86.6, 143.2, -85.5, 0];
export const ZERO_START_PROFILE = 'teleop-zero-prepare-v1';
export const AUTO_PREPARE_PROFILE = 'teleop-zero-prepare-v2';
export function parseOperatingAngles(values, limits) {
  try { return parseInitialJointAngles(values, limits); }
  catch (error) { throw new Error(error.message.replaceAll('初始角度', '操作角度')); }
}
const labels = {waiting:'等待准备', arming:'等待机械臂稳定停止', checking:'旧版本正在检查路径', moving:'正在到达操作姿态',
  pausing:'暂停并重新检查路径', settling:'等待实测稳定到位', ready:'已到位，可以操作', failed:'准备失败', cancelled:'已中止，请显式重试', legacy:'旧场景：保留原初始化方式'};
const activeStatuses = ['arming', 'checking', 'moving', 'settling', 'pausing'];

export function createArmPreparation({fields, startButton, cancelButton, defaultButton, status, getContext, send, cancelMotion, activateControl, message, changed}) {
  const inputs = [...fields.querySelectorAll('input')];
  let limits = null, pending = null, waitingControl = null, requestedAt = 0, observation = null, observedAt = 0;
  let instanceId = null, required = false, selectedProfile = AUTO_PREPARE_PROFILE, runtimeProfile = null;
  let target = null, measured = null, autoIntent = null, halted = '', wasReady = false;
  let controlRequested = false;
  const armedInstances = new Set();
  const autoMode = () => (instanceId ? runtimeProfile : selectedProfile) === AUTO_PREPARE_PROFILE
    || observation?.strategy === 'validated-waypoints-v1';
  const legacyHeld = () => !!instanceId && required && !autoMode();
  const fill = values => inputs.forEach((input, index) => {
    input.value = String(legacyHeld() && (index === 0 || index === 5) ? 0 : values[index]);
  });
  fill(DEFAULT_OPERATING_DEG);
  const read = () => parseOperatingAngles(inputs.map(input => input.value), limits);
  const fresh = () => !!observation && performance.now() - observedAt < 1500;
  const atZero = () => fresh() && Array.isArray(measured) && measured.length === 6
    && measured.every(value => typeof value === 'number' && Number.isFinite(value) && Math.abs(value) <= 0.1 * Math.PI / 180);
  function ready() {
    if (!required) return true;
    return getContext().connected && getContext().sceneReady && !halted && !pending && !waitingControl && !autoIntent && observation?.ready === true
      && (!autoMode() || observation?.status === 'ready') && fresh();
  }
  function render() {
    const context = getContext();
    const running = !!pending || !!waitingControl || !!autoIntent || (!halted && activeStatuses.includes(observation?.status));
    startButton.hidden = cancelButton.hidden = !required;
    startButton.textContent = autoMode() ? '重试零位展开' : '到达操作姿态';
    startButton.disabled = !required || !context.sceneReady || !context.connected || !context.canManageScene
      || context.estopped || document.hidden || running || !limits || !fresh() || (autoMode() && (ready() || !atZero()));
    startButton.title = autoMode() && !atZero() ? '仅允许实测六轴仍在零位时重试；请先复原场景再展开' : '';
    cancelButton.disabled = !running || !context.canManageScene;
    inputs.forEach((input, index) => {
      const held = legacyHeld() && (index === 0 || index === 5);
      const locked = autoMode() && !!instanceId;
      input.disabled = held || locked || running || (context.sceneReady && !context.canManageScene);
      input.title = held ? '旧版准备保持 J1/J6 当前角度' : locked ? '本次场景目标已锁定；结束场景后修改用于下次启动' : '';
    });
    defaultButton.disabled = (autoMode() && !!instanceId) || running || (context.sceneReady && !context.canManageScene);
    const label = halted || (waitingControl ? '正在申请控制权' : pending && observation?.request_id !== pending.request_id
      ? '正在请求准备' : autoIntent ? '等待场景、实时遥测与控制权后自动展开'
        : labels[observation?.status] || (required ? (context.sceneReady ? '等待仿真关节遥测' : '等待场景启动')
          : (context.sceneReady ? '按初始操作角度初始化（无需准备运动）' : '等待场景启动')));
    status.textContent = `${autoMode() ? '六轴零位 → 操作目标 · ' : ''}${label}${observation?.phase_count ? ` · 阶段 ${observation.phase}/${observation.phase_count}` : ''}${observation?.progress != null ? ` · ${(observation.progress*100).toFixed(0)}%` : ''}${observation?.reason ? `：${observation.reason}` : ''}${observation && !fresh() ? '（遥测已过期）' : ''}${autoMode() && required && !running && !ready() && !atZero() ? '；请先复原场景，再显式重试展开' : ''}`;
  }
  function cancel(reason = '已中止准备') {
    const running = pending || waitingControl || autoIntent || activeStatuses.includes(observation?.status);
    if (running) {
      pending = null; waitingControl = null; autoIntent = null;
      controlRequested = false;
      halted = reason;
      send(null); message(reason);
    }
    if (!getContext().connected) { observation = null; measured = null; }
    render(); changed();
  }
  function begin(angles) {
    halted = '';
    pending = {request_id: crypto.randomUUID(), joint_position_deg: angles};
    requestedAt = performance.now();
    send(pending);
    message(autoMode() ? '正在沿已验证路点从六轴零位展开；实测到位前禁止操作与采集。中止后请先复原场景再重试。'
      : '正在按旧版固定顺序到达操作姿态；J1/J6 保持当前角度。');
    render(); changed();
  }
  function requestStart(angles) {
    cancelMotion();
    if (!getContext().controlGranted) {
      waitingControl = {angles, startedAt: performance.now()};
      activateControl();
      message('正在申请控制权，授权后自动开始准备，无需再次点击。');
      render(); changed();
    } else begin(angles);
  }
  startButton.addEventListener('click', () => {
    render();
    if (startButton.disabled) return;
    let angles;
    try { angles = autoMode() ? parseOperatingAngles(target, limits) : read(); }
    catch (error) { return message(error.message); }
    requestStart(angles);
  });
  cancelButton.addEventListener('click', () => cancel());
  defaultButton.addEventListener('click', () => { if (!defaultButton.disabled) fill(DEFAULT_OPERATING_DEG); });
  render();
  return {
    read, ready, cancel, render,
    configure(template, profile = selectedProfile) {
      selectedProfile = profile;
      limits = template?.arm_joint_limits_deg ?? null;
      inputs.forEach((input, index) => {
        input.min = limits?.[index]?.[0] ?? ''; input.max = limits?.[index]?.[1] ?? '';
      });
      render();
    },
    armAutoStart(id) {
      if (!id || id !== instanceId || !autoMode() || !required || armedInstances.has(id)) return;
      armedInstances.add(id);
      if (document.hidden || halted || ['ready', 'failed', 'cancelled', ...activeStatuses].includes(observation?.status)) return;
      const context = getContext();
      if (!context.canManageScene) return;
      autoIntent = {connected: context.connected, controlGranted: context.controlGranted, canManageScene: context.canManageScene};
      render(); changed();
    },
    setRuntime(instance, isReady) {
      if ((instance?.instance_id ?? null) !== instanceId) {
        cancel('场景已变化，已中止准备');
        instanceId = instance?.instance_id ?? null;
        runtimeProfile = instance?.randomization_profile ?? null;
        required = instance?.arm_preparation_required === true;
        target = instanceId ? [...(instance.operating_arm_joint_position_deg ?? DEFAULT_OPERATING_DEG)] : null;
        observation = null; measured = null; observedAt = 0; halted = ''; wasReady = false;
        if (target) fill(target);
      }
      required = instance?.arm_preparation_required === true;
      if (!isReady) {
        if (wasReady || pending || waitingControl) cancel('场景未就绪，已停止准备');
        observation = null; measured = null; observedAt = 0;
      }
      wasReady = isReady;
      render(); changed();
    },
    update(value, telemetry = {}) {
      if (!getContext().sceneReady || !getContext().connected) return;
      if (autoMode() && (!instanceId || telemetry.scene_instance_id !== instanceId)) return;
      const previous = observation;
      observation = value ?? (required ? null : {status:'legacy', ready:true}); observedAt = performance.now();
      measured = telemetry.arm_joint_position_rad
        ?? (telemetry.joint_position_rad?.length === 8 ? telemetry.joint_position_rad.slice(0, 6) : null);
      if (legacyHeld() && value?.held_joints?.length && value.goal_deg?.length === 6) {
        for (const index of [0, 5]) inputs[index].value = String(value.goal_deg[index]);
      }
      if (['failed', 'cancelled'].includes(value?.status)) {
        if (pending ? value.request_id === pending.request_id
          : previous?.status !== value.status || previous?.request_id !== value.request_id) {
          cancel(labels[value.status]);
          halted = labels[value.status];
        }
      } else if (pending && value?.request_id === pending.request_id && value.status === 'ready' && value.ready === true) {
        pending = null; send(null);
      }
      render(); changed();
    },
    tick() {
      const context = getContext();
      render(); changed();
      if (autoIntent) {
        if (document.hidden || context.estopped || (autoIntent.connected && !context.connected)
          || (autoIntent.controlGranted && !context.controlGranted) || (autoIntent.canManageScene && !context.canManageScene)) {
          cancel('自动展开已中止，请显式重试'); return true;
        }
        autoIntent.connected ||= context.connected;
        autoIntent.controlGranted ||= context.controlGranted;
        autoIntent.canManageScene ||= context.canManageScene;
        if (!context.sceneReady || !context.connected || !context.canManageScene || !fresh() || !limits) return true;
        if (!context.controlGranted) {
          if (!controlRequested) { controlRequested = true; activateControl(); }
          return true;
        }
        if (!atZero()) { cancel('实测不在六轴零位，请先复原场景再展开'); return true; }
        let angles;
        try { angles = parseOperatingAngles(target, limits); }
        catch (error) { cancel(error.message); return true; }
        autoIntent = null;
        controlRequested = false;
        requestStart(angles);
        return true;
      }
      if (waitingControl) {
        if (!context.connected || !context.sceneReady || !context.canManageScene || context.estopped || document.hidden
          || !fresh() || (autoMode() && !atZero())) {
          cancel('准备链路不可用或实测不在零位，已取消控制权申请');
        } else if (performance.now()-waitingControl.startedAt > 5000) {
          cancel('未获得控制权，请检查当前控制页面后重试');
        } else if (context.controlGranted) {
          const angles = waitingControl.angles;
          waitingControl = null;
          begin(angles);
        }
        return true;
      }
      if (!pending) return false;
      if (!context.connected || !context.controlGranted || !context.sceneReady || !context.canManageScene
        || context.estopped || document.hidden || !fresh()) {
        cancel('准备链路不可用，已停止发送运动授权'); return true;
      }
      if (observation?.request_id !== pending.request_id && performance.now()-requestedAt > 5000) {
        cancel('仿真未确认准备请求，请检查场景连接后重试'); return true;
      }
      send(pending);
      return true;
    },
  };
}
