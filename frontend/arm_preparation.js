import { parseInitialJointAngles } from './initial_joint_angles.js';

export const DEFAULT_OPERATING_DEG = [0, -67.6, -86.6, 143.2, -85.5, 0];
export const ZERO_START_PROFILE = 'teleop-zero-prepare-v1';
export function parseOperatingAngles(values, limits) {
  try { return parseInitialJointAngles(values, limits); }
  catch (error) { throw new Error(error.message.replaceAll('初始角度', '操作角度')); }
}
const labels = {waiting:'等待准备', arming:'等待机械臂稳定停止', checking:'旧版本正在检查路径', moving:'正在到达操作姿态',
  pausing:'暂停并重新检查路径', settling:'等待实测稳定到位', ready:'已到位，可以操作', failed:'准备失败', cancelled:'已中止，请重新准备', legacy:'旧场景：保留原初始化方式'};

export function createArmPreparation({fields, startButton, cancelButton, defaultButton, status, getContext, send, cancelMotion, activateControl, message, changed}) {
  const inputs = [...fields.querySelectorAll('input')];
  let limits = null, pending = null, waitingControl = null, requestedAt = 0, observation = null, observedAt = 0, instanceId = null, required = true;
  const fill = values => inputs.forEach((input, i) => { input.value = String(i === 0 || i === 5 ? 0 : values[i]); });
  fill(DEFAULT_OPERATING_DEG);
  const read = () => parseOperatingAngles(inputs.map(input => input.value), limits);
  function ready() {
    // New scenes start directly at the requested operating pose. Keep the
    // measured readiness gate only for legacy saved scenes that still carry
    // arm_preparation_required=true.
    if (!required) return true;
    return !pending && !waitingControl && observation?.ready === true && performance.now()-observedAt < 1500;
  }
  function render() {
    const c = getContext();
    const fresh = !!observation && performance.now()-observedAt < 1500;
    const running = !!pending || !!waitingControl || ['arming','checking','moving','settling','pausing'].includes(observation?.status);
    startButton.disabled = !required || !c.sceneReady || !c.connected || !c.canManageScene || c.estopped || running || !limits || !fresh;
    cancelButton.disabled = !running || !c.canManageScene;
    inputs.forEach((input, i) => {
      const held = i === 0 || i === 5;
      input.disabled = held || running || (c.sceneReady && !c.canManageScene);
      input.title = held ? '旧版准备流程保持当前角度；直接启动场景不会执行准备运动' : '';
    });
    defaultButton.disabled = running || (c.sceneReady && !c.canManageScene);
    const label = waitingControl ? '正在申请控制权' : pending && observation?.request_id !== pending.request_id ? '正在请求准备' : labels[observation?.status] || (required ? (c.sceneReady ? '等待仿真关节遥测' : '等待场景启动') : (c.sceneReady ? '场景已直接设置操作姿态' : '等待场景启动'));
    status.textContent = `${label}${observation?.phase_count ? ` · 阶段 ${observation.phase}/${observation.phase_count}` : ''}${observation?.progress > 0 ? ` · ${(observation.progress*100).toFixed(0)}%` : ''}${observation?.reason ? `：${observation.reason}` : ''}${observation && !fresh ? '（遥测已过期）' : ''}`;
  }
  function cancel(reason = '已中止准备') {
    if (pending || waitingControl) { pending = null; waitingControl = null; send(null); message(reason); }
    render(); changed();
  }
  function begin(angles) {
    pending = {request_id: crypto.randomUUID(), joint_position_deg: angles};
    requestedAt = performance.now();
    send(pending);
    message('正在按固定顺序到达操作姿态；期间不能遥操作。Esc、中止、急停或断线会停止准备。');
    render(); changed();
  }
  startButton.addEventListener('click', () => {
    const c = getContext();
    if (startButton.disabled) return;
    let angles;
    try { angles = read(); } catch (error) { return message(error.message); }
    cancelMotion();
    if (!c.controlGranted) {
      waitingControl = {angles, startedAt: performance.now()};
      activateControl();
      message('正在申请控制权，授权后自动开始准备，无需再次点击。');
      render(); changed();
      return;
    }
    begin(angles);
  });
  cancelButton.addEventListener('click', () => cancel());
  defaultButton.addEventListener('click', () => fill(DEFAULT_OPERATING_DEG));
  render();
  return {
    read, ready, cancel, render,
    configure(template) {
      limits = template?.arm_joint_limits_deg ?? null;
      inputs.forEach((input, i) => {
        input.min = limits?.[i]?.[0] ?? ''; input.max = limits?.[i]?.[1] ?? '';
      });
      render();
    },
    setRuntime(instance, isReady) {
      required = instance?.arm_preparation_required === true;
      if ((instance?.instance_id ?? null) !== instanceId) {
        cancel('场景已变化，已中止准备');
        instanceId = instance?.instance_id ?? null;
        observation = null; observedAt = 0;
        if (instanceId) fill(instance.operating_arm_joint_position_deg ?? DEFAULT_OPERATING_DEG);
      }
      if (!isReady) { cancel('场景未就绪，已停止准备'); observation = null; observedAt = 0; }
      render();
    },
    update(value) {
      observation = value ?? (required ? null : {status:"legacy", ready:true}); observedAt = performance.now();
      if (value?.held_joints && value.goal_deg?.length === 6) {
        for (const i of [0, 5]) inputs[i].value = String(value.goal_deg[i]);
      }
      if (pending && value?.request_id === pending.request_id && ['ready','failed','cancelled'].includes(value.status)) {
        pending = null; send(null);
      }
      render(); changed();
    },
    tick() {
      const c = getContext();
      if (waitingControl) {
        if (!c.connected || !c.sceneReady || !c.canManageScene || c.estopped || document.hidden || !observation || performance.now()-observedAt > 1500) {
          cancel('准备链路不可用，已取消控制权申请');
        } else if (performance.now()-waitingControl.startedAt > 5000) {
          cancel('未获得控制权，请检查当前控制页面后重试');
        } else if (c.controlGranted) {
          const angles = waitingControl.angles;
          waitingControl = null;
          begin(angles);
        }
        return true;
      }
      if (!pending) return false;
      if (!c.connected || !c.controlGranted || !c.sceneReady || !c.canManageScene || c.estopped || document.hidden || (observedAt && performance.now()-observedAt > 1500)) {
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
