export function referenceProtectionLabel(observation) {
  const labels = {
    active: "正常跟踪",
    holding: "保持位置",
    tracking_limited: "跟踪误差限速，可尝试反向退出或松键卸载",
    saturation_limited: "持续力矩饱和，停止推进，可尝试反向退出或松键卸载",
    reference_recovery: "正在缓慢卸除旧目标误差，请松开运动键等待；Esc 可停止",
    recovery_complete: "旧目标卸载已结束，可重新操作；仍受阻请一键复原",
  };
  const label = labels[observation.reference_governor_state];
  if (!label) return "未上报（旧仿真进程或未连接）";
  const joints = Array.isArray(observation.reference_limited_joints)
    ? observation.reference_limited_joints.filter(value => Number.isInteger(value) && value >= 1 && value <= 6)
    : [];
  const scale = Number(observation.tracking_scale);
  const detail = Number.isFinite(scale) ? ` · 参考比例 ${Math.max(0, Math.min(1, scale)).toFixed(2)}` : "";
  const version = observation.reference_controller_version;
  const recoveryHint = version !== "bounded_reference_recovery_v2" ? " · 松键卸载需新版仿真" : "";
  return `${label}${joints.length ? ` · ${joints.map(value => `J${value}`).join("/")}` : ""}${detail}${recoveryHint}`;
}

export function dynamicsTimingLabel(observation) {
  const step = Number(observation.dynamics_step_s);
  if (!Number.isFinite(step) || step <= 0) return "—";
  const rate = Number(observation.ik_control_rate_hz);
  return `${(step * 1000).toFixed(2)} ms${Number.isFinite(rate) && rate > 0 ? ` · IK ${rate} Hz` : ""}`;
}
