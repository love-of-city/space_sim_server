// Measured body-frame speed / PRE-IK smoothed command, with separate SI units.
const REASONS = {
  torque_saturation: "实测执行器输出已饱和，存在关节力矩限制",
  joint_position_limit: "关节角度限位",
  joint_speed_limit: "关节速度上限",
  singularity_nearby: "同时接近奇异位形（不代表唯一原因）",
  damped_task_error: "阻尼 IK 未完全实现指令（可能伴随方向／姿态误差）",
  task_residual: "求解运动与指令存在残差",
  task_constraint: "保持运动方向／位姿的任务约束受限",
  joint_tracking_error: "实测关节未跟上目标；力矩／接触原因尚未确认",
  cause_unconfirmed: "原因待确认（尚无足够诊断证据）",
};

export function formatMotionSpeedDiagnostics(data) {
  if (!data) return {warning: false, title: "等待速度诊断", detail: "需要新版仿真遥测；阈值：预期速度的 80%。"};
  if (data.command_stale) return {warning: true, title: "控制指令已过期", detail: "运动输入已停用，当前不计算速度达成率。"};
  if (!data.measurement_valid) return {warning: false, title: "实测速度不可用", detail: "不使用 IK 预测速度冒充实测速度。"};
  const active = [["linear", "平移", "m/s"], ["angular", "旋转", "rad/s"]].filter(([key]) => data[key]?.active);
  if (!active.length) return {warning: false, title: "无有效运动指令", detail: "本体系 · sarm_ee；静止时不触发低速提示。"};
  const lines = active.map(([key, label, unit]) => {
    const c = data[key];
    const ratio = Number.isFinite(c.ratio) ? `${(c.ratio * 100).toFixed(0)}%` : "—";
    const state = c.warning ? "速度不足" : c.state === "settling" ? "启动／换向观察中" : "监测中";
    const speed = `${label} ${ratio}（${state}）：预期 ${Number(c.expected_speed).toFixed(3)} / 沿指令实际 ${Number(c.actual_speed_along_command).toFixed(3)} ${unit}`;
    const reasons = (c.reasons || []).map(r => {
      const joints = r.joints?.length ? ` [关节 ${r.joints.join("、")}]` : "";
      return `${REASONS[r.code] || "原因待确认"}${joints}${r.detail ? ` (${r.detail})` : ""}`;
    });
    return speed + (c.warning ? `\n诊断：${reasons.join("；") || REASONS.cause_unconfirmed}` : "");
  });
  const warning = active.some(([key]) => data[key].warning);
  return {warning, title: warning ? "末端速度不足（80% 阈值／85% 恢复）" : "末端速度达成率", detail: lines.join("\n")};
}
