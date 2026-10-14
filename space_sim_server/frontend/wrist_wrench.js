export function formatWristWrench(wrench) {
  if (!wrench || !wrench.valid) return { force: "—", torque: "—", detail: "未安装或尚无有效采样" };
  const valid = values => Array.isArray(values) && values.length === 3 && values.every(Number.isFinite);
  if (!valid(wrench.force_n) || !valid(wrench.torque_nm))
    return { force: "—", torque: "—", detail: "测力数据无效" };
  return {
    force: wrench.force_n.map(v => v.toFixed(3)).join(", ") + " N",
    torque: wrench.torque_nm.map(v => v.toFixed(3)).join(", ") + " N·m",
    detail: `${wrench.site_name} · 原始接口载荷 · t=${(Number(wrench.sample_time_ns) / 1e9).toFixed(4)} s`,
  };
}
