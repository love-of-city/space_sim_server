// Display only: never normalize angles, alter targets, or infer why motion stopped.
export const NEAR_LIMIT_DEG = 10;
export const AT_LIMIT_DEG = 1;
const DEG = 180 / Math.PI;
const finite = value => typeof value === "number" && Number.isFinite(value);
const angle = value => `${(Math.abs(value) < 0.05 ? 0 : value).toFixed(1)}°`;
const distance = value => value < 0
  ? `越界 ${-value < 0.1 ? "<0.1°" : angle(-value)}` : angle(value);

export function jointAngleRows(observation) {
  // An old six-element layout can contain a prismatic gripper, not six arm axes.
  const actual = observation?.arm_joint_position_rad
    ?? (observation?.joint_position_rad?.length === 8 ? observation.joint_position_rad.slice(0, 6) : []);
  return Array.from({length: 6}, (_, i) => {
    const row = {name: `J${i + 1}`, angle: "—", range: "—", lower: "—", upper: "—", status: "等待实测", level: "unknown"};
    const bounds = observation?.arm_joint_limits_rad?.[i];
    const known = Array.isArray(bounds) && bounds.length === 2;
    const continuous = known && bounds.every(x => x === null);
    const limited = known && bounds.every(finite) && bounds[0] < bounds[1];
    if (limited) row.range = `${angle(bounds[0] * DEG)} ～ ${angle(bounds[1] * DEG)}`;
    else if (continuous) row.range = "无位置限位";
    if (!finite(actual?.[i])) return row;
    row.angle = angle(actual[i] * DEG);
    if (continuous) return {...row, status: "连续关节", level: "normal"};
    if (!limited) return {...row, status: "限位未提供"};
    const low = (actual[i] - bounds[0]) * DEG;
    const high = (bounds[1] - actual[i]) * DEG;
    row.lower = distance(low); row.upper = distance(high);
    const margin = Math.min(low, high);
    const side = low <= high ? "下限" : "上限";
    if (margin < -1e-7) return {...row, status: `超出${side}`, level: "danger"};
    if (margin <= AT_LIMIT_DEG + 1e-7) return {...row, status: `距${side} ≤1°`, level: "danger"};
    if (margin <= NEAR_LIMIT_DEG + 1e-7) return {...row, status: `接近${side}`, level: "warning"};
    return {...row, status: "正常", level: "normal"};
  });
}

export function createJointAnglePanel(container, status, {clock = () => performance.now(), staleMs = 3000} = {}) {
  const doc = container.ownerDocument;
  const cards = Array.from({length: 6}, (_, i) => {
    const card = doc.createElement("div");
    card.className = "joint-angle-card unknown";
    const head = doc.createElement("div"); head.className = "joint-angle-head";
    const label = doc.createElement("strong"); label.textContent = `J${i + 1}`;
    const value = doc.createElement("output"); value.setAttribute("aria-label", `关节 ${i + 1} 实际角度`);
    head.append(label, value); card.append(head);
    const fields = {angle: value};
    for (const [key, title] of [["range", "限位"], ["lower", "距下限"], ["upper", "距上限"], ["status", "状态"]]) {
      const line = doc.createElement("div"); line.className = "joint-angle-line";
      const name = doc.createElement("span"); name.textContent = title;
      fields[key] = doc.createElement("span"); line.append(name, fields[key]); card.append(line);
    }
    container.append(card); return {card, fields};
  });
  let receivedAt = null;
  const text = (node, value) => { if (node.textContent !== value) node.textContent = value; };
  function render(rows) {
    rows.forEach((row, i) => {
      const {card, fields} = cards[i];
      const className = `joint-angle-card ${row.level}`;
      if (card.className !== className) card.className = className;
      for (const key of Object.keys(fields)) text(fields[key], row[key]);
    });
  }
  function clear(reason = "等待关节遥测") {
    receivedAt = null; render(jointAngleRows(null)); text(status, reason);
  }
  clear();
  return {
    update(observation) {
      receivedAt = clock();
      const rows = jointAngleRows(observation); render(rows);
      text(status, rows.every(r => r.angle === "—") ? "实测角度不可用"
        : rows.some(r => r.status === "限位未提供") ? "限位元数据未提供" : "实际角度 · °");
    },
    clear,
    checkFreshness() {
      if (receivedAt !== null && clock() - receivedAt >= staleMs) clear("遥测已过期，等待更新");
    },
  };
}
