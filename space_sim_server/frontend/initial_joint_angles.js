// Initialization only; never sends live teleoperation commands or wraps angles.
export function parseInitialJointAngles(values, limits) {
  if (!Array.isArray(limits) || limits.length !== 6) throw new Error("关节限位未加载，暂不能设置初始角度");
  if (values.length !== 6) throw new Error("请输入 J1～J6 共六个初始角度");
  return values.map((text, i) => {
    const value = typeof text === "number" ? text : typeof text === "string" && text.trim() !== "" ? Number(text) : NaN;
    if (!Number.isFinite(value)) throw new Error(`J${i + 1} 请输入有效的初始角度（°）`);
    const bounds = limits[i];
    if (!Array.isArray(bounds) || bounds.length !== 2) throw new Error(`J${i + 1} 限位不可用`);
    const [low, high] = bounds;
    if (low === null && high === null) return value;
    if (!Number.isFinite(low) || !Number.isFinite(high) || low >= high) throw new Error(`J${i + 1} 限位不可用`);
    if (value < low || value > high) throw new Error(`J${i + 1} 初始角度必须在 ${low}° ～ ${high}° 之间`);
    return value;
  });
}

export function createInitialJointAngles({toggle, fields, presetButton, help}) {
  const inputs = [...fields.querySelectorAll("input")];
  const ranges = [...fields.querySelectorAll("small")];
  let limits = null, preset = null, active = false, initialized = false;
  function update() {
    toggle.disabled = active || (!limits && !toggle.checked);
    fields.hidden = !toggle.checked;
    inputs.forEach(input => { input.disabled = active || !toggle.checked || !limits; });
    presetButton.disabled = active || !toggle.checked || !limits || !preset;
  }
  function fillPreset() {
    if (preset) inputs.forEach((input, i) => { input.value = String(preset[i]); });
  }
  toggle.addEventListener("change", update);
  presetButton.addEventListener("click", fillPreset);
  update();
  return {
    configure(template, home) {
      limits = template?.arm_joint_limits_deg ?? null;
      preset = home ?? null;
      if (!initialized && preset) { fillPreset(); initialized = true; }
      inputs.forEach((input, i) => {
        const [low, high] = limits?.[i] ?? [null, null];
        input.min = low === null ? "" : String(low);
        input.max = high === null ? "" : String(high);
        ranges[i].textContent = !limits ? "限位未加载" : low === null && high === null ? "连续关节，无位置限位" : `${Number(low.toFixed(4))}° ～ ${Number(high.toFixed(4))}°`;
      });
      help.textContent = limits
        ? "按模型零位输入 J1～J6，单位为度（°），不自动折返角度。启用后仅覆盖机械臂随机初始角度，夹爪及其他随机参数不变；重置恢复本次初始状态。限位校验不保证无碰撞，请选择合理姿态。"
        : template?.initial_arm_error || "关节限位未加载，暂不能设置自定义初始角度。";
      update();
    },
    setRuntime(isActive, instance) {
      active = isActive;
      if (active && instance?.instance_id) {
        const saved = instance.initial_arm_joint_position_deg;
        toggle.checked = Array.isArray(saved) && saved.length === 6;
        if (toggle.checked) inputs.forEach((input, i) => { input.value = String(saved[i]); });
      }
      update();
    },
    read() {
      return toggle.checked ? parseInitialJointAngles(inputs.map(input => input.value), limits) : null;
    },
  };
}
