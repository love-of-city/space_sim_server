// Device discovery is read-only. No WebSocket, UE command, or operation-mode mutation.
const DEADZONE = 0.12;
const finite = (value, min, max) => Number.isFinite(value) ? Math.max(min, Math.min(max, value)) : 0;
const empty = (status, hint) => ({status, hint, device: null, axes: [], buttons: [], action: null, active: false});

export function sampleGamepad(navigatorObject = globalThis.navigator) {
  if (typeof navigatorObject?.getGamepads !== "function") {
    return empty("unsupported", "浏览器未提供 Gamepad API；请检查浏览器支持及 HTTPS/localhost 访问环境。键盘仍可使用。");
  }
  let pads;
  try {
    pads = Array.from(navigatorObject.getGamepads() || []).filter(pad => pad && pad.connected !== false);
  } catch (error) {
    return empty("blocked", error?.name === "SecurityError"
      ? "浏览器策略阻止读取手柄；请直接打开本站，检查 Permissions-Policy 和浏览器权限。"
      : "读取手柄失败；请重新连接设备或刷新页面。键盘仍可使用。");
  }
  if (!pads.length) {
    return empty("disconnected", "浏览器尚未发现手柄：在本机连接设备，保持网页前台并按一下按钮后释放。RDP 不等于手柄已透传。");
  }
  const pad = pads.find(p => p.mapping === "standard") || pads[0];
  const axes = Array.from(pad.axes || [], value => finite(value, -1, 1));
  const buttons = Array.from(pad.buttons || [], button => finite(button?.value, 0, 1));
  const device = {id: pad.id || "未命名手柄", index: pad.index ?? 0, mapping: pad.mapping || "非标准", count: pads.length};
  if (pad.mapping !== "standard" || axes.length < 4 || buttons.length < 8) {
    return {...empty("unmapped", "设备已识别，但不是受支持的标准双摇杆映射；暂不输出机械臂动作。请切换手柄标准/XInput 模式或添加专用映射。"), device, axes, buttons};
  }
  const dz = value => Math.abs(value) < DEADZONE ? 0 : -value;
  const action = {
    linear: [dz(axes[1]), dz(axes[0]), buttons[7] - buttons[6]],
    angular: [buttons[1] - buttons[0], dz(axes[3]), dz(axes[2])],
    grip: buttons[3] - buttons[2], source: "gamepad",
  };
  const active = [...action.linear, ...action.angular, action.grip].some(value => Math.abs(value) > 0.01);
  return {status: "connected", hint: "左杆 XY，LT/RT Z；右杆 Pitch/Yaw，A/B Roll，X/Y 夹爪。", device, axes, buttons, action, active};
}

export class GamepadInput {
  constructor() { this.reset(); }
  reset() { this.deviceKey = null; this.centered = false; }
  read(navigatorObject = globalThis.navigator) {
    const snapshot = sampleGamepad(navigatorObject);
    if (!snapshot.action) { this.reset(); return snapshot; }
    const key = `${snapshot.device.index}:${snapshot.device.id}`;
    if (this.deviceKey !== key) { this.deviceKey = key; this.centered = false; }
    // Connecting/reconnecting a held stick must not immediately start motion.
    if (!snapshot.active) this.centered = true;
    if (!this.centered) return {...snapshot, status: "centering", action: null,
      hint: "请先将摇杆回中并释放扳机/按钮，再操作；新接入的手柄不会立即输出动作。"};
    return snapshot;
  }
}

export function gamepadControlBlockReason(state) {
  if (state.estopped) return "急停已锁存：先恢复控制，再点击画面。";
  if (state.freeCameraMode) return "自由相机模式屏蔽机械臂输入，按 C/Home 返回。";
  if (!state.sceneReady) return "等待场景运行。";
  if (!state.connected) return "控制 WebSocket 未连接。";
  if (!state.canManageScene) return "当前账号没有场景操作权限。";
  if (!state.controlGranted) return "点击画面，将控制权切换到当前页面。";
  if (!state.operationActive) return "点击实时画面进入操作；页面失焦后必须重新进入。";
  return "操作模式已开启；回中/释放输入停止目标推进，Esc 退出。";
}
