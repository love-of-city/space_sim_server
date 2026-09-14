import test from "node:test";
import assert from "node:assert/strict";
import {readFileSync} from "node:fs";
import vm from "node:vm";
import {GamepadInput, sampleGamepad, gamepadControlBlockReason} from "../gamepad_input.js";

function pad(overrides = {}) {
  return {id: "Test gamepad", index: 0, connected: true, mapping: "standard", axes: [0, 0, 0, 0],
    buttons: Array.from({length: 17}, () => ({value: 0})), ...overrides};
}
const nav = (...pads) => ({getGamepads: () => pads});

test("missing API, policy rejection and absent device are explicit and do not throw", () => {
  assert.equal(sampleGamepad({}).status, "unsupported");
  assert.equal(sampleGamepad({getGamepads() {throw Object.assign(new Error(), {name: "SecurityError"});}}).status, "blocked");
  assert.equal(sampleGamepad(nav(null, null)).status, "disconnected");
  assert.equal(sampleGamepad(nav(pad({connected: false}))).status, "disconnected");
});

test("standard axes/buttons preserve XYZ, RPY and gripper mapping", () => {
  const device = pad({axes: [-0.5, -0.75, 0.4, -0.3]});
  for (const [i, value] of [[7, 0.9], [6, 0.2], [1, 1], [3, 0.5]]) device.buttons[i].value = value;
  const result = sampleGamepad(nav(null, device));
  assert.deepEqual(result.action.linear, [0.75, 0.5, 0.7]);
  assert.deepEqual(result.action.angular, [1, 0.3, -0.4]);
  assert.equal(result.action.grip, 0.5);
  assert.equal(result.action.source, "gamepad");
  assert.equal(result.active, true);
});

test("deadzone and finite-number checks prevent drift and invalid commands", () => {
  const result = sampleGamepad(nav(pad({axes: [0.119, NaN, Infinity, -0.1]})));
  assert.equal(result.active, false);
  assert.ok(result.action.linear.every(Number.isFinite));
  assert.ok(result.action.angular.every(Number.isFinite));
});

test("nonstandard/incomplete devices are visible but cannot emit guessed robot commands", () => {
  const result = sampleGamepad(nav(pad({mapping: ""})));
  assert.equal(result.status, "unmapped");
  assert.equal(result.action, null);
  assert.equal(result.device.id, "Test gamepad");
  assert.equal(sampleGamepad(nav(pad({axes: [1, 1]}))).status, "unmapped");
  assert.equal(sampleGamepad(nav(pad({mapping: ""}), pad({index: 1}))).device.index, 1);
});

test("newly connected or reconnected held controls must return neutral before output", () => {
  const input = new GamepadInput();
  const held = pad({axes: [0, -1, 0, 0]});
  assert.equal(input.read(nav(held)).status, "centering");
  assert.equal(input.read(nav(held)).action, null);
  assert.equal(input.read(nav(pad())).status, "connected");
  assert.equal(input.read(nav(held)).action.linear[0], 1);
  assert.equal(input.read(nav(null)).status, "disconnected");
  assert.equal(input.read(nav(held)).action, null);
});

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
function harness(navigatorObject) {
  const messages = [], nodes = new Map();
  const state = {operationActive: true, freeCameraMode: false, sceneReady: true, estopped: false,
    connected: true, canManageScene: true, controlGranted: true, ws: {readyState: 1}, lastAckAt: 0};
  const context = vm.createContext({state, gamepadInput: {read: () => sampleGamepad(navigatorObject)}, gamepadControlBlockReason,
    $: id => {if (!nodes.has(id)) nodes.set(id, {textContent: "", classList: {toggle(){}}}); return nodes.get(id);},
    WebSocket: {OPEN: 1}, keyboardAction: () => ({linear: [1, 0, 0], angular: [0, 0, 0], grip: 0, source: "keyboard"}),
    transmitAction: (action, deadman) => messages.push({action, deadman}), highlightKeys(){}});
  vm.runInContext(source.slice(source.indexOf("function gamepadAction("), source.indexOf("function transmitAction(")) +
    source.slice(source.indexOf("function sendAction("), source.indexOf("function updateMotionOutputs(")), context);
  return {context, state, messages, nodes};
}

test("Gamepad API failure cannot break keyboard command processing", () => {
  for (const navigatorObject of [{}, {getGamepads() {throw new Error("blocked");}}]) {
    const h = harness(navigatorObject); h.context.sendAction();
    assert.equal(h.messages.at(-1).action.source, "keyboard");
    assert.ok(h.nodes.get("gamepadStatus").textContent);
  }
});

test("diagnostics poll while all existing motion guards still block transmission", () => {
  for (const [key, value] of Object.entries({operationActive: false, freeCameraMode: true, sceneReady: false,
    estopped: true, connected: false, canManageScene: false, controlGranted: false})) {
    const h = harness(nav(pad({axes: [0, -1, 0, 0]}))); h.state[key] = value;
    h.context.sendAction();
    assert.equal(h.messages.length, 0, key);
    assert.equal(h.nodes.get("gamepadStatus").textContent, "已连接");
    assert.ok(h.nodes.get("gamepadGate").textContent);
  }
});
