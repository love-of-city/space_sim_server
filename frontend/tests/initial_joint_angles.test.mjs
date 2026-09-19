import {readFileSync} from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";
import {parseInitialJointAngles, createInitialJointAngles} from "../initial_joint_angles.js";
const limits = [...Array.from({length: 5}, () => [-180, 180]), [-360, 360]];
const angles = [-45, -20, 25, -90, -50, 275];

test("six degree inputs preserve negative, zero and multi-turn model coordinates", () => {
  assert.deepEqual(parseInitialJointAngles(angles.map(String), limits), angles);
  assert.deepEqual(parseInitialJointAngles([0, 0, 0, 0, 0, -300], limits), [0, 0, 0, 0, 0, -300]);
  assert.equal(parseInitialJointAngles([180, -180, 0, 0, 0, 360], limits)[5], 360);
});
test("missing, blank, nonfinite, malformed and out-of-range inputs fail closed", () => {
  for (const bad of ["", " ", "NaN", "Infinity", undefined, null, true, "181", "-181"]) {
    assert.throws(() => parseInitialJointAngles([bad, 0, 0, 0, 0, 0], limits), /J1/);
  }
  assert.throws(() => parseInitialJointAngles([0], limits), /六个/);
  assert.throws(() => parseInitialJointAngles(angles, null), /限位/);
  assert.throws(() => parseInitialJointAngles(angles, [[null, 1], ...limits.slice(1)]), /限位/);
  const continuous = [[null, null], ...limits.slice(1)];
  assert.equal(parseInitialJointAngles([900, 0, 0, 0, 0, 0], continuous)[0], 900);
});

function node() {
  return {value: "", checked: false, disabled: false, hidden: false, events: {},
    addEventListener(name, fn) {this.events[name] = fn;}, fire(name) {this.events[name]();}};
}
function panel() {
  const inputs = Array.from({length: 6}, node), ranges = Array.from({length: 6}, node);
  const toggle = node(), fields = node(), presetButton = node(), help = node();
  fields.querySelectorAll = selector => selector === "input" ? inputs : ranges;
  const ui = createInitialJointAngles({toggle, fields, presetButton, help});
  return {ui, toggle, fields, presetButton, help, inputs, ranges};
}
test("opt-in form, preset fill, active locking and saved-instance restoration", () => {
  const p = panel();
  assert.equal(p.ui.read(), null);
  assert.equal(p.toggle.disabled, true);
  p.ui.configure({arm_joint_limits_deg: limits}, angles);
  assert.equal(p.toggle.disabled, false);
  p.toggle.checked = true; p.toggle.fire("change");
  assert.equal(p.fields.hidden, false);
  assert.deepEqual(p.ui.read(), angles);
  p.inputs[0].value = "17";
  p.ui.configure({arm_joint_limits_deg: limits}, [0,0,0,0,0,0]);
  assert.equal(p.inputs[0].value, "17"); // polling / profile changes must not erase edits
  p.presetButton.fire("click");
  assert.deepEqual(p.ui.read(), [0,0,0,0,0,0]);
  p.ui.setRuntime(true, {instance_id: "custom", initial_arm_joint_position_deg: angles});
  assert.equal(p.toggle.disabled, true);
  assert.ok(p.inputs.every(input => input.disabled));
  assert.deepEqual(p.ui.read(), angles);
  p.ui.setRuntime(false);
  assert.ok(p.inputs.every(input => !input.disabled));
  p.ui.setRuntime(true, {instance_id: "legacy"});
  assert.equal(p.ui.read(), null);
  assert.equal(p.fields.hidden, true);
});
test("missing template limits cannot submit a previously enabled custom pose", () => {
  const p = panel();
  p.ui.configure({arm_joint_limits_deg: limits}, angles);
  p.toggle.checked = true; p.toggle.fire("change");
  p.ui.configure({initial_arm_error: "模型缺失"}, angles);
  assert.equal(p.help.textContent, "模型缺失");
  assert.equal(p.toggle.disabled, false); // still allow opting out
  assert.throws(() => p.ui.read(), /限位/);
  p.toggle.checked = false; p.toggle.fire("change");
  assert.equal(p.ui.read(), null);
});

const appSource = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const startSource = appSource.slice(appSource.indexOf("async function startScene()"), appSource.indexOf("async function resetScene()"));
test("actual scene-start handler submits custom degrees, supports opt-out and blocks bad values", async () => {
  const p = panel();
  p.ui.configure({arm_joint_limits_deg: limits}, angles);
  const nodes = new Map();
  const $ = id => { if (!nodes.has(id)) nodes.set(id, node()); return nodes.get(id); };
  $("sceneSunlightIntensity").value = "1";
  $("sceneTemplate").value = "spacecraft-arm-teleop";
  $("randomizationProfile").value = "training-v1";
  const requests = [], messages = [];
  const ctx = vm.createContext({$, initialJoints: p.ui, state: {sceneDefaults: {}},
    setMessage: text => messages.push(text), applySceneRuntime() {}, refreshState: async () => {},
    apiRequest: async (path, options) => {requests.push(JSON.parse(options.body)); return {ok: true};},
    readApiResponse: async () => ({instance_id: "test", seed: 1}),
  });
  vm.runInContext(startSource, ctx);
  await ctx.startScene();
  assert.equal(requests.at(-1).initial_arm_joint_position_deg, null);
  p.toggle.checked = true; p.toggle.fire("change");
  await ctx.startScene();
  assert.deepEqual(requests.at(-1).initial_arm_joint_position_deg, angles);
  for (const bad of ["", "181", "-181", "NaN"]) {
    p.inputs[0].value = bad;
    await ctx.startScene();
    assert.equal(requests.length, 2);
    assert.match(messages.at(-1), /J1/);
  }
});
test("six labelled degree inputs are opt-in and separated from live telemetry", () => {
  const html = readFileSync(new URL("../index.html", import.meta.url), "utf8");
  assert.match(html, /id="sceneCustomInitialJoints"[^>]+type="checkbox"/);
  assert.match(html, /id="sceneInitialJointFields"[^>]+hidden/);
  for (let i = 1; i <= 6; i++) {
    assert.match(html, new RegExp(`for="initialJoint${i}">J${i} 初始角度（°）`));
    assert.match(html, new RegExp(`id="initialJoint${i}"[^>]+type="number"[^>]+step="any"`));
  }
});
