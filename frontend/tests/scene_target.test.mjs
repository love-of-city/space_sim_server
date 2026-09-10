import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const catalogSource = source.slice(source.indexOf("async function loadSceneCatalog()"), source.indexOf("function applySceneRuntime("));
const applySource = source.slice(source.indexOf("function applySceneRuntime("), source.indexOf("async function readApiResponse("));
const startSource = source.slice(source.indexOf("async function startScene()"), source.indexOf("async function stopScene("));
const current = "sarm-ground-validation-self-collision-grasp";
const coarse = "sarm-ground-validation-grasp";
const mesh = "sarm-ground-validation-mesh-grasp";
const legacy = "spacecraft-arm-teleop";

function setup() {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, { value: "", checked: false, disabled: false, textContent: "",
      classList: { toggle() {} }, replaceChildren(...items) { this.options = items; }, focus() {} });
    return nodes.get(id);
  };
  const requests = [];
  const ctx = vm.createContext({ $, console, state: { currentUser: { role: "admin" } },
    Option: function(label, value) { this.label = label; this.value = value; }, scenePhaseLabels: {},
    setMessage() {}, updateOperationUI() {}, updateEpisodeUI() {}, connectPixelStreaming() {},
    exitOperationMode() {}, refreshState: async () => {},
    readApiResponse: async () => ({ instance_id: "s1", template_id: $("sceneTemplate").value }),
    apiRequest: async (path, options = {}) => {
      requests.push({ path, options });
      return { ok: true, json: async () => ({ templates: [{ id: current, label: "粗碰撞体·内部碰撞" }, { id: coarse, label: "旧粗碰撞盒" }, { id: mesh, label: "高精度·实验" }, { id: legacy, label: "小方块" }],
        randomization_profiles: [{ id: "none", label: "固定" }], defaults: { template_id: current, randomization_profile: "none" } }) };
    },
  });
  vm.runInContext(catalogSource + applySource + startSource, ctx);
  return { $, ctx, requests };
}

test("HTML and catalog default to coarse satellite self-contact, not the old boxes or triangles", async () => {
  const html = readFileSync(new URL("../index.html", import.meta.url), "utf8");
  assert.match(html, /<option value="sarm-ground-validation-self-collision-grasp">/);
  assert.doesNotMatch(html.match(/<select id="sceneTemplate"[^>]*>(.*?)<\/select>/s)[1], /sarm-ground-validation-mesh-grasp/);
  assert.match(html, /id="sceneInstanceTarget"/);
  const h = setup();
  await h.ctx.loadSceneCatalog();
  assert.equal(h.$("sceneTemplate").value, current);
  assert.equal(h.$("sceneTemplate").options.map(x => x.value).join(","), `${current},${coarse},${mesh},${legacy}`);
  await h.ctx.startScene();
  const request = h.requests.find(r => r.path === "/api/scenes/start");
  assert.equal(JSON.parse(request.options.body).template_id, current);
});

test("active scenes still display their actual collision template after rollback", () => {
  const targets = [
    [current, "model/SARM/platform/sarm_ground_target_self_collision.xml", "地面验证星（粗碰撞体·内部碰撞）"],
    [coarse, "model/SARM/platform/sarm_ground_target.xml", "地面验证星（粗碰撞盒·无内部碰撞）"],
    [mesh, "model/ground_validation_satellite/mesh_collision_trial/sarm_mesh_collision.xml", "地面验证星（高精度三角网格·实验）"],
    [legacy, "model/SARM/platform/sarm_platform.xml", "原小方块目标"],
  ];
  for (const [template, runtime_model, label] of targets) {
    const h = setup();
    h.ctx.applySceneRuntime({ phase: "running", active: true, instance: { instance_id: "s1", template_id: template,
      randomization: {}, capture_target: { runtime_model, synthetic_mass_kg: 10 } } });
    assert.equal(h.$("sceneTemplate").value, template);
    assert.equal(h.$("sceneTemplate").disabled, true);
    assert.equal(h.$("sceneInstanceTarget").textContent, label);
    assert.equal(JSON.parse(h.$("sceneParameters").textContent).capture_target.runtime_model, runtime_model);
  }
});

test("stopped old coarse, mesh or legacy instances cannot replace the new self-contact selection", () => {
  for (const template of [coarse, mesh, legacy]) {
    const h = setup();
    h.$("sceneTemplate").value = current;
    h.ctx.applySceneRuntime({ phase: "stopped", active: false, instance: { instance_id: "old", template_id: template, randomization: {} } });
    assert.equal(h.$("sceneTemplate").value, current);
    assert.equal(h.$("sceneTemplate").disabled, false);
    h.ctx.applySceneRuntime({ phase: "idle", active: false });
    assert.equal(h.$("sceneInstanceTarget").textContent, "—");
  }
});

test("preserved high-precision template can still be selected explicitly", async () => {
  const h = setup();
  await h.ctx.loadSceneCatalog();
  h.$("sceneTemplate").value = mesh;
  await h.ctx.startScene();
  const request = h.requests.find(r => r.path === "/api/scenes/start");
  assert.equal(JSON.parse(request.options.body).template_id, mesh);
});

test("initialization warns about coarse internal collision and experimental triangle cost", () => {
  const html = readFileSync(new URL("../index.html", import.meta.url), "utf8");
  assert.match(html, /aria-describedby="sceneCollisionWarning"/);
  assert.match(html, /默认粗碰撞体版已启用目标内部碰撞/);
  assert.match(html, /旧粗盒版不计算目标内部碰撞/);
  assert.match(html, /高精度三角网格版仅供实验/);
  assert.match(html, /接触角度不代表真实限位/);
  assert.match(html, /运行速度显著低于实时/);
});
