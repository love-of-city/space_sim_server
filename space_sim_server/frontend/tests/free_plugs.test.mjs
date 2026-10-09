import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const selectionSource = source.slice(source.indexOf("function configureSceneSelection()"), source.indexOf("const jointAngles ="));
const catalogSource = source.slice(source.indexOf("async function loadSceneCatalog()"), source.indexOf("function applySceneRuntime("));
const startSource = source.slice(source.indexOf("async function startScene()"), source.indexOf("async function stopScene("));
const freePlugs = "sarm-task-box-free-plugs";
const directStart = "teleop-zero-prepare-v1";
const autoPrepare = "teleop-zero-prepare-v2";

function setup() {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {
      value: "", checked: false, disabled: false, hidden: false, textContent: "",
      classList: { toggle() {} }, replaceChildren(...items) { this.options = items; }, focus() {},
    });
    return nodes.get(id);
  };
  $("sceneSunlightIntensity").value = "1";
  $("sceneTemplate").value = freePlugs;
  $("randomizationProfile").value = autoPrepare;
  $("sceneSeed").value = "42";
  const requests = [], configuredProfiles = [];
  const catalog = {
    templates: [{
      id: freePlugs,
      label: "本地任务盒（两个活动插头·实验）",
      default_randomization_profile: directStart,
      unsupported_randomization_profiles: [autoPrepare],
    }],
    randomization_profiles: [
      { id: autoPrepare, label: "零位启动并自动展开" },
      { id: directStart, label: "指定操作姿态直接启动" },
    ],
    defaults: { template_id: freePlugs, randomization_profile: autoPrepare },
  };
  const state = {
    stateRequestSequence: 0,
    sceneDefaults: { simulation_rate: 1, capture_rate_hz: 30, ik_rate_hz: 120 },
    sceneTransitionPending: false,
  };
  const ctx = vm.createContext({
    AUTO_PREPARE_PROFILE: autoPrepare,
    ZERO_START_PROFILE: directStart,
    initialJointCatalog: null,
    configureInitialJoints() { configuredProfiles.push($("randomizationProfile").value); },
    initialJoints: { read: () => null, setRuntime() {} },
    armPreparation: { read: () => [90, -60, 60, 0, -90, 0], armAutoStart() {} },
    $, state, console,
    Option: function Option(label, value) { this.label = label; this.value = value; },
    setMessage() {}, applySceneRuntime() {}, refreshState: async () => {},
    apiRequest: async (path, options = {}) => {
      requests.push({ path, options });
      if (path === "/api/scenes/catalog") return { ok: true, json: async () => catalog };
      return { ok: true };
    },
    readApiResponse: async () => ({ instance_id: "s1", seed: 42 }),
  });
  vm.runInContext(selectionSource + catalogSource + startSource, ctx);
  return { $, ctx, requests, catalog, configuredProfiles };
}

test("free-plug template switches unsupported auto-prepare launches to direct start", async () => {
  const h = setup();
  await h.ctx.loadSceneCatalog();
  assert.equal(h.$("sceneTemplate").value, freePlugs);
  assert.equal(h.$("randomizationProfile").value, directStart);

  h.$("randomizationProfile").value = autoPrepare;
  h.ctx.configureSceneSelection();
  assert.equal(h.$("randomizationProfile").value, directStart);
  assert.deepEqual(h.configuredProfiles, [directStart, directStart]);

  await h.ctx.startScene();
  const request = h.requests.find(item => item.path === "/api/scenes/start");
  const body = JSON.parse(request.options.body);
  assert.equal(body.template_id, freePlugs);
  assert.equal(body.randomization_profile, directStart);
  assert.equal(body.initial_arm_joint_position_deg, null);
  assert.deepEqual(body.operating_arm_joint_position_deg, [90, -60, 60, 0, -90, 0]);
});

test("switching from the default scene works with older catalogs and preserves custom angles", async () => {
  const h = setup();
  const standard = "sarm-ground-validation-self-collision-grasp";
  const customAngles = [12, -65, -84, 140, -80, 35];
  delete h.catalog.templates[0].default_randomization_profile;
  delete h.catalog.templates[0].unsupported_randomization_profiles;
  h.catalog.templates.push({ id: standard, label: "SARM" });
  h.catalog.defaults.template_id = standard;
  h.ctx.armPreparation.read = () => customAngles;
  await h.ctx.loadSceneCatalog();
  assert.equal(h.$("randomizationProfile").value, autoPrepare);

  h.$("sceneTemplate").value = freePlugs;
  h.ctx.configureSceneSelection();
  assert.equal(h.$("randomizationProfile").value, directStart);
  assert.deepEqual(h.configuredProfiles, [autoPrepare, directStart]);
  await h.ctx.startScene();
  const body = JSON.parse(h.requests.find(item => item.path === "/api/scenes/start").options.body);
  assert.equal(body.randomization_profile, directStart);
  assert.deepEqual(body.operating_arm_joint_position_deg, customAngles);

  for (const profile of [directStart, "none", "training-v1", "teleop-balanced-v1"]) {
    h.$("randomizationProfile").value = profile;
    h.ctx.configureSceneSelection();
    assert.equal(h.$("randomizationProfile").value, profile);
  }
});

test("template and profile change events both run compatibility selection", () => {
  for (const id of ["sceneTemplate", "randomizationProfile"]) {
    assert.ok(source.includes(`$("${id}").addEventListener("change", configureSceneSelection)`));
  }
});
