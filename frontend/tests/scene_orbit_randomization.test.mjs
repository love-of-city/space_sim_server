import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const startSource = source.slice(source.indexOf("async function startScene()"), source.indexOf("async function stopScene()"));
const applySource = source.slice(source.indexOf("function applySceneRuntime("), source.indexOf("async function readApiResponse("));
const catalogSource = source.slice(source.indexOf("async function loadSceneCatalog()"), source.indexOf("function applySceneRuntime("));

function setup() {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, { value: "", checked: false, disabled: false,
      textContent: "", classList: { toggle() {} }, replaceChildren() {}, focus() {} });
    return nodes.get(id);
  };
  $("sceneSunlightIntensity").value = "1";
  $("sceneTemplate").value = "spacecraft-arm-teleop";
  $("randomizationProfile").value = "training-v1";
  const requests = [];
  const state = { sceneDefaults: { simulation_rate: 1, capture_rate_hz: 10, ik_rate_hz: 100 },
    currentUser: { role: "admin" }, scenePhase: "idle" };
  const ctx = vm.createContext({ $, state, console, Option: function() {}, scenePhaseLabels: {},
    setMessage() {}, updateOperationUI() {}, updateEpisodeUI() {}, connectPixelStreaming() {},
    exitOperationMode() {}, refreshState: async () => {}, applySceneRuntime() {},
    readApiResponse: async () => ({ instance_id: "s1", seed: 42 }),
    apiRequest: async (path, options = {}) => {
      requests.push({ path, options });
      return { ok: true, json: async () => ({ templates: [], randomization_profiles: [], defaults: {} }) };
    },
  });
  return { $, ctx, state, requests };
}

test("orbital start checkbox is accessible and off by default", () => {
  const html = readFileSync(new URL("../index.html", import.meta.url), "utf8");
  const input = html.match(/<input[^>]+id="sceneRandomizeOrbitPhase"[^>]*>/)?.[0];
  assert.ok(input);
  assert.match(input, /type="checkbox"/);
  assert.match(input, /aria-describedby="sceneOrbitPhaseHelp"/);
  assert.doesNotMatch(input, /\bchecked\b/);
  assert.match(html, /id="sceneInstanceOrbitPhase"/);
});

test("start transmits the independent orbital flag and preserves seed semantics", async () => {
  for (const enabled of [false, true]) {
    for (const seed of ["", "0", "42"]) {
      for (const profile of ["none", "training-v1"]) {
        const h = setup();
        h.$("sceneRandomizeOrbitPhase").checked = enabled;
        h.$("sceneSeed").value = seed;
        h.$("randomizationProfile").value = profile;
        vm.runInContext(startSource, h.ctx);
        await h.ctx.startScene();
        assert.equal(h.requests[0].path, "/api/scenes/start");
        const body = JSON.parse(h.requests[0].options.body);
        assert.equal(body.randomize_orbit_phase, enabled);
        assert.equal(body.seed, seed === "" ? null : Number(seed));
        assert.equal(body.randomization_profile, profile);
      }
    }
  }
});

test("catalog defaults and older backends leave the orbital option off", async () => {
  for (const value of [undefined, false, true]) {
    const h = setup();
    h.$("sceneRandomizeOrbitPhase").checked = true;
    h.ctx.apiRequest = async () => ({ ok: true, json: async () => ({ templates: [], randomization_profiles: [],
      defaults: { randomize_orbit_phase: value } }) });
    vm.runInContext(catalogSource, h.ctx);
    await h.ctx.loadSceneCatalog();
    assert.equal(h.$("sceneRandomizeOrbitPhase").checked, value === true);
  }
});

test("active scene restores and locks the saved option and displays the actual angle including zero", () => {
  for (const phase of [0, 90, 271.234567]) {
    const h = setup();
    vm.runInContext(applySource, h.ctx);
    h.ctx.applySceneRuntime({ phase: "running", active: true, instance: {
      instance_id: "s1", seed: 42, randomize_orbit_phase: true,
      environment: { orbit: { true_anomaly_deg: phase } }, randomization: {},
    } });
    assert.equal(h.$("sceneRandomizeOrbitPhase").checked, true);
    assert.equal(h.$("sceneRandomizeOrbitPhase").disabled, true);
    assert.equal(h.$("sceneInstanceOrbitPhase").textContent, `${phase.toFixed(2)}°`);
    const displayed = JSON.parse(h.$("sceneParameters").textContent);
    assert.equal(displayed.randomize_orbit_phase, true);
    assert.equal(displayed.environment.orbit.true_anomaly_deg, phase);
  }
});

test("inactive polling does not overwrite the next launch selection; old scenes stay compatible", () => {
  const h = setup();
  vm.runInContext(applySource, h.ctx);
  h.$("sceneRandomizeOrbitPhase").checked = true;
  h.ctx.applySceneRuntime({ phase: "stopped", active: false, instance: {
    instance_id: "old", randomization: {},
  } });
  assert.equal(h.$("sceneRandomizeOrbitPhase").checked, true);
  assert.equal(h.$("sceneRandomizeOrbitPhase").disabled, false);
  assert.equal(h.$("sceneInstanceOrbitPhase").textContent, "180.00°");
  h.ctx.applySceneRuntime({ phase: "running", active: true, instance: { instance_id: "old", randomization: {} } });
  assert.equal(h.$("sceneRandomizeOrbitPhase").checked, false);
  h.ctx.applySceneRuntime({ phase: "idle", active: false });
  assert.equal(h.$("sceneRandomizeOrbitPhase").disabled, false);
  assert.equal(h.$("sceneInstanceOrbitPhase").textContent, "—");
});
