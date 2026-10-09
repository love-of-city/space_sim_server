import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const startSource = source.slice(source.indexOf("async function startScene()"), source.indexOf("async function resetScene()"));
const stopSource = source.slice(source.indexOf("async function stopScene()"), source.indexOf("function connect()"));
const refreshSource = source.slice(source.indexOf("async function refreshState()"), source.indexOf("function updateEpisodeUI()"));

function deferred() {
  let resolve, reject;
  const promise = new Promise((accept, fail) => { resolve = accept; reject = fail; });
  return { promise, resolve, reject };
}

function snapshot(instanceId, phase = "running") {
  return { simulation: { connected: true }, active_episode: null,
    scene_runtime: { phase, active: phase !== "stopped", instance: { instance_id: instanceId } } };
}

function harness() {
  const state = vm.runInNewContext(source.slice(source.indexOf("const state ="), source.indexOf("const $ =")) + "state;");
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, { value: "", checked: false, disabled: false, textContent: "" });
    return nodes.get(id);
  };
  $("sceneSunlightIntensity").value = "1";
  $("randomizationProfile").value = "teleop-zero-prepare-v2";
  const requests = [], runtimes = [], armed = [], online = [], messages = [];
  const context = vm.createContext({ state, $, ZERO_START_PROFILE: "teleop-zero-prepare-v1",
    initialJoints: { read: () => null },
    armPreparation: { read: () => [0, -67.6, -86.6, 143.2, -85.5, 0], armAutoStart: id => armed.push(id) },
    apiRequest(path, options) {
      const request = { path, options, ...deferred() };
      requests.push(request);
      return request.promise;
    },
    readApiResponse: async response => response.payload,
    applySceneRuntime: runtime => runtimes.push(runtime),
    setOnline: (...args) => online.push(args),
    setMessage: message => messages.push(message),
    jointAngles: { clear() {} },
  });
  vm.runInContext(startSource + stopSource + refreshSource, context);
  return { context, state, requests, runtimes, armed, online, messages };
}

const flush = () => new Promise(resolve => setImmediate(resolve));
const respond = (request, data) => request.resolve({ json: async () => data });

for (const action of ["startScene", "stopScene"]) {
  for (const timing of ["during", "after"]) {
    for (const failure of [false, true]) {
      test(`${action} ignores old poll ${failure ? "errors" : "responses"} ${timing} transition`, async () => {
        const h = harness();
        const oldPoll = h.context.refreshState();
        const oldRequest = h.requests[0];
        const transition = h.context[action]();
        assert.equal(h.state.sceneTransitionPending, true);
        await h.context.refreshState();
        await h.context.startScene();
        await h.context.stopScene();
        assert.equal(h.requests.length, 2);
        const completeOld = async () => {
          if (failure) oldRequest.reject(new Error("old network failure"));
          else respond(oldRequest, { ...snapshot("old", "stopped"), simulation: { connected: false }, active_episode: "old-episode" });
          await oldPoll;
        };
        if (timing === "during") {
          await completeOld();
          assert.equal(h.runtimes.length, 0);
          assert.equal(h.online.length, 0);
        }
        const starting = action === "startScene";
        h.requests[1].resolve({ ok: true, payload: starting
          ? { instance_id: "new", seed: 42, phase: "launching" }
          : { instance: { instance_id: "new" }, phase: "stopped", active: false } });
        await flush();
        assert.equal(h.state.sceneTransitionPending, false);
        assert.equal(h.requests[2].path, "/api/state");
        assert.equal(h.runtimes[0].instance.instance_id, "new");
        if (starting) assert.equal(h.runtimes[0].phase, "launching");
        respond(h.requests[2], snapshot("new", starting ? "running" : "stopped"));
        await transition;
        if (timing === "after") await completeOld();
        assert.deepEqual(h.runtimes.map(runtime => runtime.instance.instance_id), ["new", "new"]);
        assert.deepEqual(h.armed, starting ? ["new"] : []);
        assert.equal(h.state.simulationConnected, true);
        assert.equal(h.state.activeEpisode, null);
        assert.equal(h.online.some(([id]) => id === "backendDot"), false);
      });
    }
  }

  test(`${action} failure releases polling without arming`, async () => {
    const h = harness();
    const transition = h.context[action]();
    h.requests[0].resolve({ ok: false, payload: { detail: "transition failed" } });
    await flush();
    assert.equal(h.state.sceneTransitionPending, false);
    respond(h.requests[1], snapshot("existing"));
    await transition;
    assert.deepEqual(h.messages.slice(-1), ["transition failed"]);
    assert.deepEqual(h.armed, []);
    assert.equal(h.runtimes.length, 1);
  });
}

test("overlapping polls cannot apply older JSON after a newer poll", async () => {
  const h = harness();
  const oldBody = deferred();
  const oldPoll = h.context.refreshState();
  h.requests[0].resolve({ json: () => oldBody.promise });
  await flush();
  const newPoll = h.context.refreshState();
  respond(h.requests[1], snapshot("new"));
  await newPoll;
  oldBody.resolve(snapshot("old", "stopped"));
  await oldPoll;
  assert.deepEqual(h.runtimes.map(runtime => runtime.instance.instance_id), ["new"]);
  assert.deepEqual(h.armed, []);
});

test("a scene transition invalidates a poll already decoding JSON", async () => {
  const h = harness();
  const oldBody = deferred();
  const oldPoll = h.context.refreshState();
  h.requests[0].resolve({ json: () => oldBody.promise });
  await flush();
  const transition = h.context.startScene();
  oldBody.resolve(snapshot("old", "stopped"));
  await oldPoll;
  assert.equal(h.runtimes.length, 0);
  h.requests[1].resolve({ ok: true, payload: { instance_id: "new", phase: "launching" } });
  await flush();
  respond(h.requests[2], snapshot("new"));
  await transition;
  assert.deepEqual(h.armed, ["new"]);
});
