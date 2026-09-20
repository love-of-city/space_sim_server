import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const resetSource = source.slice(source.indexOf("async function resetScene()"), source.indexOf("async function stopScene()"));

function harness(response) {
  const state = {sceneReady: true, resetPending: false, simulationResetting: false,
    operationRequested: true, operationActive: true, pressed: new Set(["KeyW"])};
  const button = {disabled: false, textContent: "重置状态"};
  let calls = 0, refreshed = 0;
  const messages = [];
  const ctx = vm.createContext({state, $: () => button, Error,
    exitOperationMode() {state.operationActive = false; state.pressed.clear();},
    updateEpisodeUI() {},
    async apiRequest(path, options) {
      calls++;
      assert.equal(path, "/api/scenes/reset"); assert.equal(options.method, "POST");
      return await response;
    },
    async readApiResponse(r) {return r.payload;},
    setMessage(message) {messages.push(message);},
    async refreshState() {refreshed++;},
  });
  vm.runInContext(resetSource, ctx);
  return {ctx, state, button, messages, get calls() {return calls;}, get refreshed() {return refreshed;}};
}

test("reset immediately disarms held input, blocks double clicks and does not auto-rearm", async () => {
  let done;
  const h = harness(new Promise(resolve => {done = resolve;}));
  const pending = h.ctx.resetScene();
  assert.equal(h.state.sceneReady, false);
  assert.equal(h.state.operationActive, false);
  assert.equal(h.state.operationRequested, false);
  assert.equal(h.state.pressed.size, 0);
  assert.equal(h.button.disabled, true);
  await h.ctx.resetScene(); assert.equal(h.calls, 1);
  done({ok: true, payload: {status: "completed"}});
  await pending;
  assert.equal(h.state.resetPending, false);
  assert.equal(h.state.operationActive, false);
  assert.equal(h.refreshed, 1);
  assert.match(h.messages.at(-1), /已恢复/);
});

test("reset errors are visible and do not re-enable operation", async () => {
  const h = harness({ok: false, payload: {detail: "请先结束当前采集"}});
  await h.ctx.resetScene();
  assert.equal(h.messages.at(-1), "请先结束当前采集");
  assert.equal(h.state.operationActive, false);
  assert.equal(h.state.resetPending, false);
  assert.equal(h.refreshed, 1);
});

test("server-side reset also blocks duplicate requests", async () => {
  const h = harness({ok: true, payload: {}});
  h.state.simulationResetting = true;
  await h.ctx.resetScene();
  assert.equal(h.calls, 0);
});

test("one click restore is visible beside the viewport and has a unique ID", () => {
  const html = readFileSync(new URL("../index.html", import.meta.url), "utf8");
  assert.equal((html.match(/id="resetScene"/g) || []).length, 1);
  assert.ok(html.indexOf('id="resetScene"') < html.indexOf('<aside class="control-rail"'));
  assert.match(html, /一键复原初始位置/);
});

test("recording prevents accidental scene restore", async () => {
  const h = harness({ok: true, payload: {status: "completed"}});
  h.state.activeEpisode = "recording";
  await h.ctx.resetScene();
  assert.equal(h.calls, 0);
  assert.match(h.messages.at(-1), /先结束当前采集/);
});

test("disconnected or unauthorized restore does not send a request", async () => {
  for (const flag of ["canManageScene", "resetSupported", "simulationConnected"]) {
    const h = harness({ok: true, payload: {}});
    h.state[flag] = false;
    await h.ctx.resetScene();
    assert.equal(h.calls, 0);
  }
});
