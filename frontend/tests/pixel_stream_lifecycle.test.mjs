import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const lifecycle = source.slice(source.indexOf("function disposePixelStream()"), source.indexOf("function applyControlLimits("));

function harness() {
  let nextId = 0;
  const timers = new Map();
  const players = [];
  const nodes = new Map();
  const state = { pixelStreaming: null, streamReconnectTimer: null, streamLive: false };
  const $ = (id) => {
    if (!nodes.has(id)) nodes.set(id, { style: {}, replaceChildren() {}, textContent: "", hidden: false });
    return nodes.get(id);
  };
  class Player {
    constructor() { this.handlers = new Map(); players.push(this); }
    addEventListener(name, fn) { this.handlers.set(name, fn); }
    emit(name, data = {}) { this.handlers.get(name)?.(data); }
    disconnect() { this.emit("webRtcDisconnected"); }
  }
  let reconnects = 0;
  let statsUpdates = 0;
  const context = vm.createContext({
    state, $, window: {}, PixelStreaming: Player, Config: class {},
    TextParameters: {}, OptionParameters: {}, Flags: {},
    signallingUrl: () => "ws://localhost/", setPixelStreamingInputEnabled() {},
    resetWebRtcStats() {}, updateWebRtcStats() { statsUpdates++; },
    connectPixelStreaming() { reconnects++; },
    setFreeCameraMode(enabled) { state.freeCameraMode = enabled; },
    setTimeout(fn, delay) { const id = ++nextId; timers.set(id, { fn, delay }); return id; },
    clearTimeout(id) { timers.delete(id); },
  });
  vm.runInContext(lifecycle, context);
  return { context, state, timers, players, $, get reconnects() { return reconnects; }, get statsUpdates() { return statsUpdates; } };
}

test("disconnecting a retired player cannot create an orphan reconnect timer", () => {
  const h = harness();
  h.context.createPixelStream();
  h.context.createPixelStream();
  assert.equal(h.players.length, 2);
  assert.equal(h.timers.size, 1, "only the current player's startup watchdog remains");
  assert.equal([...h.timers.values()][0].delay, 60000);
  h.players[1].emit("webRtcConnected");
  assert.equal(h.timers.size, 0);
});

test("late events from an old player cannot clear LIVE state or schedule reconnects", () => {
  const h = harness();
  h.context.createPixelStream();
  const old = h.players[0];
  h.context.createPixelStream();
  h.players[1].emit("webRtcConnected");
  h.players[1].emit("videoInitialized");
  for (const event of ["webRtcConnecting", "webRtcConnected", "videoInitialized", "playStreamRejected", "webRtcDisconnected", "webRtcFailed", "subscribeFailed", "statsReceived"]) old.emit(event);
  assert.equal(h.$("frameState").textContent, "LIVE WEBRTC");
  assert.equal(h.state.streamLive, true);
  assert.equal(h.timers.size, 0);
  assert.equal(h.statsUpdates, 0);
});

test("a genuine disconnect schedules exactly one managed retry", () => {
  const h = harness();
  h.context.createPixelStream();
  h.players[0].emit("webRtcConnected");
  h.players[0].emit("webRtcDisconnected");
  h.players[0].emit("webRtcFailed");
  assert.equal(h.timers.size, 1);
  assert.equal(h.state.streamLive, false);
  [...h.timers.values()][0].fn();
  assert.equal(h.reconnects, 1);
});

test("disposing the player clears timers without being resurrected by SDK callbacks", () => {
  const h = harness();
  h.context.createPixelStream();
  h.context.disposePixelStream();
  assert.equal(h.state.pixelStreaming, null);
  assert.equal(h.timers.size, 0);
  h.players[0].emit("videoInitialized");
  assert.equal(h.state.streamLive, false);
});


test("replacing or losing an active stream exits free-camera mode", () => {
  for (const event of ["dispose", "webRtcDisconnected", "webRtcFailed", "subscribeFailed"]) {
    const h = harness(); h.context.createPixelStream();
    h.players[0].emit("webRtcConnected"); h.state.freeCameraMode = true;
    if (event === "dispose") h.context.disposePixelStream(); else h.players[0].emit(event);
    assert.equal(h.state.freeCameraMode, false);
  }
});


test("stream discovery and RTC negotiation replace the old startup deadline", () => {
  const h = harness(); h.context.createPixelStream();
  const stream = h.players[0];
  const firstTimer = h.state.streamReconnectTimer;
  stream.emit("streamerListMessage");
  assert.equal(h.timers.has(firstTimer), false);
  const discoveryTimer = h.state.streamReconnectTimer;
  stream.emit("webRtcConnecting");
  assert.equal(h.timers.has(discoveryTimer), false);
  assert.equal(h.timers.size, 1);
  assert.equal([...h.timers.values()][0].delay, 60000);
  assert.equal(h.players.length, 1, "a newly joining player must not be replaced");
  stream.emit("webRtcConnected");
  stream.emit("streamerListMessage");
  assert.equal(h.timers.size, 0, "discovery cannot rearm a LIVE player's watchdog");
});

test("a renderer becoming running does not tear down its waiting player", () => {
  const start = source.indexOf('  if (phase === "running"');
  const end = source.indexOf("  updateEpisodeUI();", start);
  const block = source.slice(start, end);
  for (const waiting of [true, false]) {
    let reconnects = 0;
    const context = vm.createContext({phase: "running", previousPhase: "starting_renderer",
      instance: {}, state: {pixelStreaming: waiting ? {} : null, pixelConnectPromise: null},
      setMessage() {}, connectPixelStreaming() { reconnects++; }});
    vm.runInContext(block, context);
    assert.equal(reconnects, waiting ? 0 : 1);
  }
});
