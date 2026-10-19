import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";
import { resolvePreviewStreams } from '../preview_streams.js';

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const lifecycle = source.slice(source.indexOf("function disposePixelStream()"), source.indexOf("function applyControlLimits("));
const relativeView = source.slice(source.indexOf("function requestRelativeView("), source.indexOf("function setFreeCameraMode("));

function harness() {
  let nextId = 0;
  let now = 100000;
  const timers = new Map();
  const players = [];
  const discoveries = [];
  const nodes = new Map();
  const state = { pixelStreaming: null, streamReconnectTimer: null, streamLive: false };
  const $ = (id) => {
    if (!nodes.has(id)) nodes.set(id, { style: {}, replaceChildren() {}, textContent: "", hidden: false,
      querySelector() { return state.pixelStreaming?.video; } });
    return nodes.get(id);
  };
  class Player {
    constructor() {
      this.handlers = new Map(); this.responses = new Map(); this.commands = []; players.push(this);
      this.frames = 1;
      this.video = {readyState: 2, videoWidth: 1280, paused: false, currentTime: 0,
        getVideoPlaybackQuality: () => ({totalVideoFrames: this.frames}),
        requestVideoFrameCallback: callback => { this.onFrame = callback; }};
    }
    addEventListener(name, fn) { this.handlers.set(name, fn); }
    addResponseEventListener(name, fn) { this.responses.set(name, fn); }
    emitCommand(command) { this.commands.push(command); return true; }
    emit(name, data = {}) { this.handlers.get(name)?.(data); }
    disconnect() { this.emit("webRtcDisconnected"); }
  }
  let reconnects = 0;
  let statsUpdates = 0;
  const context = vm.createContext({clearWristWrench() {},clearSimulationTelemetry() {},
    state, $, window: {}, document: {hidden: false}, Date: {now: () => now}, PixelStreaming: Player, Config: class {
      constructor(options) { this.initialSettings = options.initialSettings; }
      setNumericSetting(key, value) { this.initialSettings[key] = value; }
      setOptionSettingValue(key, value) { (this.optionUpdates ||= []).push([key, value]); }
    }, queueMicrotask,
    TextParameters: {}, OptionParameters: {StreamerId: "StreamerId"}, NumericParameters: { WebRTCFPS: "WebRTCFPS", MinQuality: "MinQuality" }, Flags: {},
    signallingUrl: () => "ws://localhost/", setPixelStreamingInputEnabled() {},
    observePreviewAvailability(url, onIds) {
      const discovery = {onIds, closed: false, close() { this.closed = true; }};
      discoveries.push(discovery);
      return discovery;
    },
    resetWebRtcStats() {}, updateWebRtcStats() { statsUpdates++; },
    connectPixelStreaming() { reconnects++; },
    updatePreviewStreams(ids) {
      state.selectedStreamerId = resolvePreviewStreams(state.streamConfig.pixel_streaming_streamers, ids, state.selectedStreamerId).selectedId;
    },
    setFreeCameraMode(enabled) { state.freeCameraMode = enabled; },
    setTimeout(fn, delay) { const id = ++nextId; timers.set(id, { fn, delay }); return id; },
    clearTimeout(id) { timers.delete(id); },
  });
  vm.runInContext(relativeView + lifecycle, context);
  return { context, state, players, discoveries, $, allTimers: timers,
    get timers() { return new Map([...timers].filter(([id]) => id !== state.streamMediaTimer)); },
    tickMedia(ms) { now += ms; const id = state.streamMediaTimer; const timer = timers.get(id); timers.delete(id); timer?.fn(); },
    get reconnects() { return reconnects; }, get statsUpdates() { return statsUpdates; } };
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

test('camera discovery switches a waiting logical camera once without recreating the player', () => {
  const h = harness();
  h.state.streamConfig = {pixel_streaming_streamers: [{id: 'main'}, {id: 'arm-wrist', aliases: ['flight-wrist']}]};
  h.state.selectedStreamerId = 'arm-wrist';
  h.context.createPixelStream();
  const event = {data: {messageStreamerList: {ids: ['main', 'flight-wrist']}}};
  h.players[0].emit('streamerListMessage', event);
  h.players[0].emit('streamerListMessage', event);
  assert.equal(h.state.selectedStreamerId, 'flight-wrist');
  assert.deepEqual(h.state.pixelConfig.optionUpdates, [['StreamerId', 'flight-wrist']]);
  assert.equal(h.players.length, 1);
});

test('discovering flight cameras preserves a healthy main preview', () => {
  const h = harness();
  h.state.streamConfig = {pixel_streaming_streamers: [{id: 'main'}, {id: 'arm-wrist', aliases: ['flight-wrist']}]};
  h.state.selectedStreamerId = 'main';
  h.context.createPixelStream();
  h.players[0].emit('webRtcConnected');
  h.players[0].emit('streamerListMessage', {data: {messageStreamerList: {ids: ['main', 'flight-wrist']}}});
  assert.equal(h.state.selectedStreamerId, 'main');
  assert.equal(h.state.pixelConfig.optionUpdates, undefined);
  assert.equal(h.timers.size, 0);
});

test('late camera discovery updates a playing preview without replacing its peer', () => {
  const h = harness();
  h.state.streamConfig = {pixel_streaming_streamers: [{id:'main'}, {id:'wrist'}]};
  h.state.selectedStreamerId = 'main';
  h.context.createPixelStream();
  h.players[0].emit('webRtcConnected');
  h.discoveries[0].onIds(['main','wrist']);
  h.players[0].emit('streamerListMessage', {data:{messageStreamerList:{ids:['main']}}});
  assert.deepEqual(h.state.previewAvailableIds, ['main','wrist']);
  assert.equal(h.state.pixelConfig.optionUpdates, undefined);
  assert.equal(h.players.length, 1);
  h.context.createPixelStream();
  assert.equal(h.discoveries[0].closed, true);
  h.discoveries[0].onIds(['retired']);
  assert.equal(h.state.previewAvailableIds, null);
});

test("selected FPS is reapplied after server settings and survives reconnection", async () => {
  const harness = harnessForFps();
  harness.context.createPixelStream();
  harness.state.pixelConfig.initialSettings.WebRTCFPS = 90;
  harness.players[0].emit("initialSettings");
  await Promise.resolve();
  assert.equal(harness.state.pixelConfig.initialSettings.WebRTCFPS, 30);
  harness.context.createPixelStream();
  assert.equal(harness.state.pixelConfig.initialSettings.WebRTCFPS, 30);
  const previousConfig = harness.state.pixelConfig;
  harness.players[1].emit("initialSettings");
  harness.context.createPixelStream();
  previousConfig.initialSettings.WebRTCFPS = 60;
  await Promise.resolve();
  assert.equal(previousConfig.initialSettings.WebRTCFPS, 60);
});

function harnessForFps() {
  const result = harness();
  result.state.requestedVideoFps = 30;
  return result;
}

test("late events from an old player cannot clear LIVE state or schedule reconnects", () => {
  const h = harness();
  h.context.createPixelStream();
  const old = h.players[0];
  h.context.createPixelStream();
  h.players[1].emit("webRtcConnected");
  h.players[1].emit("videoInitialized");
  for (const event of ["webRtcConnecting", "webRtcConnected", "videoInitialized", "playStreamRejected", "webRtcDisconnected", "webRtcFailed", "subscribeFailed", "statsReceived", "dataChannelClose"]) old.emit(event);
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

test("media initialization keeps the loading view until a real frame arrives", () => {
  const h = harness(); h.context.createPixelStream();
  const player = h.players[0]; player.frames = 0;
  player.emit("webRtcConnected"); player.emit("videoInitialized");
  assert.equal(h.$("frameState").textContent, "WAITING FOR FRAMES");
  assert.notEqual(h.$("previewEmpty").style.display, "none");
  assert.equal(h.timers.size, 0, "a connected player must survive slow model loading");
  player.frames = 30; player.onFrame();
  assert.equal(h.$("frameState").textContent, "LIVE WEBRTC");
  assert.equal(h.$("previewEmpty").style.display, "none");
});

test("a retired player's delayed first-frame callback cannot hide the new loading view", () => {
  const h = harness(); h.context.createPixelStream();
  const old = h.players[0]; old.frames = 0; old.emit("videoInitialized");
  h.context.createPixelStream();
  old.frames = 30; old.onFrame();
  assert.equal(h.$("frameState").textContent, "CONNECTING");
  assert.notEqual(h.$("previewEmpty").style.display, "none");
});

test("disposing the player clears timers without being resurrected by SDK callbacks", () => {
  const h = harness();
  h.context.createPixelStream();
  h.context.disposePixelStream();
  assert.equal(h.state.pixelStreaming, null);
  assert.equal(h.timers.size, 0);
  assert.equal(h.allTimers.size, 0);
  h.players[0].emit("videoInitialized");
  assert.equal(h.state.streamLive, false);
});

test("relative view discovers a late manifest without resetting a shared camera", () => {
  const h = harness();
  h.state.streamConfig = {pixel_streaming_streamer_id: "main"};
  h.state.selectedStreamerId = "main";
  h.state.mission = {stage: "flight"};
  h.context.createPixelStream();
  const player = h.players[0];
  player.emit("dataChannelOpen");
  player.responses.get("relative-view")('{"type":"BskRelativeView","mode":""}');
  assert.equal(h.$("relativeViewSelector").disabled, true);
  player.emit("webRtcConnected");
  player.emit("statsReceived");
  assert.equal(player.commands.length, 2);
  for (const command of player.commands) assert.equal(JSON.parse(command.BskRelativeView).mode, "query");
  player.responses.get("relative-view")('{"type":"BskRelativeView","mode":"overview-fixed","range_m":30000}');
  h.state.streamLive = true;
  h.context.updateRelativeViewControl();
  assert.equal(h.$("relativeViewSelector").disabled, false);
  assert.equal(h.$("relativeViewSelector").value, "overview-fixed");
  assert.equal(h.state.relativeView.range_m, 30000);
  player.emit("statsReceived");
  assert.equal(player.commands.length, 2, "statistics cannot flood the data channel");
  h.state.mission.stage = "teleop";
  h.context.updateRelativeViewControl();
  assert.equal(h.$("relativeViewSelector").disabled, true);
});

test("retired and malformed camera responses cannot replace the current view", () => {
  const h = harness(); h.context.createPixelStream();
  const old = h.players[0];
  h.context.createPixelStream();
  const response = h.players[1].responses.get("relative-view");
  response('{"type":"BskRelativeView","mode":"overview"}');
  for (const value of ["invalid", "null", '{"type":"other","mode":"target"}', '{"type":"BskRelativeView","mode":"unsupported"}']) response(value);
  old.responses.get("relative-view")('{"type":"BskRelativeView","mode":"target"}');
  old.emit("dataChannelOpen"); old.emit("statsReceived");
  assert.equal(h.state.relativeView.mode, "overview");
  assert.equal(h.players[1].commands.length, 0);
});


test("replacing or losing an active stream exits free-camera mode", () => {
  for (const event of ["dispose", "webRtcDisconnected", "webRtcFailed", "subscribeFailed", "dataChannelClose"]) {
    const h = harness(); h.context.createPixelStream();
    h.players[0].emit("webRtcConnected"); h.state.freeCameraMode = true;
    if (event === "dispose") h.context.disposePixelStream(); else h.players[0].emit(event);
    assert.equal(h.state.freeCameraMode, false);
  }
});

test("a stalled media clock recovers even when ICE still reports connected", () => {
  const h = harness(); h.context.createPixelStream();
  const player = h.players[0]; player.emit("webRtcConnected"); player.emit("videoInitialized");
  h.tickMedia(5000);
  h.tickMedia(55000);
  assert.equal(h.reconnects, 0, "cold startup keeps its full grace interval");
  h.tickMedia(5000);
  assert.equal(h.state.streamLive, false);
  assert.equal(h.$("frameState").textContent, "VIDEO RECOVERING");
  assert.equal(h.$("previewEmpty").style.display, "grid");
  assert.equal(h.timers.size, 1);
  [...h.timers.values()][0].fn();
  assert.equal(h.reconnects, 1);
});

test("progressing, paused and background video do not trigger automatic replacement", () => {
  for (const mode of ["playing", "paused", "hidden"]) {
    const h = harness(); h.context.createPixelStream();
    const player = h.players[0]; player.emit("webRtcConnected");
    for (let i = 1; i <= 30; i++) {
      player.video.currentTime = mode === "playing" ? i : 0;
      player.video.paused = mode === "paused";
      h.context.document.hidden = mode === "hidden";
      h.tickMedia(5000);
    }
    assert.equal(h.state.streamLive, true);
    assert.equal(h.timers.size, 0);
    assert.equal(h.reconnects, 0);
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


test("frontend SDK uses the server FPS target rather than falling back to 60", () => {
  const h = harness();
  h.state.streamConfig = { pixel_streaming_fps: 120 };
  h.context.createPixelStream();
  assert.equal(h.state.pixelConfig.initialSettings.WebRTCFPS, 120);
  h.state.streamConfig = {};
  h.context.createPixelStream();
  assert.equal(h.state.pixelConfig.initialSettings.WebRTCFPS, 90);
});


test("frontend SDK preserves the server quality floor including explicit zero", () => {
  const h = harness();
  for (const quality of [0, 60, 75, 100]) {
    h.state.streamConfig = { pixel_streaming_encoder_min_quality: quality };
    h.context.createPixelStream();
    assert.equal(h.state.pixelConfig.initialSettings.MinQuality, quality);
  }
  for (const config of [undefined, {}, { pixel_streaming_encoder_min_quality: null }]) {
    h.state.streamConfig = config;
    h.context.createPixelStream();
    assert.equal(h.state.pixelConfig.initialSettings.MinQuality, 60);
  }
});
