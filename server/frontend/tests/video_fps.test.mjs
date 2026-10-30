import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import vm from "node:vm";
const source = fs.readFileSync(new URL("../app.js", import.meta.url), "utf8");
const code = source.slice(source.indexOf("function resetWebRtcStats("), source.indexOf("function disposePixelStream("));
function setup() {
  const nodes = new Map();
  const state = {};
  let time = 0, total = 0, dropped = 0;
  const player = { getVideoPlaybackQuality: () => ({ totalVideoFrames: total, droppedVideoFrames: dropped }) };
  const $ = id => { if (!nodes.has(id)) nodes.set(id, { textContent: "", querySelector: () => player }); return nodes.get(id); };
  const document = { hidden: false };
  const context = vm.createContext({ state, $, document, NumericParameters: { WebRTCFPS: "WebRTCFPS" }, performance: { now: () => time } });
  vm.runInContext(code, context);
  context.resetWebRtcStats();
  return { $, state, context, player, document, sample(t, received, decoded, lost = 0, videoStats = {}) {
    time = t; total = decoded; dropped = lost;
    context.updateWebRtcStats({ inboundVideoStats: { framesReceived: received, framesDecoded: decoded, timestamp: t, ...videoStats } });
  } };
}
test("90 received FPS does not falsely report 90 displayed FPS", () => {
  const p = setup(); p.sample(1000, 100, 100); p.sample(2000, 190, 190, 30);
  assert.equal(p.$("webrtcReceiveFps").textContent, "90");
  assert.equal(p.$("webrtcFps").textContent, "60");
});
test("stalled video reports zero, not the configured target", () => {
  const p = setup(); p.sample(1000, 50, 50); p.sample(2000, 50, 50);
  assert.equal(p.$("webrtcReceiveFps").textContent, "0");
  assert.equal(p.$("webrtcFps").textContent, "0");
});
test("reconnection clears counter baselines and invalid deltas", () => {
  const p = setup(); p.sample(1000, 100, 100); p.sample(2000, 5, 5);
  assert.equal(p.$("webrtcReceiveFps").textContent, "—");
  p.context.resetWebRtcStats(); p.sample(3000, 10, 10);
  assert.equal(p.$("webrtcFps").textContent, "—");
  assert.equal(p.$("webrtcReceiveFps").textContent, "—");
});

test("SDK bitrate is already kbps and must not be divided by 1000 again", () => {
  const p = setup();
  p.context.updateWebRtcStats({ inboundVideoStats: { bitrate: 9000 } });
  assert.equal(p.$("webrtcBitrate").textContent, "9000 kbps");
});

test("incoming frames without decoding report a stall and bound keyframe recovery", () => {
  const harness = setup();
  let requests = 0;
  harness.state.streamLive = true;
  harness.state.pixelStreaming = { requestIframe() { requests++; } };
  for (let seconds = 1; seconds <= 45; seconds++) harness.sample(seconds * 1000, seconds * 60, 100);
  assert.equal(harness.$("webrtcReceiveFps").textContent, "60");
  assert.equal(harness.$("webrtcDecodeFps").textContent, "0");
  assert.equal(harness.state.playbackHealth, "decode-stalled");
  assert.equal(harness.$("playStream").hidden, false);
  assert.equal(requests, 2);
  harness.sample(46000, 2760, 160);
  assert.equal(harness.state.playbackHealth, "playing");
  assert.equal(harness.$("playStream").hidden, true);
});

test("playing video with a stuck display counter is not classified as a decode stall", () => {
  const harness = setup();
  for (let seconds = 1; seconds <= 10; seconds++) {
    harness.player.currentTime = seconds;
    harness.sample(seconds * 1000, seconds * 60, 0, 0, { framesDecoded: seconds * 60 });
  }
  assert.equal(harness.$("webrtcFps").textContent, "0");
  assert.equal(harness.$("webrtcDecodeFps").textContent, "60");
  assert.equal(harness.state.playbackHealth, "playing");
});

test("decoding with a stopped media clock is a playback stall, not a decoding failure", () => {
  const harness = setup();
  harness.player.currentTime = 10;
  for (let seconds = 1; seconds <= 8; seconds++) harness.sample(seconds * 1000, seconds * 60, seconds * 60);
  assert.equal(harness.state.playbackHealth, "playback-stalled");
  assert.equal(harness.state.keyframeRecoveryAttempts, 0);
});

test("paused, hidden, and errored video have distinct states without keyframe retry loops", () => {
  const harness = setup();
  harness.player.paused = true;
  harness.sample(1000, 60, 0);
  harness.sample(10000, 600, 0);
  assert.equal(harness.state.playbackHealth, "paused");
  harness.document.hidden = true;
  harness.sample(11000, 660, 0);
  assert.equal(harness.state.playbackHealth, "hidden");
  harness.document.hidden = false;
  harness.player.paused = false;
  harness.sample(12000, 720, 0);
  assert.equal(harness.state.playbackHealth, "waiting");
  harness.player.error = { code: 3 };
  harness.sample(13000, 780, 0);
  assert.equal(harness.state.playbackHealth, "media-error");
  assert.equal(harness.state.videoDiagnostics.at(-1).mediaErrorCode, 3);
  harness.context.resetWebRtcStats();
  assert.equal(harness.state.playbackSample, null);
  assert.equal(harness.state.playbackHealth, "unknown");
});

test("packet loss reflects the latest interval instead of lifetime totals", () => {
  const harness = setup();
  harness.sample(1000, 100, 100, 0, { packetsReceived: 100000, packetsLost: 0 });
  assert.equal(harness.$("webrtcLoss").textContent, "—");
  harness.sample(2000, 160, 160, 0, { packetsReceived: 100090, packetsLost: 10 });
  assert.equal(harness.$("webrtcLoss").textContent, "10.0%");
  harness.sample(3000, 220, 220, 0, { packetsReceived: 100190, packetsLost: 9 });
  assert.equal(harness.$("webrtcLoss").textContent, "0.0%");
  harness.sample(4000, 280, 280, 0, { packetsReceived: 20, packetsLost: 0 });
  assert.equal(harness.$("webrtcLoss").textContent, "—");
  harness.context.resetWebRtcStats();
  assert.equal(harness.state.lastPacketSample, null);
});

test("missing loss counters and replacement tracks reset packet baselines", () => {
  const harness = setup();
  const sample = { timestamp: 1000, packetsReceived: 100, packetsLost: 3, id: "first", ssrc: 1 };
  assert.equal(harness.context.samplePacketLoss(sample), null);
  assert.equal(harness.context.samplePacketLoss({ ...sample, timestamp: 2000, id: "second" }), null);
  assert.equal(harness.context.samplePacketLoss({ ...sample, packetsLost: null }), null);
  assert.equal(harness.state.lastPacketSample, null);
});

test("FPS choices honor the deployment ceiling and send only valid targets", () => {
  const harness = setup();
  assert.equal(JSON.stringify(harness.context.videoFpsOptions()), "[30,60,90]");
  harness.state.streamConfig = { pixel_streaming_fps: 60 };
  assert.equal(JSON.stringify(harness.context.videoFpsOptions()), "[30,60]");
  const sent = [];
  harness.state.pixelConfig = { setNumericSetting: (...args) => sent.push(args) };
  assert.equal(harness.context.selectVideoFps("30"), true);
  assert.equal(harness.state.requestedVideoFps, 30);
  assert.deepEqual(sent, [["WebRTCFPS", 30]]);
  for (const invalid of [90, 0, "bad", 31]) assert.equal(harness.context.selectVideoFps(invalid), false);
  assert.equal(sent.length, 1);
});

test("diagnostics retain 60 real samples and expose no candidate address or credential", () => {
  const harness = setup();
  const stats = {
    inboundVideoStats: { framesReceived: 90, timestamp: 1000, bitrate: 4000 },
    getActiveCandidatePair: () => ({ localCandidateId: "local", remoteCandidateId: "remote", currentRoundTripTime: 0.05 }),
    localCandidates: [{ id: "local", candidateType: "relay", protocol: "udp", relayProtocol: "tcp", address: "10.0.0.1", usernameFragment: "secret-value" }],
    remoteCandidates: [{ id: "remote", candidateType: "host", address: "10.0.0.2" }],
  };
  for (let index = 0; index < 65; index++) harness.context.updateWebRtcStats(stats);
  assert.equal(harness.state.videoDiagnostics.length, 60);
  const latest = harness.state.videoDiagnostics.at(-1);
  assert.equal(latest.localCandidateType, "relay");
  assert.equal(latest.relayProtocol, "tcp");
  assert.equal(latest.rttMs, 50);
  assert.equal(latest.receivedFps, null);
  const output = JSON.stringify(harness.state.videoDiagnostics);
  assert.equal(output.includes("10.0.0."), false);
  assert.equal(output.includes("secret-value"), false);
  harness.context.resetWebRtcStats();
  assert.equal(harness.state.videoDiagnostics.length, 0);
});


test("average QP uses decoded-frame interval deltas, not lifetime or received-frame averages", () => {
  const p = setup();
  p.sample(1000, 200, 100, 0, { qpSum: 1000 });
  assert.equal(p.$("webrtcQp").textContent, "—");
  p.sample(2000, 290, 160, 0, { qpSum: 2800 });
  assert.equal(p.$("webrtcQp").textContent, "30.0");
  p.sample(3000, 380, 220, 0, { qpSum: 4960 });
  assert.equal(p.$("webrtcQp").textContent, "36.0");
});

test("unsupported or invalid QP counters show unavailable and reset the baseline", () => {
  for (const field of ["qpSum", "framesDecoded", "timestamp"]) {
    for (const invalid of [undefined, null, NaN, Infinity, "1000"]) {
      const p = setup();
      p.sample(1000, 100, 100, 0, { qpSum: 2000 });
      p.sample(2000, 190, 190, 0, { qpSum: 3800, [field]: invalid });
      assert.equal(p.$("webrtcQp").textContent, "—", `${field}=${invalid}`);
      assert.equal(p.state.lastQpSample, null);
      p.sample(3000, 280, 280, 0, { qpSum: 5600 });
      assert.equal(p.$("webrtcQp").textContent, "—");
      p.sample(4000, 370, 370, 0, { qpSum: 7400 });
      assert.equal(p.$("webrtcQp").textContent, "20.0");
    }
  }
  for (const field of ["qpSum", "framesDecoded"]) {
    const p = setup();
    p.sample(1000, 100, 100, 0, { qpSum: 2000, [field]: -1 });
    assert.equal(p.state.lastQpSample, null);
  }
});

test("missing video and reconnection clear QP instead of retaining stale quality", () => {
  for (const reset of [p => p.context.resetWebRtcStats(), p => p.context.updateWebRtcStats({})]) {
    const p = setup();
    p.sample(1000, 100, 100, 0, { qpSum: 2000 });
    p.sample(2000, 190, 190, 0, { qpSum: 3800 });
    reset(p);
    assert.equal(p.$("webrtcQp").textContent, "—");
    assert.equal(p.state.lastQpSample, null);
    p.sample(3000, 280, 280, 0, { qpSum: 5600 });
    assert.equal(p.$("webrtcQp").textContent, "—");
  }
});

test("QP rejects no-frame intervals, counter resets, and duplicate or backwards timestamps", () => {
  for (const sample of [
    { timestamp: 2000, framesDecoded: 100, qpSum: 2000 },
    { timestamp: 2000, framesDecoded: 50, qpSum: 3000 },
    { timestamp: 2000, framesDecoded: 150, qpSum: 1000 },
    { timestamp: 1000, framesDecoded: 150, qpSum: 3000 },
    { timestamp: 900, framesDecoded: 150, qpSum: 3000 },
  ]) {
    const p = setup();
    p.sample(1000, 100, 100, 0, { qpSum: 2000 });
    p.sample(2000, 190, 190, 0, sample);
    assert.equal(p.$("webrtcQp").textContent, "—");
    p.sample(3000, 280, sample.framesDecoded + 60, 0, { qpSum: sample.qpSum + 1200 });
    assert.equal(p.$("webrtcQp").textContent, "20.0");
  }
});

test("changing RTP stream, track or codec rebaselines QP even if counters increase", () => {
  for (const field of ["id", "ssrc", "trackIdentifier", "codecId"]) {
    const p = setup();
    p.sample(1000, 100, 100, 0, { qpSum: 2000, [field]: "old" });
    p.sample(2000, 190, 190, 0, { qpSum: 3800, [field]: "new" });
    assert.equal(p.$("webrtcQp").textContent, "—");
    p.sample(3000, 280, 280, 0, { qpSum: 5600, [field]: "new" });
    assert.equal(p.$("webrtcQp").textContent, "20.0");
  }
});

test("a genuine zero QP remains zero, unlike an unavailable QP", () => {
  const p = setup();
  p.sample(1000, 100, 100, 0, { qpSum: 0 });
  p.sample(2000, 190, 190, 0, { qpSum: 0 });
  assert.equal(p.$("webrtcQp").textContent, "0.0");
});
