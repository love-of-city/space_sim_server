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
  const context = vm.createContext({ state, $, performance: { now: () => time } });
  vm.runInContext(code, context);
  context.resetWebRtcStats();
  return { $, state, context, sample(t, received, decoded, lost = 0, videoStats = {}) {
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
