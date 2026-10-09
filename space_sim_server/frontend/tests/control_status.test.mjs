import assert from "node:assert/strict";
import test from "node:test";
import { referenceProtectionLabel, dynamicsTimingLabel } from "../control_status.js";

test("old telemetry does not falsely report active protection", () => {
  assert.match(referenceProtectionLabel({}), /未上报/);
  assert.equal(dynamicsTimingLabel({}), "—");
});

test("tracking restriction names affected joints and suggests recovery", () => {
  const label = referenceProtectionLabel({ reference_governor_state: "tracking_limited", reference_limited_joints: [1, 2], tracking_scale: 0.25 });
  assert.match(label, /反向退出/);
  assert.match(label, /J1\/J2/);
  assert.match(label, /0.25/);
});

test("saturation and holding are distinct states", () => {
  assert.match(referenceProtectionLabel({ reference_governor_state: "saturation_limited" }), /停止推进/);
  assert.match(referenceProtectionLabel({ reference_governor_state: "holding" }), /保持位置/);
});

test("physics step is reported independently of rendering FPS", () => {
  assert.equal(dynamicsTimingLabel({ dynamics_step_s: 0.001, ik_control_rate_hz: 100 }), "1.00 ms · IK 100 Hz");
  assert.equal(dynamicsTimingLabel({ dynamics_step_s: Infinity }), "—");
});
