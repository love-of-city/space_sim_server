import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const start = source.indexOf('const obs = message.payload;', source.indexOf('message.type === "observation"'));
const observationHandler = source.slice(start, source.indexOf('} else if (message.type === "action_ack")', start));

function observe(payload) {
  const nodes = new Map();
  let output;
  const $ = (id) => {
    if (!nodes.has(id)) nodes.set(id, { textContent: "" });
    return nodes.get(id);
  };
  vm.runInNewContext(observationHandler, {
    message: { payload }, $, setOnline() {},
    updateMotionOutputs(twist, grip) { output = { twist, grip }; },
  });
  return { output, $ };
}

test("SARM telemetry uses a slider velocity, not the sixth revolute joint", () => {
  const h = observe({
    sim_time_ns: "1000000000", applied_action_sequence: "42", jacobian_rank: 5,
    joint_velocity_rad_s: [0, 0, 0, 0, 0, 0.25, 0.005, 0.005],
    end_effector_twist_body: [0.01, 0, 0, 0, 0, 0],
  });
  assert.equal(h.output.grip, 0.005);
  assert.equal(h.$("jacobianRank").textContent, "5 / 6");
});

test("missing SARM telemetry remains safe before the first full observation", () => {
  const h = observe({ sim_time_ns: "0" });
  assert.equal(h.output.grip, 0);
  assert.equal(h.$("jacobianRank").textContent, "— / 6");
});

test("explicit SI slider telemetry takes precedence over the legacy alias", () => {
  const h = observe({sim_time_ns: "0", gripper_velocity_m_s: [0.004, 0.004], joint_velocity_rad_s: [0,0,0,0,0,0,99,99]});
  assert.equal(h.output.grip, 0.004);
});

test("attitude hold, wheel speed and saturation are displayed separately", () => {
  const h = observe({sim_time_ns: "0", attitude_control: {
    enabled: true, reference_initialized: true, saturated: true,
    attitude_error_angle_rad: Math.PI/180, angular_velocity_body_rad_s: [0.01,0,0],
  }, reaction_wheels: {speed_rad_s: [Math.PI*2,0,0], applied_motor_torque_nm: [0.2,0,0]}});
  assert.equal(h.$("attitudeMode").textContent, "惯性保持 · 饱和");
  assert.equal(h.$("attitudeError").textContent, "1.000 °");
  assert.equal(h.$("wheelSpeeds").textContent, "60.0, 0.0, 0.0 rpm");
  assert.equal(h.$("wheelTorques").textContent, "0.200, 0.000, 0.000 N·m");
});
