import { formatMotionSpeedDiagnostics } from "../motion_diagnostics.js";
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
  let jointObservation;
  const $ = (id) => {
    if (!nodes.has(id)) nodes.set(id, { textContent: "" });
    return nodes.get(id);
  };
  vm.runInNewContext(observationHandler, {
    armPreparation: { update() {} },
    message: { payload }, $, setOnline() {}, formatMotionSpeedDiagnostics,
    jointAngles: { update(obs) { jointObservation = obs; } },
    updateMotionOutputs(twist, grip) { output = { twist, grip }; },
  });
  return { output, $, jointObservation };
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

test("actual observation is forwarded unchanged to the joint-angle panel", () => {
  const payload = {sim_time_ns: "0", arm_joint_position_rad: [0,0,0,0,0,5.2],
    target_arm_joint_position_rad: [1,1,1,1,1,1], arm_joint_limits_rad: Array(6).fill([-6.28,6.28])};
  assert.equal(observe(payload).jointObservation, payload);
});

test("arch preference displays measured wrist drop with sign, not predicted height", () => {
  const h = observe({sim_time_ns: "0", ik_mode: "ik_pose", ik_elbow_preference: {
    enabled: true, wrist_enabled: true, status: "active", measured_height_m: .123,
    reference_height_m: .456, measured_wrist_drop_m: -.012, reference_wrist_drop_m: .05,
  }});
  assert.match(h.$("ikSolver").textContent, /构型偏好=active/);
  assert.match(h.$("ikSolver").textContent, /肘高=0\.123m/);
  assert.match(h.$("ikSolver").textContent, /J4−J6=-0\.012m/);
  assert.doesNotMatch(h.$("ikSolver").textContent, /0\.456/);
});

test("old/off wrist telemetry remains compatible and does not claim wrist control", () => {
  const old = observe({sim_time_ns: "0", ik_mode: "ik_pose"});
  assert.doesNotMatch(old.$("ikSolver").textContent, /J4−J6/);
  const onlyElbow = observe({sim_time_ns: "0", ik_mode: "ik_pose", ik_elbow_preference: {
    enabled: true, wrist_enabled: false, status: "active", measured_height_m: .2,
    measured_wrist_drop_m: .1,
  }});
  assert.match(onlyElbow.$("ikSolver").textContent, /肘高=0\.200m/);
  assert.doesNotMatch(onlyElbow.$("ikSolver").textContent, /J4−J6/);
});

test("joint3 negative-angle preference uses measured degrees and preserves positive/zero signs", () => {
  for (const [radians, label] of [[0, "0.0"], [Math.PI / 6, "30.0"], [-Math.PI / 18, "-10.0"]]) {
    const h = observe({sim_time_ns: "0", ik_mode: "ik_pose", ik_elbow_preference: {
      enabled: true, status: "active", joint3_enabled: true,
      measured_joint3_rad: radians, reference_joint3_rad: -1,
    }});
    assert.ok(h.$("ikSolver").textContent.includes(`J3负角偏好=${label}°`));
  }
  const off = observe({sim_time_ns: "0", ik_mode: "ik_pose", ik_elbow_preference: {
    enabled: true, status: "active", joint3_enabled: false, measured_joint3_rad: .5,
  }});
  assert.doesNotMatch(off.$("ikSolver").textContent, /J3负角偏好/);
});
