import test from "node:test";
import assert from "node:assert/strict";
import {jointAngleRows, createJointAnglePanel} from "../joint_angles.js";
const rad = degrees => degrees * Math.PI / 180;
const obs = (degrees = [0,0,0,0,0,300]) => ({
  arm_joint_position_rad: degrees.map(rad),
  target_arm_joint_position_rad: Array(6).fill(0),
  arm_joint_limits_rad: [...Array.from({length: 5}, () => [-Math.PI, Math.PI]), [-2*Math.PI, 2*Math.PI]],
});

test("six actual model-zero angles, including negative and multi-turn, never targets", () => {
  const rows = jointAngleRows(obs([-120, 20, 30, 40, 50, 300]));
  assert.equal(rows.length, 6);
  assert.equal(rows[0].angle, "-120.0°");
  assert.equal(rows[5].angle, "300.0°");
  assert.equal(rows[5].range, "-360.0° ～ 360.0°");
  assert.equal(rows[5].lower, "660.0°");
  assert.equal(rows[5].upper, "60.0°");
  assert.equal(jointAngleRows(obs([0,0,0,0,0,-300]))[5].angle, "-300.0°");
});

test("both limit directions and 10 / 1 degree inclusive thresholds", () => {
  const rows = jointAngleRows(obs([169,170,-170,179,-179,360]));
  assert.deepEqual(rows.map(r => r.level), ["normal","warning","warning","danger","danger","danger"]);
  assert.equal(rows[1].status, "接近上限");
  assert.equal(rows[2].status, "接近下限");
  assert.equal(rows[5].upper, "0.0°");
});

test("out-of-range values stay visible and distinct from merely near the limit", () => {
  const rows = jointAngleRows(obs([180.2,-181,0,0,0,400]));
  assert.equal(rows[0].upper, "越界 0.2°");
  assert.equal(rows[1].status, "超出下限");
  assert.equal(rows[5].angle, "400.0°");
  assert.equal(rows[5].upper, "越界 40.0°");
});

test("old eight-element SARM alias excludes fingers; actual SI field has precedence", () => {
  const data = {joint_position_rad: [0,0,0,0,0,rad(250),0.01,0.02]};
  assert.equal(jointAngleRows(data)[5].angle, "250.0°");
  data.arm_joint_position_rad = Array(6).fill(rad(20));
  assert.equal(jointAngleRows(data)[5].angle, "20.0°");
  assert.equal(jointAngleRows({joint_position_rad: Array(6).fill(1)})[5].angle, "—");
});

test("missing, invalid and continuous limits are not confused", () => {
  const data = obs();
  data.arm_joint_limits_rad = [undefined,[null,null],[-1,null],[1,-1],[NaN,1],[-Infinity,Infinity]];
  const rows = jointAngleRows(data);
  assert.equal(rows[0].status, "限位未提供");
  assert.equal(rows[1].range, "无位置限位");
  assert.equal(rows[1].status, "连续关节");
  for (const i of [2,3,4,5]) assert.equal(rows[i].level, "unknown");
  data.arm_joint_position_rad = [null,undefined,NaN,Infinity,"0",0];
  assert.ok(jointAngleRows(data).slice(0,5).every(r => r.angle === "—"));
});

function fakeDom() {
  const doc = {createElement(tag) {return {tag,ownerDocument: doc,textContent: "",className: "",children: [],
    append(...nodes) {this.children.push(...nodes);},setAttribute() {}};}};
  return {container: doc.createElement("div"), status: doc.createElement("span")};
}

test("DOM nodes are reused; stale/disconnect clears angles and warnings", () => {
  const {container, status} = fakeDom(); let now = 0;
  const panel = createJointAnglePanel(container,status,{clock: () => now});
  const cards = [...container.children];
  const angle = i => cards[i].children[0].children[1].textContent;
  panel.update(obs([180,0,0,0,0,300]));
  assert.equal(angle(0), "180.0°"); assert.match(cards[0].className, /danger/);
  now = 2999; panel.checkFreshness(); assert.equal(angle(5), "300.0°");
  now = 3000; panel.checkFreshness(); assert.equal(angle(5), "—"); assert.match(status.textContent,/过期/);
  assert.equal(cards[0].className, "joint-angle-card unknown");
  panel.update(obs()); assert.equal(angle(5), "300.0°");
  panel.clear("连接中断"); assert.equal(angle(5), "—"); assert.equal(status.textContent,"连接中断");
  panel.update({arm_joint_position_rad: Array(6).fill(0)});
  assert.equal(status.textContent,"限位元数据未提供");
  assert.deepEqual(container.children,cards);
});

test("asymmetric model bounds are respected; target-only packets never invent actual angles", () => {
  const data = obs([60,0,0,0,0,0]);
  data.arm_joint_limits_rad[0] = [rad(-30), rad(120)];
  const row = jointAngleRows(data)[0];
  assert.equal(row.range, "-30.0° ～ 120.0°");
  assert.equal(row.lower, "90.0°"); assert.equal(row.upper, "60.0°");
  delete data.arm_joint_position_rad;
  assert.ok(jointAngleRows(data).every(r => r.angle === "—"));
});
