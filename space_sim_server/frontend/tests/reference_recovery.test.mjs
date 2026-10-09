import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";
import { referenceProtectionLabel } from "../control_status.js";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");

test("motion commands and explicit stop never grant neutral recovery permission", () => {
  const packets = [];
  const state = {connected: true, controlGranted: true, sequence: 0, linearSpeed: .05,
    ws: {readyState: 1, send: message => packets.push(JSON.parse(message))}};
  const context = vm.createContext({state, WebSocket: {OPEN: 1}});
  vm.runInContext(source.slice(source.indexOf("function transmitAction("), source.indexOf("function updateOperationUI(")), context);
  const neutral = {linear: [0, 0, 0], angular: [0, 0, 0], grip: 0, source: "keyboard"};
  context.transmitAction(neutral, false, true);
  assert.equal(packets.at(-1).allow_reference_recovery, true);
  context.sendNeutralAction();
  assert.equal(packets.at(-1).allow_reference_recovery, false);
  context.transmitAction(neutral, true, true);
  assert.equal(packets.at(-1).allow_reference_recovery, false);
});

test("recovery is identified as reference unloading instead of requested motion", () => {
  assert.match(referenceProtectionLabel({reference_governor_state: "reference_recovery"}), /卸除旧目标/);
  assert.match(referenceProtectionLabel({reference_governor_state: "reference_recovery"}), /Esc/);
});
