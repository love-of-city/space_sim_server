import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";
import { FreeCameraController } from "../free_camera.js";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const modeSource = source.slice(source.indexOf("const freeCamera ="), source.indexOf("function setMessage("));
const eventsSource = source.slice(source.indexOf('$("viewport").addEventListener("pointerdown"'), source.indexOf('$("estop").addEventListener'));
const exitSource = source.slice(source.indexOf("function exitOperationMode("), source.indexOf("function sendAction("));
function harness() {
  const packets = [], messages = [], listeners = new Map(), nodes = new Map();
  let neutral = 0, operationEntries = 0;
  const doc = { pointerLockElement: null, hidden: false,
    addEventListener(type, fn) { listeners.set(`doc:${type}`, fn); }, removeEventListener() {},
    exitPointerLock() { this.pointerLockElement = null; listeners.get("doc:pointerlockchange")?.(); },
  };
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, { focus() {}, addEventListener(type, fn) { listeners.set(`${id}:${type}`, fn); },
      requestPointerLock() { doc.pointerLockElement = nodes.get(id); } });
    return nodes.get(id);
  };
  const state = { freeCameraMode: false, operationActive: true, pressed: new Set(), streamLive: true,
    selectedStreamerId: "BskRenderer", pixelStreaming: {emitCommand(command) {packets.push(JSON.parse(command.BskCameraInput)); return true;}} };
  const ctx = vm.createContext({ state, $, document: doc,
    window: {addEventListener(type, fn) {listeners.set(`window:${type}`, fn);}},
    controlKeys: new Set(["KeyW", "KeyS", "KeyA", "KeyD", "KeyQ", "KeyE", "KeyF", "KeyR", "ShiftLeft", "ShiftRight", "Escape"]),
    FreeCameraController: class extends FreeCameraController {
      constructor(options) {super({...options, document: doc, schedule: () => 1, cancel() {}});}
    },
    sendNeutralAction() {neutral++;}, updateOperationUI() {}, highlightKeys() {}, setPixelStreamingInputEnabled() {},
    setMessage(m) {messages.push(m);}, enterOperationMode() {operationEntries++;},
  });
  vm.runInContext(modeSource + exitSource + eventsSource, ctx);
  const dispatch = (type, code, extra = {}) => {
    const event = {code, keyCode: 229, repeat: false, target: {tagName: "DIV", closest() {return null;}},
      preventDefault() {this.prevented = true;}, stopImmediatePropagation() {this.stopped = true;}, ...extra};
    listeners.get(type)?.(event);
    return event;
  };
  return {state, packets, doc, dispatch, messages, get neutral() {return neutral;}, get operationEntries() {return operationEntries;}};
}

test("C and free-camera keys never reach SDK and never drive robot state", () => {
  const h = harness(); h.state.pressed.add("KeyW");
  assert.equal(h.dispatch("window:keydown", "KeyC").stopped, true);
  assert.equal(h.state.freeCameraMode, true); assert.equal(h.neutral, 1); assert.equal(h.state.pressed.size, 0);
  assert.equal(h.dispatch("window:keydown", "KeyW").stopped, true);
  assert.equal(h.packets.at(-1).forward, 1); assert.equal(h.state.pressed.size, 0);
  const count = h.packets.length;
  h.dispatch("window:keydown", "KeyC", {repeat: true}); assert.equal(h.packets.length, count);
  h.dispatch("viewport:pointerdown", "", {button: 0}); assert.equal(h.operationEntries, 0);
  h.dispatch("window:keydown", "KeyC"); assert.equal(h.packets.at(-1).active, false);
  h.dispatch("window:keydown", "KeyW", {repeat: true}); assert.equal(h.state.pressed.size, 0);
  h.dispatch("window:keyup", "KeyW"); assert.equal(h.state.pressed.size, 0);
});

test("Home, Escape and blur exit even when robot operation is inactive", () => {
  for (const [type, code] of [["window:keydown", "Home"], ["window:keydown", "Escape"], ["window:blur", ""]]) {
    const h = harness(); h.state.operationActive = false;
    h.dispatch("window:keydown", "KeyC"); h.dispatch("window:keydown", "KeyW");
    h.dispatch(type, code);
    assert.equal(h.state.freeCameraMode, false); assert.equal(h.packets.at(-1).active, false);
    assert.equal(h.packets.at(-1).forward, 0); assert.equal(h.doc.pointerLockElement, null);
  }
});

test("hidden page and form focus clear held camera input", () => {
  for (const type of ["doc:visibilitychange", "doc:focusin"]) {
    const h = harness(); h.dispatch("window:keydown", "KeyC"); h.dispatch("window:keydown", "KeyW");
    h.doc.hidden = true;
    h.dispatch(type, "", {target: {tagName: "INPUT"}});
    assert.equal(h.state.freeCameraMode, false); assert.equal(h.packets.at(-1).active, false);
  }
});

test("editing a page field cannot toggle UE camera or leak keyboard shortcuts", () => {
  const h = harness();
  for (const code of ["KeyC", "KeyW", "KeyM"]) {
    const event = h.dispatch("window:keydown", code, {target: {tagName: "INPUT"}});
    assert.equal(event.stopped, true); assert.equal(event.prevented, undefined);
  }
  assert.equal(h.state.freeCameraMode, false); assert.equal(h.packets.length, 0);
});

test("model camera stream does not pretend to control the main viewport", () => {
  const h = harness(); h.state.selectedStreamerId = "BskRenderer__teleop_camera_sarm_wrist_cam";
  h.dispatch("window:keydown", "KeyC");
  assert.equal(h.state.freeCameraMode, false); assert.equal(h.packets.length, 0);
  assert.match(h.messages.at(-1), /主视口/);
});


test("free-camera UI exposes actual mouse lock even without robot/scene readiness", () => {
  const uiSource = source.slice(source.indexOf("function updateOperationUI("), source.indexOf("function enterOperationMode("));
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {classList: {toggle() {}, remove() {}}, setAttribute() {}, textContent: ""});
    return nodes.get(id);
  };
  const ctx = vm.createContext({$, state: {freeCameraMode: true, sceneReady: false}, freeCamera: {locked: false}});
  vm.runInContext(uiSource, ctx);
  ctx.updateOperationUI();
  assert.match($("operationHintTitle").textContent, /鼠标未锁定/);
  assert.match($("directControl").textContent, /鼠标未锁定/);
  ctx.freeCamera.locked = true;
  ctx.updateOperationUI();
  assert.match($("operationHintTitle").textContent, /鼠标已锁定/);
  assert.match($("directControl").textContent, /鼠标已锁定/);
});
