import test from "node:test";
import assert from "node:assert/strict";
import { FreeCameraController, MAX_LOOK_EVENT_DISTANCE, MAX_LOOK_AGE_MS } from "../free_camera.js";

function setup({ sendOk = true, requestLock, now = () => 1000 } = {}) {
  const listeners = new Map();
  const doc = {
    pointerLockElement: null,
    addEventListener(type, fn) { listeners.set(type, fn); },
    removeEventListener(type) { listeners.delete(type); },
    exitPointerLock() { this.pointerLockElement = null; listeners.get("pointerlockchange")?.(); },
  };
  const packets = [];
  let tick, exits = 0, errors = 0, cancelled = 0;
  const lockStates = [];
  const element = { clientWidth: 640, clientHeight: 360, focus() {},
    requestPointerLock: requestLock || (() => { doc.pointerLockElement = element; }),
  };
  const controller = new FreeCameraController({ element, document: doc, now,
    send: command => { packets.push(JSON.parse(command.BskCameraInput)); return sendOk; },
    onExit: () => { exits++; controller.setActive(false); },
    onLockError: () => { errors++; },
    onLockState: locked => lockStates.push(locked),
    schedule: fn => { tick = fn; return 42; }, cancel: () => { cancelled++; },
  });
  const key = (code, down = true, extras = {}) => {
    const event = { code, keyCode: 229, repeat: false, ...extras,
      preventDefault() { this.prevented = true; }, stopImmediatePropagation() { this.stopped = true; } };
    controller.handleKey(event, down);
    return event;
  };
  return { controller, doc, element, packets, key, emit: (type, event) => listeners.get(type)?.(event),
    tick: () => tick?.(), get exits() { return exits; }, get errors() { return errors; }, get cancelled() { return cancelled; }, listeners, lockStates };
}

test("IME keyCode=229 uses physical WASD/QE and opposing held keys cancel", () => {
  const h = setup(); h.controller.setActive(true);
  for (const [code, field, value] of [["KeyW", "forward", 1], ["KeyS", "forward", -1],
    ["KeyA", "right", -1], ["KeyD", "right", 1], ["KeyQ", "up", -1], ["KeyE", "up", 1]]) {
    const e = h.key(code);
    assert.equal(e.stopped, true); assert.equal(e.prevented, true);
    assert.equal(h.packets.at(-1)[field], value);
    h.key(code, false); assert.equal(h.packets.at(-1)[field], 0);
  }
  h.key("KeyW"); h.key("KeyS"); assert.equal(h.packets.at(-1).forward, 0);
  h.key("KeyW", false); assert.equal(h.packets.at(-1).forward, -1);
  h.key("ShiftLeft"); assert.equal(h.packets.at(-1).boost, true);
});

test("mouse distance is invariant under resize, event batching, and idle ticks", () => {
  const h = setup(); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 0, movementY: 0}); // acquisition baseline
  h.emit("mousemove", { movementX: 10, movementY: -5 });
  h.emit("mousemove", { movementX: 20, movementY: -10 });
  h.tick(); assert.equal(h.packets.at(-1).look_dx, 30); assert.equal(h.packets.at(-1).look_dy, -15);
  h.element.clientWidth = 1920; h.element.clientHeight = 1080;
  h.emit("mousemove", { movementX: 30, movementY: -15 }); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 30); assert.equal(h.packets.at(-1).look_dy, -15);
  h.tick(); assert.equal(h.packets.at(-1).look_dx, 0);
});

test("unlock stops held movement, sends absolute main mode, and clears keepalive", () => {
  const h = setup(); h.controller.setActive(true); h.key("KeyW");
  h.doc.exitPointerLock();
  assert.equal(h.exits, 1); assert.equal(h.cancelled, 1);
  assert.deepEqual(h.packets.at(-1), {version: 1, active: false, forward: 0, right: 0, up: 0, boost: false, look_dx: 0, look_dy: 0});
  h.controller.setActive(true); assert.equal(h.packets.at(-1).forward, 0);
});

test("repeated activation is idempotent and key auto-repeat does not duplicate motion", () => {
  const h = setup(); h.controller.setActive(true);
  const before = h.packets.length;
  h.controller.setActive(true); assert.equal(h.packets.length, before);
  h.key("KeyW"); const count = h.packets.length;
  h.key("KeyW", true, {repeat: true}); assert.equal(h.packets.length, count);
  h.tick(); assert.equal(h.packets.at(-1).forward, 1);
});

test("failed send cannot leave local camera mode or timers active", () => {
  const h = setup({sendOk: false});
  assert.equal(h.controller.setActive(true), false); assert.equal(h.controller.active, false);
  assert.equal(h.controller.timer, null); assert.equal(h.doc.pointerLockElement, null);
});

test("mouse without lock and invalid displacements are ignored", () => {
  const h = setup({requestLock: () => {}}); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 50, movementY: 30}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 0);
  h.doc.pointerLockElement = h.element;
  h.emit("mousemove", {movementX: NaN, movementY: Infinity}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 0);
});

test("raw input is preferred and late lock is released after exit", async () => {
  let calls = 0, resolve;
  const h = setup({requestLock: options => {
    calls++;
    assert.deepEqual(options, {unadjustedMovement: true}, "do not force OS acceleration on every browser");
    return new Promise(r => { resolve = () => { h.doc.pointerLockElement = h.element; r(); }; });
  }});
  h.controller.setActive(true);
  await Promise.resolve(); assert.equal(calls, 1);
  h.controller.setActive(false); resolve();
  await Promise.resolve(); await Promise.resolve();
  assert.equal(h.doc.pointerLockElement, null); assert.equal(h.errors, 0);
});

test("dispose removes listeners and clears input without affecting unrelated keys", () => {
  const h = setup(); h.controller.setActive(true);
  assert.equal(h.key("KeyM").stopped, undefined);
  h.controller.dispose(); assert.equal(h.listeners.size, 0);
  assert.equal(h.controller.active, false); assert.equal(h.packets.at(-1).active, false);
});


test("a throwing closed data channel cannot prevent local exit and cleanup", () => {
  const h = setup(); h.controller.setActive(true); h.key("KeyW");
  h.controller.send = () => {throw new Error("RTCDataChannel is closed");};
  assert.doesNotThrow(() => h.controller.setActive(false));
  assert.equal(h.controller.active, false); assert.equal(h.cancelled, 1);
  assert.equal(h.controller.keys.size, 0); assert.equal(h.doc.pointerLockElement, null);
  assert.equal(h.controller.setActive(true), false); assert.equal(h.controller.timer, null);
});


test("default browser timer methods retain the global Window receiver", () => {
  const originalSchedule = globalThis.setInterval, originalCancel = globalThis.clearInterval;
  const h = setup(); let cancelled = false;
  globalThis.setInterval = function () {assert.equal(this, globalThis); return 7;};
  globalThis.clearInterval = function (id) {assert.equal(this, globalThis); assert.equal(id, 7); cancelled = true;};
  try {
    const c = new FreeCameraController({element: h.element, document: h.doc, send: () => true, onExit() {}});
    assert.doesNotThrow(() => c.setActive(true)); assert.equal(c.timer, 7);
    assert.doesNotThrow(() => c.setActive(false)); assert.equal(cancelled, true);
    c.dispose();
  } finally {
    globalThis.setInterval = originalSchedule; globalThis.clearInterval = originalCancel;
  }
});


test("lock status follows browser confirmation rather than local camera activation", () => {
  const h = setup({requestLock: () => {}});
  h.controller.setActive(true);
  assert.equal(h.controller.locked, false);
  h.doc.pointerLockElement = h.element;
  h.emit("pointerlockchange");
  assert.equal(h.controller.locked, true);
  assert.deepEqual(h.lockStates, [true]);
  h.doc.exitPointerLock();
  assert.deepEqual(h.lockStates, [true, false]);
  assert.equal(h.controller.active, false);
});

test("a rejected lock is reported once across Promise and legacy error event, and can retry", async () => {
  const h = setup({requestLock: () => Promise.reject(Object.assign(new Error(), {name: "NotAllowedError"}))});
  h.controller.setActive(true);
  await Promise.resolve();
  h.emit("pointerlockerror");
  assert.equal(h.errors, 1);
  assert.equal(h.controller.locked, false);
  h.element.requestPointerLock = () => {h.doc.pointerLockElement = h.element;};
  await h.controller.requestPointerLock();
  h.emit("pointerlockchange");
  assert.equal(h.controller.locked, true);
  assert.deepEqual(h.lockStates, [true]);
  assert.equal(h.errors, 1);
});

test("repeated clicks cannot race an outstanding pointer-lock request", async () => {
  let calls = 0, resolve;
  const h = setup({requestLock: () => {calls++; return new Promise(r => {resolve = r;});}});
  h.controller.setActive(true);
  h.controller.requestPointerLock();
  h.controller.requestPointerLock();
  assert.equal(calls, 1);
  h.doc.pointerLockElement = h.element; resolve(); await Promise.resolve();
  assert.equal(h.controller.lockRequestPending, false);
  assert.equal(h.controller.locked, true);
});

test("legacy event-only errors and late acquisitions are cleaned up", async () => {
  const h = setup({requestLock: () => {}});
  h.controller.setActive(true);
  await Promise.resolve();
  h.emit("pointerlockerror"); assert.equal(h.errors, 1);
  h.controller.setActive(false);
  await Promise.resolve();
  h.doc.pointerLockElement = h.element; h.emit("pointerlockchange");
  assert.equal(h.doc.pointerLockElement, null);
  assert.equal(h.controller.active, false);
});

test("relative motion continues with fixed desktop coordinates across repeated full turns", () => {
  const h = setup(); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 0, movementY: 0});
  for (const [dx, dy] of [[100, 0], [-100, 0], [0, 100], [0, -100]]) {
    const start = h.packets.length;
    for (let i = 0; i < 60; i++) {
      h.emit("mousemove", {movementX: dx, movementY: dy, clientX: 1919, clientY: 1079});
      h.tick();
    }
    const sent = h.packets.slice(start);
    assert.equal(sent.reduce((n, p) => n + p.look_dx, 0), dx * 60);
    assert.equal(sent.reduce((n, p) => n + p.look_dy, 0), dy * 60);
  }
});


test("only NotSupportedError allows normal-input fallback, remembered for the next activation", async () => {
  const requests = [];
  const h = setup({requestLock: options => {
    requests.push(options);
    if (options?.unadjustedMovement) return Promise.reject(Object.assign(new Error(), {name: "NotSupportedError"}));
    h.doc.pointerLockElement = h.element;
    return Promise.resolve();
  }});
  h.controller.setActive(true);
  h.emit("pointerlockerror"); // companion event must not pre-empt fallback
  await Promise.resolve(); await Promise.resolve();
  assert.deepEqual(requests, [{unadjustedMovement: true}, undefined]);
  assert.equal(h.errors, 0);
  assert.equal(h.controller.inputMode, "compatible");
  assert.equal(h.controller.locked, true);
  h.controller.setActive(false); h.controller.setActive(true);
  await Promise.resolve();
  assert.deepEqual(requests, [{unadjustedMovement: true}, undefined, undefined]);
});

test("permission/gesture errors do not silently switch input gain or retry normal input", async () => {
  let calls = 0;
  const h = setup({requestLock: () => {calls++; return Promise.reject(Object.assign(new Error(), {name: "NotAllowedError"}));}});
  h.controller.setActive(true); await Promise.resolve(); await Promise.resolve();
  h.emit("pointerlockerror");
  assert.equal(calls, 1); assert.equal(h.errors, 1);
  assert.equal(h.controller.preferRawInput, true);
});

test("successful raw mode stays raw and reports confirmed mode", async () => {
  const h = setup({requestLock: () => {h.doc.pointerLockElement = h.element; return Promise.resolve();}});
  h.controller.setActive(true); await Promise.resolve();
  assert.equal(h.controller.inputMode, "raw");
  for(let i=0;i<10;i++) h.tick();
  assert.equal(h.controller.inputMode, "raw");
  assert.equal(h.controller.preferRawInput, true);
});

test("lock acquisition drops its first sample, then preserves ordinary mouse distance", () => {
  const h = setup(); h.controller.setActive(true); h.emit("pointerlockchange");
  h.emit("mousemove", {movementX: 1200, movementY: -800}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 0); assert.equal(h.packets.at(-1).look_dy, 0);
  h.emit("mousemove", {movementX: 4, movementY: -3}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 4); assert.equal(h.packets.at(-1).look_dy, -3);
});

test("cursor-warp spikes and their reverse jumps are discarded, never clamped into a turn", () => {
  const h = setup(); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 0, movementY: 0});
  h.emit("mousemove", {movementX: 10, movementY: -5});
  h.emit("mousemove", {movementX: 1200, movementY: -800});
  h.emit("mousemove", {movementX: -1200, movementY: 800});
  h.emit("mousemove", {movementX: 20, movementY: -10}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 30); assert.equal(h.packets.at(-1).look_dy, -15);
  assert.equal(h.controller.getDiagnostics().recentLook.filter(s => s.reason === "discontinuity").length, 2);
  h.emit("mousemove", {movementX: MAX_LOOK_EVENT_DISTANCE, movementY: MAX_LOOK_EVENT_DISTANCE}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 0, "diagonal outliers use vector length, not per-axis clamps");
});

test("normal gain is independent of batching, event frequency, resize and timer cadence", () => {
  for (const [events, displacement, interval] of [[120,1,1], [12,10,10], [4,30,30]]) {
    let now = 1000;
    const h = setup({now: () => now}); h.controller.setActive(true);
    h.emit("mousemove", {movementX: 0, movementY: 0});
    for(let i=0;i<events;i++){
      now += interval;
      h.emit("mousemove", {movementX: displacement, movementY: -displacement/2, timeStamp: now});
      h.tick();
    }
    assert.equal(h.packets.reduce((n,p) => n+p.look_dx, 0), 120);
    assert.equal(h.packets.reduce((n,p) => n+p.look_dy, 0), -60);
  }
});

test("stale browser events and delayed timer batches cannot replay a queued sweep", () => {
  let now = 1000;
  const h = setup({now: () => now}); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 0, movementY: 0});
  h.emit("mousemove", {movementX: 100, movementY: 0, timeStamp: now-MAX_LOOK_AGE_MS-1});
  h.emit("mousemove", {movementX: 30, movementY: -10, timeStamp: now});
  now += MAX_LOOK_AGE_MS+1; h.tick();
  assert.equal(h.packets.at(-1).look_dx, 0); assert.equal(h.packets.at(-1).look_dy, 0);
  now += 10; h.emit("mousemove", {movementX: 30, movementY: -10, timeStamp: now}); h.tick();
  assert.equal(h.packets.at(-1).look_dx, 30);
  assert.equal(h.controller.getDiagnostics().stats.staleBatches, 1);
});

test("diagnostics are bounded snapshots and never expose writable controller state", () => {
  const h = setup(); h.controller.setActive(true);
  for(let i=0;i<200;i++) {h.emit("mousemove", {movementX: 1, movementY: 2}); h.tick();}
  const first = h.controller.getDiagnostics();
  assert.equal(first.recentLook.length, 64);
  first.stats.accepted = -1; first.recentLook[0].dx = 20000;
  const second = h.controller.getDiagnostics();
  assert.notEqual(second.stats.accepted, -1); assert.notEqual(second.recentLook[0].dx, 20000);
});

test("a wire-overflow batch is discarded instead of producing a maximum turn", () => {
  const h = setup(); h.controller.setActive(true);
  h.emit("mousemove", {movementX: 0, movementY: 0});
  for(let i=0;i<50;i++)h.emit("mousemove", {movementX: 100, movementY: 0});
  h.tick(); assert.equal(h.packets.at(-1).look_dx, 0);
  h.emit("mousemove", {movementX: 3, movementY: -2});h.tick();
  assert.equal(h.packets.at(-1).look_dx, 3);
});
