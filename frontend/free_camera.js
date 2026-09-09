// Browser-owned free-camera input. Never route these keys through the SDK's
// legacy keyCode path (IME composition reports 229), or scale mouse motion by
// the video element/encoded frame dimensions.
export const CAMERA_KEYS = new Set([
  "KeyW", "KeyS", "KeyA", "KeyD", "KeyQ", "KeyE", "ShiftLeft", "ShiftRight",
]);

// Bounds on a single discontinuous input sample, NOT on accumulated camera
// angles or regular mouse speed. Drop cursor warps rather than turning them into
// a maximum-sized rotation. Ordinary input retains exactly the same fixed gain.
export const MAX_LOOK_EVENT_DISTANCE = 256;
export const MAX_LOOK_AGE_MS = 200;
const MAX_WIRE_DISPLACEMENT = 4096;

export class FreeCameraController {
  constructor({ element, send, onExit, onLockError = () => {}, onLockState = () => {}, document: doc = document,
    schedule = (fn, ms) => globalThis.setInterval(fn, ms),
    cancel = id => globalThis.clearInterval(id), now = () => globalThis.performance.now() }) {
    this.element = element;
    this.send = send;
    this.onExit = onExit;
    this.onLockError = onLockError;
    this.onLockState = onLockState;
    this.doc = doc;
    this.schedule = schedule;
    this.cancel = cancel;
    this.now = now;
    this.active = false;
    this.keys = new Set();
    this.dx = 0;
    this.dy = 0;
    this.timer = null;
    this.lockRequestPending = false;
    this.lockErrorReported = false;
    this.preferRawInput = true;
    this.inputMode = "unlocked";
    this.ignoreNextMouseMove = true;
    this.discardLookBatch = false;
    this.lastFlushAt = null;
    this.lookStats = { accepted: 0, discarded: 0, acceptedDx: 0, acceptedDy: 0, staleBatches: 0 };
    this.recentLook = [];
    this.onMouseMove = event => {
      if (!this.active || !this.locked) return;
      const dx = event.movementX, dy = event.movementY;
      if (!Number.isFinite(dx) || !Number.isFinite(dy)) {
        this.recordLook(dx, dy, "invalid");
        return;
      }
      // A browser/RDP cursor reposition at acquisition must not rotate the view.
      if (this.ignoreNextMouseMove) {
        this.ignoreNextMouseMove = false;
        this.recordLook(dx, dy, "lock-baseline");
        return;
      }
      const time = this.now();
      // Ignore event queues from before a main-thread pause. Some legacy
      // browsers use epoch timestamps, so compare only compatible time origins.
      if (Number.isFinite(event.timeStamp) && event.timeStamp >= 0 && time >= event.timeStamp
          && time - event.timeStamp > MAX_LOOK_AGE_MS) {
        this.recordLook(dx, dy, "stale-event");
        return;
      }
      if (Math.hypot(dx, dy) > MAX_LOOK_EVENT_DISTANCE) {
        this.recordLook(dx, dy, "discontinuity");
        return;
      }
      if (this.discardLookBatch) {
        this.recordLook(dx, dy, "overflow-batch");
        return;
      }
      const nextX = this.dx + dx, nextY = this.dy + dy;
      if (Math.abs(nextX) > MAX_WIRE_DISPLACEMENT || Math.abs(nextY) > MAX_WIRE_DISPLACEMENT) {
        // Do not clamp a corrupt/backlogged batch into a 491-degree turn.
        this.dx = this.dy = 0;
        this.discardLookBatch = true;
        this.recordLook(dx, dy, "overflow-batch");
        return;
      }
      this.dx = nextX;
      this.dy = nextY;
      this.recordLook(dx, dy, "accepted");
    };
    this.onLockChange = () => {
      if (!this.active) {
        // Legacy browsers may finish a lock request after its caller exited.
        if (this.locked) this.doc.exitPointerLock?.();
        return;
      }
      this.dx = this.dy = 0;
      this.discardLookBatch = false;
      this.ignoreNextMouseMove = true;
      this.onLockState(this.locked);
      if (!this.locked) this.onExit();
    };
    // Modern requests report failure through their Promise. Suppress the
    // companion error event while trying the supported-options fallback.
    this.onPointerLockError = () => { if (!this.lockRequestPending) this.reportLockError(); };
    this.doc.addEventListener("mousemove", this.onMouseMove, true);
    this.doc.addEventListener("pointerlockchange", this.onLockChange);
    this.doc.addEventListener("pointerlockerror", this.onPointerLockError);
  }

  setActive(enabled) {
    if (enabled === this.active) return true;
    this.keys.clear();
    this.dx = this.dy = 0;
    this.discardLookBatch = false;
    this.ignoreNextMouseMove = true;
    this.lastFlushAt = null;
    if (!enabled) this.inputMode = "unlocked";
    this.active = enabled;
    const sent = this.flush(); // Absolute mode + complete held-key state, not a C toggle.
    if (enabled && !sent) {
      this.active = false;
      return false;
    }
    if (enabled) {
      // Keepalive bounds stale movement even if a key-up never reaches UE.
      this.timer = this.schedule(() => this.flush(), 1000 / 60);
      this.element.focus?.({ preventScroll: true });
      this.requestPointerLock();
    } else {
      if (this.timer != null) this.cancel(this.timer);
      this.timer = null;
      if (this.doc.pointerLockElement === this.element) this.doc.exitPointerLock?.();
    }
    return true;
  }

  get locked() { return this.doc.pointerLockElement === this.element; }

  reportLockError(error) {
    if (!this.active || this.locked || this.lockErrorReported) return;
    this.lockErrorReported = true;
    this.onLockError(error);
  }

  async requestPointerLock() {
    if (!this.active || this.locked || this.lockRequestPending) return;
    this.lockRequestPending = true;
    this.lockErrorReported = false;
    try {
      if (!this.element.requestPointerLock) {
        this.reportLockError();
        return;
      }
      // Restore unaccelerated relative input. Do not infer the browser's mouse
      // capabilities from the server's RDP session and force everyone onto the
      // OS-accelerated/recentring path. Fallback ONLY if the browser rejects raw
      // input as unsupported, never silently switch modes during a gesture.
      const request = raw => {
        this.inputMode = raw ? "raw" : "compatible";
        const pending = raw ? this.element.requestPointerLock({ unadjustedMovement: true })
                            : this.element.requestPointerLock();
        // Legacy void-returning APIs cannot confirm support for the raw option.
        if (!pending?.then) this.inputMode = "compatible";
        return pending;
      };
      try {
        await request(this.preferRawInput);
      } catch (error) {
        if (!this.active) return;
        if (!this.preferRawInput || error?.name !== "NotSupportedError") throw error;
        this.preferRawInput = false;
        await request(false);
      }
    } catch (error) {
      this.reportLockError(error);
    } finally {
      this.lockRequestPending = false;
      // An asynchronous request may complete after blur/C/disconnect.
      if (!this.active && this.locked) this.doc.exitPointerLock?.();
    }
  }

  recordLook(dx, dy, reason) {
    if (reason === "accepted") {
      this.lookStats.accepted++;
      this.lookStats.acceptedDx += dx;
      this.lookStats.acceptedDy += dy;
    } else {
      this.lookStats.discarded++;
    }
    this.recentLook.push({ dx: Number.isFinite(dx) ? dx : null, dy: Number.isFinite(dy) ? dy : null, reason });
    if (this.recentLook.length > 64) this.recentLook.shift();
  }

  // Mouse-only, bounded, read-only snapshots for real-browser diagnosis. This
  // does not infer missing motion or silently change sensitivity/input modes.
  getDiagnostics() {
    return { active: this.active, locked: this.locked, inputMode: this.inputMode,
      preferRawInput: this.preferRawInput, stats: { ...this.lookStats },
      recentLook: this.recentLook.map(sample => ({ ...sample })) };
  }

  handleKey(event, down) {
    if (!this.active || !CAMERA_KEYS.has(event.code)) return false;
    event.preventDefault();
    event.stopImmediatePropagation();
    if (down) this.keys.add(event.code);
    else this.keys.delete(event.code);
    if (!event.repeat) this.flush();
    return true;
  }

  flush() {
    const time = this.now();
    if (this.active && this.lastFlushAt != null && time - this.lastFlushAt > MAX_LOOK_AGE_MS) {
      // A delayed timer must not replay an old sweep as one large camera jump.
      if (this.dx || this.dy || this.discardLookBatch) this.lookStats.staleBatches++;
      this.dx = this.dy = 0;
    }
    this.lastFlushAt = time;
    const axis = (positive, negative) => Number(this.keys.has(positive)) - Number(this.keys.has(negative));
    const payload = {
      version: 1, active: this.active,
      forward: this.active ? axis("KeyW", "KeyS") : 0,
      right: this.active ? axis("KeyD", "KeyA") : 0,
      up: this.active ? axis("KeyE", "KeyQ") : 0,
      boost: this.active && (this.keys.has("ShiftLeft") || this.keys.has("ShiftRight")),
      look_dx: this.active ? this.dx : 0, look_dy: this.active ? this.dy : 0,
    };
    // Motion is a displacement, consumed once. Do not replay it on a failed send.
    this.dx = this.dy = 0;
    this.discardLookBatch = false;
    try {
      return this.send({ BskCameraInput: JSON.stringify(payload) }) !== false;
    } catch (_) {
      // A closing RTC data channel can throw instead of returning false. Local
      // exit/cleanup must still complete; UE's heartbeat watchdog handles loss.
      return false;
    }
  }

  dispose() {
    this.setActive(false);
    this.doc.removeEventListener("mousemove", this.onMouseMove, true);
    this.doc.removeEventListener("pointerlockchange", this.onLockChange);
    this.doc.removeEventListener("pointerlockerror", this.onPointerLockError);
  }
}
