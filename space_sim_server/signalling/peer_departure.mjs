// UE 5.6 queues participant creation/offer work asynchronously. A browser can
// disappear before that work runs. Stop forwarding immediately, but do not
// destroy its UE participant in the same startup window.
export class PeerDeparture {
  constructor(remove, {setTimer = setTimeout, clearTimer = clearTimeout} = {}) {
    this.remove = remove;
    this.setTimer = setTimer;
    this.clearTimer = clearTimer;
    this.offerSeen = false;
    this.leaving = false;
    this.done = false;
    this.timer = null;
  }
  offered() {
    if (this.done || this.offerSeen) return;
    this.offerSeen = true;
    if (this.leaving) this.schedule(500);
  }
  leave() {
    if (this.done || this.leaving) return;
    this.leaving = true;
    // The offer acknowledges that UE has finished participant setup. Bound the
    // fallback as well, so a dead/stalled streamer cannot leak slots forever.
    this.schedule(this.offerSeen ? 500 : 10000);
  }
  schedule(delay) {
    if (this.timer != null) this.clearTimer(this.timer);
    this.timer = this.setTimer(() => this.finish(true), delay);
    this.timer?.unref?.();
  }
  finish(notify) {
    if (this.done) return;
    this.done = true;
    if (this.timer != null) this.clearTimer(this.timer);
    this.timer = null;
    this.remove(notify);
  }
}
