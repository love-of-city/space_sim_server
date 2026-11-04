// Renderer-confirmed preview lighting, independent of solar and robot controls.
export class WorkLight {
  constructor({send, now = () => performance.now()}) {
    this.send = send;
    this.now = now;
    this.reset();
  }
  reset() { this.supported = false; this.enabled = false; this.lastReply = -Infinity; }
  get available() { return this.supported && this.now() - this.lastReply < 5000; }
  receive(payload) {
    let reply;
    try { reply = JSON.parse(payload); } catch { return; }
    if (reply?.type !== 'bsk_work_light' || typeof reply.supported !== 'boolean' || typeof reply.enabled !== 'boolean') return;
    this.supported = reply.supported;
    this.enabled = reply.enabled;
    this.lastReply = this.now();
  }
  query() { return this.send({BskWorkLight: '{}'}); }
  toggle() {
    if (!this.available) return false;
    return this.send({BskWorkLight: JSON.stringify({enabled: !this.enabled})});
  }
}
