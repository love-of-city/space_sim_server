import test from 'node:test';
import assert from 'node:assert/strict';
import {observePreviewAvailability} from '../preview_availability.js';

test('camera discovery never subscribes, retries a closed socket and retires cleanly', () => {
  const sockets = [], received = [], timers = new Map();
  let timerId = 0;
  class Socket {
    constructor(url) { this.url = url; this.handlers = {}; this.sent = []; sockets.push(this); }
    addEventListener(name, fn) { this.handlers[name] = fn; }
    send(data) { this.sent.push(JSON.parse(data)); }
    close() { this.handlers.close(); }
    emit(name, data) { this.handlers[name](data); }
  }
  const observer = observePreviewAvailability('ws://authorized.test', ids => received.push(ids), {
    Socket, schedule(fn) { const id = ++timerId; timers.set(id, fn); return id; }, cancel(id) { timers.delete(id); },
  });
  const first = sockets[0];
  first.emit('open');
  first.emit('message', {data:JSON.stringify({type:'config'})});
  first.emit('message', {data:'invalid'});
  first.emit('message', {data:JSON.stringify({type:'streamerList',ids:['main','wrist','wrist']})});
  assert.deepEqual(first.sent, [{type:'listStreamers'}]);
  assert.deepEqual(received, [['main','wrist']]);
  first.close();
  [...timers.values()][0](); timers.clear();
  const second = sockets[1];
  first.emit('message', {data:JSON.stringify({type:'streamerList',ids:['old']})});
  second.emit('open');
  assert.equal(received.length, 1);
  assert.deepEqual(second.sent, [{type:'listStreamers'}]);
  second.close();
  assert.equal(timers.size, 1);
  observer.close();
  assert.equal(timers.size, 0);
});
