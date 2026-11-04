import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

// Exercise the installed SDK method used by Vite, rather than a parallel model.
const source = readFileSync(new URL('../node_modules/@epicgames-ps/lib-pixelstreamingfrontend-ue5.6/dist/esm/PeerConnectionController/PeerConnectionController.js', import.meta.url), 'utf8');
assert.ok(source.includes('// SpaceSim: discard statistics from a retired media peer.'));
const start = source.indexOf('    generateStats() {'), end = source.indexOf('\n    /**', start);
const Controller = vm.runInNewContext(`class Controller {${source.slice(start,end)}}; Controller`);
const settle = () => new Promise(resolve => setImmediate(resolve));
function harness() {
  let resolve, reject;
  const pending = new Promise((yes,no) => { resolve=yes; reject=no; });
  const calls = {processed:0,video:0,receivers:0,latency:0};
  const peer = {signalingState:'stable', getStats:()=>pending, getReceivers() {calls.receivers++;return [];} };
  const controller = Object.assign(new Controller(), {
    peerConnection:peer, updateCodecSelection:false,
    aggregatedStats:{processStats() {calls.processed++;}},
    onVideoStats() {calls.video++;}, latencyCalculator:{calculate:()=>({})}, onLatencyCalculated() {calls.latency++;},
  });
  return {controller,peer,calls,resolve,reject};
}

test('pending SDK statistics cannot read a retired or replaced camera peer', async () => {
  for (const replacement of [null, {signalingState:'stable'}]) {
    const h=harness(); h.controller.generateStats();
    h.controller.peerConnection=replacement;
    h.resolve({}); await settle();
    assert.deepEqual(h.calls,{processed:0,video:0,receivers:0,latency:0});
  }
});

test('active statistics work and reentrant teardown prevents later receiver reads', async () => {
  const live=harness(); live.controller.generateStats();live.resolve({});await settle();
  assert.deepEqual(live.calls,{processed:1,video:1,receivers:1,latency:1});
  const retired=harness();
  retired.controller.onVideoStats=()=>{retired.controller.peerConnection=null;};
  retired.controller.generateStats();retired.resolve({});await settle();
  assert.equal(retired.calls.receivers,0);
  const rejected=harness();rejected.controller.generateStats();rejected.controller.peerConnection=null;
  rejected.reject(new Error('closed peer'));await settle();
  assert.equal(rejected.calls.processed,0);
});
