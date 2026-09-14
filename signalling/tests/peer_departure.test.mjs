import assert from 'node:assert/strict';
import test from 'node:test';
import { PeerDeparture } from '../peer_departure.mjs';
function fixture() {
  let next=0; const timers=new Map(), removed=[];
  const peer=new PeerDeparture(notify=>removed.push(notify), {
    setTimer(fn,delay){const id=++next;timers.set(id,{fn,delay});return id;},
    clearTimer(id){timers.delete(id);}
  });
  return {peer,timers,removed,fire(){[...timers.values()][0].fn();}};
}
test('early departure waits for UE offer rather than racing participant startup',()=>{
  const f=fixture();f.peer.leave();assert.deepEqual(f.removed,[]);
  assert.equal([...f.timers.values()][0].delay,10000);
  f.peer.offered();assert.equal(f.timers.size,1);assert.equal([...f.timers.values()][0].delay,500);
  f.fire();assert.deepEqual(f.removed,[true]);assert.equal(f.timers.size,0);
});
test('negotiated departure is short, bounded and idempotent',()=>{
  const f=fixture();f.peer.offered();assert.equal(f.timers.size,0);
  f.peer.leave();const timer=f.peer.timer;f.peer.leave();f.peer.offered();
  assert.equal(f.peer.timer,timer);assert.equal([...f.timers.values()][0].delay,500);
  f.fire();f.peer.leave();f.peer.offered();assert.deepEqual(f.removed,[true]);
});
test('a streamer that never produces an offer cannot leak a slot indefinitely',()=>{
  const f=fixture();f.peer.leave();assert.equal([...f.timers.values()][0].delay,10000);
  f.fire();f.peer.offered();assert.deepEqual(f.removed,[true]);
});
test('streamer disconnect cancels deferred notifications and frees the slot once',()=>{
  const f=fixture();f.peer.leave();const callback=[...f.timers.values()][0].fn;
  f.peer.finish(false);callback();f.peer.finish(false);assert.deepEqual(f.removed,[false]);assert.equal(f.timers.size,0);
});
