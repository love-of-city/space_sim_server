import test from 'node:test';
import assert from 'node:assert/strict';
import {createStereoVisionPanel} from '../stereo_vision.js';
const deferred=()=>{let resolve;const promise=new Promise(r=>{resolve=r;});return {promise,resolve};};
const flush=()=>new Promise(resolve=>setImmediate(resolve));
function fixture(request) {
  const images={left:{removeAttribute(){delete this.src;}},right:{removeAttribute(){delete this.src;}}};
  const created=[],revoked=[];
  const panel=createStereoVisionPanel({button:{addEventListener(){},setAttribute(){}},panel:{},status:{},images,request,
    visible:()=>true,schedule:()=>1,cancel(){},urls:{createObjectURL:()=>{const u=`blob:${created.length}`;created.push(u);return u;},revokeObjectURL:u=>revoked.push(u)}});
  return {panel,images,created,revoked};
}
test('closing the stereo panel discards a response already in flight',async()=>{
  const status=deferred();
  const f=fixture(()=>status.promise);
  f.panel.setOpen(true);f.panel.setOpen(false);
  status.resolve({ok:true,json:async()=>({stale:false})});await flush();
  assert.equal(f.created.length,0);assert.equal(f.images.left.src,undefined);
});
test('one failed eye cannot be repopulated by the other late image',async()=>{
  const right=deferred();
  const f=fixture(async path=>path.endsWith('status')?{ok:true,json:async()=>({stale:false})}:
    path.endsWith('left')?{ok:false}:right.promise);
  f.panel.setOpen(true);await flush();
  right.resolve({ok:true,blob:async()=>new Blob(['frame'])});await flush();
  assert.equal(f.created.length,0);assert.equal(f.images.right.src,undefined);
  f.panel.setOpen(false);
});
