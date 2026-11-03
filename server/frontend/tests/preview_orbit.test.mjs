import test from 'node:test';
import assert from 'node:assert/strict';
import {PreviewOrbit} from '../preview_orbit.js';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';

function harness() {
  const sent=[]; let stopped=0, allowed=true;
  const orbit=new PreviewOrbit({send:p=>{sent.push(JSON.parse(p.BskPreviewOrbit));return true;},
    available:()=>allowed, stopRobot:()=>stopped++});
  return {orbit,sent,get stopped(){return stopped;},disable(){allowed=false;}};
}
test('only middle button starts orbit and stops robot input',()=>{
  const h=harness(); assert.equal(h.orbit.begin(0,10,20),false);
  assert.equal(h.orbit.begin(1,10,20),true); assert.equal(h.stopped,1);
  h.orbit.move(30,30); assert.equal(h.sent.at(-1).yaw,5); assert.equal(h.sent.at(-1).pitch,2.5);
  h.orbit.end(); h.orbit.move(100,100); assert.equal(h.sent.length,1);
  h.orbit.begin(1,100,100);h.orbit.move(104,100);assert.equal(h.sent.at(-1).yaw,6);
  h.orbit.reset();assert.deepEqual(h.sent.at(-1),{yaw:0,pitch:0});
});
test('unavailable streams and cursor discontinuities cannot rotate',()=>{
  const h=harness();h.disable();assert.equal(h.orbit.begin(1,0,0),false);
  const j=harness();j.orbit.begin(1,0,0);j.orbit.move(1000,0);assert.equal(j.sent.length,0);
  j.orbit.move(1002,0);assert.equal(j.sent.at(-1).yaw,.5);
  j.disable();j.orbit.move(1004,0);assert.equal(j.orbit.dragging,false);
});
test('viewport routes middle drag, releases capture and never enters robot operation',()=>{
  const listeners=new Map(),packets=[];let neutral=0,entered=0;
  const node=id=>({title:'',setPointerCapture(){},addEventListener(name,fn){
    const key=id+':'+name;listeners.set(key,[...(listeners.get(key)||[]),fn]);}});
  const nodes=new Map();const $=id=>{if(!nodes.has(id))nodes.set(id,node(id));return nodes.get(id);};
  const source=readFileSync(new URL('../app.js',import.meta.url),'utf8');
  const state={streamLive:true,selectedStreamerId:'BskRenderer__teleop_camera_spacecraft_overview',
    pixelStreaming:{emitCommand:p=>{packets.push(p);return true;}}};
  const context=vm.createContext({PreviewOrbit,state,$,window:node('window'),document:node('document'),
    exitOperationMode(){neutral++;},enterOperationMode(){entered++;},setMessage(){},freeCamera:{}});
  vm.runInContext(source.slice(source.indexOf('const previewOrbit ='),source.indexOf('window.addEventListener("keydown"')),context);
  const fire=(type,extra)=>{const event={button:1,buttons:4,clientX:10,clientY:20,pointerId:1,
    target:{closest:()=>null},preventDefault(){},stopImmediatePropagation(){this.stop=true;},...extra};
    for(const fn of listeners.get(type)||[]){fn(event);if(event.stop)break;}};
  fire('viewport:pointerdown');fire('viewport:pointermove',{clientX:30});
  assert.equal(neutral,1);assert.equal(entered,0);assert.equal(JSON.parse(packets[0].BskPreviewOrbit).yaw,5);
  fire('viewport:pointerup');fire('viewport:pointermove',{clientX:50});assert.equal(packets.length,1);
  fire('viewport:pointerdown');fire('window:blur');fire('viewport:pointermove',{clientX:60});assert.equal(packets.length,1);
});

test('view offsets survive switching without leaking into another camera',()=>{
 const {orbit,sent}=harness();
 orbit.select('overview');orbit.begin(1,0,0);orbit.move(40,20);orbit.end();
 orbit.select('capture');assert.equal(orbit.yaw,0);orbit.begin(1,0,0);orbit.move(-20,0);orbit.end();
 orbit.select('overview');assert.equal(orbit.yaw,10);assert.equal(orbit.pitch,5);
 orbit.reset();orbit.select('capture');assert.equal(orbit.yaw,-5);
});

test('view offsets survive reload and same-view reconnect keeps companion changes',()=>{
 const values=new Map();const storage={getItem:k=>values.get(k),setItem:(k,v)=>values.set(k,v)};
 const make=()=>new PreviewOrbit({storage,send:()=>true,available:()=>true,stopRobot:()=>{}});
 const first=make();first.select('main');first.begin(1,0,0);first.move(40,20);first.end();
 const second=make();second.select('main');assert.equal(second.yaw,10);assert.equal(second.pitch,5);
 second.views.set('main',[25,8]);second.select('main');assert.equal(second.yaw,25);
 second.select('capture');assert.equal(second.yaw,0);
 second.select('main');assert.equal(second.yaw,25);
});
