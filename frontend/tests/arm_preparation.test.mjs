import test from 'node:test';
import assert from 'node:assert/strict';
import { webcrypto } from 'node:crypto';
import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import {createArmPreparation, parseOperatingAngles, DEFAULT_OPERATING_DEG} from '../arm_preparation.js';

globalThis.crypto ??= webcrypto;
globalThis.document = {hidden:false};
const limits = [...Array.from({length:5},()=>[-180,180]),[-360,360]];
function node() { return {value:'', disabled:false, textContent:'', events:{}, addEventListener(name,fn){this.events[name]=fn;}, click(){this.events.click();}}; }
function setup() {
 const inputs=Array.from({length:6},node), fields={querySelectorAll:()=>inputs};
 const startButton=node(),cancelButton=node(),defaultButton=node(),status=node();
 const context={sceneReady:true,connected:true,canManageScene:true,controlGranted:true,estopped:false};
 const sent=[],messages=[];let exited=0,activated=0;
 const ui=createArmPreparation({fields,startButton,cancelButton,defaultButton,status,getContext:()=>context,
  send:r=>sent.push(r),message:m=>messages.push(m),cancelMotion:()=>exited++,activateControl:()=>activated++,changed(){}});
 ui.configure({arm_joint_limits_deg:limits});ui.setRuntime({instance_id:'scene',arm_preparation_required:true},true);ui.update({status:'waiting',ready:false});
 return {ui,inputs,startButton,cancelButton,defaultButton,status,context,sent,messages,get exited(){return exited;},get activated(){return activated;}};
}
test('literal screenshot defaults and validation',()=>{
 assert.deepEqual(parseOperatingAngles(DEFAULT_OPERATING_DEG,limits),DEFAULT_OPERATING_DEG);
 assert.equal(parseOperatingAngles(DEFAULT_OPERATING_DEG,limits)[5],0);
 assert.throws(()=>parseOperatingAngles(['',0,0,0,0,0],limits),/操作角度/);
 assert.throws(()=>parseOperatingAngles([0,0,0,0,0,361],limits),/J6/);
});
test('editing does not move, button latches one heartbeat, measured ready gates operation',()=>{
 const p=setup();assert.deepEqual(p.ui.read(),DEFAULT_OPERATING_DEG);assert.equal(p.ui.ready(),false);
 p.inputs[1].value='-61';assert.equal(p.sent.length,0);
 p.startButton.click();const request=p.sent.at(-1);
 assert.equal(p.exited,1);assert.equal(request.joint_position_deg[1],-61);assert.ok(request.request_id);
 assert.equal(p.ui.tick(),true);assert.strictEqual(p.sent.at(-1),request);
 assert.equal(p.ui.ready(),false);assert.equal(p.startButton.disabled,true);
 p.ui.update({request_id:request.request_id,status:'moving',ready:false,progress:1});assert.equal(p.ui.ready(),false);
 p.ui.update({request_id:request.request_id,status:'ready',ready:true,progress:1});
 assert.equal(p.ui.ready(),true);assert.equal(p.sent.at(-1),null);assert.equal(p.ui.tick(),false);
});
test('cancel, disconnect, revocation and page hiding clear motion authorization',()=>{
 for(const cause of ['cancel','disconnect','revoke','hidden','estop']){
  const p=setup();p.startButton.click();
  if(cause==='cancel')p.cancelButton.click();
  if(cause==='disconnect')p.context.connected=false;
  if(cause==='revoke')p.context.controlGranted=false;
  if(cause==='hidden')document.hidden=true;
  if(cause==='estop')p.context.estopped=true;
  p.ui.tick();assert.equal(p.sent.at(-1),null);assert.equal(p.ui.tick(),false);
  document.hidden=false;
 }
});
test('new click requires control and failures never mark ready',()=>{
 const p=setup();p.context.controlGranted=false;p.startButton.click();assert.equal(p.activated,1);assert.equal(p.sent.length,0);
 p.context.controlGranted=true;p.ui.tick();let id=p.sent.at(-1).request_id;
 p.ui.update({request_id:id,status:'failed',ready:false,reason:'collision'});
 assert.equal(p.ui.ready(),false);assert.equal(p.ui.tick(),false);assert.match(p.status.textContent,/collision/);
 p.startButton.click();assert.notEqual(p.sent.at(-1).request_id,id);
});
test('reset and scene replacement clear readiness; refresh preserves unsent edits',()=>{
 const p=setup();p.ui.update({status:'ready',ready:true});assert.equal(p.ui.ready(),true);
 p.inputs[1].value='-65';p.ui.setRuntime({instance_id:'scene',arm_preparation_required:true},true);assert.equal(p.ui.ready(),true);assert.equal(p.inputs[1].value,'-65');
 p.ui.setRuntime({instance_id:'scene',arm_preparation_required:true},false);assert.equal(p.ui.ready(),false);
 p.ui.setRuntime({instance_id:'scene2',arm_preparation_required:true,operating_arm_joint_position_deg:[1,2,3,4,5,6]},true);
 assert.deepEqual(p.ui.read(),[0,2,3,4,5,0]);
});

test('older legacy telemetry is compatible but missing zero-start readiness fails closed',()=>{
 const p=setup();p.ui.update(undefined);assert.equal(p.ui.ready(),false);
 p.ui.setRuntime({instance_id:'legacy'},true);p.ui.update(undefined);assert.equal(p.ui.ready(),true);
 assert.equal(p.startButton.disabled,true);
});

test('control grant uses one click and cancelling the grant cannot start later',()=>{
 const p=setup();p.context.controlGranted=false;p.startButton.click();
 assert.equal(p.startButton.disabled,true);assert.match(p.status.textContent,/申请控制权/);
 p.cancelButton.click();p.context.controlGranted=true;p.ui.tick();
 assert.equal(p.sent.filter(Boolean).length,0);
 p.startButton.click();assert.equal(p.sent.filter(Boolean).length,1);
});
test('unacknowledged requests time out instead of showing requesting forever',()=>{
 const original=globalThis.performance;
 let time=100;globalThis.performance={now:()=>time};
 try {
  const p=setup();p.startButton.click();time+=5001;p.ui.update({status:'waiting',ready:false});p.ui.tick();
  assert.equal(p.sent.at(-1),null);assert.equal(p.startButton.disabled,false);
  assert.match(p.messages.at(-1),/未确认/);
 } finally {globalThis.performance=original;}
});
test('arming and phase number are visible while motion remains gated',()=>{
 const p=setup();p.startButton.click();const id=p.sent.at(-1).request_id;
 p.ui.update({request_id:id,status:'arming',ready:false});assert.match(p.status.textContent,/稳定停止/);
 p.ui.update({request_id:id,status:'moving',ready:false,phase:2,phase_count:2});
 assert.match(p.status.textContent,/阶段 2\/2/);assert.equal(p.ui.ready(),false);
});

test('simulation hello alone does not enable preparation before first measured telemetry',()=>{
 const p=setup();p.ui.update(undefined);assert.equal(p.startButton.disabled,true);
 p.startButton.click();assert.equal(p.sent.length,0);assert.match(p.status.textContent,/关节遥测/);
 p.ui.update({status:'waiting',ready:false});assert.equal(p.startButton.disabled,false);
});


test('J1 and J6 are held fields and old saved defaults cannot restore stage three',()=>{
 const p=setup();assert.equal(p.inputs[0].disabled,true);assert.equal(p.inputs[5].disabled,true);
 p.ui.setRuntime({instance_id:'old',arm_preparation_required:true,operating_arm_joint_position_deg:[-30.2,-67.6,-86.6,143.2,-85.5,337.4]},true);
 assert.deepEqual(p.ui.read(),DEFAULT_OPERATING_DEG);
 p.ui.update({status:'waiting',ready:false});p.startButton.click();
 assert.deepEqual(p.sent.at(-1).joint_position_deg,DEFAULT_OPERATING_DEG);
});

function setupAuto({controlGranted = true} = {}) {
 const p=setup();
 p.context.controlGranted=controlGranted;
 p.ui.setRuntime({instance_id:'auto',randomization_profile:'teleop-zero-prepare-v2',arm_preparation_required:true,
   operating_arm_joint_position_deg:[17,-61,-82,135,-72,23]},true);
 p.ui.update({status:'waiting',strategy:'validated-waypoints-v1',ready:false},
   {scene_instance_id:'auto',arm_joint_position_rad:[0,0,0,0,0,0]});
 return p;
}

test('v2 auto-starts once after fresh telemetry and delayed control grant',()=>{
 const p=setupAuto({controlGranted:false});
 assert.deepEqual(p.ui.read(),[17,-61,-82,135,-72,23]);
 p.ui.armAutoStart('auto');
 assert.equal(p.ui.tick(),true);
 assert.equal(p.activated,1);
 assert.equal(p.sent.filter(Boolean).length,0);
 p.context.controlGranted=true;
 p.ui.tick();
 assert.equal(p.sent.filter(Boolean).length,1);
 const request=p.sent.find(Boolean);
 assert.deepEqual(request.joint_position_deg,[17,-61,-82,135,-72,23]);
 p.ui.armAutoStart('auto');
 p.ui.tick();
 assert.equal(p.sent.filter(Boolean).length,2);
});

test('v2 preserves non-zero J1/J6 target and locks edits after creation',()=>{
 const p=setupAuto();
 assert.deepEqual(p.ui.read(),[17,-61,-82,135,-72,23]);
 assert.equal(p.inputs[0].disabled,true);
 assert.equal(p.inputs[5].disabled,true);
 assert.equal(p.inputs[0].value,'17');
 assert.equal(p.inputs[5].value,'23');
});

const appSource=readFileSync(new URL('../app.js',import.meta.url),'utf8');
function appSection(start,end) {
 const first=appSource.indexOf(start),last=appSource.indexOf(end,first+start.length);
 assert.ok(first>=0 && last>first,`missing app callback section: ${start}`);
 return appSource.slice(first,last);
}

function setupApp(testContext,controlGranted) {
 const originalDocument=globalThis.document;
 const listeners=new Map(),nodes=new Map(),packets=[];
 const doc={hidden:false,addEventListener(name,callback){listeners.set(name,callback);}};
 globalThis.document=doc;
 testContext.after(()=>{globalThis.document=originalDocument;});
 const inputs=Array.from({length:6},node);
 const $=id=>{
  if(!nodes.has(id))nodes.set(id,node());
  return nodes.get(id);
 };
 $('operatingJointFields').querySelectorAll=()=>inputs;
 const state={sceneReady:true,connected:true,canManageScene:true,controlGranted,estopped:false,
  operationActive:false,operationRequested:false,freeCameraMode:false,pressed:new Set(),sequence:0};
 const instance={instance_id:'auto',randomization_profile:'teleop-zero-prepare-v2',arm_preparation_required:true,
  operating_arm_joint_position_deg:[17,-61,-82,135,-72,23]};
 class Socket {
  static OPEN=1;
  readyState=Socket.OPEN;
  send(packet){packets.push(JSON.parse(packet));}
 }
 let ui;
 const context=vm.createContext({state,$,document:doc,createArmPreparation,WebSocket:Socket,URL,URLSearchParams,
  location:{protocol:'http:',host:'test.invalid',search:''},
  setOnline(){},setMessage(){},updateOperationUI(){},updateEpisodeUI(){},highlightKeys(){},
  jointAngles:{clear(){}},
  async apiRequest(path,options){
   assert.equal(path,'/api/scenes/reset');assert.equal(options.method,'POST');
   return {ok:true};
  },
  async readApiResponse(){return {status:'completed'};},
  async refreshState(){
   state.sceneReady=true;
   ui.setRuntime(instance,true);
   ui.update({status:'waiting',ready:false,strategy:'validated-waypoints-v1'},
    {scene_instance_id:'auto',arm_joint_position_rad:[0,0,0,0,0,0]});
  },
 });
 vm.runInContext([
  appSection('const armPreparation = createArmPreparation(', 'let initialJointCatalog'),
  appSection('function connect() {','async function connectPixelStreaming()'),
  appSection('function transmitAction(', 'function sendNeutralAction('),
  appSection('function exitOperationMode(', 'function sendAction()'),
  appSection('document.addEventListener("visibilitychange"', 'document.addEventListener("focusin"'),
  appSection('$("estop").addEventListener("click"', '$("linearSpeed").addEventListener'),
  appSection('async function resetScene()', 'async function stopScene()'),
 ].join('\n'),context);
 ui=vm.runInContext('armPreparation',context);
 context.connect();
 ui.configure({arm_joint_limits_deg:limits});ui.setRuntime(instance,true);
 ui.update({status:'waiting',ready:false,strategy:'validated-waypoints-v1'},
  {scene_instance_id:'auto',arm_joint_position_rad:[0,0,0,0,0,0]});
 const message=type=>state.ws.onmessage({data:JSON.stringify({type})});
 return {ui,state,$,packets,context,doc,message,listeners,
  requests:()=>packets.filter(packet=>packet.arm_preparation)};
}

for(const phase of ['waiting-control','moving']) {
 for(const cause of ['cancel','hidden','disconnect','revoke','estop']) {
  test(`v2 frontend ${cause} callback during ${phase} never auto-rearms after restore`,async testContext=>{
   const app=setupApp(testContext,phase==='moving');
   app.ui.armAutoStart('auto');assert.equal(app.ui.tick(),true);
   const original=app.requests()[0]?.arm_preparation;
   if(phase==='moving') {
    assert.ok(original);
    app.ui.update({request_id:original.request_id,status:'moving',ready:false},
     {scene_instance_id:'auto',arm_joint_position_rad:[0.2,0,0,0,0,0]});
   } else {
    assert.equal(app.requests().length,0);
    assert.equal(app.packets.filter(packet=>packet.type==='activate_control').length,1);
   }
   const requestCount=app.requests().length,packetCount=app.packets.length;
   if(cause==='cancel')app.$('cancelPrepareArm').click();
   if(cause==='hidden'){app.doc.hidden=true;app.listeners.get('visibilitychange')();}
   if(cause==='disconnect')app.state.ws.onclose();
   if(cause==='revoke')app.message('control_revoked');
   if(cause==='estop')app.$('estop').click();
   assert.equal(app.ui.ready(),false);
   if(phase==='moving' && ['cancel','hidden','estop'].includes(cause)) {
    assert.equal(app.packets.length,packetCount+1);
    assert.equal(app.packets.at(-1).deadman,false);
    assert.equal(app.packets.at(-1).arm_preparation,undefined);
   }
   if(cause==='hidden'){app.doc.hidden=false;app.listeners.get('visibilitychange')();}
   if(cause==='disconnect'){app.context.connect();app.state.ws.onopen();}
   if(cause==='estop')app.$('estop').click();
   app.message('control_granted');
   assert.equal(app.ui.tick(),false,'recovery before the next heartbeat must not revive intent');
   app.ui.update({request_id:original?.request_id,status:'cancelled',ready:false},
    {scene_instance_id:'auto',arm_joint_position_rad:[0.2,0,0,0,0,0]});
   assert.equal(app.$('prepareArm').disabled,true);
   app.$('prepareArm').click();assert.equal(app.requests().length,requestCount);
   await app.context.resetScene();
   assert.equal(app.$('prepareArm').disabled,false);
   app.ui.armAutoStart('auto');
   for(let heartbeat=0;heartbeat<3;heartbeat++)assert.equal(app.ui.tick(),false);
   assert.equal(app.requests().length,requestCount,'zero restoration must not authorize motion');
   assert.equal(app.ui.ready(),false);
   app.$('prepareArm').click();
   assert.equal(app.requests().length,requestCount+1);
   const retry=app.requests().at(-1).arm_preparation;
   assert.ok(retry.request_id);assert.notEqual(retry.request_id,original?.request_id);
   assert.deepEqual(retry.joint_position_deg,[17,-61,-82,135,-72,23]);
   assert.equal(app.requests().at(-1).deadman,true);
   assert.equal(app.ui.ready(),false);
   app.ui.update({request_id:retry.request_id,status:'moving',ready:false,progress:1},
    {scene_instance_id:'auto',arm_joint_position_rad:retry.joint_position_deg.map(value=>value*Math.PI/180)});
   assert.equal(app.ui.ready(),false);assert.equal(app.ui.tick(),true);
   assert.equal(app.requests().at(-1).arm_preparation.request_id,retry.request_id);
   app.ui.update({request_id:retry.request_id,status:'ready',ready:true},
    {scene_instance_id:'auto',arm_joint_position_rad:retry.joint_position_deg.map(value=>value*Math.PI/180)});
   assert.equal(app.ui.ready(),true);assert.equal(app.ui.tick(),false);
   assert.equal(app.packets.at(-1).deadman,false);
  });
 }
}

test('v2 requires matching scene telemetry before consuming auto-start intent',()=>{
 for (const sceneId of [undefined, null, 'old']) {
  for (const status of ['waiting', 'ready', 'failed', 'cancelled', 'moving']) {
   const p=setupAuto();
   const instance={instance_id:'auto',randomization_profile:'teleop-zero-prepare-v2',arm_preparation_required:true};
   p.ui.setRuntime(instance,false);
   p.ui.setRuntime(instance,true);
   p.ui.armAutoStart('auto');
   p.ui.update({status,ready:status==='ready',strategy:'validated-waypoints-v1'},
    {scene_instance_id:sceneId,arm_joint_position_rad:[0,0,0,0,0,0]});
   assert.equal(p.ui.ready(),false);
   assert.equal(p.ui.tick(),true);
   assert.equal(p.sent.filter(Boolean).length,0);
   p.ui.update({status:'waiting',ready:false,strategy:'validated-waypoints-v1'},
    {scene_instance_id:'auto',arm_joint_position_rad:[0,0,0,0,0,0]});
   p.ui.tick();
   assert.equal(p.sent.filter(Boolean).length,1);
  }
 }
});

test('v2 ignores foreign completion and failure without clearing its pending request',()=>{
 const p=setupAuto();
 p.ui.armAutoStart('auto');
 p.ui.tick();
 const request=p.sent.find(Boolean);
 for (const sceneId of [undefined, 'old']) {
  for (const status of ['ready', 'failed', 'cancelled']) {
   p.ui.update({request_id:request.request_id,status,ready:status==='ready'},
    {scene_instance_id:sceneId,arm_joint_position_rad:[1,1,1,1,1,1]});
   assert.equal(p.ui.ready(),false);
   assert.equal(p.ui.tick(),true);
   assert.strictEqual(p.sent.at(-1),request);
  }
 }
 p.ui.update({request_id:request.request_id,status:'ready',ready:true},
  {scene_instance_id:'auto',arm_joint_position_rad:[1,1,1,1,1,1]});
 assert.equal(p.ui.ready(),true);
 assert.equal(p.sent.at(-1),null);
 assert.equal(p.ui.tick(),false);
});

test('v2 runtime refresh and reconnect do not arm a request',()=>{
 const p=setupAuto();
 p.context.connected=false;
 p.ui.tick();
 p.context.connected=true;
 p.ui.setRuntime({instance_id:'auto',randomization_profile:'teleop-zero-prepare-v2',arm_preparation_required:true},true);
 p.ui.update({status:'waiting',ready:false},
  {scene_instance_id:'auto',arm_joint_position_rad:[0,0,0,0,0,0]});
 assert.equal(p.ui.tick(),false);
 assert.equal(p.sent.filter(Boolean).length,0);
});
