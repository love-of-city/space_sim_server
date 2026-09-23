import test from 'node:test';
import assert from 'node:assert/strict';
import { webcrypto } from 'node:crypto';
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
   {arm_joint_position_rad:[0,0,0,0,0,0]});
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

test('v2 cancellation and disconnect never revive an automatic request',()=>{
 for (const cause of ['cancel','disconnect']) {
  const p=setupAuto();
  p.ui.armAutoStart('auto');
  p.ui.tick();
  const sentBefore = p.sent.filter(Boolean).length;
  if (cause==='cancel') p.cancelButton.click();
  else p.context.connected=false;
  p.ui.tick();
  p.context.connected=true;
  p.context.controlGranted=true;
  p.ui.update({status:'waiting',strategy:'validated-waypoints-v1',ready:false}, {arm_joint_position_rad:[0,0,0,0,0,0]});
  p.ui.tick();
  assert.equal(p.sent.filter(Boolean).length,sentBefore);
 }
});

test('v2 explicit retry stays disabled away from zero and reopens after measured restore',()=>{
 const p=setupAuto();
 p.ui.armAutoStart('auto');
 p.ui.tick();
 p.cancelButton.click();
 p.ui.update({status:'cancelled',strategy:'validated-waypoints-v1',ready:false},
   {arm_joint_position_rad:[0.2,0,0,0,0,0]});
 assert.equal(p.startButton.disabled,true);
 p.ui.update({status:'cancelled',strategy:'validated-waypoints-v1',ready:false},
   {arm_joint_position_rad:[0,0,0,0,0,0]});
 assert.equal(p.startButton.disabled,false);
 p.startButton.click();
 assert.equal(p.sent.filter(Boolean).length,2);
});
