import test from 'node:test';
import assert from 'node:assert/strict';
import {formatWristWrench} from '../wrist_wrench.js';
test('missing and initial invalid sensor never display a measured zero',()=>{
  assert.equal(formatWristWrench(null).force,'—');
  assert.equal(formatWristWrench({valid:false}).force,'—');
});
test('force and torque remain separate and retain SI units',()=>{
  const result=formatWristWrench({valid:true,force_n:[1,-2,3],torque_nm:[.1,.2,-.3],site_name:'wrist_ft_site',sample_time_ns:'100000000'});
  assert.equal(result.force,'1.000, -2.000, 3.000 N');
  assert.equal(result.torque,'0.100, 0.200, -0.300 N·m');
  assert.match(result.detail,/0.1000 s/);
});
test('non finite data cannot appear as a valid measurement',()=>{
  assert.equal(formatWristWrench({valid:true,force_n:[NaN,0,0],torque_nm:[0,0,0]}).force,'—');
});

 test('disconnect/reset clear all last measured wrench fields',async()=>{
   const {readFileSync}=await import('node:fs');
   const {default:vm}=await import('node:vm');
   const source=readFileSync(new URL('../app.js',import.meta.url),'utf8');
   const fn=source.slice(source.indexOf('function clearWristWrench()'),source.indexOf('const gamepadInput'));
   const nodes=new Map(['wristForce','wristTorque','wristWrenchDetail'].map(id=>[id,{textContent:'old'}]));
   vm.runInNewContext(fn+';clearWristWrench();',{$:id=>nodes.get(id)});
   assert.equal(nodes.get('wristForce').textContent,'—');
   assert.equal(nodes.get('wristTorque').textContent,'—');
   assert.equal(nodes.get('wristWrenchDetail').textContent,'无实时测量');
 });
