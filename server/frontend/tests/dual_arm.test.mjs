import test from 'node:test';
import assert from 'node:assert/strict';
import {armObservation, MainCompanion} from '../dual_arm.js';

test('capture telemetry never presents operating arm joint angles or force',()=>{
 const raw={arm_joint_position_rad:[9],wrist_wrench:{force:1},capture_arm:{joint_position_rad:[1,2,3,4,5,6],end_effector_position_body_m:[1,0,0]}};
 const obs=armObservation(raw,'capture');
 assert.deepEqual(obs.arm_joint_position_rad,[1,2,3,4,5,6]);
 assert.equal(obs.wrist_wrench,null);
 assert.equal(armObservation(raw,'operating'),raw);
});

test('companion subscribes to existing main stream and never issues camera commands',()=>{
 const connections=[];
 const controller=new MainCompanion({create:id=>{const stream={id,disconnect(){this.closed=true;},emitCommand(){throw Error('must not reset orbit');}};connections.push(stream);return stream;},show:()=>{}});
 controller.update('Main__teleop_camera_capture_wrist_cam','Main');
 assert.equal(connections[0].id,'Main');
 controller.update('Main__teleop_camera_capture_wrist_cam','Main');
 assert.equal(connections.length,1);
 controller.update('Main','Main');assert.equal(connections[0].closed,true);
 controller.update('Main__teleop_camera_capture_wrist_cam','Main');
 assert.equal(connections.length,2);
 controller.dispose();assert.equal(connections[1].closed,true);
});
