import test from 'node:test';
import assert from 'node:assert/strict';
import { wristViewState, videoContentRect } from '../wrist_vision.js';

const good={valid:true,status:'tracking',age_s:.1,stale_after_s:1.5,session_id:'s',
  source_frame_id:'7',marker_count:4,result:{alignment:{valid:true,
    interface_position_display_m:[.001,-.006,.499],position_error_display_mm:[1,-6,-1],
    rotation_error_display_deg:[-2,0,1]},detections:[]}};
test('success is read-only and values are cleared on recognition failure',()=>{
  assert.equal(wristViewState(good,0,100).valid,true);
  const lost=wristViewState({...good,valid:false,status:'lost',result:null},0,100);
  assert.equal(lost.valid,false);assert.equal(lost.result,null);assert.match(lost.text,/识别失败/);
  assert.doesNotMatch(lost.text,/499/);
  assert.doesNotMatch(wristViewState({...good,valid:false,status:'error',result:null,reason:'private worker traceback'},0,100).text,/private/);
});
test('stale result or shifted inspection camera never draws old boxes',()=>{
  assert.equal(wristViewState(good,0,1500).result,null);
  assert.match(wristViewState(good,0,100,{orbitShifted:true}).text,/Home/);
  assert.equal(wristViewState(good,0,100,{session:'different'}).result,null);
});
test('letterboxed native image maps inside its actual video rectangle',()=>{
  assert.deepEqual(videoContentRect({x:0,y:0,width:1000,height:700},1024,1024),{x:150,y:0,width:700,height:700});
});

test('active visual probe is labelled as control instead of read-only observation',()=>{
  const active=wristViewState(good,0,100,{visualProbe:true});
  assert.match(active.text,/视觉闭环短程试验/);
  assert.doesNotMatch(active.text,/仅观察/);
  assert.match(wristViewState({...good,valid:false,status:'lost'},0,100,{visualProbe:true}).text,/丢失.*过期即停/);
});
