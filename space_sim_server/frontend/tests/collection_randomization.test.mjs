import test from 'node:test';
import assert from 'node:assert/strict';
import {serializeRandomization} from '../pages/collection_randomization.js';
test('each checkbox serializes an explicit boolean with typed parameters, including disabled choices',()=>{
  const out=serializeRandomization([{key:'wrist_wrench_noise',enabled:false,fields:[{name:'force_std_n',value:'0.1',type:'number'}]},
    {key:'external_disturbance',enabled:true,fields:[{name:'body',value:'ee_base',type:'string'},{name:'interval_s',value:'2',type:'number'}]}]);
  assert.deepEqual(out,{wrist_wrench_noise:{enabled:false,force_std_n:.1},external_disturbance:{enabled:true,body:'ee_base',interval_s:2}});
});
