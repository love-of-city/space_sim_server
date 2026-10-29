import test from 'node:test';
import assert from 'node:assert/strict';
import {collectionLayoutError} from '../pages/collection_layout_status.js';
const base={tasks:[{id:'plug',requires_color_layout:true},{id:'drawer'}],fixed_layout:{available:false,configured:false},layout_randomization:{available:false}};
test('colour tasks need an actual fixed layout, while legacy drawer remains available',()=>{
  assert.match(collectionLayoutError(base,'plug',false),/固定操作台/);
  assert.equal(collectionLayoutError(base,'drawer',false),'');
  assert.equal(collectionLayoutError({...base,fixed_layout:{available:true,configured:true}},'plug',false),'');
});
test('the selected layout mode controls readiness without silently switching modes',()=>{
  const pool={...base,layout_randomization:{available:true}};
  assert.match(collectionLayoutError(pool,'plug',false),/固定操作台/);
  assert.equal(collectionLayoutError(pool,'plug',true),'');
  assert.match(collectionLayoutError(base,'drawer',true),/随机布局/);
  assert.match(collectionLayoutError({...base,fixed_layout:{available:false,configured:true}},'drawer',false),/未通过校验/);
});
