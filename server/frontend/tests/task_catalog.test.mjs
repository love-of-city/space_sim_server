import test from 'node:test';
import assert from 'node:assert/strict';
import {createTaskCatalog} from '../task_catalog.js';

const tasks = [
  ['aviation_plug_insert','航空插头拿取与插入'],
  ['guide_plug_insert','导向插头拿取与插入'],
  ['cylindrical_plug_insert','圆柱插头拿取与插入'],
  ['drawer_open','抽屉拉起'], ['drawer_close','抽屉放回'],
  ['locking_rod_fold','锁杆装置放倒'], ['locking_rod_restore','锁杆装置复位'],
].map(([id,label])=>({id,label}));
const response = {tasks, task_aliases:{plug_insert:'cylindrical_plug_insert'}};

test('dataset and collection labels share canonical IDs without classifying every plug as cylindrical', async()=>{
  const catalog=createTaskCatalog(async()=>response);
  await catalog.load();
  for(const task of tasks){
    assert.equal(catalog.label(task.id),task.label);
    assert.equal(catalog.label(task.id+':saved-instruction'),task.label);
  }
  assert.equal(catalog.label('plug_insert:legacy'),tasks[2].label);
  assert.equal(catalog.label('future_plug_insert:unknown'),'future_plug_insert:unknown');
  assert.equal(catalog.label('spacecraft arm teleoperation'),'遥操作采集');
});

test('concurrent page activations share one catalog request and failed loading can be retried', async()=>{
  let calls=0,resolve;
  const catalog=createTaskCatalog(()=>{calls++;return new Promise(accept=>{resolve=accept;});});
  const first=catalog.load(),second=catalog.load();
  await Promise.resolve();assert.equal(calls,1);
  resolve(response);assert.equal(await first,await second);
  await catalog.load();assert.equal(calls,1);
  let attempts=0;
  const failing=createTaskCatalog(async()=>{if(++attempts===1)throw Error('offline');return response;});
  await assert.rejects(failing.load(),/offline/);
  assert.equal(failing.label('aviation_plug_insert'),'aviation_plug_insert');
  await failing.load();assert.equal(failing.label('aviation_plug_insert'),tasks[0].label);
});
