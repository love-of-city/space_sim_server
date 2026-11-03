import test from 'node:test';
import assert from 'node:assert/strict';
import {groupSceneTemplates,libraryFamilyId,resolveSceneTask} from '../library_templates.js';

const counts={'01_self_operation':5,'02_docked_operation':5,'03_rendezvous':1,'04_cross_transfer':3};
const tasks=Object.entries(counts).flatMap(([family,count])=>Array.from({length:count},(_,i)=>({
  id:`library/${family}/task-${i}`,label:`${family} / Task ${i}`,library_scene:true,
})));

test('four launch scenes retain all fourteen task initialization profiles',()=>{
  const original={id:'original-platform',label:'Original'};
  const visible=groupSceneTemplates([original,...tasks]);
  assert.equal(visible[0],original);
  assert.equal(visible.filter(item=>item.library_family).length,4);
  assert.deepEqual(visible.slice(1).map(item=>item.tasks.length),[5,5,1,3]);
  assert.deepEqual(visible.slice(1).flatMap(item=>item.tasks.map(t=>t.id)),tasks.map(t=>t.id));
});

test('selection resolves the exact task and cannot retain a task from another family',()=>{
  const selected=tasks.at(-1),family=libraryFamilyId(selected.id);
  assert.equal(resolveSceneTask(tasks,family,selected.id),selected);
  assert.equal(resolveSceneTask(tasks,family,tasks[0].id),tasks[11]);
  assert.equal(resolveSceneTask(tasks,selected.id),selected); // Existing script aliases remain valid.
});
