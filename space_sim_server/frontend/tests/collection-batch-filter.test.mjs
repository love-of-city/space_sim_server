import test from 'node:test';
import assert from 'node:assert/strict';
import {filterRecords} from '../pages/dataset_filters.js';
test('all dates and multiple private collection batches remain independently selectable',()=>{
 const records=[{id:'old'}, {id:'a',collection_batch:{batch_id:'day1'}}, {id:'b',collection_batch:{batch_id:'day2'}}, {id:'c',collection_batch:{batch_id:'day3'}}];
 assert.equal(filterRecords(records).length,4);
 assert.deepEqual(filterRecords(records,{batches:['day1','day2']}).map(r=>r.id),['a','b']);
 assert.deepEqual(filterRecords(records,{batches:['missing']}),[]);
 assert.deepEqual(filterRecords(records,{batches:['day1','day2'],ids:new Set(['b'])}).map(r=>r.id),['b']);
});
