import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {navigationUrl,routeForUrl} from '../platform_shell.js';
import {filterRecords,chartCategory} from '../pages/dataset_filters.js';

test('hidden chart groups retain their sampling clock without drawing a zero-size canvas',()=>{
  const datasetSource=readFileSync(new URL('../pages/datasets.js',import.meta.url),'utf8');
  const context=vm.createContext({state:{active:true,data:{times:[0,0.02]},degree:true}});
  vm.runInContext(datasetSource.slice(datasetSource.indexOf('class Chart {'),datasetSource.indexOf('function exportCSV()'))+'\nthis.Chart=Chart;',context);
  const chart=Object.create(context.Chart.prototype);
  Object.assign(chart,{group:{unit:'N',times:[0,0.005]},unit:{},w:320,h:200,
    wrap:{getBoundingClientRect:()=>({width:0,height:0})},
    canvas:{getContext(){throw new Error('hidden canvas must not be drawn');}}});
  chart.rebuild();chart.draw();
  assert.equal(chart.w,0);assert.equal(chart.h,0);assert.deepEqual(chart.times,[0,0.005]);
  assert.equal(chart.unit.textContent,'N');
});

test('deep links resolve the correct page and retain episode and batch selection',()=>{
  const route=routeForUrl('/datasets?episode=episode-one&batch=collection-one');
  assert.equal(route.page,'datasets');assert.equal(route.params.get('episode'),'episode-one');assert.equal(route.params.get('batch'),'collection-one');
  assert.equal(routeForUrl('/users').page,'users');assert.equal(routeForUrl('/collection').page,'collection');
});
test('page navigation preserves streaming access keys and rejects external destinations',()=>{
  assert.equal(navigationUrl('/datasets?episode=one','http://localhost:8000/?access_key=test-key'),'/datasets?episode=one&access_key=test-key');
  assert.throws(()=>navigationUrl('https://elsewhere.invalid/','http://localhost:8000/'));
});
test('dataset filters compose without mixing another batch or sampling clock',()=>{
  const records=[{id:'episode-20261016-120000-a',source:'auto',outcome:'success',task:'drawer',frames:6000,fps:50},{id:'episode-20261015-120000-b',source:'manual',outcome:'failure',task:'teleop',frames:3000,fps:30}];
  assert.deepEqual(filterRecords(records,{source:'auto',outcome:'success',from:'2026-10-16',to:'2026-10-16'}),[records[0]]);
  assert.deepEqual(filterRecords(records,{ids:new Set()}),[]);
  assert.deepEqual(filterRecords(records,{query:'teleop',ids:new Set([records[0].id])}),[]);
  assert.equal(chartCategory('force_200hz'),'wrench');assert.equal(chartCategory('torque'),'wrench');assert.equal(chartCategory('gripper_opening'),'joints');assert.equal(chartCategory('drawer_buckle'),'task');
});
const source=readFileSync(new URL('../app.js',import.meta.url),'utf8');
function section(start,end){return source.slice(source.indexOf(start),source.indexOf(end,source.indexOf(start)));}
test('hidden workbench never polls a motion command or arms preparation',()=>{
  let polls=0,ticks=0;
  const context=vm.createContext({state:{activePage:'datasets'},gamepadAction(){polls++;},armPreparation:{tick(){ticks++;}},console});
  vm.runInContext(section('function sendAction()','function updateMotionOutputs('),context);context.sendAction();
  assert.equal(polls,0);assert.equal(ticks,0);
});
test('leaving workbench stops motion before releasing its connection',()=>{
  const events=[];
  const state={operationActive:true,freeCameraMode:false,pressed:new Set(['KeyW']),operationRequested:true,connected:true,controlGranted:true,ws:{close(){events.push('close');}}};
  const context=vm.createContext({state,armPreparation:{cancel(){events.push('cancel');}},sendNeutralAction(){events.push('neutral');},highlightKeys(){},updateOperationUI(){},setMessage(){},disposePixelStream(){events.push('video');},setOnline(){},$(){return {};}});
  vm.runInContext(section('function exitOperationMode(','function sendAction()')+section('function leaveWorkbench(){','const platform='),context);context.leaveWorkbench();
  assert.ok(events.indexOf('neutral')<events.indexOf('close'));assert.equal(state.operationActive,false);assert.equal(state.operationRequested,false);assert.equal(state.ws,null);assert.equal(state.pressed.size,0);
});
