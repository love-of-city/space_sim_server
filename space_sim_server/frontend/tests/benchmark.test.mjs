import test from 'node:test';
import assert from 'node:assert/strict';
import {benchmarkMetric,benchmarkRate,renderBenchmarkJob} from '../pages/benchmark_view.js';
import {navigationUrl,routeForUrl} from '../platform_shell.js';

test('benchmark deep links use existing pages and preserve the selected batch',()=>{
  const url=navigationUrl('/datasets?benchmark=benchmark-one','http://localhost/?access_key=example');
  const route=routeForUrl(url);
  assert.equal(route.page,'datasets');assert.equal(route.params.get('benchmark'),'benchmark-one');
  assert.equal(route.params.get('access_key'),'example');
  const collection=routeForUrl('/collection?mode=benchmark');
  assert.equal(collection.page,'collection');assert.equal(collection.params.get('mode'),'benchmark');
});

test('displayed success rate uses all planned cases and separates angle and position units',()=>{
  assert.equal(benchmarkRate({success_rate_all_planned:.25,successes:1,planned:4}),'25.0%（1/4）');
  assert.equal(benchmarkMetric({final_attitude_error_rad:Math.PI/180}),'1.0000°');
  assert.equal(benchmarkMetric({final_position_error_m:.125}),'0.1250 m');
  assert.equal(benchmarkMetric({final_orbit:{impulse_vector_error_m_s:.001}}),'Δv 误差 0.0010 m/s');
  assert.equal(benchmarkMetric({final_orbit:{semimajor_axis_error_m:2,eccentricity:1e-6}}),'半长轴误差 2.00 m；e=1.00e-6');
  assert.equal(benchmarkMetric({position_rmse_m:.2,velocity_rmse_m_s:.003,final_position_error_m:1}),'位置 RMSE 0.2000 m；速度 RMSE 0.0030 m/s');
  assert.equal(benchmarkMetric({path_requirement:'full_duration_relative_hold',maximum_position_error_m:.25,longest_dwell_s:300,final_position_error_m:.01}),'最大位置误差 0.2500 m；连续保持 300.00 s');
  assert.equal(benchmarkMetric({path_requirement:'axis_approach_corridor',maximum_cross_track_m:2.5,final_position_error_m:.125}),'位置误差 0.1250 m；最大横向偏差 2.5000 m');
  assert.equal(benchmarkMetric({signed_revolutions:1.0001,maximum_radial_error_m:.25,maximum_plane_error_m:.01}),'绕行 1.0001 圈；最大径向偏差 0.2500 m；平面偏差 0.0100 m');
  assert.equal(benchmarkMetric({signed_revolutions:0,maximum_radial_error_m:null}),'—');
  assert.equal(benchmarkMetric(null),'—');
});

class Node {
  constructor(tag){this.tag=tag;this.children=[];this.dataset={};this.textContent='';}
  append(...nodes){this.children.push(...nodes);}
  flatten(){return [this,...this.children.flatMap(child=>child.flatten())];}
}
const job={id:'benchmark-one',configuration:'s02-wheel-attitude-v1',method:'attitude-wheel-pd-v1',status:'stopped',
  request:{split:'test',selection:'split'},owner:{username:'<img onerror=alert(1)>'},started_at:1,can_resume:true,
  summary:{planned:3,finished:1,successes:0,success_rate_all_planned:0,wilson_95_interval:null,
    trials:[{trial_id:'0000-seed-201',seed:201,outcome:'cancelled'},{trial_id:'0001-seed-202',seed:202,outcome:'pending'}]}};

test('results keep cancelled cases visible, offer only pending continuation, and render owner as text',()=>{
  globalThis.document={createElement:tag=>new Node(tag)};
  try{
    const nodes=renderBenchmarkJob(job,{api(){},notice(){},refresh(){},detail:true}).flatten();
    assert.ok(nodes.some(node=>node.textContent==='已取消'));
    assert.ok(nodes.some(node=>node.tag==='button'&&node.textContent==='继续未启动样本'));
    assert.ok(nodes.some(node=>node.textContent.startsWith(job.owner.username)));
    assert.ok(!nodes.some(node=>node.tag==='img'));
    assert.ok(nodes.some(node=>node.textContent.includes('暂不报告置信区间')));
    assert.equal(nodes.filter(node=>node.href?.includes('/artifacts/')).length,0);
  }finally{delete globalThis.document;}
});
