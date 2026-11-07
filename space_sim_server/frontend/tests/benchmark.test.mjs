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
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-momentum-unload-score/1',wheel_momentum_norm_reduction_nm_s:4.5,final_maximum_wheel_speed_rad_s:314.125}),'轮系角动量模长降低 4.500 N·m·s；末端最大轮速 314.13 rad/s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-momentum-unload-score/1',wheel_momentum_norm_reduction_nm_s:null,final_maximum_wheel_speed_rad_s:null}),'轮系角动量模长降低 —；末端最大轮速 —');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-wheel-recovery-score/1',diagnosis_latency_s:.86,final_attitude_error_rad:Math.PI/180}),'异常诊断延迟 0.86 s；姿态误差 1.0000°');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-debris-avoidance-score/1',minimum_range_m:201.5,final_coast_s:200}),'最小碎片间距 201.50 m；末段无推力滑行 200.00 s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-wheel-recovery-score/1',diagnosis_latency_s:null,final_attitude_error_rad:null}),'异常诊断延迟 未确认；姿态误差 —');
  assert.equal(benchmarkMetric({final_boresight_error_rad:Math.PI/180,failed_tracking_samples:1,tracking_samples:9001}),'指向误差 1.0000°；跟踪时窗超限 1/9001 帧');
  assert.equal(benchmarkMetric({target_count:3,completed_targets:[{target_index:0}],final_attitude_error_rad:0}), '已完成姿态 1/3；当前姿态误差 0.0000°');
  assert.equal(benchmarkMetric({final_position_error_m:.125}),'0.1250 m');
  assert.equal(benchmarkMetric({final_orbit:{impulse_vector_error_m_s:.001}}),'Δv 误差 0.0010 m/s');
  assert.equal(benchmarkMetric({path_requirement:'continuous_low_thrust',longest_continuous_thrust_s:660.5,semimajor_axis_increase_m:1300.125,final_orbit:{impulse_vector_error_m_s:.001}}),'最长连续推力 660.50 s；半长轴增加 1300.13 m');
  assert.equal(benchmarkMetric({path_requirement:'orbital_plane_adjustment',target_plane_error_rad:Math.PI/180000,achieved_plane_change_rad:Math.PI/3600}),'轨道面误差 0.00100°；实际改变 0.05000°');
  assert.equal(benchmarkMetric({final_orbit:{semimajor_axis_error_m:2,eccentricity:1e-6}}),'半长轴误差 2.00 m；e=1.00e-6');
  assert.equal(benchmarkMetric({path_requirement:'continuous_orbit_maintenance',maximum_maintenance_axis_error_m:28.5,failed_maintenance_samples:1,maintenance_samples:54001,final_orbit:{semimajor_axis_error_m:2,eccentricity:1e-6}}),'维持时窗最大半长轴误差 28.50 m；超限 1/54001 帧');
  assert.equal(benchmarkMetric({position_rmse_m:.2,velocity_rmse_m_s:.003,final_position_error_m:1}),'位置 RMSE 0.2000 m；速度 RMSE 0.0030 m/s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-navigation-recovery-score/1',maximum_outage_position_error_m:1.25,recovery_confirmation_latency_s:5.5}),'中断期间最大位置误差 1.2500 m；恢复确认延迟 5.50 s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-navigation-recovery-score/1',maximum_outage_position_error_m:null,recovery_confirmation_latency_s:null}),'中断期间最大位置误差 —；恢复确认延迟 未确认');
  assert.equal(benchmarkMetric({path_requirement:'full_duration_relative_hold',maximum_position_error_m:.25,longest_dwell_s:300,final_position_error_m:.01}),'最大位置误差 0.2500 m；连续保持 300.00 s');
  assert.equal(benchmarkMetric({path_requirement:'axis_approach_corridor',maximum_cross_track_m:2.5,final_position_error_m:.125}),'位置误差 0.1250 m；最大横向偏差 2.5000 m');
  assert.equal(benchmarkMetric({path_requirement:'abort_and_retreat',reversal_confirmed_s:25.12,retreat_progress_m:99.9}),'向外运动确认 25.12 s；撤离距离 99.90 m');
  assert.equal(benchmarkMetric({path_requirement:'far_to_proximity_rendezvous',range_reduction_m:6850,final_range_m:150}),'交会缩短距离 6850.00 m；当前距离 150.00 m');
  assert.equal(benchmarkMetric({path_requirement:'abort_and_retreat',reversal_confirmed_s:null,retreat_progress_m:-10}),'向外运动确认 未确认；撤离距离 -10.00 m');
  assert.equal(benchmarkMetric({signed_revolutions:1.0001,maximum_radial_error_m:.25,maximum_plane_error_m:.01}),'绕行 1.0001 圈；最大径向偏差 0.2500 m；平面偏差 0.0100 m');
  assert.equal(benchmarkMetric({signed_revolutions:0,maximum_radial_error_m:null}),'—');
  assert.equal(benchmarkMetric(null),'—');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-rendered-pose-score/1',available_pose_count:38,expected_estimates:40,position_rmse_m:.004,orientation_rmse_rad:Math.PI/180}),'有效估计 38/40；位置 RMSE 4.00 mm；姿态 RMSE 1.000°');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-rendered-pose-score/1',available_pose_count:0,expected_estimates:40,position_rmse_m:null,orientation_rmse_rad:null}),'有效估计 0/40');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-rendered-spin-score/1',available_spin_count:36,expected_spin_estimates:36,angular_velocity_rmse_rad_s:.004}),'有效角速度估计 36/36；角速度 RMSE 0.00400 rad/s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-rendered-spin-score/1',available_spin_count:0,expected_spin_estimates:36,angular_velocity_rmse_rad_s:null}),'有效角速度估计 0/36');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-visual-approach-score/1',position_error_m:.025,optical_relative_speed_m_s:.001,dwell_s:5}),'位置误差 0.0250 m；相对速度 0.0010 m/s；连续保持 5.00 s');
  assert.equal(benchmarkMetric({schema_version:'spacesim-autonomy-visual-approach-score/1',position_error_m:null,optical_relative_speed_m_s:null,dwell_s:0}),'—');
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
