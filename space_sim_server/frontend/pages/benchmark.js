import template from './benchmark.html?raw';
import {mountView,element} from '../ui.js';
import {benchmarkTerminal,methodLabels,renderBenchmarkJob} from './benchmark_view.js';

export function createBenchmarkPanel(root,{api}){
  const $=mountView(root,template);
  let catalog=null,active=false,epoch=0,timer=null,busy=false,refreshing=false,jobs=[];
  function notice(message='',error=true){$('notice').textContent=message;$('notice').hidden=!message;$('notice').classList.toggle('error',error);}
  function syncPlan(changeMethod=false){
    const task=catalog?.configurations.find(item=>item.id===$('configuration').value);if(!task)return;
    if(changeMethod){$('method').replaceChildren(...task.methods.map(id=>new Option(methodLabels[id]||id,id)));$('method').value=task.default_method;}
    const spec=task.specification,seeds=spec.seeds[$('split').value],previous=$('seed').value;
    $('seed').replaceChildren(...seeds.map(seed=>new Option(String(seed),String(seed))));if(seeds.includes(Number(previous)))$('seed').value=previous;
    const single=$('selection').value==='single';$('seed-field').hidden=!single;
    $('plan').textContent=`计划 ${single?1:seeds.length} 个样本；种子 ${single?$('seed').value:seeds.join('、')}。每个样本只尝试一次，停止后可继续尚未启动的样本。`;
    const objective=spec.schema_version==='spacesim-autonomy-wheel-recovery/1'
      ?`单飞轮发生力矩衰减；依据带噪力矩遥测在异常后 ${spec.objective.diagnosis_deadline_after_onset_s} s 内完成诊断，恢复指定姿态并连续保持 ${spec.objective.dwell_s} s`
      :spec.schema_version==='spacesim-autonomy-visual-approach/1'
      ?`双目真实图像闭环控制；已知被动标志物，无目标遥测；接近后位置误差 ≤ ${spec.objective.position_tolerance_m} m、相对速度 ≤ ${spec.objective.optical_relative_speed_tolerance_m_s} m/s，连续保持 ${spec.objective.dwell_s} s`
      :spec.schema_version==='spacesim-autonomy-rendered-spin/1'
      ?`双目真实图像与自身陀螺；已知被动标志物，无目标遥测；观测 ${(spec.objective.spin_warmup_ns/1e9).toFixed(1)} s 后连续评估目标角速度，RMSE ≤ ${spec.objective.angular_velocity_rmse_limit_rad_s} rad/s`
      :spec.schema_version==='spacesim-autonomy-rendered-pose/1'
      ?`${spec.cameras.length===1?'单目':'双目'}真实图像；已知被动标志物，无目标遥测；位置 RMSE ≤ ${spec.objective.position_rmse_limit_m*1000} mm，姿态 RMSE ≤ ${(spec.objective.orientation_rmse_limit_rad*180/Math.PI).toFixed(2)}°；仅输出位姿估计`
      :spec.objective.kind==='cooperative_rendezvous'
      ?`双星主动交会；实际缩短距离至少 ${spec.objective.minimum_range_reduction_m} m，每颗星推力积分 ≥ ${spec.objective.minimum_per_spacecraft_impulse_n_s} N·s；双方连续同时机动 ${spec.objective.simultaneous_maneuver_dwell_s} s，到达后保持 ${spec.objective.dwell_s} s`
      :spec.schema_version==='spacesim-autonomy-formation/1'
      ?`${spec.controlled_spacecraft} 颗星分别受控；所有成员位置误差 ≤ ${spec.objective.position_tolerance_m} m，连续保持 ${spec.objective.dwell_s} s；任意两星间距 ≥ ${spec.safety.minimum_pair_distance_m} m`
      :spec.objective.kind==='navigation_outage_recovery'
      ?`GNSS 解中断 ${spec.fault.minimum_duration_s}–${spec.fault.maximum_duration_s} s；中断期间持续估计，位置误差 ≤ ${spec.objective.outage_maximum_position_error_m} m；信号恢复后 ${spec.objective.recovery_deadline_after_return_s} s 内确认连续 ${spec.objective.recovered_dwell_s} s 精度恢复`
      :spec.objective.kind==='navigation_accuracy'
      ?`全程位置 RMSE ≤ ${spec.objective.position_rmse_limit_m} m、速度 RMSE ≤ ${spec.objective.velocity_rmse_limit_m_s} m/s；方法只输出估计，不控制推进器`
      :spec.objective.kind==='continuous_low_thrust_impulse'
      ?`推力不超过 ${spec.safety.maximum_force_n} N；实际推力 ≥ ${spec.objective.minimum_active_force_n} N 连续 ${spec.objective.minimum_continuous_thrust_s} s，完成规定推力积分并使半长轴增加至少 ${spec.objective.minimum_semimajor_axis_increase_m} m，末段无推力保持 ${spec.objective.dwell_s} s`
      :spec.objective.kind==='orbital_plane_adjustment'
      ?`目标轨道倾角 ${spec.objective.inclination_deg}°、升交点赤经 ${spec.objective.raan_deg}°；实际轨道面误差 ≤ ${(spec.objective.plane_tolerance_rad*180/Math.PI).toFixed(4)}°，满足目标轨道并无推力保持 ${spec.objective.dwell_s} s`
      :spec.objective.kind==='complete_signed_revolution'
        ?`在 T-R 平面完成一整圈绕飞；半径 ${spec.objective.radius_m} ± ${spec.objective.radial_tolerance_m} m，平面偏差 ≤ ${spec.objective.plane_tolerance_m} m`
        :spec.path_requirement?.kind==='abort_and_retreat'
          ?`接近中止：${spec.path_requirement.reversal_deadline_s} s 内确认持续 ${spec.path_requirement.reversal_confirmation_s} s 的向外运动；撤离至少 ${spec.path_requirement.minimum_retreat_distance_m} m，至目标保持点连续驻留 ${spec.objective.dwell_s} s`
          :`连续保持 ${spec.objective.dwell_s} s`;
    const corridor=spec.path_requirement?.kind==='axis_approach_corridor'?`；全程横向偏差 ≤ ${spec.path_requirement.maximum_cross_track_m} m`:'';
    $('description').textContent=`${task.source_id} · ${spec.spacecraft_count} 颗航天器 · ${spec.vision_mode==='V0'?'无视觉观测':'视觉参与'} · 时限 ${spec.duration_s} s · ${objective}${corridor}。该配置的目标与安全限制固定，结果可与同配置的方法比较。`;
    const dependencies=task.dependencies||catalog.dependencies;
    $('availability').textContent=dependencies.problems.join('；');
    $('submit').disabled=busy||!active||!dependencies.available||jobs.some(job=>!benchmarkTerminal.has(job.status));
  }
  async function refresh(){
    if(!active||refreshing)return;refreshing=true;const version=epoch;
    try{const result=await api('/api/benchmarks');if(!active||version!==epoch)return;jobs=result.jobs;
      $('jobs').replaceChildren(...(jobs.length?jobs.map(job=>renderBenchmarkJob(job,{api,notice,refresh,
        title:catalog?.configurations.find(item=>item.id===job.configuration)?.title||job.configuration})):[element('div','panel empty','尚无驾驶评测批次')]));
      $('refresh-state').textContent=`更新于 ${new Date().toLocaleTimeString()}`;syncPlan();
    }catch(error){if(active&&version===epoch)notice(error.message);}finally{refreshing=false;}
  }
  $('configuration').onchange=()=>syncPlan(true);
  for(const name of ['split','selection','seed'])$(name).onchange=()=>syncPlan();
  $('form').onsubmit=async event=>{
    event.preventDefault();if(busy)return;busy=true;notice();syncPlan();const version=epoch;
    const payload={configuration:$('configuration').value,method:$('method').value,split:$('split').value,selection:$('selection').value};
    if(payload.selection==='single')payload.seed=Number($('seed').value);
    try{await api('/api/benchmarks',{method:'POST',body:JSON.stringify(payload)});}
    catch(error){if(active&&version===epoch)notice(error.message);}
    finally{busy=false;if(active&&version===epoch)await refresh();}
  };
  return {
    async activate(){active=true;const version=++epoch;
      try{catalog=await api('/api/benchmarks/catalog');if(!active||version!==epoch)return;
        $('configuration').replaceChildren(...catalog.configurations.map(task=>new Option(`${task.source_id} · ${task.title}`,task.id)));
        const mapped=catalog.capability_matrix?.registered_requirements??new Set(catalog.configurations.map(task=>task.source_id)).size;
        $('coverage').textContent=`${catalog.configurations.length} 个可运行配置 · 对应 ${mapped} / ${catalog.requirements.requirements} 项来源需求`;
        $('availability').textContent=catalog.dependencies.problems.join('；');syncPlan(true);await refresh();
        if(active&&version===epoch){clearInterval(timer);timer=setInterval(refresh,2000);}
      }catch(error){if(active&&version===epoch)notice(error.message);}
    },
    deactivate(){active=false;epoch++;clearInterval(timer);timer=null;},
    reset(){this.deactivate();catalog=null;jobs=[];$('jobs').replaceChildren();notice();},
  };
}

export function createBenchmarkResults(root,{api}){
  const heading=element('div','page-heading'),title=element('h1','','驾驶评测结果'),back=element('a','button','返回评测管理');
  back.href='/collection?mode=benchmark';back.dataset.route='';heading.append(title,back);
  const message=element('p','notice'),content=element('div');message.hidden=true;root.replaceChildren(heading,message,content);
  let active=false,epoch=0,timer=null,key='',refreshing=false;
  const notice=(text,error=true)=>{message.textContent=text;message.hidden=!text;message.classList.toggle('error',error);};
  async function refresh(){if(!active||refreshing)return;refreshing=true;const version=epoch;
    try{const job=await api(`/api/benchmarks/${encodeURIComponent(key)}`);if(!active||epoch!==version)return;
      content.replaceChildren(renderBenchmarkJob(job,{api,notice,refresh,detail:true}));
    }catch(error){if(active&&epoch===version)notice(error.message);}finally{refreshing=false;}}
  return {
    async activate(id){active=true;key=id;epoch++;notice('');await refresh();if(active){clearInterval(timer);timer=setInterval(refresh,2000);}},
    deactivate(){active=false;epoch++;clearInterval(timer);timer=null;},
    reset(){this.deactivate();content.replaceChildren();notice('');},
  };
}
