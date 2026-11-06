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
    const objective=spec.schema_version==='spacesim-autonomy-rendered-pose/1'
      ?`${spec.cameras.length===1?'单目':'双目'}真实图像；已知被动标志物，无目标遥测；位置 RMSE ≤ ${spec.objective.position_rmse_limit_m*1000} mm，姿态 RMSE ≤ ${(spec.objective.orientation_rmse_limit_rad*180/Math.PI).toFixed(2)}°；仅输出位姿估计`
      :spec.schema_version==='spacesim-autonomy-formation/1'
      ?`${spec.controlled_spacecraft} 颗星分别受控；所有成员位置误差 ≤ ${spec.objective.position_tolerance_m} m，连续保持 ${spec.objective.dwell_s} s；任意两星间距 ≥ ${spec.safety.minimum_pair_distance_m} m`
      :spec.objective.kind==='navigation_accuracy'
      ?`全程位置 RMSE ≤ ${spec.objective.position_rmse_limit_m} m、速度 RMSE ≤ ${spec.objective.velocity_rmse_limit_m_s} m/s；方法只输出估计，不控制推进器`
      :spec.objective.kind==='complete_signed_revolution'
        ?`在 T-R 平面完成一整圈绕飞；半径 ${spec.objective.radius_m} ± ${spec.objective.radial_tolerance_m} m，平面偏差 ≤ ${spec.objective.plane_tolerance_m} m`
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
