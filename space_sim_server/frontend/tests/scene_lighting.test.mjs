import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const startSource = source.slice(source.indexOf("async function startScene()"), source.indexOf("async function stopScene()"));
const applySource = source.slice(source.indexOf("function applySceneRuntime("), source.indexOf("async function readApiResponse("));
const catalogSource = source.slice(source.indexOf("async function loadSceneCatalog()"), source.indexOf("function applySceneRuntime("));
function setup(value = "1") {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {value:"", textContent:"", disabled:false, checked:false,
      classList: {toggle(){}}, focus(){this.focused=true;}, replaceChildren(){}});
    return nodes.get(id);
  };
  $("sceneSunlightIntensity").value = value;
  $("sceneSeed").value = "42";
  $("sceneTemplate").value = "spacecraft-arm-teleop";
  $("randomizationProfile").value = "none";
  const requests=[], messages=[];
  const state={stateRequestSequence:0,sceneDefaults:{simulation_rate:1,capture_rate_hz:30,ik_rate_hz:120}, currentUser:{role:"admin",user_id:"admin"}, scenePhase:"idle"};
  const ctx=vm.createContext({clearWristWrench() {},clearSimulationTelemetry() {}, ZERO_START_PROFILE: "teleop-zero-prepare-v1", armPreparation: {read: () => [0,-67.6,-86.6,143.2,-85.5,0], ready: () => true, tick: () => false, cancel() {}, setRuntime() {}},  initialJointCatalog: null, configureInitialJoints() {}, configureSceneSelection() {},
    initialJoints: { read: () => null, setRuntime() {} },$,state,console,Option:function(){}, scenePhaseLabels:{running:"运行中"},
    setMessage:m=>messages.push(m), updateOperationUI(){},updateEpisodeUI(){},connectPixelStreaming(){},exitOperationMode(){},refreshState:async()=>{},
    apiRequest:async (path, options={})=>{requests.push({path,options});return {ok:true,json:async()=>({templates:[],randomization_profiles:[],defaults:{}})};},
    readApiResponse:async()=>({instance_id:"s1",seed:42}),applySceneRuntime(){}
  });
  vm.runInContext(startSource,ctx);
  return {$,ctx,requests,messages,state};
}

test("scene start transmits sunlight multiplier including zero",async()=>{
  for(const value of ["0","0.5","1","2.5","12500","20000"]){
    const h=setup(value);await h.ctx.startScene();
    assert.equal(JSON.parse(h.requests[0].options.body).sunlight_intensity_scale,Number(value));
  }
});

test("invalid or empty sunlight prevents launch",async()=>{
  for(const value of [""," ","-1","20000.1","NaN","Infinity","oops"]){
    const h=setup(value);await h.ctx.startScene();
    assert.equal(h.requests.length,0);
    assert.match(h.messages[0],/0～20,000/);
    assert.equal(h.$("sceneSunlightIntensity").focused,true);
    assert.equal(h.$("startScene").disabled,false);
  }
});

test("active scene displays its persisted lighting and locks initialization",()=>{
  const h=setup("2");vm.runInContext(applySource,h.ctx);
  h.ctx.applySceneRuntime({phase:"running",active:true,instance:{instance_id:"s1",seed:42,
    randomization:{},environment:{lighting:{sunlight_intensity_scale:0}}}});
  assert.equal(h.$("sceneSunlightIntensity").value,"0");
  assert.equal(h.$("sceneSunlightIntensity").disabled,true);
  assert.equal(h.$("sceneInstanceSunlight").textContent,"0 倍");
  assert.equal(JSON.parse(h.$("sceneParameters").textContent).environment.lighting.sunlight_intensity_scale,0);
  h.ctx.applySceneRuntime({phase:"idle",active:false});
  assert.equal(h.$("sceneSunlightIntensity").disabled,false);
  h.ctx.applySceneRuntime({phase:"running",active:true,instance:{instance_id:"old",randomization:{}}});
  assert.equal(h.$("sceneSunlightIntensity").value,"1");
});

test("legacy catalog supplies default one without replacing zero",async()=>{
  const h=setup("2");vm.runInContext(catalogSource,h.ctx);await h.ctx.loadSceneCatalog();
  assert.equal(h.$("sceneSunlightIntensity").value,"1");
  h.ctx.apiRequest=async()=>({ok:true,json:async()=>({templates:[],randomization_profiles:[],defaults:{sunlight_intensity_scale:0}})});
  await h.ctx.loadSceneCatalog();assert.equal(h.$("sceneSunlightIntensity").value,"0");
});

test("a running process cannot enable controls using absent or previous-scene telemetry",()=>{
  const h=setup();vm.runInContext(applySource,h.ctx);
  const runtime={phase:"running",active:true,instance:{instance_id:"current",randomization:{}}};
  for(const [connected,id] of [[false,null],[false,"current"],[true,null],[true,"previous"]]){
    h.state.simulationConnected=connected;h.state.observationSceneId=id;
    h.ctx.applySceneRuntime(runtime);
    assert.equal(h.state.sceneReady,false);
    assert.equal(h.$("linearSpeed").disabled,true);
  }
  h.state.simulationConnected=true;h.state.observationSceneId="current";
  h.ctx.applySceneRuntime(runtime);
  assert.equal(h.state.sceneReady,true);
  assert.equal(h.$("linearSpeed").disabled,false);
  h.state.simulationResetting=true;h.ctx.applySceneRuntime(runtime);
  assert.equal(h.state.sceneReady,false);
});

test("wrist camera selection is submitted, restored and locked for an active scene",async()=>{
  for(const mode of ["rgb","fisheye","monochrome"]){
    const h=setup();h.$("sceneWristCameraMode").value=mode;
    await h.ctx.startScene();
    assert.equal(JSON.parse(h.requests[0].options.body).wrist_camera_mode,mode);
    vm.runInContext(applySource,h.ctx);
    h.ctx.applySceneRuntime({phase:"running",active:true,instance:{instance_id:"s1",randomization:{},wrist_camera_mode:mode}});
    assert.equal(h.$("sceneWristCameraMode").value,mode);
    assert.equal(h.$("sceneWristCameraMode").disabled,true);
    assert.equal(JSON.parse(h.$("sceneParameters").textContent).wrist_camera_mode,mode);
    h.ctx.applySceneRuntime({phase:"idle",active:false});
    assert.equal(h.$("sceneWristCameraMode").disabled,false);
  }
  const h=setup();vm.runInContext(applySource,h.ctx);
  h.ctx.applySceneRuntime({phase:"running",active:true,instance:{instance_id:"legacy",randomization:{}}});
  assert.equal(h.$("sceneWristCameraMode").value,"rgb");
});

test("overview randomization is submitted and restored without reselecting an active camera",async()=>{
  const h=setup();h.$("sceneRandomizeOverviewCamera").checked=true;
  await h.ctx.startScene();
  assert.equal(JSON.parse(h.requests[0].options.body).randomize_overview_camera,true);
  vm.runInContext(applySource,h.ctx);
  const camera={preset_id:"P4",label:"+Y 内侧安装点 · 30 cm",position_body_m:[-.10,.58,.30]};
  h.ctx.applySceneRuntime({phase:"running",active:true,instance:{instance_id:"s1",randomization:{},template_id:"sarm-task-box-20261012",randomize_overview_camera:true,overview_camera:camera}});
  assert.equal(h.$("sceneRandomizeOverviewCamera").checked,true);
  assert.equal(h.$("sceneRandomizeOverviewCamera").disabled,true);
  assert.equal(h.$("sceneInstanceOverviewCamera").textContent,"P4 · +Y 内侧安装点 · 30 cm");
  assert.deepEqual(JSON.parse(h.$("sceneParameters").textContent).overview_camera,camera);
  h.ctx.applySceneRuntime({phase:"idle",active:false});
  assert.equal(h.$("sceneRandomizeOverviewCamera").disabled,false);
});
