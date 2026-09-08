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
  const state={sceneDefaults:{simulation_rate:1,capture_rate_hz:10,ik_rate_hz:100}, currentUser:{role:"admin",user_id:"admin"}, scenePhase:"idle"};
  const ctx=vm.createContext({$,state,console,Option:function(){}, scenePhaseLabels:{running:"运行中"},
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
