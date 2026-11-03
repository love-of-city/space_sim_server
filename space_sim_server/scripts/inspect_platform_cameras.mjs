// Actual workbench/WebRTC screenshots. Does not change camera transforms.
import {spawn} from 'node:child_process';
import {mkdir,writeFile,copyFile} from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';

const output=path.resolve(process.env.SPACE_SIM_INSPECTION_OUTPUT || 'run/platform-camera-inspection');
await mkdir(output,{recursive:true});
const server=net.createServer();
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const port=server.address().port;
await new Promise(resolve=>server.close(resolve));
const browser=spawn(process.env.SPACE_SIM_EDGE || 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',[
  '--headless=new','--no-first-run','--no-default-browser-check',
  `--user-data-dir=${path.join(output,'browser-profile')}`,`--remote-debugging-port=${port}`,
  '--window-size=1600,1000','--autoplay-policy=no-user-gesture-required',
  '--disable-background-timer-throttling','--disable-backgrounding-occluded-windows','about:blank'
],{windowsHide:true,stdio:'ignore'});
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let ws, closeBrowser;
try {
  let pages;
  for(let i=0;i<150;i++) {
    try {pages=await (await fetch(`http://127.0.0.1:${port}/json`)).json();if(pages.some(p=>p.type==='page'))break;}catch{}
    await sleep(200);
  }
  ws=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
  const pending=new Map();let nextId=0;
  ws.onmessage=event=>{const m=JSON.parse(event.data),p=pending.get(m.id);if(p){pending.delete(m.id);clearTimeout(p.timer);m.error?p.reject(new Error(JSON.stringify(m.error))):p.resolve(m.result);}};
  const call=(method,params={},timeoutMs=30000)=>new Promise((resolve,reject)=>{const id=++nextId;const timer=setTimeout(()=>reject(new Error(`CDP timeout ${method}`)),timeoutMs);pending.set(id,{resolve,reject,timer});ws.send(JSON.stringify({id,method,params}));});
  // Edge can relaunch itself, leaving spawn().pid already exited. CDP closes
  // the actual browser and its children rather than only that launcher PID.
  closeBrowser=()=>call('Browser.close',{},3000);
  const evaluate=async expression=>{const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value;};
  await call('Page.enable');
  await call('Page.addScriptToEvaluateOnNewDocument',{source:`window.__inspectionPeers=[];const NativePeer=window.RTCPeerConnection;window.RTCPeerConnection=new Proxy(NativePeer,{construct(Target,args){const p=new Target(...args);window.__inspectionPeers.push(p);return p;}});`});
  await call('Page.navigate',{url:process.env.SPACE_SIM_INSPECTION_URL || 'http://127.0.0.1:8000'});
  for(let i=0;i<300;i++){if(await evaluate("typeof window.__gamepadDiagnostics==='function'"))break;await sleep(200);}
  await sleep(2000);
  await evaluate(`document.getElementById('loginUsername').value=${JSON.stringify(process.env.SPACE_SIM_ADMIN_USERNAME || 'admin')};document.getElementById('loginPassword').value=${JSON.stringify(process.env.SPACE_SIM_ADMIN_PASSWORD || 'ChangeMe123!')};document.getElementById('loginForm').requestSubmit()`);
  let ready=false;
  for(let i=0;i<600;i++){
    if(i>0 && i%15===0)await evaluate("(()=>{const form=document.getElementById('loginForm');if(form && !document.getElementById('loginScreen')?.hidden && document.getElementById('loginPassword').value)form.requestSubmit();})()");
    ready=await evaluate("!!window.__pixelStream && document.querySelector('#pixelStream video')?.readyState===4");
    if(ready)break;
    if(i%30===0)console.log('Waiting for actual platform video',i,'s');
    await sleep(1000);
  }
  const getState=()=>evaluate("fetch('/api/state',{cache:'no-store'}).then(r=>r.json())");
  if(!ready){await writeFile(path.join(output,'failed-state.json'),JSON.stringify(await getState(),null,2));throw new Error('No live platform WebRTC video');}
  let nativeReady=false;
  for(let i=0;i<600;i++){
    const s=await getState(), runtime=s.scene_runtime||{}, observation=s.simulation?.latest_observation;
    nativeReady=(runtime.phase==='running' && s.simulation?.connected && observation?.scene_instance_id===runtime.instance?.instance_id && Number(observation?.sim_time_ns)>1000000000)
      || (s.mission?.stage==='flight' && s.mission?.telemetry?.connected);
    if(nativeReady)break;
    if(i%30===0)console.log('Waiting for current native scene',i,'s');
    await sleep(1000);
  }
  if(!nativeReady)throw new Error('Native scene not ready');
  await sleep(15000);
  const readOptions=()=>evaluate("[...document.getElementById('streamSelector').options].map(o=>({id:o.value,label:o.textContent,disabled:o.disabled}))");
  let options=await readOptions();
  const cameraCount=(await getState()).scene_runtime?.instance?.template_id==='sarm-dual-capture26'?4:3;
  // A player opened before the manifest arrives can retain the initial main-only
  // discovery list. Reopen the page after native readiness, not the scene.
  if(options.filter(o=>!o.disabled).length<cameraCount){
    await call('Page.reload',{ignoreCache:true});
    for(let i=0;i<120;i++){
      if(await evaluate("!!document.getElementById('streamSelector') && !!window.__pixelStream && document.querySelector('#pixelStream video')?.readyState===4")){
        options=await readOptions();
        if(options.filter(o=>!o.disabled).length>=cameraCount)break;
      }
      await sleep(500);
    }
  }
  const report={source:'actual platform UI and decoded UE WebRTC frames',camera_transforms_changed:false,streams:[],state:await getState()};
  report.scene_menu=await evaluate("({scenes:[...document.getElementById('sceneTemplate').options].filter(o=>o.value.startsWith('library-family/')).map(o=>({id:o.value,label:o.textContent})),selected_scene:document.getElementById('sceneTemplate').value,tasks:[...document.getElementById('sceneTaskProfile').options].map(o=>({id:o.value,label:o.textContent})),selected_task:document.getElementById('sceneTaskProfile').value})");
  let previousId=await evaluate("document.getElementById('streamSelector').value");
  for(const option of options){
    if(option.disabled)continue;
    const beforePeers=await evaluate('window.__inspectionPeers.length');
    await evaluate(`document.getElementById('streamSelector').value=${JSON.stringify(option.id)};document.getElementById('streamSelector').dispatchEvent(new Event('change'))`);
    await sleep(6000);
    let frame;
    for(let i=0;i<40;i++){
      frame=await evaluate(`(()=>{const peers=window.__inspectionPeers;if(peers.at(-1)?.connectionState!=='connected'||(${JSON.stringify(option.id!==previousId)}&&peers.length<=${beforePeers}))return null;const v=document.querySelector('#pixelStream video');if(!v||v.readyState<3||!v.videoWidth)return null;const c=document.createElement('canvas');c.width=v.videoWidth;c.height=v.videoHeight;c.getContext('2d').drawImage(v,0,0);return {image:c.toDataURL('image/png'),width:v.videoWidth,height:v.videoHeight,time:v.currentTime,frames:v.getVideoPlaybackQuality().totalVideoFrames};})()`);
      if(frame?.frames>0)break;
      await sleep(500);
    }
    previousId=option.id;
    if(!frame?.frames){report.streams.push({...option,available:false});continue;}
    await sleep(1000);
    const later=await evaluate("document.querySelector('#pixelStream video')?.getVideoPlaybackQuality().totalVideoFrames || 0");
    if(later<=frame.frames){report.streams.push({...option,available:false,reason:'no fresh decoded frames'});continue;}
    const stem=option.id.replace(/[^a-zA-Z0-9_-]/g,'_');
    await writeFile(path.join(output,stem+'.png'),Buffer.from(frame.image.split(',')[1],'base64'));
    const shot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
    await writeFile(path.join(output,stem+'--workbench.png'),Buffer.from(shot.data,'base64'));
    report.streams.push({...option,available:true,image:stem+'.png',workbench:stem+'--workbench.png',width:frame.width,height:frame.height,decoded_frames:frame.frames,state:await getState()});
    console.log(JSON.stringify({stream:option.id,width:frame.width,height:frame.height,captured:true}));
  }
  const latest=await getState();
  const preheld=!!latest.scene_runtime?.instance?.preheld_capture;
  report.vision=preheld?{excluded:'already docked; docking stereo outside review scope'}:
    await evaluate("fetch('/api/vision/status',{cache:'no-store'}).then(r=>r.json())");
  report.vision_images=[];
  const currentSession=latest.simulation?.latest_observation?.render_session_id;
  report.docking_stereo_required=!preheld;
  if(!preheld && currentSession && report.vision.session_id===currentSession && !report.vision.stale && latest.scene_runtime?.instance?.template_id==='sarm-dual-capture26'){
    for(const eye of ['left','right']){
      const data=await evaluate(`fetch('/api/vision/image/${eye}',{cache:'no-store'}).then(async r=>{if(!r.ok)return null;const b=await r.blob();return new Promise(resolve=>{const f=new FileReader();f.onload=()=>resolve(f.result);f.readAsDataURL(b);});})`);
      if(data){const file=`vision-${eye}-verified.png`;await writeFile(path.join(output,file),Buffer.from(data.split(',')[1],'base64'));report.vision_images.push({eye,file,session_id:currentSession});}
    }
  }
  await writeFile(path.join(output,'inspection.json'),JSON.stringify(report,null,2));
  const adapter=latest.scene_runtime?.adapter_root;
  if(adapter)await copyFile(path.join(adapter,'Unreal/BskUnrealRenderer/Saved/Logs/BskUnrealRenderer.log'),path.join(output,'renderer.log'));
} finally {
  try {await closeBrowser?.();} catch {}
  ws?.close();
  browser.kill();
}
