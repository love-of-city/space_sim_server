// Opt-in end-to-end regression against a DEDICATED running backend and UE.
// UE must use -BskCameraDiagnostics. Never point this at an operator's live scene:
// the test drives the main camera. See docs/FREE_CAMERA_INPUT.md.
// Node >=22; no Playwright install or synthetic DOM MouseEvent required.
import { spawn } from 'node:child_process';
import { mkdir, writeFile } from 'node:fs/promises';
import path from 'node:path';
import net from 'node:net';
import assert from 'node:assert/strict';

const url = process.env.BSK_CAMERA_TEST_URL;
const username = process.env.BSK_CAMERA_TEST_USER;
const password = process.env.BSK_CAMERA_TEST_PASSWORD;
if (!url || !username || !password) throw new Error('Set BSK_CAMERA_TEST_URL/USER/PASSWORD for an isolated test instance.');
const evidence = path.resolve(process.env.BSK_CAMERA_TEST_OUTPUT || 'run/free-camera-runtime');
const edge = process.env.BSK_CAMERA_TEST_EDGE || 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';
await mkdir(evidence, { recursive: true });
const sleep = ms => new Promise(r => setTimeout(r, ms));
const portServer = net.createServer();
await new Promise(r => portServer.listen(0, '127.0.0.1', r));
const debugPort = portServer.address().port;
await new Promise(r => portServer.close(r));
const profile = path.join(evidence, `edge-${Date.now()}`);
// Start on the actual origin. Navigating a headless about:blank target via CDP
// can yield WrongDocumentError on pointer-lock (not a production input failure).
const browser = spawn(edge, ['--headless=new', '--no-first-run', '--no-default-browser-check',
  `--user-data-dir=${profile}`, `--remote-debugging-port=${debugPort}`, '--window-size=1600,1000',
  '--autoplay-policy=no-user-gesture-required', '--disable-background-timer-throttling',
  '--disable-backgrounding-occluded-windows', url], { windowsHide: true, stdio: 'ignore' });
const result = { passed: false, snapshots: [], limitation: 'CDP generates trusted browser input but does not emulate physical RDP mouse packets.' };
let ws, call;
const multiply = ([x,y,z,w], [a,b,c,d]) => [w*a+x*d+y*c-z*b, w*b-x*c+y*d+z*a, w*c+x*b-y*a+z*d, w*d-x*a-y*b-z*c];
const normalize = q => { const n = Math.hypot(...q); return q.map(x => x/n); };
const angularError = (a,b) => 2*Math.acos(Math.min(1, Math.abs(normalize(a).reduce((n,x,i) => n+x*normalize(b)[i],0))))*180/Math.PI;
try {
  let pages;
  for (let i=0; i<100; i++) {
    try { pages = await (await fetch(`http://127.0.0.1:${debugPort}/json`)).json(); if (pages.some(p=>p.type==='page')) break; } catch {}
    await sleep(100);
  }
  assert.ok(pages?.some(p=>p.type==='page'), 'browser must expose a page');
  ws = new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((r,j)=>{ws.onopen=r;ws.onerror=j;});
  const pending = new Map(); let id=0;
  ws.onmessage = event => { const m=JSON.parse(event.data), p=pending.get(m.id); if (p) {pending.delete(m.id);m.error?p.reject(new Error(JSON.stringify(m.error))):p.resolve(m.result);} };
  call = (method, params={}) => new Promise((resolve,reject)=>{pending.set(++id,{resolve,reject});ws.send(JSON.stringify({id,method,params}));});
  const evaluate = async expression => {
    const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true,userGesture:true});
    if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails)); return r.result.value;
  };
  for(let i=0;i<100;i++){if(await evaluate("Boolean(document.getElementById('loginForm') && document.body.classList.contains('auth-required'))"))break;await sleep(100);}
  await evaluate(`document.getElementById('loginUsername').value=${JSON.stringify(username)};document.getElementById('loginPassword').value=${JSON.stringify(password)};document.getElementById('loginForm').requestSubmit()`);
  let ready=false;
  for(let i=0;i<180;i++){
    ready=await evaluate("Boolean(window.__pixelStream && document.querySelector('video')?.videoWidth && document.querySelector('video').readyState===4)");
    if(ready)break;await sleep(500);
  }
  assert.ok(ready,'live decoded WebRTC video required');
  await evaluate(`
    window.cameraProbe={packets:[],responses:[],locks:[],attempts:[],moves:[]};
    const stream=window.__pixelStream, emit=stream.emitCommand.bind(stream);
    stream.emitCommand=command=>{if(command.BskCameraInput)cameraProbe.packets.push(JSON.parse(command.BskCameraInput));return emit(command);};
    stream.addResponseEventListener('camera-runtime-test',s=>{try{const d=JSON.parse(s);if(d.type==='BskCameraDiagnostics')cameraProbe.responses.push(d);}catch{}});
    const lock=HTMLElement.prototype.requestPointerLock;
    HTMLElement.prototype.requestPointerLock=function(options){cameraProbe.attempts.push(options||null);return lock.call(this,options);};
    document.addEventListener('pointerlockchange',()=>cameraProbe.locks.push(document.pointerLockElement?.id||null));
    document.addEventListener('mousemove',e=>cameraProbe.moves.push({dx:e.movementX,dy:e.movementY,trusted:e.isTrusted}));
    document.getElementById('pixelStream').focus();
  `);
  const press=async(key,code,vk)=>{
    await call('Input.dispatchKeyEvent',{type:'keyDown',key,code,windowsVirtualKeyCode:vk});
    await call('Input.dispatchKeyEvent',{type:'keyUp',key,code,windowsVirtualKeyCode:vk});
  };
  await press('c','KeyC',67);await sleep(150);
  assert.equal(await evaluate('document.pointerLockElement?.id'),'pixelStream','C must acquire real browser pointer-lock');
  assert.deepEqual(await evaluate('cameraProbe.attempts'),[{unadjustedMovement:true}],'prefer raw unaccelerated pointer-lock');
  assert.equal(await evaluate('__freeCameraDiagnostics().inputMode'),'raw');
  let expected, baseline, baselineDx=0, baselineDy=0, usedPackets=0;
  const snapshot=async(label,verify=true)=>{
    // Wait for application timer -> SDK -> WebRTC -> game frame -> POV.
    await sleep(100);
    const n=await evaluate('cameraProbe.responses.length');
    assert.equal(await evaluate("__pixelStream.emitCommand({BskCameraProbe:'state'})"),true);
    for(let i=0;i<80;i++){if(await evaluate('cameraProbe.responses.length')>n)break;await sleep(25);}
    assert.ok(await evaluate('cameraProbe.responses.length')>n,'UE must respond; launch with -BskCameraDiagnostics');
    const s=await evaluate(`({ue:cameraProbe.responses.at(-1),packets:cameraProbe.packets,locked:document.pointerLockElement?.id||null,title:document.getElementById('operationHintTitle').textContent,frames:document.querySelector('video').getVideoPlaybackQuality().totalVideoFrames})`);
    const dx=s.packets.reduce((n,p)=>n+p.look_dx,0),dy=s.packets.reduce((n,p)=>n+p.look_dy,0);
    if(!baseline){baseline=s.ue;baselineDx=dx;baselineDy=dy;expected=baseline.orientation;usedPackets=s.packets.length;}
    const saved={label,...s,packetCount:s.packets.length,input:await evaluate('__freeCameraDiagnostics()')};delete saved.packets;
    result.snapshots.push(saved);
    if(verify){
      assert.equal(s.locked,'pixelStream'); assert.match(s.title,/鼠标已锁定/);
      assert.equal(s.ue.free,true);assert.equal(s.ue.remote,true);
      for(const p of s.packets.slice(usedPackets)){
        const size=Math.hypot(p.look_dx,p.look_dy);if(!size)continue;
        const a=size*0.12*Math.PI/180/2,k=Math.sin(a)/size;
        expected=normalize(multiply(expected,[0,p.look_dy*k,p.look_dx*k,Math.cos(a)]));
      }
      assert.equal(s.ue.look_dx-baseline.look_dx,dx-baselineDx,'UE receives full horizontal displacement');
      assert.equal(s.ue.look_dy-baseline.look_dy,dy-baselineDy,'UE receives full vertical displacement');
      for(const field of ['orientation','actor','camera','pov']) assert.ok(angularError(expected,s.ue[field])<0.001,`${label}: ${field} must follow full rotation, no clamp/overwrite`);
    }
    usedPackets=s.packets.length;
    console.log(label,JSON.stringify({locked:s.locked,dx:s.ue.look_dx,dy:s.ue.look_dy,frames:s.frames}));return s;
  };
  await snapshot('initial');
  let x=300,y=300;
  await call('Input.dispatchMouseEvent',{type:'mouseMoved',x,y});await sleep(40);
  const beforeWarp=await snapshot('before-injected-warp');
  // Trusted browser events with discontinuities shaped like a cursor warp.
  // These deliberately injected deltas are NOT a recording of physical RDP input.
  x+=1200;y-=800;
  await call('Input.dispatchMouseEvent',{type:'mouseMoved',x,y});
  const afterWarp=await snapshot('injected-warp-discarded');
  assert.equal(afterWarp.ue.look_dx,beforeWarp.ue.look_dx);
  assert.equal(afterWarp.ue.look_dy,beforeWarp.ue.look_dy);
  assert.ok(await evaluate("__freeCameraDiagnostics().recentLook.some(s=>s.reason==='discontinuity' && Math.abs(s.dx)>=1000)"));
  x-=1200;y+=800;
  await call('Input.dispatchMouseEvent',{type:'mouseMoved',x,y});
  const afterReverse=await snapshot('reverse-warp-discarded');
  assert.equal(afterReverse.ue.look_dx,beforeWarp.ue.look_dx);
  assert.equal(afterReverse.ue.look_dy,beforeWarp.ue.look_dy);
  // +/- 720 degrees about each local axis; trusted events cross the viewport
  // coordinates many times. No DOM dispatchEvent or direct UE angle assignment.
  for(const [label,dx,dy] of [['yaw+720',100,0],['yaw-720',-100,0],['pitch+720',0,100],['pitch-720',0,-100]]){
    for(let i=1;i<=60;i++){
      x+=dx;y+=dy;await call('Input.dispatchMouseEvent',{type:'mouseMoved',x,y});await sleep(18);
      if(i%10===0)await snapshot(`${label}/${i}`);
    }
  }
  assert.equal(await evaluate('cameraProbe.moves.every(m=>m.trusted)'),true);
  const before=await snapshot('movement-before');
  await call('Input.dispatchKeyEvent',{type:'keyDown',key:'w',code:'KeyW',windowsVirtualKeyCode:87});await sleep(150);
  await call('Input.dispatchKeyEvent',{type:'keyUp',key:'w',code:'KeyW',windowsVirtualKeyCode:87});
  const after=await snapshot('movement-after');assert.notEqual(after.ue.location,before.ue.location,'W must translate the rendered camera');
  await press('Home','Home',36);await sleep(100);
  const exit=await snapshot('exit',false);assert.equal(exit.ue.free,false);assert.equal(exit.locked,null);
  const screenshot=await call('Page.captureScreenshot',{format:'png'});
  await writeFile(path.join(evidence,'browser.png'),Buffer.from(screenshot.data,'base64'));
  assert.ok(exit.frames>result.snapshots[0].frames+100,'stream continues presenting frames throughout the test');
  result.browser=await call('Browser.getVersion');result.passed=true;
} catch(error) {result.error=String(error.stack||error);throw error;}
finally {
  await writeFile(path.join(evidence,'result.json'),JSON.stringify(result,null,2));
  if(ws?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch{}ws.close();}
  browser.kill();
}
