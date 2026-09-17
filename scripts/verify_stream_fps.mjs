// Opt-in, read-only video test against a DEDICATED backend/UE instance.
// Selects each camera but never starts/resets a scene or sends robot commands.
// Credentials stay in environment variables; no production URL is assumed.
import { spawn } from 'node:child_process';
import { mkdir, writeFile } from 'node:fs/promises';
import net from 'node:net';
import path from 'node:path';
import assert from 'node:assert/strict';

const url = process.env.BSK_STREAM_TEST_URL;
const user = process.env.BSK_STREAM_TEST_USER;
const password = process.env.BSK_STREAM_TEST_PASSWORD;
if (!url || !user || !password) throw new Error('Set BSK_STREAM_TEST_URL/USER/PASSWORD for an isolated instance.');
const seconds = Number(process.env.BSK_STREAM_TEST_SECONDS || 60);
const warmup = Number(process.env.BSK_STREAM_TEST_WARMUP || 15);
const cycles = Number(process.env.BSK_STREAM_TEST_CYCLES || 1);
if (!Number.isInteger(cycles) || cycles < 1 || cycles > 20) throw new Error('Invalid cycle count.');
if (!Number.isFinite(seconds) || seconds < 10 || !Number.isFinite(warmup) || warmup < 0) throw new Error('Invalid test duration.');
const output = path.resolve(process.env.BSK_STREAM_TEST_OUTPUT || 'run/stream-fps');
await mkdir(output, { recursive: true });
const server = net.createServer();
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
const debugPort = server.address().port;
await new Promise(resolve => server.close(resolve));
const edge = process.env.BSK_STREAM_TEST_EDGE || 'C:\\Program Files (x86)\\Microsoft\\Edge\\Application\\msedge.exe';
const sleep = ms => new Promise(resolve => setTimeout(resolve, ms));
const browser = spawn(edge, ['--headless=new', '--no-first-run', '--no-default-browser-check',
  `--user-data-dir=${path.join(output, `edge-${Date.now()}`)}`, `--remote-debugging-port=${debugPort}`,
  '--window-size=1600,1000', '--autoplay-policy=no-user-gesture-required',
  '--disable-background-timer-throttling', '--disable-backgrounding-occluded-windows', 'about:blank'],
  { windowsHide: true, stdio: 'ignore' });
// Do not disable GPU/vsync/frame-rate limits: measure the real browser path,
// rather than manufacturing a high "displayed FPS" counter for this test.
const result = { startedAt: new Date().toISOString(), passed: false, secondsPerStream: seconds,
  scope: 'Received/decoded FPS on the test client. Not a guarantee for a different network/display.', streams: [] };
let ws, call;
try {
  let pages;
  for (let i=0;i<150;i++) {
    try { pages = await (await fetch(`http://127.0.0.1:${debugPort}/json`)).json(); if (pages.some(p=>p.type==='page')) break; } catch {}
    await sleep(100);
  }
  assert.ok(pages?.some(p=>p.type==='page'), 'Browser debugging endpoint unavailable');
  ws = new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl);
  await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
  const pending = new Map(); let nextId=0;
  ws.onmessage = event => { const m=JSON.parse(event.data), p=pending.get(m.id); if(p){pending.delete(m.id);clearTimeout(p.timer);m.error?p.reject(new Error(JSON.stringify(m.error))):p.resolve(m.result);} };
  call = (method,params={}) => new Promise((resolve,reject)=>{const id=++nextId;const timer=setTimeout(()=>{pending.delete(id);reject(new Error(`CDP timeout: ${method}`));},20000);pending.set(id,{resolve,reject,timer});ws.send(JSON.stringify({id,method,params}));});
  const evaluate = async expression => { const r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true}); if(r.exceptionDetails)throw new Error(JSON.stringify(r.exceptionDetails));return r.result.value; };
  const installProbe = `
    window.__fpsTestConnections=[];
    const Native=window.RTCPeerConnection;
    window.RTCPeerConnection=new Proxy(Native,{construct(Target,args){const pc=new Target(...args);window.__fpsTestConnections.push(pc);return pc;}});
    window.__fpsTestSnapshot=async()=>{
      const pc=window.__fpsTestConnections.findLast(p=>p.connectionState==='connected');
      if(!pc)return null;
      const all=[...(await pc.getStats()).values()];
      const v=all.find(s=>s.type==='inbound-rtp'&&s.kind==='video');
      const pair=all.find(s=>s.type==='candidate-pair'&&s.state==='succeeded'&&s.nominated);
      const player=document.querySelector('#pixelStream video');
      const q=player?.getVideoPlaybackQuality();
      if(!v||!q)return null;
      return {...v,wallTime:performance.now(),displayed:q.totalVideoFrames-q.droppedVideoFrames,
        dropped:q.droppedVideoFrames,rtt:pair?.currentRoundTripTime,
        visibility:document.visibilityState,dimensions:[player.videoWidth,player.videoHeight]};
    };`;
  await call('Page.enable');
  await call('Page.navigate',{url});
  for(let i=0;i<100;i++){if(await evaluate("Boolean(document.getElementById('loginForm') && document.body.classList.contains('auth-required'))"))break;await sleep(100);}
  await evaluate(installProbe);
  await evaluate(`document.getElementById('loginUsername').value=${JSON.stringify(user)};document.getElementById('loginPassword').value=${JSON.stringify(password)};document.getElementById('loginForm').requestSubmit()`);
  let ready = false;
  for(let i=0;i<240;i++){ready=await evaluate("Boolean(window.__pixelStream && document.querySelector('#pixelStream video')?.readyState===4)");if(ready)break;await sleep(500);}
  if (!ready) {
    result.setup = await evaluate(`({authClass:document.body.className,loginError:document.getElementById('loginError')?.textContent,
      frameState:document.getElementById('frameState')?.textContent,hasPlayer:!!window.__pixelStream,
      peers:window.__fpsTestConnections.map(p=>({connection:p.connectionState,ice:p.iceConnectionState})),
      videos:[...document.querySelectorAll('video')].map(v=>({ready:v.readyState,w:v.videoWidth,h:v.videoHeight,paused:v.paused}))})`);
  }
  assert.ok(ready,'No live WebRTC video after login');
  result.userAgent = await evaluate('navigator.userAgent');
  const options=await evaluate("[...document.getElementById('streamSelector').options].map(o=>({id:o.value,label:o.textContent}))");
  assert.ok(options.length>0,'No selectable streams');
  const sequence = Array.from({length:cycles}, (_,index)=>options.map(option=>({...option,cycle:index+1}))).flat();
  for(const option of sequence){
    console.log(`Measuring ${option.id}: ${seconds}s after ${warmup}s warmup`);
    await evaluate(`document.getElementById('streamSelector').value=${JSON.stringify(option.id)};document.getElementById('streamSelector').dispatchEvent(new Event('change'))`);
    await sleep(warmup*1000);
    let prev=await evaluate('window.__fpsTestSnapshot()');
    const entry={...option,samples:[]};result.streams.push(entry);
    for(let i=0;i<seconds;i++){
      await sleep(1000);
      const now=await evaluate('window.__fpsTestSnapshot()');
      if(!now || !prev || now.id!==prev.id || now.timestamp<=prev.timestamp){entry.samples.push({connected:false});prev=now;continue;}
      const interval=(now.timestamp-prev.timestamp)/1000;
      const wall=(now.wallTime-prev.wallTime)/1000;
      entry.samples.push({connected:true,receivedFps:(now.framesReceived-prev.framesReceived)/interval,
        decodedFps:(now.framesDecoded-prev.framesDecoded)/interval,displayedFps:(now.displayed-prev.displayed)/wall,
        receivedDelta:now.framesReceived-prev.framesReceived,decodedDelta:now.framesDecoded-prev.framesDecoded,
        interval,freezeDelta:(now.freezeCount||0)-(prev.freezeCount||0),rtt:now.rtt,
        lossDelta:now.packetsLost-prev.packetsLost,bitrateMbps:(now.bytesReceived-prev.bytesReceived)*8/interval/1e6,
        width:now.frameWidth,height:now.frameHeight,visibility:now.visibility});
      prev=now;
    }
    const connected=entry.samples.filter(s=>s.connected);
    const describe=key=>{const values=connected.map(s=>s[key]).filter(Number.isFinite).sort((a,b)=>a-b);return values.length?{min:values[0],p05:values[Math.floor((values.length-1)*0.05)],mean:values.reduce((a,b)=>a+b,0)/values.length,max:values.at(-1)}:null;};
    entry.received=describe('receivedFps');entry.decoded=describe('decodedFps');entry.displayed=describe('displayedFps');
    const shot=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
    await writeFile(path.join(output,`${option.id.replace(/[^a-zA-Z0-9_-]/g,'_')}-cycle${option.cycle}.png`),Buffer.from(shot.data,'base64'));
    entry.passed=connected.length===entry.samples.length && entry.received?.min>=60 && entry.decoded?.min>=60;
    console.log(JSON.stringify({id:entry.id,received:entry.received,decoded:entry.decoded,displayed:entry.displayed,passed:entry.passed}));
    await writeFile(path.join(output,'fps-results.json'),JSON.stringify(result,null,2));
  }
  result.passed=result.streams.every(s=>s.passed);
} catch(error){result.error=String(error.stack||error);console.error(result.error);}
finally {
  result.finishedAt=new Date().toISOString();
  await writeFile(path.join(output,'fps-results.json'),JSON.stringify(result,null,2));
  if(call)try{await call('Browser.close');}catch{}
  ws?.close();
  if(browser.exitCode==null)browser.kill();
}
if(!result.passed)process.exitCode=1;
