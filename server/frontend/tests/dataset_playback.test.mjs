import test from 'node:test';
import assert from 'node:assert/strict';
import {createDatasetPlayback} from '../pages/dataset_playback.js';

function harness(sync=true){
  const nodes=new Map(),frames=[],messages=[];
  const $=id=>{if(!nodes.has(id))nodes.set(id,{value:'1',setAttribute(){}});return nodes.get(id);};
  const video=()=>({readyState:1,currentTime:0,pause(){this.paused=true;},async play(){this.paused=false;}});
  const state={data:{times:[0,.02,.04,.06],frames:[11,12,13,14],stages:['grip','insert','insert','release'],
    count:4,fps:50,duration:.06,sync_ok:sync,start_sim_time_ns:1e9},index:0,playing:false,
    videos:[video(),video()],charts:[],range:[0,.06]};
  const playback=createDatasetPlayback({state,$,clock:String,fmt:String,stageNames:{insert:'插入'},notice:s=>messages.push(s),
    performance:{now:()=>1000},requestAnimationFrame:callback=>frames.push(callback)});
  return {state,$,frames,messages,...playback};
}

test('sample seeking uses the video frame grid while independent force samples retain their own clock',()=>{
  const h=harness();h.setIndex(2);
  assert.deepEqual(h.state.videos.map(v=>v.currentTime),[.04,.04]);
  assert.equal(h.$('stage').textContent,'阶段 插入');
  assert.equal(h.nearest(.014,[0,.005,.010,.015,.020]),3);
  h.setIndex(100);assert.equal(h.state.index,3);
});

test('synchronized playback follows the master video and only corrects material camera drift',async()=>{
  const h=harness();await h.play();
  h.state.videos[0].currentTime=.04;h.state.videos[1].currentTime=.2;
  h.frames.shift()(1010);
  assert.equal(h.state.index,2);assert.equal(h.state.videos[1].currentTime,.04);
  h.state.videos[0].currentTime=.06;h.frames.shift()(1020);
  assert.equal(h.state.playing,false);assert.ok(h.state.videos.every(v=>v.paused));
});

test('without synchronized video playback follows elapsed time and speed rather than camera timestamps',async()=>{
  const h=harness(false);h.$('speed').value='2';await h.play();
  h.frames.shift()(1020);assert.equal(h.state.index,2);
  h.pause();h.frames.shift()(1050);assert.equal(h.state.index,2);
});

test('a pending browser play request cannot resume sample advancement after the page pauses',async()=>{
  const h=harness();let resolve;
  h.state.videos[0].play=()=>new Promise(accept=>{resolve=accept;});
  const starting=h.play();h.pause();resolve();await starting;
  h.frames.shift()(5000);assert.equal(h.state.playing,false);assert.equal(h.state.index,0);
});

test('range validation uses one recorded frame and never alters the force sampling clock',()=>{
  const h=harness();let rebuilds=0;
  h.state.charts.push({rebuild(){rebuilds++;}});
  h.setRange(.01,.015);assert.equal(rebuilds,0);assert.equal(h.messages.length,1);
  h.setRange(0,.04);assert.deepEqual(h.state.range,[0,.04]);assert.equal(rebuilds,1);
});
