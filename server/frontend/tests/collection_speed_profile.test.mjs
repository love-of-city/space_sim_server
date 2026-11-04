import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {createTaskCatalog} from '../task_catalog.js';
import {collectionLayoutError} from '../pages/collection_layout_status.js';

const source=readFileSync(new URL('../pages/collection.js',import.meta.url),'utf8');

async function setup(lineEnding){
  const nodes=new Map(), requests=[];
  const node=()=>({value:'',disabled:false,hidden:false,checked:false,options:[],
    replaceChildren(){},append(){},setAttribute(){}});
  const $=id=>{
    if(!nodes.has(id))nodes.set(id,node());
    return nodes.get(id);
  };
  $('task').value='drawer_cycle';
  $('threaded-speed-profile').value='fast_collection';
  const context=vm.createContext({template:'',mountView:()=>$,collectionLayoutError,createTaskCatalog,
    FormData:class {
      *[Symbol.iterator](){
        yield ['task',$('task').value];
        // Include stale values deliberately: submission must protect against
        // a task switch even before a browser recomputes successful controls.
        yield ['threaded_speed_profile',$('threaded-speed-profile').value];
      }
    },
    createRandomizationControls:()=>({value:()=>({})}),
    document:{createElement:node},setInterval:()=>1,clearInterval(){},
  });
  // Exercise the checked-out source with either Git line-ending style. Imports
  // are supplied by the fixture because the HTML ?raw import belongs to Vite.
  const script=source.replace(/\r?\n/g,lineEnding)
    .replace(/^import .*;\r?\n/gm,'')
    .replace('export function createCollectionPage','function createCollectionPage');
  vm.runInContext(script+'\nthis.createCollectionPage=createCollectionPage;',context);
  const page=context.createCollectionPage({querySelectorAll:()=>[]},{api:async(path,options)=>{
    if(options){requests.push({path,options});return {};}
    if(path==='/api/collections/catalog')return {
      tasks:['drawer_cycle','drawer_open','drawer_close','aviation_bolt_insert','locking_rod_cycle','locking_rod_fold','locking_rod_restore','aviation_plug_insert']
        .map(id=>({id,label:id})),
      randomization_options:[],rates:{},cameras:{},baseline_joint_deg:[],max_workers:4,
    };
    return {jobs:[]};
  }});
  await page.activate();
  return {$,requests,context};
}

for(const [style,lineEnding] of [['LF','\n'],['CRLF','\r\n']]){
test(`threaded speed control follows the selected task and never affects drawer speed (${style})`,async()=>{
  const h=await setup(lineEnding);
  for(const [task,threaded,drawer] of [
    ['aviation_bolt_insert',true,false],['locking_rod_cycle',true,false],
    ['aviation_plug_insert',false,false],['drawer_cycle',false,true],
    ['drawer_open',false,true],['drawer_close',false,true],
    ['locking_rod_fold',true,false],['locking_rod_restore',true,false],
  ]){
    h.$('task').value=task;h.$('task').onchange();
    assert.equal(h.$('threaded-speed-field').hidden,!threaded);
    assert.equal(h.$('threaded-speed-profile').disabled,!threaded);
    assert.equal(h.$('linear-speed').disabled,!drawer);
  }
});

test(`submission preserves either threaded profile and drops stale values after a task switch (${style})`,async()=>{
  const h=await setup(lineEnding);
  for(const [task,profile,expected] of [
    ['aviation_bolt_insert','fast_collection','fast_collection'],
    ['locking_rod_cycle','default','default'],
    ['aviation_plug_insert','default',undefined],
    ['drawer_cycle','fast_collection',undefined],
    ['drawer_open','fast_collection',undefined],['drawer_close','default',undefined],
    ['locking_rod_fold','default','default'],['locking_rod_restore','fast_collection','fast_collection'],
  ]){
    h.$('task').value=task;h.$('threaded-speed-profile').value=profile;
    h.$('task').onchange();
    await h.$('collection-form').onsubmit({preventDefault(){},currentTarget:{}});
    const request=h.requests.at(-1);
    assert.equal(request.path,'/api/collections');
    assert.equal(JSON.parse(request.options.body).threaded_speed_profile,expected);
  }
});
}
