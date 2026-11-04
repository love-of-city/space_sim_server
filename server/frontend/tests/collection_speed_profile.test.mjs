import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {collectionLayoutError} from '../pages/collection_layout_status.js';

const source=readFileSync(new URL('../pages/collection.js',import.meta.url),'utf8')
  .replace(/^import .*;\r?\n/gm,'').replace('export function createCollectionPage','function createCollectionPage');

async function setup(){
  const nodes=new Map(), requests=[];
  const node=()=>({value:'',disabled:false,hidden:false,checked:false,options:[],
    replaceChildren(){},append(){},setAttribute(){}});
  const $=id=>{
    if(!nodes.has(id))nodes.set(id,node());
    return nodes.get(id);
  };
  $('task').value='drawer_cycle';
  $('threaded-speed-profile').value='fast_collection';
  const context=vm.createContext({template:'',mountView:()=>$,collectionLayoutError,
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
  vm.runInContext(source+'\nthis.createCollectionPage=createCollectionPage;',context);
  const page=context.createCollectionPage({querySelectorAll:()=>[]},{api:async(path,options)=>{
    if(options){requests.push({path,options});return {};}
    if(path==='/api/collections/catalog')return {
      tasks:['drawer_cycle','aviation_bolt_insert','locking_rod_cycle','aviation_plug_insert']
        .map(id=>({id,label:id})),
      randomization_options:[],rates:{},cameras:{},baseline_joint_deg:[],max_workers:4,
    };
    return {jobs:[]};
  }});
  await page.activate();
  return {$,requests,context};
}

test('threaded speed control follows the selected task and never affects drawer speed',async()=>{
  const h=await setup();
  for(const [task,threaded,drawer] of [
    ['aviation_bolt_insert',true,false],['locking_rod_cycle',true,false],
    ['aviation_plug_insert',false,false],['drawer_cycle',false,true],
  ]){
    h.$('task').value=task;h.$('task').onchange();
    assert.equal(h.$('threaded-speed-field').hidden,!threaded);
    assert.equal(h.$('threaded-speed-profile').disabled,!threaded);
    assert.equal(h.$('linear-speed').disabled,!drawer);
  }
});

test('submission preserves either threaded profile and drops stale values after a task switch',async()=>{
  const h=await setup();
  for(const [task,profile,expected] of [
    ['aviation_bolt_insert','fast_collection','fast_collection'],
    ['locking_rod_cycle','default','default'],
    ['aviation_plug_insert','default',undefined],
    ['drawer_cycle','fast_collection',undefined],
  ]){
    h.$('task').value=task;h.$('threaded-speed-profile').value=profile;
    h.$('task').onchange();
    await h.$('collection-form').onsubmit({preventDefault(){},currentTarget:{}});
    const request=h.requests.at(-1);
    assert.equal(request.path,'/api/collections');
    assert.equal(JSON.parse(request.options.body).threaded_speed_profile,expected);
  }
});
