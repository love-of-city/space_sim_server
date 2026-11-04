import test from 'node:test';
import assert from 'node:assert/strict';
import {WorkLight} from '../work_light.js';

test('light state changes only after renderer acknowledgement; stale state disables toggle', () => {
  let time = 0;
  const sent = [];
  const light = new WorkLight({send: command => { sent.push(command); return true; }, now: () => time});
  assert.equal(light.toggle(), false);
  light.receive(JSON.stringify({type:'bsk_work_light', supported:true, enabled:true}));
  assert.equal(light.toggle(), true);
  assert.equal(light.enabled, true);
  assert.deepEqual(JSON.parse(sent[0].BskWorkLight), {enabled:false});
  light.receive(JSON.stringify({type:'bsk_work_light', supported:true, enabled:false}));
  assert.equal(light.enabled, false);
  time = 6000;
  assert.equal(light.toggle(), false);
});

test('unsupported scenes and malformed replies cannot enable work light', () => {
  const light = new WorkLight({send: () => true});
  for (const reply of ['bad', '{}', '{"type":"bsk_work_light","supported":true,"enabled":"false"}']) light.receive(reply);
  assert.equal(light.available, false);
  light.receive('{"type":"bsk_work_light","supported":false,"enabled":false}');
  assert.equal(light.toggle(), false);
});
