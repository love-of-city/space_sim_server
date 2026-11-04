import test from 'node:test';
import assert from 'node:assert/strict';
import {resolvePreviewStreams} from '../preview_streams.js';

const configured = [{id: 'main', label: '主视口'},
  {id: 'arm-wrist', label: '腕部相机', aliases: ['flight-wrist']},
  {id: 'arm-overview', label: '总览相机', aliases: ['flight-overview']}];

test('flight discovery resolves both cameras without changing the main player', () => {
  const resolved = resolvePreviewStreams(configured, ['main', 'flight-wrist', 'flight-overview'], 'main');
  assert.deepEqual(resolved.streams.map(s => s.id), ['main', 'flight-wrist', 'flight-overview']);
  assert.equal(resolved.selectedId, 'main');
  assert.ok(resolved.streams.every(s => !s.disabled));
});

test('handoff retains the selected logical wrist camera across namespaces', () => {
  const resolved = resolvePreviewStreams(configured, ['main', 'arm-wrist', 'arm-overview'], 'flight-wrist');
  assert.equal(resolved.selectedId, 'arm-wrist');
  assert.equal(resolved.streams[1].label, '腕部相机');
});

test('unrelated streamers are never exposed and absent cameras are disabled', () => {
  const resolved = resolvePreviewStreams(configured, ['main', 'foreign-camera'], 'main');
  assert.equal(resolved.streams.length, 3);
  assert.ok(resolved.streams.slice(1).every(s => s.disabled));
  assert.ok(resolved.streams.every(s => s.id !== 'foreign-camera'));
});

test('reconnecting preserves a known alias until discovery is available', () => {
  assert.equal(resolvePreviewStreams(configured, null, 'flight-wrist').selectedId, 'flight-wrist');
  assert.equal(resolvePreviewStreams(configured, null, 'foreign-camera').selectedId, 'main');
});
