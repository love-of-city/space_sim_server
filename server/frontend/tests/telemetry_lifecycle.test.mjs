import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';

const source = readFileSync(new URL('../app.js', import.meta.url), 'utf8');
const functions = ['clearWristWrench', 'clearSimulationTelemetry', 'refreshState'].map(name => {
  const match = source.match(new RegExp(`^(?:async )?function ${name}\\([^]*?^}`, 'm'));
  assert.ok(match, name);
  return match[0];
}).join('\n');
const fields = ['toolPosition', 'actionSequence', 'jacobianRank', 'ikSolver', 'wheelSpeeds', 'wheelTorques', 'vx', 'grip'];

function harness(data, fail = false) {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {textContent: 'previous-scene-value', className: 'warning'});
    return nodes.get(id);
  };
  const state = {stateRequestSequence: 0, activePage: 'operate', sceneReady: true, lastAckAt: 123};
  const sceneMissions = [];
  const context = vm.createContext({$, state,
    apiRequest: async () => {if (fail) throw new Error('disconnected'); return {ok: true, json: async () => data};},
    setOnline() {}, setMessage() {}, updateOperationUI() {},
    applySceneRuntime(runtime, mission) {state.sceneReady = runtime.ready; sceneMissions.push(mission);},
    jointAngles: {clear() {}}, missionPanel: {render(mission) {state.mission = mission;}, failed() {}},
  });
  vm.runInContext(functions, context);
  return {$, state, sceneMissions, refresh: () => context.refreshState()};
}

test('flight refresh clears previous scene values and displays only live flight telemetry', async () => {
  const h = harness({simulation: {connected: false}, scene_runtime: {ready: false},
    mission: {stage: 'flight', telemetry: {connected: true, sim_time_s: 123.5,
      state: {bodies: [{body_id: 'rendezvous/chaser', angular_velocity_rad_s: [0, 0, .001]}]}}}});
  await h.refresh();
  for (const id of fields) assert.equal(h.$(id).textContent, '—', id);
  assert.equal(h.$('simTime').textContent, '123.500 s');
  assert.equal(h.$('bodyRate').textContent, '0.0000, 0.0000, 0.0010 rad/s');
  assert.equal(h.state.lastAckAt, 0);
  assert.equal(h.sceneMissions[0].stage, 'flight');
  assert.equal(h.sceneMissions[0].telemetry.sim_time_s, 123.5);
});

test('disconnected and resetting scenes do not retain old measured state', async () => {
  for (const simulation of [{connected: false}, {connected: true, resetting: true}]) {
    const h = harness({simulation, scene_runtime: {ready: true}, mission: {stage: 'teleop'}});
    await h.refresh();
    for (const id of [...fields, 'simTime', 'bodyRate']) assert.equal(h.$(id).textContent, '—', id);
  }
});

test('live teleoperation retains current observation values', async () => {
  const h = harness({simulation: {connected: true}, scene_runtime: {ready: true}, mission: {stage: 'teleop'}});
  await h.refresh();
  for (const id of fields) assert.equal(h.$(id).textContent, 'previous-scene-value', id);
  assert.equal(h.state.lastAckAt, 123);
});

test('failed state reads invalidate telemetry without retaining a previous source', async () => {
  const h = harness(null, true);
  await h.refresh();
  for (const id of [...fields, 'simTime', 'bodyRate']) assert.equal(h.$(id).textContent, '—', id);
  assert.equal(h.state.stateReadPending, false);
});
