import test from 'node:test';
import assert from 'node:assert/strict';
import {missionPresentation, createMissionPanel} from '../mission_panel.js';

test('flight keeps robot controls closed and exposes stop only to its owner', () => {
  const run = {stage: 'flight', phase: 'flight', active: true, can_manage: true};
  assert.equal(missionPresentation(run).flightView, true);
  assert.equal(missionPresentation(run).canStart, false);
  assert.equal(missionPresentation(run).canStop, true);
  assert.equal(missionPresentation({...run, can_manage: false}).canStop, false);
});

test('an accepted manual flight offers an explicit owned handoff', () => {
  const ready = {stage: 'flight', phase: 'ready', active: false, can_manage: true, can_handoff: true};
  assert.equal(missionPresentation(ready).canHandoff, true);
  assert.equal(missionPresentation(ready).flightView, true);
  assert.equal(missionPresentation({...ready, can_manage: false}).canHandoff, false);
  assert.equal(missionPresentation({...ready, can_handoff: false}).showHandoff, false);
});

test('continuous start uses the shared receiver defaults in the existing mission request', async () => {
  const previousDocument = globalThis.document, previousOption = globalThis.Option;
  globalThis.document = {body: {classList: {toggle() {}}}};
  globalThis.Option = class { constructor(label, value) { this.value = value; } };
  const nodes = new Map(), requests = [];
  const ctx = {clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}};
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {value: id === 'missionMode' ? 'team-in-orbit' : '', textContent: '', checked: true,
      addEventListener() {}, replaceChildren() {}, querySelectorAll: () => [], getContext: () => ctx, width: 620, height: 200});
    return nodes.get(id);
  };
  const receiver = {absolute_position_sigma_m: .577, absolute_velocity_sigma_m_s: .001155,
    relative_position_sigma_m: .03, relative_velocity_sigma_m_s: .0015, sample_rate_hz: 10, delay_s: 0};
  try {
    const panel = createMissionPanel({$, state: {streamLive: false}, message() {}, reconnect() {}, configureScene() {},
      refresh: async () => {}, apiRequest: async (path, init) => {
        requests.push([path, init]);
        return {ok: true, status: 200, json: async () => path.endsWith('catalog') ? {
          modes: [{id: 'team-in-orbit'}], continuous_default: true,
          flight_defaults: {seed: 7, handoff_distance_m: 110, gnss_position_sigma_m: .577,
            gnss_velocity_sigma_m_s: .001155, initial_position_sigma_m: 5, orbit_acceleration_sigma_m_s2: 0,
            differential_gnss: receiver}} : {stage: 'flight', phase: 'starting', active: true}};
      }});
    await panel.load();
    panel.render({});
    $('missionGnssPosition').value = '.6';
    await panel.startFlight();
    const [path, init] = requests.at(-1), request = JSON.parse(init.body);
    assert.equal(path, '/api/mission/start');
    assert.equal(request.continuous, true);
    assert.equal(request.mode, 'team-in-orbit');
    assert.deepEqual(request.options.differential_gnss, {...receiver, absolute_position_sigma_m: .6});
    assert.equal(request.options.gnss_position_sigma_m, .6);
  } finally { globalThis.document = previousDocument; globalThis.Option = previousOption; }
});

test('the same mission returns to original teleoperation panels only after handoff', () => {
  const run = {stage: 'teleop', phase: 'running', active: true, mode: 'team-in-orbit', can_manage: true};
  assert.equal(missionPresentation(run, 'team-in-orbit').flightView, false);
  assert.equal(missionPresentation(run, 'team-in-orbit').canHandoff, false);
});

test('idle selection changes panels without starting a task; stopping stays disabled', () => {
  assert.equal(missionPresentation({}, 'team-orbit-raise').flightView, true);
  assert.equal(missionPresentation({}, 'teleop').flightView, false);
  assert.equal(missionPresentation({active: true, phase: 'stopping', can_manage: true}).canStop, false);
});

test('completed logical capture keeps its result view and does not offer distant-flight handoff', () => {
  const mission = {stage: 'flight', phase: 'completed', active: false, mode: 'near-field', can_manage: true, can_handoff: false};
  const view = missionPresentation(mission, 'near-field');
  assert.equal(view.flightView, true);
  assert.equal(view.canStart, true);
  assert.equal(view.canStop, false);
  assert.equal(view.showHandoff, false);
});

test('actual flight transition reconnects a waiting player once and updates the startup message', () => {
  const previousDocument = globalThis.document;
  globalThis.document = {body: {classList: {toggle() {}}}};
  const nodes = new Map(), messages = [];
  const ctx = {clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}};
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {value: id === 'missionMode' ? 'team-in-orbit' : '', textContent: '',
      addEventListener() {}, querySelectorAll: () => [], getContext: () => ctx, width: 620, height: 200});
    return nodes.get(id);
  };
  let reconnects = 0;
  const state = {missionMode: 'team-in-orbit', streamLive: false};
  try {
    const panel = createMissionPanel({$, state, message: text => messages.push(text), reconnect: () => reconnects++});
    const mission = {mission_id: 'test', mode: 'team-in-orbit', stage: 'flight', active: true, can_manage: true};
    panel.render({...mission, phase: 'starting'});
    assert.equal(reconnects, 0);
    panel.render({...mission, phase: 'flight'});
    panel.render({...mission, phase: 'flight'});
    assert.equal(reconnects, 1);
    assert.equal(messages.at(-1), '交会运行中');
    state.streamLive = true;
    panel.render({...mission, phase: 'starting'});
    panel.render({...mission, phase: 'flight'});
    assert.equal(reconnects, 1, 'A live stream must not be recreated');
  } finally { globalThis.document = previousDocument; }
});

test('login recovery waits for restored state before accepting a different mission mode', async () => {
  const previousDocument = globalThis.document, previousOption = globalThis.Option;
  globalThis.document = {body: {classList: {toggle() {}}}};
  globalThis.Option = class { constructor(label, value) { this.value = value; } };
  const nodes = new Map();
  const ctx = {clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}};
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {value: id === 'missionMode' ? 'teleop' : '', textContent: '',
      addEventListener() {}, replaceChildren() {}, querySelectorAll: () => [], getContext: () => ctx, width: 620, height: 200});
    return nodes.get(id);
  };
  try {
    const panel = createMissionPanel({$, state: {streamLive: false}, message() {}, reconnect() {}, configureScene() {},
      apiRequest: async () => ({ok: true, status: 200, json: async () => ({modes: [{id: 'teleop'}, {id: 'team-in-orbit'}]})})});
    await panel.load();
    assert.equal($('missionMode').disabled, true, 'catalog alone must not unlock a selection');
    const saved = {mission_id: 'saved', mode: 'team-in-orbit', stage: 'flight', phase: 'stopped', active: false};
    panel.render(saved);
    assert.equal($('missionMode').value, 'team-in-orbit');
    assert.equal($('missionMode').disabled, false);
    $('missionMode').value = 'teleop'; panel.configure(); panel.render(saved);
    assert.equal($('missionMode').value, 'teleop', 'subsequent polls preserve the user selection');
    panel.failed();
    assert.equal($('missionMode').disabled, true);
    panel.render(saved);
    assert.equal($('missionMode').disabled, false);
    assert.equal($('missionMode').value, 'teleop');
  } finally { globalThis.document = previousDocument; globalThis.Option = previousOption; }
});
