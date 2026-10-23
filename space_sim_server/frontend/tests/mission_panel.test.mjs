import test from 'node:test';
import assert from 'node:assert/strict';
import {missionPresentation, createMissionPanel, missionArtifactLinks} from '../mission_panel.js';

test('starting conditions and the full mission lifecycle keep the same workbench and gate inputs', async () => {
  const previousOption = globalThis.Option;
  globalThis.Option = class { constructor(label, value) { this.value = value; } };
  const nodes = new Map(), requests = [];
  const ctx = {clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}};
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {id, hidden: false, disabled: false, value: '', textContent: '',
      addEventListener() {}, replaceChildren() {}, querySelectorAll: () => [], getContext: () => ctx,
      width: 620, height: 200});
    return nodes.get(id);
  };
  const stable = ['controlHeading', 'controlDescription', 'teleopSettings', 'flightSettings',
    'flightTelemetry', 'sceneCollisionWarning', 'sceneInstanceSummary', 'sceneDetails', 'continueMission'];
  $('controlHeading').textContent = '团队末端六维遥操作';
  $('controlDescription').textContent = '团队操作说明';
  $('missionMode').value = 'teleop';
  const original = stable.map(id => [id, $(id).hidden, $(id).textContent]);
  const state = {streamLive: true};
  const modes = ['teleop', 'team-in-orbit', 'team-orbit-raise'];
  const panel = createMissionPanel({$, state, message() {}, reconnect() {}, configureScene() {},
    apiRequest: async path => {
      requests.push(path);
      return {ok: true, status: 200, json: async () => ({modes: modes.map(id => ({id})), continuous_default: true})};
    }});
  const assertStable = () => assert.deepEqual(stable.map(id => [id, $(id).hidden, $(id).textContent]), original);
  try {
    await panel.load(); panel.render({});
    for (const mode of modes) {
      $('missionMode').value = mode; panel.configure();
      assertStable();
      assert.equal($('teleopSettings').disabled, mode !== 'teleop');
      assert.equal($('flightSettings').disabled, mode === 'teleop');
      assert.equal($('startScene').disabled, false);
      assert.equal($('stopScene').disabled, true);
      assert.equal($('continueMission').disabled, true);
    }
    assert.deepEqual(requests, ['/api/mission/catalog'], 'selection must not start or stop a task');
    const mission = {mission_id:'continuous', mode:'team-in-orbit', can_manage:true};
    for (const [stage, phase, flight_phase] of [['flight','starting','far'], ['flight','flight','far'],
      ['flight','transitioning','near'], ['flight','flight','near'], ['flight','handoff','near'], ['teleop','running','near']]) {
      panel.render({...mission, stage, phase, flight_phase, active:true});
      assertStable();
      assert.equal($('missionMode').disabled, true);
      assert.equal($('flightSettings').disabled, true);
      assert.equal($('teleopSettings').disabled, true, 'inherited state must not expose ignored initial inputs');
      assert.equal($('stopScene').disabled, false);
      assert.equal($('continueMission').disabled, true);
    }
    panel.render({...mission, stage:'flight', phase:'ready', active:false, can_handoff:true});
    assert.equal($('continueMission').disabled, false);
    panel.render({...mission, stage:'flight', phase:'ready', active:false, can_handoff:true, can_manage:false});
    assert.equal($('continueMission').disabled, true);
    panel.failed(); assertStable();
    assert.equal($('missionMode').disabled, true);
    assert.equal($('flightSettings').disabled, true);
    for (const phase of ['failed', 'stopped']) {
      panel.render({...mission, stage:'flight', phase, active:false}); assertStable();
      assert.equal($('missionMode').disabled, false);
    }
    state.sceneTransitionPending = true; panel.configure();
    assert.equal($('missionMode').disabled, true);
    assert.equal($('startScene').disabled, true);
    panel.reset(); assertStable();
  } finally { globalThis.Option = previousOption; }
});

test('completed stages keep separate downloads after entering the original operator scene', () => {
  const links = missionArtifactLinks({mission_id:'mission-123', stage:'teleop', stages:[{phase:'far'}, {phase:'near'}]});
  assert.equal(links.length, 6);
  assert.equal(links[0].href, '/api/rendezvous/artifact/mission_report.json?mission_id=mission-123&phase=far');
  assert.equal(links[4].href, '/api/rendezvous/artifact/handoff.json?mission_id=mission-123&phase=near');
  assert.deepEqual(missionArtifactLinks({stages:[{phase:'far'}]}), []);
  assert.deepEqual(missionArtifactLinks({mission_id:'id', stages:[{phase:'unexpected'}]}), []);
});

test('an active flight exposes stop only to its owner and cannot be restarted', () => {
  const run = {stage: 'flight', phase: 'flight', active: true, can_manage: true};
  assert.equal(missionPresentation(run).canStart, false);
  assert.equal(missionPresentation(run).canStop, true);
  assert.equal(missionPresentation({...run, can_manage: false}).canStop, false);
});

test('delayed missions expose acquisition audit only after the far stage completes', () => {
  const mission = {mission_id:'mission-delayed', options:{differential_gnss:{delay_s:.2}}, stages:[]};
  assert.deepEqual(missionArtifactLinks(mission), []);
  const links = missionArtifactLinks({...mission, stages:[{phase:'far'}, {phase:'near'}]});
  assert.equal(links.length, 7);
  const samples = links.filter(link => link.href.includes('gnss_measurements.npz'));
  assert.equal(samples.length, 1);
  assert.ok(samples[0].href.endsWith('phase=far'));
  assert.equal(missionArtifactLinks({...mission, options:{differential_gnss:{delay_s:0}}, stages:[{phase:'far'}, {phase:'near'}]}).length, 6);
});

test('an accepted manual flight offers an explicit owned handoff', () => {
  const ready = {stage: 'flight', phase: 'ready', active: false, can_manage: true, can_handoff: true};
  assert.equal(missionPresentation(ready).canHandoff, true);
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
    assert.equal(Number($('missionGnssDelay').value), 0);
    $('missionGnssDelay').value = '.12';
    await panel.startFlight();
    const [path, init] = requests.at(-1), request = JSON.parse(init.body);
    assert.equal(path, '/api/mission/start');
    assert.equal(request.continuous, true);
    assert.equal(request.mode, 'team-in-orbit');
    assert.deepEqual(request.options.differential_gnss, {...receiver, absolute_position_sigma_m: .6, delay_s: .12});
    assert.equal(request.options.gnss_position_sigma_m, .6);
  } finally { globalThis.document = previousDocument; globalThis.Option = previousOption; }
});

test('handoff preserves the owned mission lifecycle', () => {
  const run = {stage: 'teleop', phase: 'running', active: true, mode: 'team-in-orbit', can_manage: true};
  assert.equal(missionPresentation(run).canStart, false);
  assert.equal(missionPresentation(run).canStop, true);
  assert.equal(missionPresentation(run, 'team-in-orbit').canHandoff, false);
});

for (const stateFirst of [false, true]) for (const savedMode of ['teleop', 'team-in-orbit', 'team-orbit-raise']) for (const savedDelay of [undefined, .17]) {
  test(`restored ${savedMode} mode and options with ${stateFirst ? 'state' : 'catalog'} arriving first, delay ${savedDelay}`, async () => {
    const hasSavedOptions = savedMode !== 'teleop';
    const previousDocument = globalThis.document, previousOption = globalThis.Option;
    globalThis.document = {body: {classList: {toggle() {}}}};
    globalThis.Option = class { constructor(label, value) { this.value = value; } };
    const nodes = new Map(), requests = [];
    const ctx = {clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}};
    const $ = id => {
      if (!nodes.has(id)) nodes.set(id, {
        value: id === 'missionMode' ? 'teleop' : id === 'missionGnssPosition' ? '3' : id === 'missionGnssVelocity' ? '.5' : '',
        textContent: '', checked: true, addEventListener() {},
        replaceChildren(...options) { this.value = options[0]?.value || ''; },
        querySelectorAll: () => [], getContext: () => ctx, width: 620, height: 200,
      });
      return nodes.get(id);
    };
    const defaults = {seed:7, handoff_distance_m:110, gnss_position_sigma_m:.577,
      gnss_velocity_sigma_m_s:.001155, initial_position_sigma_m:10, orbit_acceleration_sigma_m_s2:.00002,
      differential_gnss:{relative_position_sigma_m:.03, relative_velocity_sigma_m_s:.0015, sample_rate_hz:10, delay_s:0}};
    const saved = {mission_id:'saved', mode:savedMode,
      stage:'teleop', phase:'stopped', active:false,
      ...(hasSavedOptions ? {options:{seed:0, gnss_position_sigma_m:3, gnss_velocity_sigma_m_s:.5,
        ...(savedDelay != null ? {differential_gnss:{...defaults.differential_gnss, delay_s:savedDelay}} : {})}} : {})};
    let releaseCatalog;
    const pendingCatalog = new Promise(resolve => { releaseCatalog = resolve; });
    try {
      const panel = createMissionPanel({$, state:{streamLive:false}, message() {}, reconnect() {}, configureScene() {},
        refresh:async () => {}, apiRequest:async (path, init) => {
          requests.push([path, init]);
          if (path.endsWith('catalog')) await pendingCatalog;
          return {ok:true, status:200, json:async () => path.endsWith('catalog')
            ? {modes:[{id:'teleop'},{id:'team-in-orbit'},{id:'team-orbit-raise'}], continuous_default:true, flight_defaults:defaults}
            : {stage:'flight', phase:'starting', active:true}};
        }});
      const loading = panel.load();
      if (stateFirst) panel.render(saved);
      releaseCatalog(); await loading;
      if (!stateFirst) panel.render(saved);
      assert.equal($('missionMode').value, savedMode, 'initial restoration must not depend on response order');
      assert.equal(Number($('missionGnssPosition').value), hasSavedOptions ? 3 : defaults.gnss_position_sigma_m);
      assert.equal(Number($('missionGnssVelocity').value), hasSavedOptions ? .5 : defaults.gnss_velocity_sigma_m_s);
      assert.equal(Number($('missionSeed').value), hasSavedOptions ? 0 : 7);
      assert.equal(Number($('missionGnssDelay').value), hasSavedOptions ? savedDelay ?? 0 : 0);
      $('missionMode').value = 'team-orbit-raise'; panel.configure();
      $('missionGnssDelay').value = '.12';
      $('missionGnssPosition').value = '.8'; panel.render(saved);
      assert.equal($('missionMode').value, 'team-orbit-raise', 'later polls preserve the selected starting condition');
      assert.equal($('missionGnssPosition').value, '.8', 'later state polls preserve user edits');
      assert.equal($('missionGnssDelay').value, '.12', 'later state polls preserve delay edits');
      await panel.startFlight();
      const request = JSON.parse(requests.at(-1)[1].body);
      assert.equal(request.mode, 'team-orbit-raise');
      assert.equal(request.continuous, true);
      assert.equal(request.options.gnss_position_sigma_m, .8);
      assert.equal(request.options.differential_gnss.absolute_position_sigma_m, .8);
      assert.equal(request.options.differential_gnss.absolute_velocity_sigma_m_s, hasSavedOptions ? .5 : defaults.gnss_velocity_sigma_m_s);
      assert.equal(request.options.differential_gnss.delay_s, .12);
    } finally { globalThis.document = previousDocument; globalThis.Option = previousOption; }
  });
}

test('idle selection does not change lifecycle permissions; stopping stays disabled', () => {
  assert.deepEqual(missionPresentation({}, 'team-orbit-raise'), missionPresentation({}, 'teleop'));
  assert.equal(missionPresentation({active: true, phase: 'stopping', can_manage: true}).canStop, false);
});

test('completed logical capture keeps its result view and does not offer distant-flight handoff', () => {
  const mission = {stage: 'flight', phase: 'completed', active: false, mode: 'near-field', can_manage: true, can_handoff: false};
  const view = missionPresentation(mission, 'near-field');
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


test('near-field telemetry shows the physical node clock and current readiness without claiming fusion weights', () => {
  const nodes = new Map();
  const $ = id => {
    if (!nodes.has(id)) nodes.set(id, {value: '', textContent: '', addEventListener() {},
      querySelectorAll: () => [], getContext: () => ({clearRect() {}, beginPath() {}, moveTo() {}, lineTo() {}, stroke() {}}),
      width: 620, height: 200});
    return nodes.get(id);
  };
  const panel = createMissionPanel({$, state: {streamLive:true}, message() {}, reconnect() {}});
  const meta = {phase:'VisualApproach', navigation_source:'GNSS + stereo fusion', visual_ready:false,
    optical_depth_m:5.5, near_guidance:{node_id:'VisualReady5p5m', subsegment:'VisualReady5p5mHold',
    nodes:{VisualReady5p5m:{dwell_s:1.2,wait_s:13.4,completed:false}}}};
  const mission = {mission_id:'near',stage:'flight',phase:'flight',active:true,mode:'team-in-orbit',
    telemetry:{phase:'VisualApproach',state:{metadata:meta}}};
  panel.render(mission);
  assert.match($('missionPhase').textContent, /5.50 m/);
  assert.match($('missionPhase').textContent, /1.20 s.*13.40 s/);
  assert.match($('missionNavigation').textContent, /视觉未就绪/);
  assert.equal($('nearCameraDepth').textContent, '5.5000 m');
  meta.visual_ready = true;
  panel.render(mission);
  assert.match($('missionNavigation').textContent, /视觉就绪/);
  assert.doesNotMatch($('missionPhase').textContent, /主导/);
});
