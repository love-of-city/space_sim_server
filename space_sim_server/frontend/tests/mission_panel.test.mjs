import test from 'node:test';
import assert from 'node:assert/strict';
import {missionPresentation} from '../mission_panel.js';

test('flight keeps robot controls closed and exposes stop only to its owner', () => {
  const run = {stage: 'flight', phase: 'flight', active: true, can_manage: true};
  assert.equal(missionPresentation(run).flightView, true);
  assert.equal(missionPresentation(run).canStart, false);
  assert.equal(missionPresentation(run).canStop, true);
  assert.equal(missionPresentation({...run, can_manage: false}).canStop, false);
});

test('an accepted flight offers an explicit owned handoff, never automatic control', () => {
  const ready = {stage: 'flight', phase: 'ready', active: false, can_manage: true, can_handoff: true};
  assert.equal(missionPresentation(ready).canHandoff, true);
  assert.equal(missionPresentation(ready).flightView, true);
  assert.equal(missionPresentation({...ready, can_manage: false}).canHandoff, false);
  assert.equal(missionPresentation({...ready, can_handoff: false}).showHandoff, false);
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
