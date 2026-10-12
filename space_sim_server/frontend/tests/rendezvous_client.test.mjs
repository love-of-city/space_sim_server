import test from 'node:test';
import assert from 'node:assert/strict';
import { RendezvousSession, requestJson } from '../rendezvous_client.js';

function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return {promise, resolve, reject};
}

test('slow state reads cannot overlap or grow a request queue', async () => {
  const session = new RendezvousSession(), pending = deferred(), rendered = [];
  let reads = 0;
  const load = () => { reads += 1; return pending.promise; };
  const first = session.refresh(load, value => rendered.push(value), assert.fail);
  for (let i = 0; i < 100; i++) await session.refresh(load, assert.fail, assert.fail);
  assert.equal(reads, 1);
  pending.resolve('latest');
  await first;
  assert.deepEqual(rendered, ['latest']);
});

for (const fail of [false, true]) {
  test(`a pre-action state ${fail ? 'failure' : 'response'} cannot overwrite the completed action`, async () => {
    const session = new RendezvousSession(), pending = deferred();
    let signal;
    const first = session.refresh(s => { signal = s; return pending.promise; }, assert.fail, assert.fail);
    await session.act(async () => {});
    assert.equal(signal.aborted, true);
    if (fail) pending.reject(new Error('obsolete')); else pending.resolve('obsolete');
    await first;
    const seen = [];
    await session.refresh(async () => 'new run', x => seen.push(x), assert.fail);
    assert.deepEqual(seen, ['new run']);
  });
}

test('double clicks and polling during a mission command do not start extra work', async () => {
  const session = new RendezvousSession(), pending = deferred();
  const first = session.act(() => pending.promise);
  assert.equal(session.busy, true);
  assert.equal(await session.act(assert.fail), false);
  await session.refresh(assert.fail, assert.fail, assert.fail);
  pending.resolve();
  assert.equal(await first, true);
  assert.equal(session.busy, false);
});

test('a failed mutation is not retried and releases the controls for recovery', async () => {
  const session = new RendezvousSession();
  let calls = 0;
  await assert.rejects(session.act(async () => { calls++; throw new Error('rejected'); }), /rejected/);
  assert.equal(calls, 1);
  assert.equal(session.busy, false);
  assert.equal(await session.act(async () => { calls++; }), true);
  assert.equal(calls, 2);
});

test('failed reads recover, including transport aborts', async () => {
  const session = new RendezvousSession(), errors = [], states = [];
  await session.refresh(async () => { throw new Error('offline'); }, assert.fail, e => errors.push(e.message));
  await session.refresh(async () => ({running: true}), x => states.push(x), assert.fail);
  assert.deepEqual(errors, ['offline']);
  assert.deepEqual(states, [{running: true}]);
  await assert.rejects(requestJson('/state', {}, async () => { throw new DOMException('timeout', 'AbortError'); }), /超时/);
});

test('HTTP failures, expired sessions and malformed responses have usable errors', async () => {
  const response = (status, body) => async () => new Response(body, {status});
  await assert.rejects(requestJson('/x', {}, response(401, '')), /登录/);
  await assert.rejects(requestJson('/x', {}, response(503, '<html>down</html>')), /HTTP 503/);
  await assert.rejects(requestJson('/x', {}, response(409, '{"detail":"请先停止场景"}')), /请先停止场景/);
  await assert.rejects(requestJson('/x', {}, response(422, '{"detail":[{"loc":["body"]}]}')), /HTTP 422/);
  for (const body of ['null', '[]', '12']) await assert.rejects(requestJson('/x', {}, response(200, body)), /无效数据/);
});

test('request transport preserves method, body and cancellation without retries', async () => {
  const controller = new AbortController();
  let calls = 0;
  const data = await requestJson('/start', {method: 'POST', body: '{}', signal: controller.signal}, async (url, options) => {
    calls++;
    assert.equal(url, '/start');
    assert.equal(options.method, 'POST');
    assert.equal(options.body, '{}');
    assert.equal(options.signal, controller.signal);
    assert.equal(options.cache, 'no-store');
    return new Response('{"running":true}');
  });
  assert.deepEqual(data, {running: true});
  assert.equal(calls, 1);
});
