import {test} from 'node:test';
import assert from 'node:assert/strict';
import {requestJson} from '../api_client.js';

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

test('network and timeout errors are distinct and never retry writes', async () => {
  let calls = 0;
  await assert.rejects(requestJson('/start', {method: 'POST'}, async () => {calls++; throw new TypeError('offline');}), /连接/);
  assert.equal(calls, 1);
  await assert.rejects(requestJson('/state', {}, async () => {throw new DOMException('timeout', 'AbortError');}), /超时/);
});
