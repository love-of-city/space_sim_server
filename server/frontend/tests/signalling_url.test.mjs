import { readFileSync } from 'node:fs';
import vm from 'node:vm';
import test from 'node:test';
import assert from 'node:assert/strict';

const source = readFileSync(new URL('../app.js', import.meta.url), 'utf8');
const definition = source.slice(source.indexOf('function signallingUrl()'), source.indexOf('function resetWebRtcStats()'));
function resolve(config, href = 'http://127.0.0.1:18000/') {
  const location = new URL(href);
  const context = vm.createContext({ state: { streamConfig: config }, location, URL });
  vm.runInContext(definition, context);
  return new URL(context.signallingUrl());
}

test('default local signalling includes the JWT needed by secure Linux deployment', () => {
  const url = resolve({ pixel_streaming_player_port: 18080, pixel_streaming_access_token: 'signed-token' });
  assert.equal(url.origin, 'ws://127.0.0.1:18080');
  assert.equal(url.searchParams.get('token'), 'signed-token');
});

test('explicit gateway URL preserves its path and query while replacing old JWT', () => {
  const url = resolve({ pixel_streaming_signalling_url: '/stream?camera=main&token=old', pixel_streaming_access_token: 'new' }, 'https://example.com/');
  assert.equal(url.pathname, '/stream');
  assert.equal(url.searchParams.get('camera'), 'main');
  assert.equal(url.searchParams.get('token'), 'new');
});

test('legacy local signalling does not require a JWT and HTTPS selects WSS', () => {
  const url = resolve({ pixel_streaming_player_port: 8080 }, 'https://example.com/');
  assert.equal(url.origin, 'wss://example.com:8080');
  assert.equal(url.searchParams.has('token'), false);
});
