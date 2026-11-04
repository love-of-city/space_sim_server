import { resolveSignallingUrl } from '../pixel_stream_session.js';
import test from 'node:test';
import assert from 'node:assert/strict';

function resolve(config, href = 'http://127.0.0.1:18000/') {
  return new URL(resolveSignallingUrl(config, new URL(href)));
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
