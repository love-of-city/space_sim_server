import test from 'node:test';
import assert from 'node:assert/strict';
import { rendezvousPlayerUrl } from '../rendezvous_player.js';

test('rendezvous viewer targets its own custom player and streamer', () => {
  const url = new URL(rendezvousPlayerUrl('http://127.0.0.1:28000/static/rendezvous.html?private=discard#anchor', {
    pixel_streaming_player_port: 28080, pixel_streaming_streamer_id: 'YipuWang & SpaceSim',
  }));
  assert.equal(url.origin, 'http://127.0.0.1:28080');
  assert.equal(url.pathname, '/player.html');
  assert.equal(url.searchParams.get('StreamerId'), 'YipuWang & SpaceSim');
  assert.equal(url.searchParams.get('AutoPlayVideo'), 'true');
  assert.equal(url.searchParams.has('private'), false);
  assert.equal(url.hash, '');
});
test('default viewer configuration and IPv6 addresses remain supported', () => {
  const url = new URL(rendezvousPlayerUrl('http://[::1]:8000/', {pixel_streaming_player_port:8080,pixel_streaming_streamer_id:'BskRenderer'}));
  assert.equal(url.origin, 'http://[::1]:8080');
  assert.equal(url.searchParams.get('StreamerId'), 'BskRenderer');
});
test('invalid player configuration never silently selects another instance', () => {
  for (const port of [0, 65536, 1.5, 'not-a-port']) assert.throws(() => rendezvousPlayerUrl('http://localhost/', {pixel_streaming_player_port:port,pixel_streaming_streamer_id:'id'}));
  assert.throws(() => rendezvousPlayerUrl('http://localhost/', {pixel_streaming_player_port:8080,pixel_streaming_streamer_id:''}));
});
