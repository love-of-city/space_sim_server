// Pinned UE 5.6 SDK workaround: a pending getStats() can finish after camera
// switching disconnects its peer. Keep the same peer and discard stale results.
import {readFileSync, writeFileSync} from 'node:fs';
import {fileURLToPath, pathToFileURL} from 'node:url';
import {join} from 'node:path';

const marker = '// SpaceSim: discard statistics from a retired media peer.';
export function guardStatsSource(source) {
  if (source.includes(marker)) return source;
  const start = source.indexOf('    generateStats() {');
  const end = source.indexOf('\n    /**', start);
  if (start < 0 || end < 0) throw new Error('Pinned Pixel Streaming stats method changed; review the workaround.');
  let method = source.slice(start, end);
  for (const expected of ['this.peerConnection.getStats()', 'this.peerConnection.getReceivers()', 'this.onVideoStats(this.aggregatedStats);']) {
    if (method.split(expected).length !== 2) throw new Error('Unexpected Pixel Streaming stats implementation.');
  }
  method = method.replace('        this.peerConnection.getStats()',
    `        ${marker}\n        const peer = this.peerConnection;\n        if (!peer || peer.signalingState === 'closed') return;\n        peer.getStats()`);
  const callback = /(\.then\(\(statsData(?:: RTCStatsReport)?\) => \{\r?\n)/;
  if (!callback.test(method)) throw new Error('Unexpected Pixel Streaming stats callback.');
  const guard = "            if (this.peerConnection !== peer || peer.signalingState === 'closed') return;";
  method = method.replace(callback, `$1${guard}\n`)
    .replace('this.peerConnection.getReceivers()', 'peer.getReceivers()')
    .replace('this.onVideoStats(this.aggregatedStats);', `this.onVideoStats(this.aggregatedStats);\n${guard}`);
  const closing = method.lastIndexOf('        });');
  if (closing < 0 || method.slice(closing+11).trim() !== '}') throw new Error('Unexpected stats method ending.');
  method = method.slice(0, closing) + `        }, error => {\n            if (this.peerConnection === peer && peer.signalingState !== 'closed') throw error;\n        });` + method.slice(closing+11);
  return source.slice(0, start) + method + source.slice(end);
}

export function patchInstalledSdk() {
  const root = fileURLToPath(new URL('../node_modules/@epicgames-ps/lib-pixelstreamingfrontend-ue5.6/', import.meta.url));
  if (JSON.parse(readFileSync(join(root, 'package.json'), 'utf8')).version !== '0.2.5') {
    throw new Error('Review the Pixel Streaming stats workaround before changing SDK version.');
  }
  for (const file of ['dist/esm/PeerConnectionController/PeerConnectionController.js',
    'dist/cjs/PeerConnectionController/PeerConnectionController.js', 'src/PeerConnectionController/PeerConnectionController.ts']) {
    const path = join(root, file);
    const source = readFileSync(path, 'utf8');
    const patched = guardStatsSource(source);
    if (patched !== source) writeFileSync(path, patched);
  }
}

if (process.argv[1] && import.meta.url === pathToFileURL(process.argv[1]).href) patchInstalledSdk();
