import test from "node:test";
import assert from "node:assert/strict";
import http from "node:http";
import net from "node:net";
import { once } from "node:events";
import { spawn } from "node:child_process";
import { fileURLToPath } from "node:url";
import WebSocket from "ws";
import { validateSessionEndpoint, sessionAccess } from "../session_access.mjs";

const origins = new Set(["http://127.0.0.1:28000"]);
const endpoint = "http://127.0.0.1:28000/api/client-config";
test("session mode is explicit and cannot expose cookie authentication on a public listener", () => {
  assert.equal(validateSessionEndpoint(undefined, "0.0.0.0", new Set(), "jwt"), null);
  assert.equal(validateSessionEndpoint(endpoint, "127.0.0.1", origins, "").href, endpoint);
  for (const args of [
    [endpoint, "0.0.0.0", origins, ""], [endpoint, "127.0.0.1", new Set(), ""],
    [endpoint, "127.0.0.1", origins, "jwt"],
    ["http://remote.test/api/client-config", "127.0.0.1", origins, ""],
    [endpoint + "?redirect=1", "127.0.0.1", origins, ""],
  ]) assert.throws(() => validateSessionEndpoint(...args));
});

async function freePort() {
  const server = net.createServer().listen(0, "127.0.0.1");
  await once(server, "listening");
  const port = server.address().port;
  await new Promise(resolve => server.close(resolve));
  return port;
}

test("SSH signalling authenticates actual upgrades through the platform session and emits TURN to both peers", {timeout: 25000}, async t => {
  let authorized = true;
  let streams = [{id: "test", aliases: ["camera"]}];
  const backend = http.createServer((request, response) => {
    const ok = authorized && request.headers.cookie === "space_arm_session=valid" && request.url === "/api/client-config";
    response.writeHead(ok ? 200 : 401, {"Content-Type": "application/json"});
    response.end(JSON.stringify(ok ? {pixel_streaming_streamers: streams} : {}));
  }).listen(0, "127.0.0.1");
  await once(backend, "listening");
  t.after(() => backend.close());
  const authUrl = `http://127.0.0.1:${backend.address().port}/api/client-config`;
  assert.deepEqual(await sessionAccess(authUrl, "space_arm_session=valid"), {streamer_ids: ["test", "camera"]});
  const playerPort = await freePort();
  let streamerPort = await freePort();
  while (streamerPort === playerPort) streamerPort = await freePort();
  const env = {...process.env, PS_JWT_SECRET: "", PS_SESSION_AUTH_URL: authUrl,
    PS_PLAYER_HOST: "127.0.0.1", PS_STREAMER_HOST: "127.0.0.1",
    PS_PLAYER_PORT: String(playerPort), PS_STREAMER_PORT: String(streamerPort),
    PS_ALLOWED_ORIGINS: JSON.stringify([...origins]), PS_ICE_SERVERS_JSON: "[]",
    PS_TURN_URLS_JSON: '["turn:relay.test:3478"]', PS_TURN_AUTH_SECRET: "test-secret"};
  delete env.NODE_TEST_CONTEXT;
  const child = spawn(process.execPath, ["server.mjs"], {
    cwd: fileURLToPath(new URL("..", import.meta.url)), env, windowsHide: true,
    stdio: ["ignore", "pipe", "pipe"],
  });
  let output = "";
  child.stdout.on("data", data => {output += data;});
  child.stderr.on("data", data => {output += data;});
  t.after(async () => {if (child.exitCode === null) {child.kill(); await once(child, "exit");}});
  const deadline = Date.now() + 10000;
  while (!output.includes('"event":"ready"') && Date.now() < deadline && child.exitCode === null) {
    await new Promise(resolve => setTimeout(resolve, 25));
  }
  assert.match(output, /"event":"ready"/);
  const upgrade = options => new Promise((resolve, reject) => {
    const ws = new WebSocket(`ws://127.0.0.1:${playerPort}`, {handshakeTimeout: 5000, ...options});
    ws.on("unexpected-response", (_, response) => {response.resume(); ws.terminate(); resolve(response.statusCode);});
    ws.on("error", e => {if (!e.message.includes("closed before")) reject(e);});
    ws.on("message", raw => {const msg = JSON.parse(raw); if (msg.type === "config") {ws.close(); resolve(msg);}});
  });
  const valid = {origin: [...origins][0], headers: {Cookie: "space_arm_session=valid"}};
  assert.equal(await upgrade({origin: valid.origin}), 401);
  assert.equal(await upgrade({...valid, origin: "http://evil.test"}), 403);
  assert.equal(await upgrade({...valid, headers: {Cookie: "space_arm_session=expired"}}), 401);
  const config = await upgrade(valid);
  assert.deepEqual(config.peerConnectionOptions.iceServers[0].urls, ["turn:relay.test:3478"]);
  assert.ok(config.peerConnectionOptions.iceServers[0].credential);
  assert.ok(!JSON.stringify(config).includes("test-secret"));
  const ue = new WebSocket(`ws://127.0.0.1:${streamerPort}`);
  t.after(() => ue.terminate());
  const ueConfig = await new Promise((resolve, reject) => {
    ue.on("error", reject);
    ue.on("message", raw => {const msg = JSON.parse(raw); if (msg.type === "config") resolve(msg);});
  });
  assert.ok(ueConfig.peerConnectionOptions.iceServers[0].credential);
  ue.close();
  streams = [{id: "test", aliases: "not-an-array"}];
  assert.equal(await upgrade(valid), 401, "malformed permissions must fail closed");
  streams = [{id: "test", aliases: ["camera"]}];
  authorized = false;
  assert.equal(await upgrade(valid), 401, "logout/revocation must be checked on every new connection");
  backend.closeAllConnections();
  await new Promise(resolve => backend.close(resolve));
  assert.equal(await upgrade(valid), 401, "backend failure must fail closed");
});
