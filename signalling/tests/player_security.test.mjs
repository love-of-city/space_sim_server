import test from "node:test";
import assert from "node:assert/strict";
import crypto from "node:crypto";
import net from "node:net";
import {spawn} from "node:child_process";
import {once} from "node:events";
import {fileURLToPath} from "node:url";
import WebSocket from "ws";

const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
async function freePort() {
  const listener = net.createServer(); listener.listen(0, "127.0.0.1");
  await once(listener, "listening"); const {port} = listener.address();
  await new Promise(resolve => listener.close(resolve)); return port;
}
function token(secret) {
  const encode = value => Buffer.from(JSON.stringify(value)).toString("base64url");
  const body = `${encode({alg: "HS256", typ: "JWT"})}.${encode({exp: Math.floor(Date.now()/1000)+60, streamer_ids: ["test"]})}`;
  return `${body}.${crypto.createHmac("sha256", secret).update(body).digest("base64url")}`;
}

async function rejectedUpgrade(url, options) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(url, {...options, handshakeTimeout: 3000});
    ws.on("unexpected-response", (_request, response) => {response.resume(); ws.terminate(); resolve(response.statusCode);});
    ws.on("error", error => { if (!error.message.includes("closed before")) reject(error); });
    ws.on("open", () => {ws.terminate(); reject(new Error("Unexpected authorized upgrade"));});
  });
}
async function receiveConfig(url, options) {
  return new Promise((resolve, reject) => {
    const ws = new WebSocket(url, {...options, handshakeTimeout: 3000});
    const timer = setTimeout(() => {ws.terminate(); reject(new Error("config timed out"));}, 5000);
    ws.on("error", error => {clearTimeout(timer); reject(error);});
    ws.on("message", data => {
      const message = JSON.parse(data.toString());
      if (message.type === "config") {clearTimeout(timer); ws.close(); resolve(message);}
    });
  });
}

test("actual isolated signalling server enforces player Origin and gives both peers ICE settings", {timeout: 30000}, async t => {
  const playerPort = await freePort();
  let streamerPort = await freePort();
  while (streamerPort === playerPort) streamerPort = await freePort();
  const secret = "isolated-test-jwt-secret";
  const childEnvironment = {...process.env};
  delete childEnvironment.NODE_TEST_CONTEXT; // A service child is not a Node test worker.
  const child = spawn(process.execPath, ["server.mjs"], {
    cwd: fileURLToPath(new URL("..", import.meta.url)), windowsHide: true,
    stdio: ["ignore", "pipe", "pipe"],
    env: {...childEnvironment, PS_PLAYER_HOST: "127.0.0.1", PS_PLAYER_PORT: String(playerPort),
      PS_STREAMER_HOST: "127.0.0.1", PS_STREAMER_PORT: String(streamerPort), PS_JWT_SECRET: secret,
      PS_ALLOWED_ORIGINS: '["https://sim.test"]', PS_ICE_SERVERS_JSON: '[{"urls":"stun:stun.test:3478"}]',
      PS_TURN_URLS_JSON: '["turn:turn.test:3478"]', PS_TURN_AUTH_SECRET: "isolated-test-turn-secret"},
  });
  let output = "";
  child.stdout.on("data", data => {output += data.toString();});
  child.stderr.on("data", data => {output += data.toString();});
  t.after(() => { if(child.exitCode !== null && child.exitCode !== 0) console.error(output); });
  t.after(async () => {if (child.exitCode === null) {child.kill(); await once(child, "exit");}});
  const deadline = Date.now() + 15000;
  while (!output.includes('"event":"ready"') && Date.now() < deadline && child.exitCode === null) await delay(50);
  assert.match(output, /"event":"ready"/);
  const url = `ws://127.0.0.1:${playerPort}/stream?token=${token(secret)}`;
  assert.equal(await rejectedUpgrade(url, {origin: "https://evil.test"}), 403);
  assert.equal(await rejectedUpgrade(url, {}), 403);
  const browser = await receiveConfig(url, {origin: "https://sim.test"});
  assert.equal(browser.peerConnectionOptions.iceServers[0].urls, "stun:stun.test:3478");
  assert.deepEqual(browser.peerConnectionOptions.iceServers[1].urls, ["turn:turn.test:3478"]);
  assert.ok(browser.peerConnectionOptions.iceServers[1].credential);
  assert.ok(!JSON.stringify(browser).includes("isolated-test-turn-secret"));
  // The UE-side socket is local and does not carry a browser Origin or JWT.
  const ue = await receiveConfig(`ws://127.0.0.1:${streamerPort}`, {});
  assert.ok(ue.peerConnectionOptions.iceServers.length === 2);

  // Real wire regression: list refreshes may repeat subscribe while UE starts.
  // It must announce one peer, not disconnect/reconnect the same player ID.
  const messages=[];
  const streamer=new WebSocket(`ws://127.0.0.1:${streamerPort}`);
  streamer.on("message", raw=>messages.push(JSON.parse(raw)));
  t.after(()=>streamer.terminate());
  await once(streamer,"open");
  streamer.send(JSON.stringify({type:"endpointId",id:"test"}));
  const waitFor=async predicate=>{
    const until=Date.now()+5000;
    while(!predicate() && Date.now()<until) await delay(20);
    assert.ok(predicate(), "wire condition timed out");
  };
  await waitFor(()=>messages.some(m=>m.type==="endpointIdConfirm"));
  const player=new WebSocket(url,{origin:"https://sim.test"});
  t.after(()=>player.terminate());
  await once(player,"open");
  player.send(JSON.stringify({type:"subscribe",streamerId:"test"}));
  await waitFor(()=>messages.some(m=>m.type==="playerConnected"));
  const playerId=messages.find(m=>m.type==="playerConnected").playerId;
  for(let i=0;i<4;i++)player.send(JSON.stringify({type:"subscribe",streamerId:"test"}));
  await delay(150);
  assert.equal(messages.filter(m=>m.type==="playerConnected").length,1);
  assert.equal(messages.filter(m=>m.type==="playerDisconnected").length,0);
  player.close();await once(player,"close");
  await delay(200);
  assert.equal(messages.filter(m=>m.type==="playerDisconnected").length,0,"must await UE setup acknowledgement");
  streamer.send(JSON.stringify({type:"offer",playerId,sdp:"test SDP - no real UE/browser"}));
  await waitFor(()=>messages.some(m=>m.type==="playerDisconnected"));
  assert.equal(messages.filter(m=>m.type==="playerDisconnected").length,1);
  // Late ICE cannot silently attach a not-yet-subscribed player to a random camera.
  const idle=new WebSocket(url,{origin:"https://sim.test"});
  t.after(()=>idle.terminate());await once(idle,"open");
  idle.send(JSON.stringify({type:"iceCandidate",candidate:{candidate:"",sdpMid:"0",sdpMLineIndex:0}}));
  await delay(100);
  assert.equal(messages.filter(m=>m.type==="playerConnected").length,1);
});
