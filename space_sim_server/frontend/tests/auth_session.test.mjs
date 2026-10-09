import { readFileSync } from "node:fs";
import vm from "node:vm";
import test from "node:test";
import assert from "node:assert/strict";

const source = readFileSync(new URL("../app.js", import.meta.url), "utf8");
const apiSource = source.slice(source.indexOf("async function apiRequest("), source.indexOf("function applyAuthenticatedUser("));
const streamSource = source.slice(source.indexOf("async function connectPixelStreaming("), source.indexOf("function resetWebRtcStats("));

function harness(detail, signedIn = true) {
  const state = { currentUser: signedIn ? { username: "admin" } : null };
  const response = { status: 401, ok: false, clone: () => ({ json: async () => ({ detail }) }) };
  const nodes = new Map();
  const metrics = { logins: 0, shutdowns: 0, retries: 0, requests: 0, disposals: 0 };
  const context = vm.createContext({
    state, URLSearchParams, location: { search: "" }, console,
    fetch: async () => { metrics.requests++; return response; },
    shutdownAuthenticatedApp: () => { metrics.shutdowns++; },
    showLogin: () => { metrics.logins++; state.currentUser = null; },
    $: id => { if (!nodes.has(id)) nodes.set(id, { style: {}, textContent: "" }); return nodes.get(id); },
    disposePixelStream: () => { metrics.disposals++; },
    schedulePixelStreamingReconnect: () => { metrics.retries++; },
    setMessage: message => { metrics.message = message; },
  });
  vm.runInContext(apiSource + streamSource, context);
  return { state, response, metrics, context, nodes };
}

test("a stream-key rejection does not expire the account session", async () => {
  const context = harness("a valid stream access key is required");
  assert.equal(await context.context.apiRequest("/api/client-config"), context.response);
  assert.equal(context.metrics.logins, 0);
  assert.equal(context.metrics.shutdowns, 0);
  assert.equal(context.state.currentUser.username, "admin");
});

test("expired account sessions transition to login only once", async () => {
  const context = harness("login required");
  await context.context.apiRequest("/api/state");
  await context.context.apiRequest("/api/state");
  assert.equal(context.metrics.logins, 1);
  assert.equal(context.metrics.shutdowns, 1);
});

test("missing stream key shows an actionable error without a login or retry loop", async () => {
  const context = harness("a valid stream access key is required");
  await context.context.connectPixelStreaming();
  assert.equal(context.metrics.logins, 0);
  assert.equal(context.metrics.retries, 0);
  assert.equal(context.metrics.requests, 1);
  assert.equal(context.state.streamConfig, null);
  assert.equal(context.nodes.get("frameState").textContent, "ACCESS KEY REQUIRED");
  assert.match(context.metrics.message, /access_key/);
});

test("expired session during stream configuration cannot resurrect its retry loop", async () => {
  const context = harness("login required");
  await context.context.connectPixelStreaming();
  assert.equal(context.metrics.logins, 1);
  assert.equal(context.metrics.retries, 0);
  assert.equal(context.state.currentUser, null);
});

test("retired callbacks do not request streaming configuration on the login form", async () => {
  const context = harness("login required", false);
  await context.context.connectPixelStreaming();
  assert.equal(context.metrics.requests, 0);
  assert.equal(context.metrics.logins, 0);
});
