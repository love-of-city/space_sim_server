import { spawn } from "node:child_process";
import path from "node:path";

const address = new URL(process.argv[2] ?? "http://127.0.0.1:18000/");
const username = process.env.PIXEL_STREAMING_USERNAME;
const password = process.env.PIXEL_STREAMING_PASSWORD;
const accessKey = process.env.PIXEL_STREAMING_ACCESS_KEY;
const expectAccessKeyRequired = process.env.PIXEL_STREAMING_EXPECT_ACCESS_KEY_REQUIRED === "1";
if (!username || !password) throw new Error("Test account must be supplied through environment variables.");
if (accessKey) address.searchParams.set("access_key", accessKey);
const port = Number(process.env.CHROME_DEBUG_PORT ?? 9349);
const browser = spawn(process.env.CHROME_PATH ?? "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe", [
  "--headless=new", "--no-first-run", `--remote-debugging-port=${port}`,
  `--user-data-dir=${path.resolve("run", `login-input-${Date.now()}`)}`, "about:blank",
], { stdio: "ignore", windowsHide: true });
const delay = milliseconds => new Promise(resolve => setTimeout(resolve, milliseconds));
let socket;
let send;
let browserError;
browser.on("error", error => { browserError = error; });
try {
  let targets;
  for (let attempt = 0; attempt < 50; attempt++) {
    if (browserError) throw browserError;
    try { targets = await (await fetch(`http://127.0.0.1:${port}/json/list`)).json(); break; }
    catch { await delay(200); }
  }
  const page = targets?.find(target => target.type === "page");
  if (!page) throw new Error("Browser page unavailable.");
  socket = new WebSocket(page.webSocketDebuggerUrl);
  await new Promise((resolve, reject) => { socket.onopen = resolve; socket.onerror = reject; });
  let sequence = 0;
  const pending = new Map();
  socket.onmessage = event => {
    const result = JSON.parse(event.data);
    const waiter = pending.get(result.id);
    if (!waiter) return;
    pending.delete(result.id);
    result.error ? waiter.reject(new Error(result.error.message)) : waiter.resolve(result.result);
  };
  send = (method, params = {}) => new Promise((resolve, reject) => {
    const identifier = ++sequence;
    const timer = setTimeout(() => { pending.delete(identifier); reject(new Error(`Timed out: ${method}`)); }, 15000);
    pending.set(identifier, {
      resolve: value => { clearTimeout(timer); resolve(value); },
      reject: error => { clearTimeout(timer); reject(error); },
    });
    socket.send(JSON.stringify({ id: identifier, method, params }));
  });
  const evaluate = async expression => {
    const result = await send("Runtime.evaluate", { expression, returnByValue: true, awaitPromise: true });
    if (result.exceptionDetails) throw new Error("Browser evaluation failed.");
    return result.result?.value;
  };
  await send("Page.navigate", { url: address.href });
  let ready = false;
  for (let attempt = 0; attempt < 60; attempt++) {
    ready = await evaluate("Boolean(document.getElementById('loginPassword')) && !document.body.classList.contains('auth-pending')");
    if (ready) break;
    await delay(250);
  }
  if (!ready) throw new Error("Login page did not initialize.");
  await evaluate(`document.getElementById('loginUsername').value = ${JSON.stringify(username)}; document.getElementById('loginPassword').focus();`);
  for (const character of password) {
    await send("Input.dispatchKeyEvent", { type: "keyDown", key: character, text: character });
    await send("Input.dispatchKeyEvent", { type: "keyUp", key: character });
    await delay(70);
  }
  await delay(2000);
  const before = await evaluate(`document.getElementById('loginPassword').value.length === ${password.length}`);
  if (!before) throw new Error("Password disappeared before login submission.");
  await evaluate("document.getElementById('loginForm').requestSubmit()");
  await delay(5000);
  const result = await evaluate(`(async () => ({
    sessionStatus: (await fetch('/api/auth/me')).status,
    loginHidden: document.getElementById('authGate').hidden,
    frameState: document.getElementById('frameState').textContent,
    loginError: document.getElementById('loginError').textContent
  }))()`);
  if (result.sessionStatus !== 200 || !result.loginHidden) throw new Error(`Login did not remain stable: ${JSON.stringify(result)}`);
  if (expectAccessKeyRequired && result.frameState !== "ACCESS KEY REQUIRED") throw new Error("Missing-key guidance was not displayed.");
  if (!expectAccessKeyRequired && result.frameState === "ACCESS KEY REQUIRED") throw new Error("Unexpected access-key requirement.");
  await evaluate("document.getElementById('logoutButton').click()");
  await delay(500);
  await evaluate("document.getElementById('loginPassword').focus()");
  await send("Input.insertText", { text: "typing-only" });
  await delay(4000);
  const stableAfterLogout = await evaluate("document.getElementById('loginPassword').value === 'typing-only'");
  if (!stableAfterLogout) throw new Error("Retired authenticated callbacks cleared new login input.");
  console.log(JSON.stringify({ ok: true, origin: address.origin, accessKeySupplied: Boolean(accessKey),
    passwordTypingStable: before, stableAfterLogout, ...result }, null, 2));
} finally {
  if (send && socket?.readyState === WebSocket.OPEN) {
    try { await send("Runtime.evaluate", { expression: "fetch('/api/auth/logout', {method:'POST'})", awaitPromise: true }); } catch {}
  }
  socket?.close();
  browser.kill();
}
