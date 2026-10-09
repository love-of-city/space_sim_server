import { spawn } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";

const requestedUrl = new URL(process.argv[2] ?? "http://127.0.0.1:8000/");
if (process.env.PIXEL_STREAMING_ACCESS_KEY) requestedUrl.searchParams.set("access_key", process.env.PIXEL_STREAMING_ACCESS_KEY);
const appUrl = requestedUrl.href;
const safeUrl = (value) => { const url = new URL(value); return url.origin + url.pathname; };
const requestedStreamerId = process.argv[3] ?? "";
const chromePath = process.env.CHROME_PATH
  ?? "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe";
const debugPort = Number(process.env.CHROME_DEBUG_PORT ?? 9333);
const warmupMs = Number(process.env.PIXEL_STREAMING_WARMUP_MS ?? 15000);
const forceRelay = process.env.PIXEL_STREAMING_FORCE_RELAY === "1";
const turnTransport = process.env.PIXEL_STREAMING_TURN_TRANSPORT ?? "";
if (turnTransport && !["udp", "tcp"].includes(turnTransport)) throw new Error("TURN transport must be udp or tcp.");
const runDir = path.resolve("run", `pixel-streaming-smoke-${Date.now()}`);
await mkdir(runDir, { recursive: true });

const chrome = spawn(chromePath, [
  "--headless=new",
  "--no-first-run",
  "--autoplay-policy=no-user-gesture-required",
  "--window-size=1440,900",
  `--remote-debugging-port=${debugPort}`,
  `--user-data-dir=${path.join(runDir, "profile")}`,
  "about:blank",
], { stdio: "ignore", windowsHide: true });
let chromeError;
chrome.on("error", (error) => { chromeError = error; });

const delay = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));

async function getJson(url, timeoutMs = 20000) {
  const deadline = Date.now() + timeoutMs;
  let lastError;
  while (Date.now() < deadline) {
    if (chromeError) throw chromeError;
    try {
      const response = await fetch(url);
      if (response.ok) return await response.json();
    } catch (error) {
      lastError = error;
    }
    await delay(250);
  }
  throw lastError ?? new Error(`Timed out loading ${url}`);
}

async function connectCdp(webSocketDebuggerUrl) {
  const socket = new WebSocket(webSocketDebuggerUrl);
  await new Promise((resolve, reject) => {
    socket.addEventListener("open", resolve, { once: true });
    socket.addEventListener("error", reject, { once: true });
  });
  let sequence = 0;
  const pending = new Map();
  socket.addEventListener("message", (event) => {
    const message = JSON.parse(event.data);
    const waiter = pending.get(message.id);
    if (!waiter) return;
    pending.delete(message.id);
    if (message.error) waiter.reject(new Error(message.error.message));
    else waiter.resolve(message.result);
  });
  return {
    socket,
    send(method, params = {}) {
      const id = ++sequence;
      socket.send(JSON.stringify({ id, method, params }));
      return new Promise((resolve, reject) => {
        const timer = setTimeout(() => {
          pending.delete(id);
          reject(new Error(`CDP timed out: ${method}`));
        }, 20000);
        pending.set(id, {
          resolve: (value) => { clearTimeout(timer); resolve(value); },
          reject: (error) => { clearTimeout(timer); reject(error); },
        });
      });
    },
  };
}

let failure;
let relayNavigation;
try {
  await getJson(`http://127.0.0.1:${debugPort}/json/version`);
  const initialTargets = await getJson(`http://127.0.0.1:${debugPort}/json/list`);
  const initialPage = initialTargets.find((item) => item.type === "page");
  if (!initialPage?.webSocketDebuggerUrl) throw new Error("Browser page is unavailable.");
  const navigation = await connectCdp(initialPage.webSocketDebuggerUrl);
  try {
    if (forceRelay) {
      relayNavigation = navigation;
      await navigation.send("Page.enable");
      await navigation.send("Page.addScriptToEvaluateOnNewDocument", {
        source: `(() => {
          const OriginalPeerConnection = window.RTCPeerConnection;
          const transport = ${JSON.stringify(turnTransport)};
          const configure = configuration => ({
            ...configuration,
            iceTransportPolicy: 'relay',
            iceServers: (configuration?.iceServers ?? []).map(server => ({
              ...server,
              urls: [server.urls].flat().filter(url => !transport || url.includes('transport=' + transport))
            })).filter(server => server.urls.length)
          });
          window.__spaceSimPeerConnections = [];
          window.RTCPeerConnection = class extends OriginalPeerConnection {
            constructor(configuration, constraints) {
              super(configure(configuration), constraints);
              window.__spaceSimPeerConnections.push(this);
            }
            setConfiguration(configuration) { super.setConfiguration(configure(configuration)); }
          };
        })();`,
      });
    }
    await navigation.send("Page.navigate", { url: appUrl });
  } finally {
    if (!forceRelay) navigation.socket.close();
  }
  if (process.env.PIXEL_STREAMING_USERNAME && process.env.PIXEL_STREAMING_PASSWORD) {
    const loginTargets = await getJson(`http://127.0.0.1:${debugPort}/json/list`);
    const loginPage = loginTargets.find((item) => item.id === initialPage.id);
    if (!loginPage?.webSocketDebuggerUrl) throw new Error("Login page is unavailable.");
    const loginCdp = await connectCdp(loginPage.webSocketDebuggerUrl);
    try {
      const readyDeadline = Date.now() + 20000;
      let pageReady = false;
      while (Date.now() < readyDeadline) {
        try {
          const ready = await loginCdp.send("Runtime.evaluate", {
            expression: `location.origin === ${JSON.stringify(new URL(appUrl).origin)} && document.readyState !== 'loading'`,
            returnByValue: true,
          });
          pageReady = ready.result?.value === true;
          if (pageReady) break;
        } catch {
        }
        await delay(250);
      }
      if (!pageReady) throw new Error("Browser did not finish loading the operator page before login.");
      const login = await loginCdp.send("Runtime.evaluate", {
        expression: `(async () => {
          const response = await fetch('/api/auth/login', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(${JSON.stringify({ username: process.env.PIXEL_STREAMING_USERNAME, password: process.env.PIXEL_STREAMING_PASSWORD })})
          });
          return response.status;
        })()`,
        awaitPromise: true,
        returnByValue: true,
      });
      if (login.exceptionDetails) throw new Error(`Browser login evaluation failed: ${login.exceptionDetails.text}`);
      if (login.result?.value !== 200) throw new Error(`Browser login failed: HTTP ${login.result?.value}`);
      await loginCdp.send("Page.reload");
    } finally {
      loginCdp.socket.close();
    }
  }
  // Use real elapsed time: virtual-time acceleration can terminate WebRTC before ICE settles.
  await delay(warmupMs);
  if (requestedStreamerId) {
    const switchTargets = await getJson(`http://127.0.0.1:${debugPort}/json/list`);
    const page = switchTargets.find((item) => item.type === "page" && item.url === appUrl);
    if (!page?.webSocketDebuggerUrl) throw new Error("Operator page is unavailable for camera switching.");
    const cdp = await connectCdp(page.webSocketDebuggerUrl);
    try {
      const switched = await cdp.send("Runtime.evaluate", {
        expression: `(() => {
          const selector = document.getElementById("streamSelector");
          if (!selector || ![...selector.options].some(option => option.value === ${JSON.stringify(requestedStreamerId)})) return false;
          selector.value = ${JSON.stringify(requestedStreamerId)};
          selector.dispatchEvent(new Event("change", { bubbles: true }));
          return true;
        })()`,
        returnByValue: true,
      });
      if (!switched.result.value) throw new Error(`Streamer is not present in selector: ${requestedStreamerId}`);
    } finally {
      cdp.socket.close();
    }
    await delay(12000);
  }
  const targets = await getJson(`http://127.0.0.1:${debugPort}/json/list`);
  const inspected = [];
  const relayPaths = [];
  let screenshotWritten = false;
  for (const target of targets.filter((item) => item.webSocketDebuggerUrl && ["page", "iframe"].includes(item.type))) {
    const cdp = await connectCdp(target.webSocketDebuggerUrl);
    try {
      if (forceRelay && target.type === "page") {
        const relay = await cdp.send("Runtime.evaluate", {
          expression: `(async () => {
            const paths = [];
            for (const connection of window.__spaceSimPeerConnections ?? []) {
              if (connection.connectionState === 'closed') continue;
              const stats = await connection.getStats();
              for (const transport of stats.values()) {
                if (transport.type !== 'transport' || !transport.selectedCandidatePairId) continue;
                const pair = stats.get(transport.selectedCandidatePairId);
                const local = stats.get(pair?.localCandidateId);
                const remote = stats.get(pair?.remoteCandidateId);
                paths.push({ state: pair?.state, localType: local?.candidateType,
                  remoteType: remote?.candidateType, relayProtocol: local?.relayProtocol,
                  bytesReceived: pair?.bytesReceived ?? 0 });
              }
            }
            return paths;
          })()`,
          awaitPromise: true,
          returnByValue: true,
        });
        relayPaths.push(...(relay.result?.value ?? []));
      }
      await cdp.send("Page.enable");
      const frameTree = await cdp.send("Page.getFrameTree");
      const frameIds = [];
      const collectFrameIds = (node) => {
        if (!node?.frame?.id) return;
        frameIds.push(node.frame.id);
        for (const child of node.childFrames ?? []) collectFrameIds(child);
      };
      collectFrameIds(frameTree.frameTree);
      for (const frameId of frameIds) {
        try {
          const world = await cdp.send("Page.createIsolatedWorld", {
            frameId,
            worldName: "bsk-pixel-streaming-smoke",
          });
          const result = await cdp.send("Runtime.evaluate", {
            contextId: world.executionContextId,
            expression: `(async () => {
          const video = document.querySelector("video");
          const initialFrames = video?.getVideoPlaybackQuality().totalVideoFrames ?? 0;
          const initialTime = video?.currentTime ?? 0;
          await new Promise(resolve => setTimeout(resolve, 3000));
          let frame = null;
          if (video && video.readyState >= 2 && video.videoWidth > 0 && video.videoHeight > 0) {
            const canvas = document.createElement("canvas");
            canvas.width = 64;
            canvas.height = 36;
            const context = canvas.getContext("2d", { willReadFrequently: true });
            context.drawImage(video, 0, 0, canvas.width, canvas.height);
            const pixels = context.getImageData(0, 0, canvas.width, canvas.height).data;
            let sum = 0;
            let maximum = 0;
            for (let index = 0; index < pixels.length; index += 4) {
              const luminance = 0.2126 * pixels[index] + 0.7152 * pixels[index + 1] + 0.0722 * pixels[index + 2];
              sum += luminance;
              maximum = Math.max(maximum, luminance);
            }
            frame = { meanLuminance: sum / (pixels.length / 4), maxLuminance: maximum };
          }
          const rect = video?.getBoundingClientRect();
          return {
            url: location.href,
            video: video ? {
              readyState: video.readyState,
              videoWidth: video.videoWidth,
              videoHeight: video.videoHeight,
              currentTime: video.currentTime,
              advancedSeconds: video.currentTime - initialTime,
              decodedFrames: video.getVideoPlaybackQuality().totalVideoFrames,
              advancedFrames: video.getVideoPlaybackQuality().totalVideoFrames - initialFrames,
              paused: video.paused,
              display: getComputedStyle(video).display,
              visibility: getComputedStyle(video).visibility,
              opacity: getComputedStyle(video).opacity,
              rect: rect ? { x: rect.x, y: rect.y, width: rect.width, height: rect.height } : null,
              frame
            } : null,
            frameState: document.getElementById("frameState")?.textContent ?? null
          };
        })()`,
            awaitPromise: true,
            returnByValue: true,
          });
          const details = result.result.value;
          if (details?.url) details.url = safeUrl(details.url);
          inspected.push({ targetType: target.type, targetUrl: safeUrl(target.url), ...details });
        } catch (error) {
          inspected.push({ targetType: target.type, targetUrl: safeUrl(target.url), inspectionError: error.message });
        }
      }
      if (!screenshotWritten && target.url === appUrl) {
        const capture = await cdp.send("Page.captureScreenshot", { format: "png" });
        await writeFile(path.join(runDir, "platform.png"), Buffer.from(capture.data, "base64"));
        screenshotWritten = true;
      }
    } finally {
      cdp.socket.close();
    }
  }
  const liveVideo = inspected.find((item) => item?.video
    && item.video.readyState >= 2
    && item.video.videoWidth > 0
    && item.video.videoHeight > 0
    && item.video.advancedSeconds > 0
    && item.video.advancedFrames > 0);
  const relayVerified = relayPaths.some(candidate => candidate.state === "succeeded"
    && candidate.localType === "relay" && candidate.bytesReceived > 0
    && (!turnTransport || candidate.relayProtocol === turnTransport));
  console.log(JSON.stringify({ ok: Boolean(liveVideo) && (!forceRelay || relayVerified),
    requestedStreamerId, forceRelay, turnTransport, relayPaths, runDir, inspected }, null, 2));
  if (!liveVideo) throw new Error("Pixel Streaming player connected but no decoded video frame was observed.");
  if (forceRelay && !relayVerified) throw new Error("Live video did not use the required TURN relay path.");
} catch (error) {
  failure = error;
} finally {
  relayNavigation?.socket.close();
  if (!chrome.killed) chrome.kill();
}

if (failure) {
  console.error(failure.stack ?? String(failure));
  process.exitCode = 1;
}
