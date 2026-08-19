import { spawn } from "node:child_process";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";

const appUrl = process.argv[2] ?? "http://127.0.0.1:8000/";
const requestedStreamerId = process.argv[3] ?? "";
const chromePath = process.env.CHROME_PATH
  ?? "C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe";
const debugPort = Number(process.env.CHROME_DEBUG_PORT ?? 9333);
const warmupMs = Number(process.env.PIXEL_STREAMING_WARMUP_MS ?? 15000);
const runDir = path.resolve("run", `pixel-streaming-smoke-${Date.now()}`);
await mkdir(runDir, { recursive: true });

const chrome = spawn(chromePath, [
  "--headless=new",
  "--no-first-run",
  "--autoplay-policy=no-user-gesture-required",
  "--window-size=1440,900",
  `--remote-debugging-port=${debugPort}`,
  `--user-data-dir=${path.join(runDir, "profile")}`,
  appUrl,
], { stdio: "ignore", windowsHide: true });

const delay = (milliseconds) => new Promise((resolve) => setTimeout(resolve, milliseconds));

async function getJson(url, timeoutMs = 20000) {
  const deadline = Date.now() + timeoutMs;
  let lastError;
  while (Date.now() < deadline) {
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
      return new Promise((resolve, reject) => pending.set(id, { resolve, reject }));
    },
  };
}

let failure;
try {
  await getJson(`http://127.0.0.1:${debugPort}/json/version`);
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
  let screenshotWritten = false;
  for (const target of targets.filter((item) => item.webSocketDebuggerUrl && ["page", "iframe"].includes(item.type))) {
    const cdp = await connectCdp(target.webSocketDebuggerUrl);
    try {
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
            expression: `(() => {
          const video = document.querySelector("video");
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
            returnByValue: true,
          });
          inspected.push({ targetType: target.type, targetUrl: target.url, ...result.result.value });
        } catch (error) {
          inspected.push({ targetType: target.type, targetUrl: target.url, inspectionError: error.message });
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
    && item.video.currentTime > 0);
  console.log(JSON.stringify({ ok: Boolean(liveVideo), requestedStreamerId, runDir, inspected }, null, 2));
  if (!liveVideo) throw new Error("Pixel Streaming player connected but no decoded video frame was observed.");
} catch (error) {
  failure = error;
} finally {
  if (!chrome.killed) chrome.kill();
}

if (failure) {
  console.error(failure.stack ?? String(failure));
  process.exitCode = 1;
}
