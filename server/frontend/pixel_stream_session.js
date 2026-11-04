import { resolvePreviewStreams } from './preview_streams.js';
import { observePreviewAvailability } from './preview_availability.js';
export function resolveSignallingUrl(config, location) {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  const url = new URL(config.pixel_streaming_signalling_url
    || `${scheme}//${location.hostname}:${config.pixel_streaming_player_port}`, location.href);
  if (config.pixel_streaming_access_token) {
    url.searchParams.set("token", config.pixel_streaming_access_token);
  }
  return url.toString();
}

// One owned Pixel Streaming player, negotiation deadline and bounded reconnection.
export function createPixelStreamSession({ state, $, apiRequest, setFreeCameraMode,
  setPixelStreamingInputEnabled, setMessage, diagnostics,
  observeAvailability = observePreviewAvailability,
  mainCompanion = {dispose() {}, update() {}}, workLight = {available: false, reset() {}, query() {}, receive() {}},
  updateWorkLight = () => {}, requestRelativeView = () => {}, updateRelativeViewControl = () => {},
  previewOrbit = {select() {}, restore() {}, key: ""},
  Config, Flags, OptionParameters, NumericParameters, PixelStreaming, TextParameters,
  document = globalThis.document, window = globalThis.window, location = globalThis.location,
  setTimeout = globalThis.setTimeout, clearTimeout = globalThis.clearTimeout,
}) {
  const { resetWebRtcStats, configureVideoFps, updateWebRtcStats } = diagnostics;
  async function connectPixelStreaming() {
    if (!state.currentUser || (state.activePage && state.activePage !== "operate")) return;
    // Reconnect callbacks, the 10-second startup watchdog, and manual reconnect
    // can otherwise overlap.  That creates two Pixel Streaming players for the
    // main viewport and makes the displayed stream flash while old participants
    // are being removed.
    if (state.pixelConnectPromise) return state.pixelConnectPromise;
    let pending;
    pending = (async () => {
      try {
      const accessKey = new URLSearchParams(location.search).get("access_key");
      const response = await apiRequest("/api/client-config", {
        cache: "no-store",
        headers: accessKey ? { "X-Space-Arm-Access-Key": accessKey } : {},
      });
      if (!state.currentUser || (state.activePage && state.activePage !== "operate")) return;
      if (response.status === 401) {
        disposePixelStream();
        state.streamConfig = null;
        $("previewEmpty").style.display = "grid";
        $("previewEmptyTitle").textContent = "访问链接缺少密钥或密钥无效";
        $("previewEmptyHint").textContent = "账号已登录。请从服务器访问窗口复制完整链接，包含 access_key，再在本机浏览器打开。";
        $("frameState").textContent = "ACCESS KEY REQUIRED";
        setMessage("登录成功，但视频和控制需要有效访问密钥。这不是密码错误，请使用包含 access_key 的完整访问链接。");
        return;
      }
      if (!response.ok) throw new Error(`config ${response.status}`);
      const config = await response.json();
      state.streamConfig = config;
      configureVideoFps();
      updatePreviewStreams(null);
      createPixelStream();
      } catch (error) {
        console.warn("Pixel Streaming configuration is unavailable", error);
        $("frameState").textContent = "WEBRTC OFFLINE";
      if (state.currentUser) schedulePixelStreamingReconnect();
      }
    })().finally(() => {
      if (state.pixelConnectPromise === pending) state.pixelConnectPromise = null;
    });
    state.pixelConnectPromise = pending;
    return pending;
  }

  function signallingUrl() {
    return resolveSignallingUrl(state.streamConfig, location);
  }

  function disposePixelStream() {
    state.previewAvailability?.close();
    state.previewAvailability = null;
    state.previewAvailableIds = null;
    mainCompanion.dispose();
    if (state.freeCameraMode) setFreeCameraMode(false);
    if (state.streamMediaTimer != null) {
      clearTimeout(state.streamMediaTimer);
      state.streamMediaTimer = null;
    }
    if (state.streamReconnectTimer != null) {
      clearTimeout(state.streamReconnectTimer);
      state.streamReconnectTimer = null;
    }
    const previousStream = state.pixelStreaming;
    try {
      setPixelStreamingInputEnabled(false, false);
    } catch (_) { /* best effort */ }
    // Invalidate ownership BEFORE disconnect(): the SDK can synchronously emit
    // webRtcDisconnected here. A retired player must never schedule a reconnect
    // or overwrite the new player's LIVE state/stats.
    state.pixelStreaming = null;
    state.pixelConfig = null;
    try {
      previousStream?.disconnect();
    } catch (_) { /* best effort */ }
    state.freeCameraMode = false;
    state.streamLive = false;
    $("pixelStream").replaceChildren();
    resetWebRtcStats();
  }

  function schedulePixelStreamingReconnect(delayMs = 1500) {
    if (state.streamReconnectTimer != null) clearTimeout(state.streamReconnectTimer);
    state.streamReconnectTimer = setTimeout(() => {
      state.streamReconnectTimer = null;
      connectPixelStreaming();
    }, delayMs);
  }

  function armPixelStreamingWatchdog(stream, delayMs = 60000) {
    if (state.streamReconnectTimer != null) clearTimeout(state.streamReconnectTimer);
    state.streamReconnectTimer = setTimeout(() => {
      state.streamReconnectTimer = null;
      if (state.pixelStreaming === stream && !state.streamLive) connectPixelStreaming();
    }, delayMs);
  }

  function createPixelStream() {
    disposePixelStream();
    mainCompanion.update(state.selectedStreamerId,state.streamConfig?.pixel_streaming_streamer_id);
    $("frameState").textContent = "CONNECTING";
    $("frameState").style.color = "var(--danger)";
    $("previewEmpty").style.display = "grid";
    $("previewEmptyTitle").textContent = "等待实时画面";
    $("previewEmptyHint").textContent = "UE 与 BSK/MJScene 就绪后，WebRTC 视频将在此显示";
    $("playStream").hidden = true;
    const config = new Config({
      initialSettings: {
        [TextParameters.SignallingServerUrl]: signallingUrl(),
        [OptionParameters.StreamerId]: state.selectedStreamerId,
        [NumericParameters.WebRTCFPS]: state.requestedVideoFps ?? state.streamConfig?.pixel_streaming_fps ?? 90,
        // Match the UE launch setting; nullish fallback must preserve an explicit 0.
        [NumericParameters.MinQuality]: state.streamConfig?.pixel_streaming_encoder_min_quality ?? 60,
        [Flags.AutoConnect]: true,
        [Flags.AutoPlayVideo]: true,
        [Flags.StartVideoMuted]: true,
        [Flags.WaitForStreamer]: true,
        [Flags.HoveringMouseMode]: false,
        // Retain SDK keyboard input for non-camera UE shortcuts only. Camera
        // keys use physical codes and explicit BskCameraInput commands.
        [Flags.KeyboardInput]: true,
        [Flags.MouseInput]: false,
        [Flags.TouchInput]: false,
        [Flags.GamepadInput]: false,
      },
    });
    const stream = new PixelStreaming(config, { videoElementParent: $("pixelStream") });
    state.relativeView = null;
    updateRelativeViewControl();
    state.pixelConfig = config;
    state.pixelStreaming = stream;
    const applyAvailableStreams = available => {
      if (state.pixelStreaming !== stream) return;
      const previous = state.selectedStreamerId;
      updatePreviewStreams(available);
      if (previous !== state.selectedStreamerId && available.includes(state.selectedStreamerId)) {
        config.setOptionSettingValue(OptionParameters.StreamerId, state.selectedStreamerId);
      }
    };
    state.previewAvailability = observeAvailability(signallingUrl(), available => {
      if (state.pixelStreaming !== stream) return;
      state.previewAvailableIds = available;
      applyAvailableStreams(available);
    });
    let lastRelativeQuery = 0;
    stream.addResponseEventListener("relative-view", response => {
      if (state.pixelStreaming !== stream) return;
      let data;
      try { data = JSON.parse(response); } catch { return; }
      if (data?.type !== "BskRelativeView" || !["", "overview", "overview-fixed", "chaser", "target", "free"].includes(data.mode)) return;
      state.relativeView = data;
      if (["overview", "overview-fixed", "chaser", "target"].includes(data.mode)) $("relativeViewSelector").value = data.mode;
      updateRelativeViewControl();
    });
    stream.addEventListener("dataChannelOpen", () => {
      if (state.pixelStreaming === stream) requestRelativeView();
    });
    stream.addEventListener("dataChannelClose", () => {
      if (state.pixelStreaming !== stream) return;
      state.streamLive = false;
      if (state.freeCameraMode) setFreeCameraMode(false);
      $("frameState").textContent = "CHANNEL RECONNECTING";
      updateRelativeViewControl();
      schedulePixelStreamingReconnect();
    });
    workLight.reset();
    updateWorkLight();
    stream.addResponseEventListener("work-light", response => {
      if (state.pixelStreaming !== stream) return;
      workLight.receive(response);
      updateWorkLight();
    });
    stream.addResponseEventListener("preview-orbit", response => {
      if(state.pixelStreaming !== stream)return;
      try {
        const reply=JSON.parse(response);
        if(reply.type==='bsk_preview_orbit' && Number.isFinite(reply.yaw) && Number.isFinite(reply.pitch))
          setMessage(reply.yaw===0 && reply.pitch===0 ? '已恢复当前视角'
            : `展示视角偏转 ${reply.yaw.toFixed(1)}° / ${reply.pitch.toFixed(1)}°；Home 恢复，卫星姿态不变`);
      } catch (_) { /* Ignore unrelated renderer responses. */ }
    });
    stream.addEventListener("initialSettings", () => {
      queueMicrotask(() => {
        if (state.pixelStreaming === stream && state.requestedVideoFps != null) {
          config.setNumericSetting(NumericParameters.WebRTCFPS, state.requestedVideoFps);
        }
      });
    });
    window.__pixelStream = stream;
    stream.addEventListener("webRtcConnecting", () => {
      if (state.pixelStreaming !== stream) return;
      $("frameState").textContent = "CONNECTING";
      // Give negotiation its own deadline, independent of how long UE took to load.
      armPixelStreamingWatchdog(stream);
    });
    stream.addEventListener("streamerListMessage", (event) => {
      if (state.pixelStreaming !== stream) return;
      const available = event?.data?.messageStreamerList?.ids;
      if (Array.isArray(available)) {
        applyAvailableStreams(state.previewAvailableIds ?? available);
      }
      if (state.streamLive) return;
      // Healthy signalling / streamer discovery is progress, not a reason to
      // periodically replace a player while the renderer is coming online.
      armPixelStreamingWatchdog(stream);
    });
    stream.addEventListener("webRtcConnected", () => {
      if (state.pixelStreaming !== stream) return;
      // WebRTC connection is already the authoritative readiness signal for the
      // main viewport.  Waiting only for videoInitialized made the watchdog
      // tear down a healthy player every 10 seconds in Pixel Streaming 2, which
      // produced repeated PlayerN connections and visible main-view flashing.
      state.streamLive = true;
      if (state.streamReconnectTimer != null) {
        clearTimeout(state.streamReconnectTimer);
        state.streamReconnectTimer = null;
      }
      $("frameState").textContent = "MEDIA SETUP";
    });
    stream.addEventListener("videoInitialized", () => {
      if (state.pixelStreaming !== stream) return;
      state.streamLive = true;
      previewOrbit.select(state.selectedStreamerId);
      previewOrbit.restore();
      requestRelativeView();
      // Media setup can precede the first decoded frame during a cold asset
      // load. Keep the waiting view until actual video arrives.
      if (!showReceivedVideo(stream)) {
        $("frameState").textContent = "WAITING FOR FRAMES";
        $("previewEmptyTitle").textContent = "等待视频首帧";
        $("previewEmptyHint").textContent = "首次载入模型时可能较慢，收到实际画面后会自动显示。";
        $("pixelStream").querySelector("video")?.requestVideoFrameCallback?.(() => showReceivedVideo(stream));
      }
    });
    stream.addEventListener("playStreamRejected", () => {
      if (state.pixelStreaming !== stream) return;
      $("playStream").hidden = false;
      $("frameState").textContent = "CLICK TO PLAY";
    });
    stream.addEventListener("webRtcDisconnected", () => {
      if (state.pixelStreaming !== stream) return;
      if (state.freeCameraMode) setFreeCameraMode(false);
      state.streamLive = false;
      $("frameState").textContent = "DISCONNECTED";
      $("frameState").style.color = "var(--danger)";
      resetWebRtcStats();
      if (state.pixelStreaming === stream) schedulePixelStreamingReconnect();
    });
    stream.addEventListener("webRtcFailed", () => {
      if (state.pixelStreaming !== stream) return;
      if (state.freeCameraMode) setFreeCameraMode(false);
      state.streamLive = false;
      $("frameState").textContent = "WEBRTC FAILED";
      $("frameState").style.color = "var(--danger)";
      if (state.pixelStreaming === stream) schedulePixelStreamingReconnect();
    });
    stream.addEventListener("subscribeFailed", (event) => {
      if (state.pixelStreaming !== stream) return;
      if (state.freeCameraMode) setFreeCameraMode(false);
      state.streamLive = false;
      $("frameState").textContent = event?.message || "STREAM UNAVAILABLE";
      if (state.pixelStreaming === stream) schedulePixelStreamingReconnect(1000);
    });
    stream.addEventListener("statsReceived", (event) => {
      if (state.pixelStreaming !== stream) return;
      updateWebRtcStats(event?.data?.aggregatedStats);
      if (state.streamLive) showReceivedVideo(stream);
      // Discover manifests arriving after WebRTC and camera changes made by
      // another viewer. Reuse the SDK cadence instead of adding a timer.
      if (state.streamLive && Date.now() - lastRelativeQuery >= 2000) {
        lastRelativeQuery = Date.now();
        requestRelativeView();
      }
    });
    // Bound a genuinely stalled setup, but never recycle players on an absolute
    // 10-second cycle: UE startup and public ICE negotiation can take much longer.
    armPixelStreamingWatchdog(stream);
    monitorPixelStream(stream);
  }

  function updatePreviewStreams(availableIds) {
    const config = state.streamConfig;
    const configured = config.pixel_streaming_streamers || [{id: config.pixel_streaming_streamer_id, label: "主视口"}];
    const resolved = resolvePreviewStreams(configured, availableIds, state.selectedStreamerId);
    const selector = $("streamSelector");
    selector.replaceChildren();
    for (const item of resolved.streams) {
      const option = document.createElement("option");
      const labels = {WristCamera: "机械臂腕部相机", SpacecraftOverview: "卫星总览相机"};
      option.value = item.id;
      option.textContent = item.id.endsWith("__teleop_camera_capture_wrist_cam")
        ? "抓取臂末端相机" : labels[item.label] || item.label || item.id;
      option.disabled = item.disabled;
      selector.append(option);
    }
    selector.value = resolved.selectedId;
    state.selectedStreamerId = resolved.selectedId;
  }

  function showReceivedVideo(stream) {
    if (state.pixelStreaming !== stream) return false;
    const video = $("pixelStream").querySelector("video");
    const frames = video?.getVideoPlaybackQuality?.().totalVideoFrames;
    if (!video || video.readyState < 2 || !video.videoWidth || video.paused === true || frames === 0) return false;
    $("previewEmpty").style.display = "none";
    $("pixelStream").style.display = "block";
    $("frameState").textContent = "LIVE WEBRTC";
    $("frameState").style.color = "var(--accent)";
    return true;
  }

  function monitorPixelStream(stream) {
    let previousTime;
    let progressedAt = Date.now();
    const check = () => {
      if (state.pixelStreaming !== stream) return;
      const video = $("pixelStream").querySelector("video");
      const mediaTime = video?.currentTime;
      if (!state.streamLive || document.hidden || video?.paused === true
          || (Number.isFinite(mediaTime) && mediaTime !== previousTime)) {
        progressedAt = Date.now();
      }
      previousTime = mediaTime;
      // ICE can remain connected after a renderer restart while the media/data
      // channels have stopped. A historic frame count is not current video.
      // Allow slow cold asset loading without repeatedly interrupting negotiation.
      if (state.streamLive && Date.now() - progressedAt >= 60000) {
        state.streamLive = false;
        if (state.freeCameraMode) setFreeCameraMode(false);
        $("frameState").textContent = "VIDEO RECOVERING";
        $("previewEmpty").style.display = "grid";
        $("previewEmptyTitle").textContent = "视频暂时中断，正在重新连接";
        $("previewEmptyHint").textContent = "仿真继续运行；收到新画面后自动恢复。";
        updateRelativeViewControl();
        schedulePixelStreamingReconnect(0);
        state.streamMediaTimer = null;
        return;
      }
      state.streamMediaTimer = setTimeout(check, 5000);
    };
    state.streamMediaTimer = setTimeout(check, 5000);
  }

  return { connectPixelStreaming, signallingUrl, disposePixelStream, schedulePixelStreamingReconnect, armPixelStreamingWatchdog, createPixelStream };

}
