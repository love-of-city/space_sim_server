// Measure received, decoded and displayed frames on their actual clocks.
export function createVideoDiagnostics({ state, $, NumericParameters, document = globalThis.document, performance = globalThis.performance }) {
  function resetWebRtcStats() {
    state.playbackSample = null;
    state.playbackHealth = "unknown";
    state.playbackAlert = false;
    state.keyframeRecoveryAttempts = 0;
    state.lastKeyframeRequestAt = null;
    $("webrtcDecodeFps").textContent = "—";
    $("webrtcPlayback").textContent = "等待视频";
    state.lastPacketSample = null;
    state.videoDiagnostics = [];
    state.lastReceivedFrames = null;
    state.lastReceivedTimestamp = null;
    $("webrtcReceiveFps").textContent = "—";
    state.lastPresentedFrames = null;
    state.lastPresentedTimestamp = null;
    state.lastQpSample = null;
    $("webrtcQp").textContent = "—";
    $("webrtcRtt").textContent = "—";
    $("webrtcBitrate").textContent = "—";
    $("webrtcLoss").textContent = "—";
    $("webrtcFps").textContent = "—";
    $("webrtcResolution").textContent = "—";
  }

  function videoFpsOptions() {
    const maximum = Number(state.streamConfig?.pixel_streaming_fps ?? 90);
    return [...new Set([30, 60, 90, 120, maximum])]
      .filter(value => Number.isFinite(value) && value > 0 && value <= maximum)
      .sort((left, right) => left - right);
  }

  function configureVideoFps() {
    const options = videoFpsOptions();
    // Most browser/RDP displays present 60 Hz; keep higher server-supported
    // choices available without decoding/rendering discarded frames by default.
    if (!options.includes(state.requestedVideoFps)) state.requestedVideoFps = options.filter(fps => fps <= 60).at(-1) ?? options[0];
    const selector = $("videoFpsSelector");
    selector.replaceChildren();
    for (const fps of options) {
      const option = document.createElement("option");
      option.value = String(fps);
      option.textContent = `${fps} FPS`;
      selector.append(option);
    }
    selector.value = String(state.requestedVideoFps);
  }

  function selectVideoFps(value) {
    const fps = Number(value);
    if (!videoFpsOptions().includes(fps)) return false;
    state.requestedVideoFps = fps;
    state.pixelConfig?.setNumericSetting(NumericParameters.WebRTCFPS, fps);
    return true;
  }

  function samplePacketLoss(video) {
    const sample = {
      received: video.packetsReceived, lost: video.packetsLost, timestamp: video.timestamp,
      id: video.id, ssrc: video.ssrc,
    };
    const previous = state.lastPacketSample;
    if (![sample.received, sample.lost, sample.timestamp].every(Number.isFinite)) {
      state.lastPacketSample = null;
      return null;
    }
    state.lastPacketSample = sample;
    if (!previous || sample.id !== previous.id || sample.ssrc !== previous.ssrc
        || sample.timestamp <= previous.timestamp || sample.received < previous.received) return null;
    const lost = Math.max(0, sample.lost - previous.lost);
    const total = sample.received - previous.received + lost;
    return total > 0 ? lost * 100 / total : null;
  }

  // qpSum is cumulative across decoded frames. Difference consecutive samples so
  // a long clear period cannot conceal a recent compression-quality drop.
  function sampleAverageQp(video) {
    if (!video || !Number.isFinite(video.qpSum) || video.qpSum < 0
        || !Number.isFinite(video.framesDecoded) || video.framesDecoded < 0
        || !Number.isFinite(video.timestamp)) {
      state.lastQpSample = null;
      return null;
    }
    const sample = {
      qpSum: video.qpSum, framesDecoded: video.framesDecoded, timestamp: video.timestamp,
      id: video.id, ssrc: video.ssrc, trackIdentifier: video.trackIdentifier, codecId: video.codecId,
    };
    const previous = state.lastQpSample;
    state.lastQpSample = sample;
    if (!previous || ["id", "ssrc", "trackIdentifier", "codecId"].some(key => sample[key] !== previous[key])
        || sample.timestamp <= previous.timestamp || sample.framesDecoded <= previous.framesDecoded
        || sample.qpSum < previous.qpSum) return null;
    const average = (sample.qpSum - previous.qpSum) / (sample.framesDecoded - previous.framesDecoded);
    return Number.isFinite(average) ? average : null;
  }

  function samplePlayback(video, player, now) {
    const sample = {
      id: video.id, ssrc: video.ssrc, timestamp: video.timestamp,
      decoded: video.framesDecoded, received: video.framesReceived, currentTime: player?.currentTime,
    };
    const previous = state.playbackSample;
    state.playbackSample = sample;
    const validInterval = previous && Number.isFinite(sample.timestamp) && Number.isFinite(previous.timestamp)
      && sample.timestamp > previous.timestamp && sample.id === previous.id && sample.ssrc === previous.ssrc;
    let decodedFps = null;
    if (validInterval && Number.isFinite(sample.decoded) && Number.isFinite(previous.decoded)
        && sample.decoded >= previous.decoded) {
      decodedFps = (sample.decoded - previous.decoded) * 1000 / (sample.timestamp - previous.timestamp);
    }
    if (!validInterval || decodedFps == null || document.hidden) {
      state.lastDecodeProgressAt = now;
      state.lastMediaProgressAt = now;
      state.keyframeRecoveryAttempts = 0;
      state.lastKeyframeRequestAt = null;
    }
    if (decodedFps > 0) state.lastDecodeProgressAt = now;
    const hasMediaClock = Number.isFinite(sample.currentTime) && Number.isFinite(previous?.currentTime);
    if (!hasMediaClock || sample.currentTime !== previous.currentTime) state.lastMediaProgressAt = now;
    const receiving = validInterval && Number.isFinite(sample.received) && Number.isFinite(previous.received)
      && sample.received > previous.received;
    let health = "waiting";
    if (document.hidden) health = "hidden";
    else if (player?.error) health = "media-error";
    else if (player?.paused === true) health = "paused";
    else if (receiving && decodedFps === 0 && now - state.lastDecodeProgressAt >= 5000) health = "decode-stalled";
    else if (decodedFps > 0 && hasMediaClock && now - state.lastMediaProgressAt >= 5000) health = "playback-stalled";
    else if (decodedFps > 0) health = "playing";
    if (health === "playing") {
      state.keyframeRecoveryAttempts = 0;
      state.lastKeyframeRequestAt = null;
    }
    if (health === "decode-stalled" && state.streamLive && state.keyframeRecoveryAttempts < 2
        && (state.lastKeyframeRequestAt == null || now - state.lastKeyframeRequestAt >= 15000)) {
      state.lastKeyframeRequestAt = now;
      state.keyframeRecoveryAttempts++;
      try { state.pixelStreaming?.requestIframe?.(); } catch (_) { }
    }
    state.playbackHealth = health;
    return { decodedFps, health };
  }

  function updateWebRtcStats(aggregatedStats) {
    const pair = aggregatedStats?.getActiveCandidatePair?.();
    const video = aggregatedStats?.inboundVideoStats;
    const averageQp = sampleAverageQp(video);
    $("webrtcQp").textContent = averageQp == null ? "—" : averageQp.toFixed(1);
    if (!video) return;
    const packetLoss = samplePacketLoss(video);
    // Received frames and displayed frames are different counters. A 60 Hz client
    // can receive 90 FPS without presenting 90 unique frames on its screen.
    const receivedFrames = Number(video.framesReceived);
    const receivedTimestamp = Number(video.timestamp);
    let receivedFps = null;
    if (Number.isFinite(receivedFrames) && Number.isFinite(receivedTimestamp)) {
      if (state.lastReceivedTimestamp != null && receivedTimestamp > state.lastReceivedTimestamp
          && receivedFrames >= state.lastReceivedFrames) {
        receivedFps = (receivedFrames - state.lastReceivedFrames) * 1000
          / (receivedTimestamp - state.lastReceivedTimestamp);
      }
      state.lastReceivedFrames = receivedFrames;
      state.lastReceivedTimestamp = receivedTimestamp;
    }
    $("webrtcReceiveFps").textContent = receivedFps == null ? "—" : receivedFps.toFixed(0);
    // WebRTC's inbound framesPerSecond is the receive/decode rate. It can be
    // much higher than what the browser actually presents because decoded frames
    // may be dropped by the compositor. Measure non-dropped video frames instead.
    const player = $("pixelStream").querySelector("video");
    const quality = player?.getVideoPlaybackQuality?.();
    const totalVideoFrames = Number(quality?.totalVideoFrames ?? player?.webkitDecodedFrameCount);
    const droppedVideoFrames = Number(quality?.droppedVideoFrames ?? player?.webkitDroppedFrameCount ?? 0);
    const presentedFrames = totalVideoFrames - droppedVideoFrames;
    const presentedTimestamp = performance.now();
    const playback = samplePlayback(video, player, presentedTimestamp);
    $("webrtcDecodeFps").textContent = playback.decodedFps == null ? "—" : playback.decodedFps.toFixed(0);
    const playbackLabels = {
      waiting: "等待新帧", hidden: "页面在后台", playing: "播放中", paused: "已暂停，请播放",
      "decode-stalled": "接收正常，解码停滞", "playback-stalled": "解码正常，播放停滞",
      "media-error": "播放器错误，请重连",
    };
    $("webrtcPlayback").textContent = playbackLabels[playback.health];
    const playbackFault = ["paused", "decode-stalled", "playback-stalled", "media-error"].includes(playback.health);
    if (playbackFault) {
      $("playStream").textContent = playback.health === "paused" ? "继续播放实时画面" : "重新连接恢复画面";
      $("playStream").hidden = false;
      $("frameState").textContent = "VIDEO PAUSED / STALLED";
      state.playbackAlert = true;
    } else if (playback.health === "playing" && state.playbackAlert) {
      $("playStream").hidden = true;
      if (state.streamLive) $("frameState").textContent = "LIVE WEBRTC";
      state.playbackAlert = false;
    }
    let fps = null;
    if (Number.isFinite(presentedFrames) && presentedFrames >= 0) {
      if (state.lastPresentedTimestamp != null
          && presentedTimestamp > state.lastPresentedTimestamp
          && presentedFrames >= state.lastPresentedFrames) {
        fps = (presentedFrames - state.lastPresentedFrames) * 1000
          / (presentedTimestamp - state.lastPresentedTimestamp);
      }
      state.lastPresentedTimestamp = presentedTimestamp;
      state.lastPresentedFrames = presentedFrames;
    }
    $("webrtcRtt").textContent = pair?.currentRoundTripTime != null
      ? `${(pair.currentRoundTripTime * 1000).toFixed(0)} ms` : "—";
    $("webrtcBitrate").textContent = video.bitrate != null
      ? `${Number(video.bitrate).toFixed(0)} kbps` : "—";
    $("webrtcLoss").textContent = packetLoss == null ? "—" : `${packetLoss.toFixed(1)}%`;
    $("webrtcFps").textContent = fps == null ? "—" : fps.toFixed(0);
    $("webrtcResolution").textContent = video.frameWidth && video.frameHeight
      ? `${video.frameWidth}×${video.frameHeight}` : "—";
    const local = aggregatedStats.localCandidates?.find(candidate => candidate.id === pair?.localCandidateId);
    const remote = aggregatedStats.remoteCandidates?.find(candidate => candidate.id === pair?.remoteCandidateId);
    state.videoDiagnostics ??= [];
    state.videoDiagnostics.push({
      time: new Date().toISOString(), targetFps: state.requestedVideoFps ?? null,
      receivedFps, decodedFps: playback.decodedFps, displayedFps: fps, bitrateKbps: video.bitrate ?? null,
      playbackHealth: playback.health, paused: player?.paused ?? null, readyState: player?.readyState ?? null,
      mediaTime: player?.currentTime ?? null, mediaErrorCode: player?.error?.code ?? null,
      visibility: document.hidden ? "hidden" : "visible", keyFramesDecoded: video.keyFramesDecoded ?? null,
      pliCount: video.pliCount ?? null, decoderImplementation: video.decoderImplementation ?? null,
      keyframeRecoveryAttempts: state.keyframeRecoveryAttempts,
      rttMs: pair?.currentRoundTripTime == null ? null : pair.currentRoundTripTime * 1000,
      lossPercent: packetLoss, averageQp, jitterMs: video.jitter == null ? null : video.jitter * 1000,
      localCandidateType: local?.candidateType ?? null, remoteCandidateType: remote?.candidateType ?? null,
      protocol: local?.protocol ?? null, relayProtocol: local?.relayProtocol ?? null,
      width: video.frameWidth ?? null, height: video.frameHeight ?? null,
    });
    if (state.videoDiagnostics.length > 60) state.videoDiagnostics.shift();
  }

  return { resetWebRtcStats, videoFpsOptions, configureVideoFps, selectVideoFps, samplePacketLoss, updateWebRtcStats };

}
