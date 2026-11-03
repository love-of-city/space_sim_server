// Synchronize recorded sample navigation, videos and the playback clock.
export function createDatasetPlayback({ state, $, clock, fmt, stageNames, notice,
  performance = globalThis.performance, requestAnimationFrame = globalThis.requestAnimationFrame,
}) {
  function nearest(time, a = state.data.times) {
    let l = 0,
      r = a.length - 1;
    while (l < r) {
      const m = (l + r) >> 1;
      if (a[m] < time) l = m + 1;
      else r = m;
    }
    return l > 0 && Math.abs(a[l - 1] - time) < Math.abs(a[l] - time) ? l - 1 : l;
  }
  function setIndex(index, seek = true) {
    if (!state.data) return;
    state.index = Math.max(0, Math.min(state.data.count - 1, Math.round(index)));
    const d = state.data;
    const t = d.times[state.index];
    $("timeline").value = state.index;
    $("current-time").textContent = clock(t);
    $("source-frame").textContent =
      `源帧 ${d.frames[state.index] || "—"} · 样本 ${state.index + 1} / ${d.count}`;
    $("stage").textContent =
      "阶段 " +
      (stageNames[d.stages[state.index]] || d.stages[state.index] || "未记录");
    $("sim-time").textContent =
      `仿真时间 ${fmt(Number(d.start_sim_time_ns) / 1e9 + t, 3)} s`;
    if (seek && d.sync_ok)
      for (const video of state.videos)
        if (video.readyState >= 1) video.currentTime = state.index / d.fps;
    for (const chart of state.charts) chart.draw();
  }
  function pause() {
    state.playing = false;
    for (const v of state.videos) v.pause();
    $("play").textContent = "▶";
    $("play").setAttribute("aria-label", "播放");
  }
  async function play() {
    if (!state.data) return;
    if (state.playing) {
      pause();
      return;
    }
    if (state.index >= state.data.count - 1) setIndex(0);
    state.anchor = performance.now();
    state.anchorTime = state.data.times[state.index];
    state.playing = true;
    $("play").textContent = "Ⅱ";
    $("play").setAttribute("aria-label", "暂停");
    if (state.data.sync_ok && state.videos.length) {
      try {
        await Promise.all(
          state.videos.map((v) => {
            v.playbackRate = Number($("speed").value);
            return v.play();
          }),
        );
      } catch (e) {
        pause();
        notice("视频尚未就绪或播放受阻，请稍后重试。" + e.message);
        return;
      }
    }
    requestAnimationFrame(tick);
  }
  function tick(now) {
    if (!state.playing) return;
    let index;
    if (state.data.sync_ok && state.videos.length) {
      const master = state.videos[0];
      index = Math.floor(master.currentTime * state.data.fps + 1e-5);
      for (const video of state.videos.slice(1))
        if (
          !video.seeking &&
          Math.abs(video.currentTime - master.currentTime) > 0.08
        )
          video.currentTime = master.currentTime;
    } else
      index = nearest(
        state.anchorTime +
          ((now - state.anchor) / 1000) * Number($("speed").value),
      );
    if (index !== state.index) setIndex(index, false);
    if (state.index >= state.data.count - 1) {
      pause();
      return;
    }
    requestAnimationFrame(tick);
  }
  function setRange(start, end) {
    if (!state.data) return;
    const max = Math.max(state.data.duration, 1 / state.data.fps);
    start = Math.max(0, Number(start));
    end = Math.min(max, Number(end));
    if (
      !Number.isFinite(start) ||
      !Number.isFinite(end) ||
      end - start < 1 / state.data.fps
    ) {
      notice("时间范围无效，请确保终点晚于起点至少一帧。");
      return;
    }
    state.range = [start, end];
    updateRangeFields();
    for (const chart of state.charts) chart.rebuild();
  }
  function updateRangeFields() {
    $("range-start").value = state.range[0].toFixed(3);
    $("range-end").value = state.range[1].toFixed(3);
    $("range-label").textContent =
      `${fmt(state.range[0], 2)} — ${fmt(state.range[1], 2)} s`;
  }

  return { nearest, setIndex, pause, play, tick, setRange, updateRangeFields };

}
