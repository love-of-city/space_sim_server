import template from './datasets.html?raw';
import {mountView} from '../ui.js';
import {filterRecords,chartCategory} from './dataset_filters.js';
export function createDatasetPage(root,{api,navigate}) {
const $=mountView(root,template);
let viewGeneration=0,listingRequest=0,batchId='',batchIds=null;
const state = {
  active: false,
  group: "joints",
  list: [],
  filter: "all",
  data: null,
  index: 0,
  range: [0, 1],
  charts: [],
  videos: [],
  playing: false,
  degree: true,
  target: true,
  request: 0,
  anchor: 0,
  anchorTime: 0,
};
const colors = [
  "#2563eb",
  "#16805f",
  "#b66d0d",
  "#8755bd",
  "#c84464",
  "#168195",
];
const stageNames = {
  settle: "初始停稳",
  raise: "抬高夹爪",
  above_pickup: "到达取件位上方",
  above_drawer: "到达抽屉上方",
  approach: "连续接近 / 转向 / 张爪",
  align: "调整抓取姿态",
  open: "张开夹爪",
  descend: "下降抓取",
  grip: "夹持",
  lift: "抬起",
  return: "放回抽屉",
  transfer: "搬运",
  above_hole: "到达装配孔上方",
  align_hole: "对孔",
  insert: "插入",
  seat: "保持到位",
  release: "松爪",
  retract: "撤离",
  verify: "停稳检查",
};
const taskName = (r) =>
  /drawer_cycle/.test(r.task || "")
    ? "抽屉取出与放回"
    : /plug_insert/.test(r.task || "")
    ? "圆柱插头搬运与插入"
    : /drawer/.test(r.task || "")
      ? "抽屉提拉任务"
      : r.task === "spacecraft arm teleoperation"
        ? "遥操作采集"
        : r.task || "遥操作采集";
const outcomeName = (s) =>
  ({ success: "成功", failure: "失败", aborted: "已中止", unknown: "未标记" })[
    s
  ] || "未标记";
const fmt = (n, digits = 3) =>
  Number.isFinite(n)
    ? n.toLocaleString("en-US", {
        minimumFractionDigits: digits,
        maximumFractionDigits: digits,
      })
    : "—";
const clock = (seconds) =>
  `${String(Math.floor(seconds / 60)).padStart(2, "0")}:${(seconds % 60).toFixed(3).padStart(6, "0")}`;
function notice(text = "") {
  $("notice").hidden = !text;
  $("notice").textContent = text;
}
function el(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined) e.textContent = text;
  return e;
}
function dateLabel(r) {
  const match = r.id.match(
    /episode-(\d{4})(\d{2})(\d{2})-(\d{2})(\d{2})(\d{2})/,
  );
  return match
    ? `${match[1]}.${match[2]}.${match[3]}  ${match[4]}:${match[5]}:${match[6]}`
    : r.id;
}
function listUrl(){return '/datasets'+(batchId?'?batch='+encodeURIComponent(batchId):'');}
function episodeUrl(id){return '/datasets?episode='+encodeURIComponent(id)+(batchId?'&batch='+encodeURIComponent(batchId):'');}
function showList(){pause();$('list-page').hidden=false;$('workspace').hidden=true;}
function renderList(){
 const matches=filterRecords(state.list,{query:$('search').value,source:$('source').value,outcome:$('outcome-filter').value,task:$('task-filter').value,from:$('date-from').value,to:$('date-to').value,ids:batchIds});
 $('episodes').replaceChildren(...matches.map(record=>{
  const row=el('tr'),identity=el('td');identity.append(el('strong','',taskName(record)),el('small','',dateLabel(record)),el('small','',record.id));
  const source=el('td');source.append(el('span','',record.source==='auto'?'自动采集':'手动录制'),el('small','',record.owner||'未记录采集用户'));
  const result=el('td');result.append(el('span','badge '+(record.outcome==='success'?'success':'neutral'),outcomeName(record.outcome)));
  const duration=el('td');duration.append(el('span','',Number.isFinite(record.frames)&&record.fps>0?fmt(record.frames/record.fps,2)+' s':'—'),el('small','',record.frames==null?'帧数未记录':record.frames.toLocaleString()+' 帧'));
  const action=el('td'),open=el('button','button text-button',record.readable?'回放与分析':'数据未就绪');open.type='button';open.disabled=!record.readable;open.onclick=()=>navigate(episodeUrl(record.id));action.append(open);
  row.append(identity,source,result,duration,action);return row;
 }));
 $('episode-count').textContent=matches.length+' / '+state.list.length+' 条记录';$('list-empty').hidden=matches.length>0;
}
async function refresh(){
 const version=viewGeneration,request=++listingRequest;$('refresh').disabled=true;
 try{
  const result=await api('/api/datasets');if(!state.active||version!==viewGeneration||request!==listingRequest)return;
  state.list=result.episodes;
  const selected=$('task-filter').value;
  $('task-filter').replaceChildren(new Option('全部任务','all'),...[...new Set(state.list.map(record=>record.task).filter(Boolean))].map(task=>new Option(taskName({task}),task)));
  if([...$('task-filter').options].some(option=>option.value===selected))$('task-filter').value=selected;
  $('connection').textContent='平台已连接';renderList();
  if(!result.exists)notice('暂无录制数据。完成一次手动或自动采集后，记录会显示在这里。');
 }catch(error){if(state.active&&version===viewGeneration){notice(error.message);$('connection').textContent='读取失败';}}
 finally{if(version===viewGeneration&&request===listingRequest)$('refresh').disabled=false;}
}
async function loadEpisode(id) {
  pause();
  const request = ++state.request;
  notice("正在读取记录与完整时间序列…");
  $("export").disabled = true;
  try {
    const data = await api("/api/datasets/" + encodeURIComponent(id));
    if (!state.active || request !== state.request) return;
    for (const chart of state.charts) chart.destroy();
    for (const v of state.videos) {
      v.pause();
      v.removeAttribute("src");
      v.load();
    }
    for (const group of data.groups) group.series.forEach((series,index)=>{series.color=colors[index%colors.length];});
    state.data = data;
    state.index = 0;
    state.range = [0, Math.max(data.duration, 1 / data.fps)];
    state.charts = [];
    state.videos = [];
    $("list-page").hidden = true;
    $("workspace").hidden = false;
    $("episode-id").textContent = data.id;
    $("task-name").textContent = taskName(data.metadata);
    $("instruction").textContent =
      data.metadata.instruction || "此记录没有任务说明";
    $("outcome").textContent = outcomeName(data.metadata.outcome);
    $("outcome").className =
      "badge" + (data.metadata.outcome === "success" ? "" : " warning");
    $("stat-duration").textContent = fmt(data.count / data.fps, 2) + " s";
    $("stat-frames").textContent = data.count.toLocaleString();
    $("stat-rate").textContent = fmt(data.fps, 0) + " Hz";
    $("stat-wrench").textContent =
      fmt((100 * data.valid_wrench_count) / data.count, 1) + " %";
    $("timeline").max = data.count - 1;
    $("timeline").value = 0;
    $("total-time").textContent = clock(data.duration);
    $("export").disabled = false;
    $("range-end").max = data.duration;
    $("range-start").max = data.duration;
    renderVideos();
    renderCharts();
    filterCharts();
    updateRangeFields();
    setIndex(0, false);
    renderList();
    notice(data.warning);

  } catch (error) {
    if (state.active && request === state.request) {
      notice(error.message);
      $("export").disabled = !state.data;
    }
  }
}
function renderVideos() {
  const container = $("videos");
  container.replaceChildren();
  const data = state.data;
  $("video-note").textContent = data.videos.length
    ? `${data.videos.length} 路相机 · ${data.sync_ok ? "帧级时间轴联动" : "独立视频播放"}`
    : "无视频 · 仍可回放数值";
  if (!data.videos.length) {
    container.append(
      el("div", "empty", "这段记录没有可读取的 MP4 视频，数值曲线仍可浏览。"),
    );
    return;
  }
  const sorted = [...data.videos].sort(
    (a, b) =>
      Number(b.camera.includes("overview")) -
      Number(a.camera.includes("overview")),
  );
  for (const entry of sorted) {
    const box = el("div", "video-box");
    const video = el("video");
    video.src = entry.url;
    video.preload = "auto";
    video.muted = true;
    video.playsInline = true;
    video.controls = !data.sync_ok;
    video.playbackRate = Number($("speed").value);
    const camera = entry.camera.includes("overview")
      ? "全景相机"
      : entry.camera.includes("wrist")
        ? "腕部相机"
        : entry.camera.replace("observation.images.", "");
    const cameraLabel = el("div", "camera-name", camera);
    box.append(video, cameraLabel);
    container.append(box);
    state.videos.push(video);
    video.addEventListener("error", () => {
      box.append(el("span", "camera-state", "视频无法解码或文件不可用"));
      pause();
      notice("相机视频加载失败，数值曲线仍可查看。");
    });
    video.addEventListener("loadedmetadata", () => {
      cameraLabel.textContent = `${camera} · ${video.videoWidth} × ${video.videoHeight}`;
      if (state.data !== data) return;
      if (
        data.sync_ok &&
        Math.abs(video.duration - data.count / data.fps) > 0.15
      ) {
        pause();
        data.sync_ok = false;
        state.videos.forEach((v) => (v.controls = true));
        notice("视频时长与采样帧数不匹配，已关闭自动联动，避免错误对齐。");
        $("video-note").textContent = "视频时长不匹配 · 独立播放";
      } else if (data.sync_ok) video.currentTime = state.index / data.fps;
    });
    video.addEventListener("ended", () => {
      if (video === state.videos[0]) {
        setIndex(data.count - 1, false);
        pause();
      }
    });
  }
}
function renderCharts() {
  const root = $("charts");
  root.replaceChildren();
  for (const group of state.data.groups) {
    const available = group.series.some((s) => s.available);
    if (!available && ["gripper_opening", "gripper_m", "drawer", "drawer_buckle", "insertion"].includes(group.id)) continue;
    const card = el(
      "article",
      "chart-card" + (group.id === "joints" ? " wide" : ""),
    );
    card.dataset.chartCategory=chartCategory(group.id);
    const heading = el("div", "chart-heading");
    heading.append(el("h4", "", group.title));
    const unit = el("span");
    heading.append(unit);
    card.append(heading);
    const legend = el("div", "series-legend");
    card.append(legend);
    const wrap = el("div", "chart-wrap");
    const canvas = el("canvas");
    canvas.setAttribute("aria-label", group.title + "随时间变化曲线");
    canvas.setAttribute("role", "img");
    const tip = el("div", "chart-tooltip");
    tip.hidden = true;
    wrap.append(canvas, tip);
    card.append(wrap);
    root.append(card);
    const chart = new Chart(group, canvas, wrap, tip, unit);
    state.charts.push(chart);
    for (const s of group.series) {
      const label = el("label");
      const check = el("input");
      check.type = "checkbox";
      check.checked = s.available;
      check.disabled = !s.available;
      check.style.accentColor = s.color;
      check.setAttribute("aria-label", group.title + " " + s.label);
      chart.enabled[s.id] = s.available;
      const name = el("span", "", s.label);
      name.style.color = s.color;
      const sample = el("span", "sample", "—");
      chart.labels[s.id] = sample;
      label.append(check, name, sample);
      legend.append(label);
      check.onchange = () => {
        chart.enabled[s.id] = check.checked;
        chart.rebuild();
      };
    }
    if (!available) wrap.append(el("div", "chart-empty", "此记录未采集该数据"));
    chart.rebuild();
  }
}
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
class Chart {
  constructor(group, canvas, wrap, tip, unit) {
    this.group = group;
    this.canvas = canvas;
    this.wrap = wrap;
    this.tip = tip;
    this.unit = unit;
    this.enabled = {};
    this.labels = {};
    this.base = document.createElement("canvas");
    this.drag = null;
    this.w = 0;
    this.h = 0;
    this.pad = { l: 55, r: 13, t: 15, b: 27 };
    this.observer = new ResizeObserver(() => this.rebuild());
    this.observer.observe(wrap);
    canvas.addEventListener("pointerdown", (e) => {
      if (e.button !== 0) return;
      this.drag = { start: this.px(e), end: this.px(e) };
      canvas.setPointerCapture(e.pointerId);
    });
    canvas.addEventListener("pointermove", (e) => {
      const x = this.px(e);
      if (this.drag) {
        this.drag.end = x;
        this.draw();
        this.tip.hidden = true;
      } else this.tooltip(x);
    });
    canvas.addEventListener("pointerup", (e) => {
      if (!this.drag) return;
      const { start, end } = this.drag;
      this.drag = null;
      canvas.releasePointerCapture(e.pointerId);
      if (Math.abs(end - start) > 8)
        setRange(
          this.time(Math.min(start, end)),
          this.time(Math.max(start, end)),
        );
      else {
        pause();
        setIndex(nearest(this.time(end)));
      }
      this.draw();
    });
    canvas.addEventListener("pointercancel", () => {
      this.drag = null;
      this.draw();
    });
    canvas.addEventListener("pointerleave", () => {
      this.tip.hidden = true;
    });
    canvas.addEventListener("dblclick", () =>
      setRange(0, Math.max(state.data.duration, 1 / state.data.fps)),
    );
  }
  destroy() {
    this.observer.disconnect();
  }
  factor() {
    return this.group.angle && state.degree ? 180 / Math.PI : 1;
  }
  unitText() {
    return this.group.angle && state.degree
      ? this.group.unit.replace("rad", "°")
      : this.group.unit;
  }
  px(e) {
    return Math.max(
      this.pad.l,
      Math.min(
        this.w - this.pad.r,
        e.clientX - this.canvas.getBoundingClientRect().left,
      ),
    );
  }
  time(x) {
    return (
      state.range[0] +
      ((x - this.pad.l) / (this.w - this.pad.l - this.pad.r)) *
        (state.range[1] - state.range[0])
    );
  }
  x(t) {
    return (
      this.pad.l +
      ((t - state.range[0]) / (state.range[1] - state.range[0])) *
        (this.w - this.pad.l - this.pad.r)
    );
  }
  y(v) {
    return (
      this.pad.t +
      ((this.max - v) / (this.max - this.min)) *
        (this.h - this.pad.t - this.pad.b)
    );
  }
  rebuild() {
    if (!state.data) return;
    this.times = this.group.times || state.data.times;
    this.unit.textContent = this.unitText();
    const rect = this.wrap.getBoundingClientRect();
    // Inactive pages and unselected chart groups have no layout box.
    // ResizeObserver will rebuild them when they become visible again.
    if (!state.active || rect.width <= 0 || rect.height <= 0) {
      this.w = this.h = 0;
      return;
    }
    this.w = Math.max(100, rect.width);
    this.h = rect.height;
    const dpr = window.devicePixelRatio || 1;
    for (const c of [this.canvas, this.base]) {
      c.width = Math.round(this.w * dpr);
      c.height = Math.round(this.h * dpr);
    }
    const ctx = this.base.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.lo = Math.max(0, nearest(state.range[0], this.times) - 1);
    this.hi = Math.min(this.times.length - 1, nearest(state.range[1], this.times) + 1);
    let min = Infinity,
      max = -Infinity;
    const f = this.factor();
    for (const s of this.group.series)
      if (this.enabled[s.id])
        for (const arr of [
          s.values,
          ...(state.target && s.targetValues ? [s.targetValues] : []),
        ])
          for (let i = this.lo; i <= this.hi; i++) {
            if (arr[i] !== null) {
              min = Math.min(min, arr[i] * f);
              max = Math.max(max, arr[i] * f);
            }
          }
    if (!Number.isFinite(min)) {
      min = -1;
      max = 1;
    }
    const gap = Math.max((max - min) * 0.12, Math.abs(max) * 0.015, 1e-5);
    this.min = min - gap;
    this.max = max + gap;
    ctx.font = "10px Segoe UI, Microsoft YaHei";
    ctx.lineWidth = 1;
    ctx.textAlign = "right";
    ctx.textBaseline = "middle";
    for (let i = 0; i <= 4; i++) {
      const v = this.min + ((this.max - this.min) * i) / 4,
        y = this.y(v);
      ctx.strokeStyle = "#dfe5ea";
      ctx.beginPath();
      ctx.moveTo(this.pad.l, y);
      ctx.lineTo(this.w - this.pad.r, y);
      ctx.stroke();
      ctx.fillStyle = "#65717c";
      ctx.fillText(this.axis(v), this.pad.l - 9, y);
    }
    ctx.textAlign = "center";
    ctx.textBaseline = "top";
    for (let i = 0; i <= 5; i++) {
      const t = state.range[0] + ((state.range[1] - state.range[0]) * i) / 5,
        x = this.x(t);
      ctx.fillStyle = "#65717c";
      ctx.fillText(
        t.toFixed(state.range[1] - state.range[0] < 5 ? 2 : 1),
        x,
        this.h - this.pad.b + 9,
      );
    }
    ctx.textAlign = "right";
    ctx.fillText("s", this.w - 1, this.h - 13);
    ctx.save();
    ctx.beginPath();
    ctx.rect(
      this.pad.l,
      this.pad.t,
      this.w - this.pad.l - this.pad.r,
      this.h - this.pad.t - this.pad.b,
    );
    ctx.clip();
    for (const s of this.group.series)
      if (this.enabled[s.id]) {
        this.line(ctx, s.values, s.color, false);
        if (state.target && s.targetValues)
          this.line(ctx, s.targetValues, s.color, true);
      }
    ctx.restore();
    this.draw();
  }
  axis(v) {
    const a = Math.abs(v);
    return a !== 0 && (a < 0.001 || a >= 1e5)
      ? v.toExponential(1)
      : v.toFixed(a < 0.1 ? 4 : a < 2 ? 2 : 1);
  }
  line(ctx, values, color, dashed) {
    ctx.strokeStyle = color;
    ctx.lineWidth = dashed ? 1.1 : 1.5;
    ctx.globalAlpha = dashed ? 0.55 : 1;
    ctx.setLineDash(dashed ? [4, 4] : []);
    ctx.beginPath();
    let started = false;
    const f = this.factor();
    // Preserve first/min/max/last samples in each pixel bucket, including breaks for missing data.
    let bucket = [],
      lastPixel = -1;
    const flush = () => {
      if (!bucket.length) return;
      let low = bucket[0],
        high = low;
      for (const i of bucket) {
        if (values[i] < values[low]) low = i;
        if (values[i] > values[high]) high = i;
      }
      const picks = [
        ...new Set([bucket[0], low, high, bucket[bucket.length - 1]]),
      ].sort((a, b) => a - b);
      for (const i of picks) {
        const x = this.x(this.times[i]),
          y = this.y(values[i] * f);
        if (started) ctx.lineTo(x, y);
        else {
          ctx.moveTo(x, y);
          started = true;
        }
      }
      bucket = [];
    };
    for (let i = this.lo; i <= this.hi; i++) {
      if (values[i] === null) {
        flush();
        started = false;
        lastPixel = -1;
        continue;
      }
      const p = Math.floor(this.x(this.times[i]));
      if (p !== lastPixel) {
        flush();
        lastPixel = p;
      }
      bucket.push(i);
    }
    flush();
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.globalAlpha = 1;
  }
  draw() {
    if (!this.w || !this.h || !state.active || !state.data) return;
    const ctx = this.canvas.getContext("2d");
    const dpr = window.devicePixelRatio || 1;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
    ctx.drawImage(this.base, 0, 0);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const cursorIndex = nearest(state.data.times[state.index], this.times);
    const x = this.x(state.data.times[state.index]);
    if (x >= this.pad.l && x <= this.w - this.pad.r) {
      ctx.strokeStyle = "#65717c99";
      ctx.lineWidth = 1;
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(x, this.pad.t);
      ctx.lineTo(x, this.h - this.pad.b);
      ctx.stroke();
      ctx.setLineDash([]);
      for (const s of this.group.series)
        if (this.enabled[s.id] && s.values[cursorIndex] !== null) {
          ctx.fillStyle = s.color;
          ctx.beginPath();
          ctx.arc(
            x,
            this.y(s.values[cursorIndex] * this.factor()),
            3,
            0,
            Math.PI * 2,
          );
          ctx.fill();
        }
    }
    if (this.drag) {
      ctx.fillStyle = "#2563eb22";
      ctx.fillRect(
        Math.min(this.drag.start, this.drag.end),
        this.pad.t,
        Math.abs(this.drag.end - this.drag.start),
        this.h - this.pad.t - this.pad.b,
      );
    }
    for (const s of this.group.series) {
      if (this.labels[s.id])
        this.labels[s.id].textContent =
          s.values[cursorIndex] === null
            ? "—"
            : fmt(s.values[cursorIndex] * this.factor(), 3);
    }
  }
  tooltip(x) {
    const i = nearest(this.time(x), this.times);
    this.tip.replaceChildren(
      el("strong", "", `${fmt(this.times[i], 3)} s · 样本 ${i + 1}`),
    );
    for (const s of this.group.series)
      if (this.enabled[s.id]) {
        const v = s.values[i];
        const target = s.targetValues?.[i];
        const line = el(
          "div",
          "",
          `${s.label}  ${v === null ? "缺失" : fmt(v * this.factor())} ${this.unitText()}${state.target && target != null ? "  / 目标 " + fmt(target * this.factor()) : ""}`,
        );
        line.style.color = s.color;
        this.tip.append(line);
      }
    this.tip.hidden = false;
    this.tip.style.top = "8px";
    this.tip.style.left =
      Math.max(0, Math.min(this.w - this.tip.offsetWidth - 5, x + 12)) + "px";
  }
}
function exportCSV() {
  if (!state.data) return;
  const groups = state.charts;
  const columns = [];
  for (const c of groups)
    for (const s of c.group.series)
      if (c.enabled[s.id]) {
        columns.push({
          label: `${c.group.title}_${s.label}_实际(${c.unitText()})`,
          values: s.values,
          times: c.times,
          factor: c.factor(),
        });
        if (state.target && s.targetValues)
          columns.push({
            label: `${c.group.title}_${s.label}_目标(${c.unitText()})`,
            values: s.targetValues,
            times: c.times,
            factor: c.factor(),
          });
      }
  const rows = [
    ["time_s", "source_frame", ...columns.map((c) => c.label)].join(","),
  ];
  const exportTimes = [...new Set(columns.flatMap(c => c.times))].sort((a,b) => a-b);
  for (const t of exportTimes) {
    const i = nearest(t);
    if (t < state.range[0] || t > state.range[1]) continue;
    rows.push(
      [
        t,
        Math.abs(state.data.times[i]-t) < 1e-9 ? state.data.frames[i] : "",
        ...columns.map((c) => {
          const j = nearest(t, c.times);
          return Math.abs(c.times[j]-t) > 1e-9 || c.values[j] == null ? "" : c.values[j] * c.factor;
        }),
      ].join(","),
    );
  }
  const blob = new Blob(["\uFEFF" + rows.join("\r\n")], {
    type: "text/csv;charset=utf-8",
  });
  const url = URL.createObjectURL(blob);
  const a = el("a");
  a.href = url;
  a.download = `${state.data.id}_${state.range[0].toFixed(3)}-${state.range[1].toFixed(3)}s.csv`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(url), 10000);
}
$("refresh").onclick = refresh;
$("search").oninput = renderList;
for(const id of ['source','outcome-filter','task-filter','date-from','date-to'])$(id).onchange=renderList;
$('back-list').onclick=()=>navigate(listUrl());
function filterCharts(){root.querySelectorAll('[data-chart-category]').forEach(card=>{card.hidden=card.dataset.chartCategory!==state.group;});root.querySelectorAll('[data-chart-group]').forEach(button=>button.setAttribute('aria-pressed',String(button.dataset.chartGroup===state.group)));state.charts.forEach(chart=>chart.rebuild());}
root.querySelectorAll('[data-chart-group]').forEach(button=>button.onclick=()=>{state.group=button.dataset.chartGroup;filterCharts();});
$("play").onclick = play;
$("timeline").oninput = () => {
  pause();
  setIndex(Number($("timeline").value));
};
$("prev-frame").onclick = () => {
  pause();
  setIndex(state.index - 1);
};
$("next-frame").onclick = () => {
  pause();
  setIndex(state.index + 1);
};
$("speed").onchange = () => {
  state.anchor = performance.now();
  state.anchorTime = state.data?.times[state.index] || 0;
  state.videos.forEach((v) => (v.playbackRate = Number($("speed").value)));
};
$("angle-unit").onchange = () => {
  state.degree = $("angle-unit").value === "deg";
  state.charts.forEach((c) => c.rebuild());
};
$("show-target").onchange = () => {
  state.target = $("show-target").checked;
  state.charts.forEach((c) => c.rebuild());
};
$("reset-zoom").onclick = () => {
  if (state.data)
    setRange(0, Math.max(state.data.duration, 1 / state.data.fps));
};
$("apply-range").onclick = () =>
  setRange($("range-start").value, $("range-end").value);
$("export").onclick = exportCSV;
document.addEventListener("keydown", (e) => {
  if (!state.active || $("workspace").hidden || !state.data) return;
  if (
    ["INPUT", "SELECT", "TEXTAREA", "BUTTON"].includes(
      document.activeElement.tagName,
    )
  )
    return;
  if (e.code === "Space") {
    e.preventDefault();
    play();
  }
  if (e.code === "ArrowRight") {
    e.preventDefault();
    pause();
    setIndex(state.index + 1);
  }
  if (e.code === "ArrowLeft") {
    e.preventDefault();
    pause();
    setIndex(state.index - 1);
  }
});

function clearData(){
 pause();state.request++;for(const chart of state.charts)chart.destroy();for(const video of state.videos){video.pause();video.removeAttribute('src');video.load();}state.charts=[];state.videos=[];state.data=null;$('videos').replaceChildren();$('charts').replaceChildren();$('export').disabled=true;
}
return {
 async activate(params){
  state.active=true;const version=++viewGeneration;notice();
  const requestedBatch=params.get('batch')||'';batchId=requestedBatch;batchIds=null;
  $('batch-scope').hidden=!requestedBatch;$('batch-label').textContent=requestedBatch?'批次：'+requestedBatch:'';
  showList();
  if(requestedBatch){try{const job=await api('/api/collections/'+encodeURIComponent(requestedBatch));if(!state.active||version!==viewGeneration)return;batchIds=new Set(job.accepted_episodes||[]);}catch(error){if(state.active&&version===viewGeneration){batchIds=new Set();notice(error.message);}}}
  if(!state.active||version!==viewGeneration)return;await refresh();if(!state.active||version!==viewGeneration)return;
  const desired=params.get('episode');
  if(desired){if(state.data?.id===desired){$('list-page').hidden=true;$('workspace').hidden=false;filterCharts();}else await loadEpisode(desired);}
 },
 deactivate(){state.active=false;viewGeneration++;state.request++;pause();},
 reset(){this.deactivate();clearData();state.list=[];batchId='';batchIds=null;state.group='joints';$('search').value='';$('source').value='all';$('outcome-filter').value='all';$('task-filter').replaceChildren(new Option('全部任务','all'));$('date-from').value='';$('date-to').value='';notice();showList();renderList();},
};
}
