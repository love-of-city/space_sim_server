// Draw each dataset group on its own recorded sampling clock.
export function createDatasetChart({ state, setRange, pause, setIndex, nearest, fmt, el,
  document = globalThis.document, window = globalThis.window, ResizeObserver = globalThis.ResizeObserver,
}) {
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

  return Chart;

}
