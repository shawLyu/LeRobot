'use strict';

// Coordinates are stored in data units, so resizing never changes a measurement.
class ChartMeasurement {
  constructor(canvas, redraw, activate) {
    this.canvas = canvas;
    this.redraw = redraw;
    this.activate = activate;
    this.points = [];
    this.active = 0;
    this.drag = null;
    this.view = null;
    this.panel = document.createElement('div');
    this.panel.className = 'measurement';
    this.panel.dataset.chart = canvas.id;
    const title = canvas.getAttribute('aria-label');
    this.panel.innerHTML = `
      <div class="measurement-tools">
        <label>测量<select aria-label="${title}测量模式">
          <option value="off">关闭</option><option value="x">横轴 · Δt</option>
          <option value="y">纵轴 · Δy</option><option value="xy">双轴 · Δt / Δy</option>
        </select></label>
        <div class="cursor-picks" aria-label="${title}当前游标">
          <button type="button" data-cursor="0" aria-pressed="true">A</button>
          <button type="button" data-cursor="1" aria-pressed="false">B</button>
        </div>
        <label class="check-line"><input type="checkbox" />吸附采样点</label>
        <button type="button" class="clear-cursors">重置游标</button>
      </div>
      <output class="measurement-values mono" aria-label="${title}测量读数"></output>
      <p class="help measurement-hint">点击放置 A/B，拖动或用方向键微调；差值 = B − A。未吸附时测量自由坐标。</p>`;
    canvas.parentElement.after(this.panel);
    this.mode = this.panel.querySelector('select');
    this.snap = this.panel.querySelector('input');
    this.output = this.panel.querySelector('output');
    this.buttons = [...this.panel.querySelectorAll('[data-cursor]')];
    this.mode.addEventListener('change', () => {
      if (this.mode.value !== 'off') this.activate();
      this.redraw();
    });
    this.buttons.forEach((button, index) =>
      button.addEventListener('click', () => {
        this.active = index;
        this.redraw();
        canvas.focus({ preventScroll: true });
      }),
    );
    this.panel.querySelector('.clear-cursors').addEventListener('click', () => {
      this.points = [];
      this.active = 0;
      this.redraw();
    });
    canvas.tabIndex = 0;
    canvas.addEventListener('pointerdown', (event) => this.pointerDown(event));
    canvas.addEventListener('pointermove', (event) => {
      if (this.drag === event.pointerId) this.place(event);
    });
    canvas.addEventListener('pointerup', (event) => {
      if (this.drag !== event.pointerId) return;
      this.place(event);
      this.drag = null;
      this.active = 1 - this.active;
      canvas.releasePointerCapture(event.pointerId);
      this.redraw();
    });
    canvas.addEventListener('lostpointercapture', () => {
      this.drag = null;
    });
    canvas.addEventListener('keydown', (event) => this.keyDown(event));
  }

  reset() {
    this.mode.value = 'off';
    this.points = [];
    this.active = 0;
    this.view = null;
    this.drag = null;
  }

  enabled() {
    return this.mode.value !== 'off' && this.view?.hasData;
  }

  pixel(point) {
    const v = this.view;
    return {
      x: v.left + ((point.x - v.xmin) / (v.xmax - v.xmin)) * v.width,
      y: v.top + ((v.ymax - point.y) / (v.ymax - v.ymin)) * v.height,
    };
  }

  pointerDown(event) {
    if (!this.enabled() || event.button !== 0) return;
    const rect = this.canvas.getBoundingClientRect();
    const x = event.clientX - rect.left,
      y = event.clientY - rect.top,
      v = this.view;
    if (x < v.left || x > v.left + v.width || y < v.top || y > v.top + v.height) return;
    event.preventDefault();
    this.canvas.focus({ preventScroll: true });
    const distances = this.points.map((point) => {
      const p = this.pixel(point);
      if (this.mode.value === 'x') return Math.abs(p.x - x);
      if (this.mode.value === 'y') return Math.abs(p.y - y);
      return Math.hypot(p.x - x, p.y - y);
    });
    const nearest = distances.indexOf(Math.min(...distances));
    if (distances[nearest] <= 14) this.active = nearest;
    this.drag = event.pointerId;
    this.canvas.setPointerCapture(event.pointerId);
    this.place(event);
  }

  place(event) {
    const v = this.view,
      rect = this.canvas.getBoundingClientRect();
    const fx = Math.max(0, Math.min(1, (event.clientX - rect.left - v.left) / v.width));
    const fy = Math.max(0, Math.min(1, (event.clientY - rect.top - v.top) / v.height));
    this.setPoint({ x: v.xmin + fx * (v.xmax - v.xmin), y: v.ymax - fy * (v.ymax - v.ymin) });
  }

  setPoint(point) {
    const v = this.view;
    if (this.snap.checked) {
      const pixel = this.pixel(point);
      let best = Infinity;
      for (const series of v.series)
        for (const [x, y] of series.data) {
          if (
            !Number.isFinite(x) ||
            !Number.isFinite(y) ||
            x < v.xmin ||
            x > v.xmax ||
            y < v.ymin ||
            y > v.ymax
          )
            continue;
          const p = this.pixel({ x, y });
          const distance = Math.hypot(p.x - pixel.x, p.y - pixel.y);
          if (distance < best) {
            best = distance;
            point = { x, y, label: series.label };
          }
        }
    }
    this.points[this.active] = point;
    this.redraw();
  }

  keyDown(event) {
    if (!this.enabled() || !this.points[this.active]) return;
    const mode = this.mode.value;
    const horizontal = mode !== 'y' && ['ArrowLeft', 'ArrowRight'].includes(event.key);
    const vertical = mode !== 'x' && ['ArrowUp', 'ArrowDown'].includes(event.key);
    if (!horizontal && !vertical) return;
    event.preventDefault();
    const v = this.view,
      point = { ...this.points[this.active] };
    const step = event.shiftKey ? 0.01 : 0.001;
    if (horizontal) point.x += (event.key === 'ArrowRight' ? 1 : -1) * (v.xmax - v.xmin) * step;
    if (vertical) point.y += (event.key === 'ArrowUp' ? 1 : -1) * (v.ymax - v.ymin) * step;
    point.x = Math.max(v.xmin, Math.min(v.xmax, point.x));
    point.y = Math.max(v.ymin, Math.min(v.ymax, point.y));
    delete point.label;
    this.setPoint(point);
  }

  draw(view) {
    if (this.view?.key !== view.key) this.points = [];
    this.view = view;
    const enabled = this.enabled();
    this.mode.disabled = !view.hasData;
    this.canvas.classList.toggle('measuring', !!enabled);
    this.panel.querySelectorAll('button, input').forEach((input) => {
      input.disabled = !enabled;
    });
    this.buttons.forEach((button, index) =>
      button.setAttribute('aria-pressed', String(index === this.active)),
    );
    if (!enabled) {
      this.output.textContent = view.hasData ? '测量已关闭' : '暂无可测数据';
      return;
    }
    if (!this.points.length)
      this.points = [0.3, 0.7].map((fraction) => ({
        x: view.xmin + fraction * (view.xmax - view.xmin),
        y: view.ymin + fraction * (view.ymax - view.ymin),
      }));
    const [a, b] = this.points,
      mode = this.mode.value;
    const format = (point, name) =>
      `${name}: ${mode !== 'y' ? `t=${point.x.toFixed(4)} s` : ''}${mode === 'xy' ? ' · ' : ''}${mode !== 'x' ? `y=${point.y.toFixed(3)} ${view.yUnit}` : ''}${point.label ? ` (${point.label})` : ''}`;
    const delta = [
      mode !== 'y' ? `Δt=${((b.x - a.x) * 1000).toFixed(2)} ms` : '',
      mode !== 'x' ? `Δy=${(b.y - a.y).toFixed(3)} ${view.yUnit}` : '',
    ]
      .filter(Boolean)
      .join(' · ');
    this.output.replaceChildren(
      ...[format(a, 'A'), format(b, 'B'), delta].map((text) => {
        const span = document.createElement('span');
        span.textContent = text;
        return span;
      }),
    );
    const ctx = this.canvas.getContext('2d');
    ctx.save();
    ctx.beginPath();
    ctx.rect(view.left, view.top, view.width, view.height);
    ctx.clip();
    ctx.setLineDash([5, 3]);
    this.points.forEach((point, index) => {
      const p = this.pixel(point);
      ctx.strokeStyle = ctx.fillStyle = index ? '#9b6200' : '#8b3cb3';
      ctx.lineWidth = index === this.active ? 1.6 : 1;
      ctx.beginPath();
      if (mode !== 'y') {
        ctx.moveTo(p.x, view.top);
        ctx.lineTo(p.x, view.top + view.height);
      }
      if (mode !== 'x') {
        ctx.moveTo(view.left, p.y);
        ctx.lineTo(view.left + view.width, p.y);
      }
      ctx.stroke();
      ctx.textAlign = 'left';
      ctx.font = 'bold 11px sans-serif';
      ctx.fillText(
        index ? 'B' : 'A',
        mode === 'y' ? view.left + 5 : Math.min(p.x + 5, view.left + view.width - 12),
        mode === 'x' ? view.top + 13 : Math.max(view.top + 12, p.y - 5),
      );
    });
    ctx.restore();
  }
}
