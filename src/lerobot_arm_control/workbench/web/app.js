'use strict';

const $ = (id) => document.getElementById(id);
const JOINTS = [
  'shoulder_pan',
  'shoulder_lift',
  'elbow_flex',
  'wrist_flex',
  'wrist_roll',
  'gripper',
];
const COLORS = ['#1858ba', '#d27624', '#188576', '#8b54a6', '#b74362', '#5d798b'];
const FIELD_UNITS = {
  position: 'tick',
  velocity: 'raw',
  load: '×0.1%',
  current_raw: '×6.5 mA',
  voltage: 'V',
  temperature: '°C',
};
const measurements = new Map();
const token = document.querySelector('meta[name="workbench-token"]').content;
const escapeHTML = (value) =>
  String(value).replace(
    /[&<>"']/g,
    (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c],
  );
let info,
  pack = {},
  active = JOINTS[0],
  selected = new Set(),
  history = [],
  paused = false;
let params = {},
  edits = {},
  targets = {},
  boundsDrafts = {},
  result = null,
  fetchedResult = null,
  loadingResult = false;
let busy = false,
  currentTab = 'live',
  lastSeq = -1,
  review = null,
  drawQueued = false;
let receivedAt = 0;

function notice(text, kind = '') {
  $('notice').textContent = text;
  $('notice').className = `notice ${kind}`;
  $('notice').hidden = !text;
}

async function api(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? 'GET' : 'POST',
    headers: {
      'X-Workbench-Token': token,
      ...(body === undefined ? {} : { 'Content-Type': 'application/json' }),
    },
    body: body === undefined ? undefined : JSON.stringify(body),
    signal: AbortSignal.timeout(20000),
  });
  const data = await response.json();
  if (!response.ok || !data.ok) throw new Error(data.error || `HTTP ${response.status}`);
  return data.result;
}

const command = (operation, payload = {}) => api('/api/command', { operation, payload });
const state = () => pack.state || {};
const connected = () =>
  !!pack.connected && (pack.age_seconds || 0) + (performance.now() - receivedAt) / 1000 < 2;
const online = (joint) => connected() && !!state().discovery?.[joint]?.online;
const checked = () => JOINTS.filter((j) => selected.has(j) && online(j));
const hasMotion = () => connected() && state().motion !== 'idle';
const enabled = (joint) => connected() && (state().enabled || []).includes(joint);
const value = (id) => Number($(id).value);
const ms = (seconds) =>
  seconds === null || seconds === undefined ? '—' : `${(seconds * 1000).toFixed(1)} ms`;
const median = (values) => {
  const a = values.filter((v) => v !== null && v !== undefined).sort((x, y) => x - y);
  return a.length
    ? (a[Math.floor((a.length - 1) / 2)] + a[Math.ceil((a.length - 1) / 2)]) / 2
    : null;
};

async function action(fn, success) {
  try {
    busy = true;
    renderControls();
    const response = await fn();
    if (success) notice(typeof success === 'function' ? success(response) : success, 'success');
    return response;
  } catch (error) {
    notice(error.message, 'error');
    return undefined;
  } finally {
    busy = false;
    renderControls();
  }
}

function clearSessionData() {
  measurements.forEach((measurement) => measurement.reset());
  paused = false;
  $('pause-display').textContent = '暂停显示';
  params = {};
  edits = {};
  history = [];
  targets = {};
  boundsDrafts = {};
  result = null;
  fetchedResult = null;
  lastSeq = -1;
}

function resetSession() {
  pack = {};
  clearSessionData();
  selected.clear();
  $('motion-confirmed').checked = false;
  $('write-dialog').close();
  render();
  renderParameters();
  renderResponse();
}

function selectArm() {
  const arm = info.arms.find((a) => a.name === $('arm').value);
  $('device-ids').replaceChildren(
    ...arm.ids.map((id, i) => {
      const input = document.createElement('input');
      input.type = 'number';
      input.min = '1';
      input.max = '253';
      input.value = id;
      input.setAttribute('aria-label', `通道 ${i + 1} 舵机 ID`);
      input.title = `J${i + 1} · ${JOINTS[i]}`;
      return input;
    }),
  );
  $('baudrate').value = '1000000';
  $('port-label').textContent = `${arm.port} · 连接本身不会使电机运动`;
}

async function connectArm() {
  const hello = await action(() =>
    api('/api/connect', {
      arm: $('arm').value,
      ids: [...$('device-ids').querySelectorAll('input')].map((e) => Number(e.value)),
      baudrate: value('baudrate'),
    }),
  );
  if (!hello) return;
  clearSessionData();
  pack = { connected: true, hello, state: hello.state, age_seconds: 0 };
  receivedAt = performance.now();
  active = JOINTS.find((j) => hello.state.discovery[j]?.online) || JOINTS[0];
  selected = new Set(hello.state.discovery[active]?.online ? [active] : []);
  const count = Object.values(hello.state.discovery).filter((d) => d.online).length;
  notice(
    `已连接 ${hello.arm}：${count}/6 个电机在线。当前仅监测，请确认动作范围后使能。`,
    count ? 'success' : 'error',
  );
  render();
  renderResponse();
  if (count) await loadParameters();
}

function chooseJoint(joint) {
  if (joint !== active && Object.keys(edits).length)
    return notice('当前有未写入参数，请先写入或撤销修改，再切换电机。', 'error');
  active = joint;
  edits = {};
  render();
  renderParameters();
  if (online(joint) && !hasMotion()) loadParameters();
}

function renderMotors() {
  const container = $('motors');
  if (!container.children.length) {
    JOINTS.forEach((joint, i) => {
      const card = document.createElement('article');
      card.className = 'motor';
      card.dataset.joint = joint;
      card.tabIndex = 0;
      card.style.setProperty('--channel', COLORS[i]);
      card.innerHTML = `<div class="motor-heading"><label><input class="motor-check" type="checkbox" aria-label="选择电机 ${i + 1}">J${i + 1} · <span class="motor-id"></span></label><span class="status-tag"></span></div><div class="joint-name">${joint}</div><div class="position-value"><span class="position-number">—</span><small>tick</small></div><div class="motor-footer"><span class="motor-voltage">— V</span><span class="motor-temperature">— °C</span><span class="motor-torque">未使能</span></div>`;
      card.addEventListener('click', (event) => {
        if (!event.target.closest('label')) chooseJoint(joint);
      });
      card.addEventListener('keydown', (event) => {
        if (event.key === 'Enter' && event.target === card) chooseJoint(joint);
      });
      card.querySelector('input').addEventListener('change', (event) => {
        if (event.target.checked) selected.add(joint);
        else selected.delete(joint);
        renderControls();
        queueDraw();
      });
      container.append(card);
    });
  }
  for (const card of container.children) {
    const joint = card.dataset.joint,
      found = online(joint),
      m = found ? state().frame?.motors[joint] : null;
    const index = JOINTS.indexOf(joint);
    card.classList.toggle('active', joint === active);
    card.classList.toggle('offline', !found);
    card.querySelector('input').checked = selected.has(joint);
    card.querySelector('input').disabled = !found || busy;
    card.querySelector('.motor-id').textContent =
      state().discovery?.[joint]?.id ?? $('device-ids').children[index]?.value ?? index + 1;
    const badge = card.querySelector('.status-tag');
    badge.textContent = !found ? '离线' : m?.status ? `状态 ${m.status}` : '在线';
    badge.classList.toggle('alarm', !!m?.status);
    card.querySelector('.position-number').textContent = m?.position ?? '—';
    card.querySelector('.motor-voltage').textContent = `${m?.voltage?.toFixed(1) ?? '—'} V`;
    card.querySelector('.motor-temperature').textContent = `${m?.temperature ?? '—'} °C`;
    card.querySelector('.motor-torque').textContent = !m
      ? '无反馈'
      : m.torque === 1
        ? '扭矩开启'
        : m.torque === 2
          ? '阻尼'
          : '扭矩关闭';
  }
}

function renderBounds() {
  const applied = state().bounds?.[active];
  const draft = boundsDrafts[active];
  const pending = draft && draft.some((v, i) => v === '' || Number(v) !== applied?.[i]);
  let reason = '';
  if (!online(active)) reason = '请先连接当前电机。';
  else if (busy) reason = '正在处理请求，请稍候。';
  else if (hasMotion()) reason = '请先停止运动，再设置范围。';
  else if (enabled(active)) reason = '请先释放当前电机扭矩，再设置范围。';
  else if (!$('motion-confirmed').checked) reason = '请先勾选行程无干涉确认。';
  $('save-bounds').disabled = !!reason;
  $('bounds-status').textContent =
    `${applied ? `已应用 ${applied[0]} — ${applied[1]} tick。` : '尚未设置有效范围。'} ${pending ? '输入尚未应用。' : ''}${reason || '可设置；范围须包含当前位置，并处于 EEPROM 限位内。'}`;
}

function renderControls() {
  const isConnected = connected(),
    available = online(active),
    motion = hasMotion();
  const confirmed = $('motion-confirmed').checked,
    joints = checked();
  $('connect').disabled = busy || isConnected;
  $('disconnect').disabled = busy || !pack.connected;
  $('arm').disabled = busy;
  $('baudrate').disabled = busy || isConnected;
  [...$('device-ids').children].forEach((input) => {
    input.disabled = busy || isConnected;
  });
  $('hold').disabled = !pack.connected;
  $('release-all').disabled =
    !pack.connected || !JOINTS.some((j) => state().discovery?.[j]?.online);
  $('enable').disabled = busy || !confirmed || !joints.length || motion || !!state().fault;
  $('release').disabled = busy || !joints.length;
  renderBounds();
  for (const id of ['send-target', 'jog-minus', 'jog-plus'])
    $(id).disabled =
      busy || !available || !confirmed || !enabled(active) || motion || !!state().fault;
  $('send-multi').disabled =
    busy ||
    !confirmed ||
    !joints.length ||
    joints.some((j) => !enabled(j)) ||
    motion ||
    !!state().fault;
  $('start-step').disabled = $('send-multi').disabled;
  $('refresh-parameters').disabled = busy || !available || motion;
  $('review-write').disabled = busy || !available || !Object.keys(edits).length || motion;
  $('export-parameters').disabled = !params[active];
  for (const id of ['export-live-csv', 'export-json']) $(id).disabled = !isConnected || motion;
  for (const id of ['export-response-csv', 'export-response-json'])
    $(id).disabled = !result || motion;
  $('active-id').textContent =
    `J${JOINTS.indexOf(active) + 1} · ID ${state().discovery?.[active]?.id ?? '—'}`;
  $('active-name').textContent = active;
  $('dirty-count').textContent = Object.keys(edits).length
    ? `${Object.keys(edits).length} 待写入`
    : '';
  const progress = state().job;
  $('job-progress').textContent = progress
    ? `测试中 · ${progress.index + 1}/${progress.total} 段`
    : state().motion === 'moving'
      ? '目标轨迹正在远端执行'
      : result
        ? result.completed
          ? '最近测试采集完成'
          : `最近测试中断：${result.error}`
        : '尚无进行中的测试';
}

function render() {
  const isConnected = connected();
  $('connection-status').textContent = isConnected
    ? `${pack.hello.arm} · 已连接`
    : pack.error
      ? '连接中断'
      : '未连接';
  $('connection-status').classList.toggle('online', isConnected);
  if (isConnected) {
    $('port-label').textContent =
      `${pack.hello.port} · ${pack.hello.baudrate.toLocaleString()} baud · 高频采样在远端运行`;
    $('freshness').textContent = `最近反馈 ${Math.round((pack.age_seconds || 0) * 1000)} ms 前`;
  } else $('freshness').textContent = '';
  const recent = history.filter((f) => history.at(-1)?.t - f.t <= 2);
  $('sample-rate').textContent =
    isConnected && recent.length > 1
      ? `${((recent.at(-1).seq - recent[0].seq) / (recent.at(-1).t - recent[0].t)).toFixed(0)} Hz`
      : '— Hz';
  $('fault-banner').hidden = !isConnected || !state().fault;
  $('fault-text').textContent = state().fault || '';
  renderMotors();
  const m = isConnected ? state().frame?.motors[active] : null;
  if (m && targets[active] === undefined) targets[active] = m.position;
  if (document.activeElement !== $('target')) $('target').value = targets[active] ?? '';
  const bounds = boundsDrafts[active] || state().bounds?.[active];
  if (document.activeElement !== $('bound-low')) $('bound-low').value = bounds?.[0] ?? '';
  if (document.activeElement !== $('bound-high')) $('bound-high').value = bounds?.[1] ?? '';
  JOINTS.forEach((joint) => {
    const input = $(`multi-${joint}`),
      actual = isConnected ? state().frame?.motors[joint] : null;
    if (actual && targets[joint] === undefined) targets[joint] = actual.position;
    if (document.activeElement !== input) input.value = targets[joint] ?? '';
    input.disabled = !online(joint);
  });
  renderControls();
  queueDraw();
}

async function poll() {
  try {
    const previousConnected = pack.connected;
    pack = await api('/api/state');
    receivedAt = performance.now();
    const frame = state().frame;
    if (connected() && frame && frame.seq !== lastSeq) {
      lastSeq = frame.seq;
      if (!paused) {
        history.push(frame);
        while (history.length > 1500 || (history.length && frame.t - history[0].t > 21))
          history.shift();
      }
    }
    if (pack.error && previousConnected) notice(pack.error, 'error');
    render();
    if (state().result_id && state().result_id !== fetchedResult && !loadingResult && connected()) {
      loadingResult = true;
      try {
        result = await api('/api/result');
        fetchedResult = state().result_id;
        renderResponse();
      } finally {
        loadingResult = false;
      }
    }
  } catch (error) {
    pack.age_seconds = 2;
    notice(`读取工作台失败：${error.message}`, 'error');
    render();
  } finally {
    setTimeout(poll, 100);
  }
}

async function loadParameters() {
  $('parameter-state').textContent = '正在读取当前电机参数…';
  const joint = active;
  const response = await action(() => command('parameters', { joint }));
  if (response) {
    params[joint] = response;
    if (active === joint) edits = {};
  }
  renderParameters();
}

function renderParameters() {
  const record = params[active],
    search = $('parameter-search').value.trim().toLowerCase();
  const group = $('parameter-group').value;
  const rows =
    info?.schema.filter(
      (r) =>
        (group === 'all' || group === r.group || (group === 'readonly' && !r.writable)) &&
        `${r.key} ${r.label} ${r.address} ${r.note}`.toLowerCase().includes(search),
    ) || [];
  $('parameter-rows').innerHTML = rows
    .map((r) => {
      const current = record?.values[r.key],
        draft = edits[r.key] ?? current;
      return `<tr data-key="${escapeHTML(r.key)}" class="${r.key in edits ? 'edited' : ''}"><td class="mono">${r.address}<div class="group-label">${r.group}</div></td><td><div class="register-label">${escapeHTML(r.label)}</div><div class="register-key">${escapeHTML(r.key)}</div>${r.note ? `<div class="register-note">${escapeHTML(r.note)}</div>` : ''}</td><td class="register-value">${escapeHTML(current ?? '—')}</td><td>${r.writable ? `<input type="number" step="1" min="${r.minimum}" max="${r.maximum}" value="${escapeHTML(draft ?? '')}" aria-label="${escapeHTML(r.label)} 新值" ${current === undefined ? 'disabled' : ''}>` : '<span class="muted">只读</span>'}</td><td>${escapeHTML(r.unit)}<div class="group-label">${r.writable ? `${r.minimum}…${r.maximum}` : '只读'}</div></td></tr>`;
    })
    .join('');
  $('parameter-rows')
    .querySelectorAll('input')
    .forEach((input) =>
      input.addEventListener('input', () => {
        const key = input.closest('tr').dataset.key,
          next = Number(input.value);
        if (input.value === '' || next === params[active].values[key]) delete edits[key];
        else edits[key] = next;
        input.closest('tr').classList.toggle('edited', key in edits);
        renderControls();
      }),
    );
  $('parameter-state').textContent = record
    ? `${active} · 已读取 ${Object.keys(record.values).length} 项${Object.keys(record.errors).length ? `，${Object.keys(record.errors).length} 项读取失败` : ''}；写入前会比较原值，防止覆盖已变化的设置。`
    : '连接后点击“读取当前电机”以加载参数。';
  renderControls();
}

function reviewWrite() {
  const source = params[active];
  if (!source || !Object.keys(edits).length) return;
  for (const [key, v] of Object.entries(edits)) {
    const r = info.schema.find((r) => r.key === key);
    if (!Number.isInteger(v) || v < r.minimum || v > r.maximum)
      return notice(`${r.label} 必须是 ${r.minimum}..${r.maximum} 的整数`, 'error');
  }
  review = {
    joint: active,
    changes: { ...edits },
    expected: Object.fromEntries(Object.keys(edits).map((k) => [k, source.values[k]])),
    persist: $('persist').checked,
  };
  $('write-scope').textContent =
    `${pack.hello.arm} / ${active} / ID ${state().discovery[active].id} · ${Object.keys(edits).length} 项修改`;
  $('write-preview').innerHTML =
    `<table><thead><tr><th>参数</th><th>原值</th><th>新值</th></tr></thead><tbody>${Object.entries(
      edits,
    )
      .map(
        ([k, v]) =>
          `<tr><td>${escapeHTML(k)}</td><td>${source.values[k]}</td><td>${escapeHTML(v)}</td></tr>`,
      )
      .join('')}</tbody></table>`;
  $('write-mode').textContent = review.persist
    ? '保存到 EEPROM：断电后保留。需要先释放所有在线电机的扭矩；写完自动重新锁定。'
    : '临时写入：EEPROM 参数在本次上电期间生效，断电恢复已保存值；RAM 参数不支持掉电保存。';
  $('write-dialog').showModal();
}

async function confirmWrite() {
  $('confirm-write').disabled = true;
  const response = await action(() => command('write', review), '参数写入并读回一致。');
  $('confirm-write').disabled = false;
  if (!response) return;
  if (response.log_error)
    notice(`参数已写入并读回，但远端记录保存失败：${response.log_error}`, 'error');
  $('write-dialog').close();
  if (response.ids) {
    JOINTS.forEach((j, i) => {
      $('device-ids').children[i].value = response.ids[j];
    });
    if ('Baud_Rate' in review.changes) {
      const baud = info.baudrates[review.changes.Baud_Rate];
      $('baudrate').value = baud;
      pack.hello.baudrate = baud;
    }
  }
  edits = {};
  await loadParameters();
}

function switchTab(tab) {
  currentTab = tab;
  document
    .querySelectorAll('[data-tab]')
    .forEach((b) => b.classList.toggle('active', b.dataset.tab === tab));
  for (const name of ['live', 'parameters', 'response']) $(`tab-${name}`).hidden = name !== tab;
  if (tab === 'parameters') renderParameters();
  queueDraw();
}

function queueDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => {
    drawQueued = false;
    drawCharts();
  });
}

function plot(canvasId, series, options = {}) {
  const canvas = $(canvasId),
    rect = canvas.getBoundingClientRect();
  if (rect.width < 2 || rect.height < 2) return;
  const ratio = Math.min(2, window.devicePixelRatio || 1);
  if (
    canvas.width !== Math.round(rect.width * ratio) ||
    canvas.height !== Math.round(rect.height * ratio)
  ) {
    canvas.width = Math.round(rect.width * ratio);
    canvas.height = Math.round(rect.height * ratio);
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  ctx.clearRect(0, 0, rect.width, rect.height);
  const margin = { left: 51, right: 14, top: 15, bottom: 28 },
    width = rect.width - margin.left - margin.right,
    height = rect.height - margin.top - margin.bottom;
  const points = series
    .flatMap((s) => s.data)
    .filter((p) => Number.isFinite(p[0]) && Number.isFinite(p[1]));
  const xs = points.map((p) => p[0]),
    ys = points.map((p) => p[1]);
  let xmin = options.xmin ?? (xs.length ? Math.min(...xs) : 0),
    xmax = options.xmax ?? (xs.length ? Math.max(...xs) : 20);
  let ymin = options.ymin ?? (ys.length ? Math.min(...ys) : 0),
    ymax = options.ymax ?? (ys.length ? Math.max(...ys) : 1);
  if (xmax === xmin) xmax = xmin + 1;
  if (ymax === ymin) {
    ymin -= 2;
    ymax += 2;
  } else if (options.ymin === undefined && options.ymax === undefined) {
    const pad = (ymax - ymin) * 0.12;
    ymin -= pad;
    ymax += pad;
  }
  const X = (x) => margin.left + ((x - xmin) / (xmax - xmin)) * width;
  const Y = (y) => margin.top + ((ymax - y) / (ymax - ymin)) * height;
  ctx.font = '10px ui-monospace, SFMono-Regular, monospace';
  ctx.fillStyle = '#7c8c9c';
  ctx.lineWidth = 1;
  if (options.band) {
    ctx.fillStyle = '#e9eff6';
    ctx.fillRect(margin.left, Y(options.band[1]), width, Y(options.band[0]) - Y(options.band[1]));
  }
  for (let i = 0; i <= 4; i++) {
    const y = ymin + ((ymax - ymin) * i) / 4,
      py = Y(y);
    ctx.strokeStyle = '#e7edf3';
    ctx.beginPath();
    ctx.moveTo(margin.left, py);
    ctx.lineTo(rect.width - margin.right, py);
    ctx.stroke();
    ctx.fillStyle = '#7c8c9c';
    ctx.textAlign = 'right';
    ctx.fillText(Math.abs(y) >= 100 ? y.toFixed(0) : y.toFixed(1), margin.left - 8, py + 3);
  }
  for (let i = 0; i <= 5; i++) {
    const x = xmin + ((xmax - xmin) * i) / 5,
      px = X(x);
    ctx.strokeStyle = '#eef2f6';
    ctx.beginPath();
    ctx.moveTo(px, margin.top);
    ctx.lineTo(px, margin.top + height);
    ctx.stroke();
    ctx.fillStyle = '#7c8c9c';
    ctx.textAlign = 'center';
    ctx.fillText(x.toFixed(xmax - xmin > 5 ? 0 : 2), px, rect.height - 9);
  }
  ctx.save();
  ctx.beginPath();
  ctx.rect(margin.left, margin.top, width, height);
  ctx.clip();
  if (options.reference !== undefined) {
    ctx.strokeStyle = '#a3afbc';
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(X(xmin), Y(options.reference));
    ctx.lineTo(X(xmax), Y(options.reference));
    ctx.stroke();
    ctx.setLineDash([]);
  }
  if (options.timeReference !== undefined) {
    ctx.strokeStyle = '#9aaabd';
    ctx.setLineDash([2, 4]);
    ctx.beginPath();
    ctx.moveTo(X(options.timeReference), margin.top);
    ctx.lineTo(X(options.timeReference), margin.top + height);
    ctx.stroke();
    ctx.setLineDash([]);
  }
  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = s.dashed ? 1.2 : 1.7;
    ctx.setLineDash(s.dashed ? [5, 4] : []);
    ctx.globalAlpha = s.alpha ?? 1;
    ctx.beginPath();
    let begun = false;
    for (const [x, y] of s.data) {
      if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
      if (!begun) {
        ctx.moveTo(X(x), Y(y));
        begun = true;
      } else ctx.lineTo(X(x), Y(y));
    }
    ctx.stroke();
  }
  ctx.restore();
  ctx.globalAlpha = 1;
  ctx.setLineDash([]);
  ctx.fillStyle = '#7c8c9c';
  ctx.textAlign = 'right';
  ctx.fillText(`t / ${options.unit || 's'}`, rect.width - margin.right, 10);
  ctx.textAlign = 'left';
  ctx.fillText(options.yUnit, margin.left, 10);
  measurements.get(canvasId)?.draw({
    xmin,
    xmax,
    ymin,
    ymax,
    left: margin.left,
    top: margin.top,
    width,
    height,
    series,
    hasData: points.length > 0,
    yUnit: options.yUnit,
    key: options.measureKey,
  });
}

function drawCharts() {
  if (currentTab === 'live') {
    const field = $('plot-field').value,
      joints = checked(),
      last = history.at(-1)?.t || 0;
    const curves = [],
      errors = [];
    for (const j of joints) {
      const color = COLORS[JOINTS.indexOf(j)];
      curves.push({
        color,
        label: `J${JOINTS.indexOf(j) + 1} 实测`,
        data: history.filter((f) => f.motors[j]).map((f) => [f.t, f.motors[j][field]]),
      });
      if (field === 'position')
        curves.push({
          color,
          label: `J${JOINTS.indexOf(j) + 1} 目标`,
          dashed: true,
          alpha: 0.6,
          data: history.filter((f) => f.motors[j]).map((f) => [f.t, f.motors[j].goal]),
        });
      errors.push({
        color,
        label: `J${JOINTS.indexOf(j) + 1} 误差`,
        data: history
          .filter((f) => f.motors[j])
          .map((f) => [f.t, f.motors[j].goal - f.motors[j].position]),
      });
    }
    const domain = { xmin: Math.max(0, last - 20), xmax: Math.max(20, last) };
    plot('live-chart', curves, {
      ...domain,
      yUnit: FIELD_UNITS[field],
      measureKey: `${field}:${joints}`,
    });
    plot('error-chart', errors, {
      ...domain,
      reference: 0,
      yUnit: 'tick',
      measureKey: String(joints),
    });
    $('live-empty').hidden = connected() && history.length > 0 && joints.length > 0;
    $('live-empty').textContent = connected()
      ? joints.length
        ? '等待反馈…'
        : '勾选在线电机以显示曲线'
      : '连接机械臂以开始读取反馈';
    $('plot-legend').replaceChildren(
      ...joints.map((j) => {
        const span = document.createElement('span');
        span.className = 'legend-item';
        span.style.setProperty('--channel', COLORS[JOINTS.indexOf(j)]);
        span.innerHTML = `<i></i>J${JOINTS.indexOf(j) + 1}`;
        return span;
      }),
    );
  }
  if (currentTab === 'response') {
    const joint = $('response-joint').value;
    const segments = (result?.segments || []).filter((s) => s.kind === 'step' && s.joint === joint);
    const curves = segments.map((s) => ({
      color: s.direction === 'positive' ? COLORS[0] : COLORS[1],
      label: `${s.direction === 'positive' ? '正向' : '反向'} ${s.repeat + 1}`,
      alpha: 0.8,
      dashed: s.direction === 'negative',
      data: s.frames.map((f) => [f.t, (100 * (f.position - s.start)) / (s.target - s.start)]),
    }));
    const tails = segments.map((s) => ({
      color: s.direction === 'positive' ? COLORS[0] : COLORS[1],
      label: `${s.direction === 'positive' ? '正向' : '反向'} ${s.repeat + 1}`,
      alpha: 0.8,
      dashed: s.direction === 'negative',
      data: s.frames
        .filter((f) => f.t >= s.frames.at(-1).t - 0.3)
        .map((f) => [f.t - s.frames.at(-1).t, s.target - f.position]),
    }));
    plot('response-chart', curves, {
      xmin: 0,
      xmax: result?.dwell || 1.2,
      ymin: -5,
      ymax: 115,
      reference: 100,
      timeReference: 0.05,
      yUnit: '%',
      measureKey: `${result?.id}:${joint}`,
    });
    plot('settled-chart', tails, {
      xmin: -0.3,
      xmax: 0,
      band: [-1, 1],
      reference: 0,
      yUnit: 'tick',
      measureKey: `${result?.id}:${joint}`,
    });
    $('response-empty').hidden = segments.length > 0;
  }
}

function renderResponse() {
  const segments = (result?.segments || []).filter((s) => s.kind === 'step'),
    current = $('response-joint').value;
  const joints = [...new Set(segments.map((s) => s.joint))];
  $('response-joint').innerHTML = joints
    .map((j) => `<option value="${j}">J${JOINTS.indexOf(j) + 1} · ${j}</option>`)
    .join('');
  if (joints.includes(current)) $('response-joint').value = current;
  else if (joints.includes(active)) $('response-joint').value = active;
  const chosen = segments
    .filter((s) => s.joint === $('response-joint').value)
    .map((s) => s.metrics);
  const summary = [
    ['到达 90% · 中位数', ms(median(chosen.map((m) => m.time_to_90_s))), '从远端命令发送开始'],
    [
      '10–90% 上升 · 中位数',
      ms(median(chosen.map((m) => m.rise_time_10_90_s))),
      '不等同于到达时间',
    ],
    [
      '稳态误差 · 最大',
      chosen.length ? `${Math.max(...chosen.map((m) => m.steady_peak_error_ticks))} tick` : '—',
      '最后 300 ms 峰值',
    ],
    [
      '超调 · 最大',
      chosen.length ? `${Math.max(...chosen.map((m) => m.overshoot_ticks))} tick` : '—',
      '跨该电机完整阶跃',
    ],
  ];
  $('response-summary').innerHTML = summary
    .map(
      ([label, note, help]) =>
        `<div class="metric"><span>${label}</span><strong>${note}</strong><small>${help}</small></div>`,
    )
    .join('');
  $('response-description').textContent = result
    ? `${result.completed ? '采集完成' : '测试中断'} · ${result.amplitude_deg}° 完整阶跃 · ${segments.length} 个完整响应${result.error ? ` · ${result.error}` : ''}${result.log_error ? ` · 保存失败：${result.log_error}` : ''}`
    : '完成一次测试后显示原始响应和各电机指标。';
  $('response-rows').innerHTML = segments
    .map(
      (s) =>
        `<tr><td>J${JOINTS.indexOf(s.joint) + 1}</td><td>${s.direction === 'positive' ? '正向' : '反向'} ${s.repeat + 1}</td><td>${ms(s.metrics.time_to_90_s)}</td><td>${ms(s.metrics.rise_time_10_90_s)}</td><td>${ms(s.metrics.settling_time_s)}</td><td>${s.metrics.steady_peak_error_ticks} tick</td><td>${s.metrics.overshoot_ticks} tick</td></tr>`,
    )
    .join('');
  $('result-count').textContent = segments.length ? `${segments.length} 段` : '';
  renderControls();
  queueDraw();
}

function download(data, name, mime) {
  const blob =
    data instanceof Blob
      ? data
      : new Blob([typeof data === 'string' ? data : JSON.stringify(data, null, 2)], {
          type: mime || 'application/json',
        });
  const url = URL.createObjectURL(blob),
    anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = name;
  anchor.click();
  setTimeout(() => URL.revokeObjectURL(url), 2000);
}

async function downloadRemote(format, scope = 'live') {
  await action(async () => {
    const response = await fetch(`/api/export?format=${format}&scope=${scope}`, {
      headers: { 'X-Workbench-Token': token },
    });
    if (!response.ok) {
      const error = await response.json();
      throw new Error(error.error);
    }
    if (format === 'csv') download(await response.blob(), `servo-${scope}-${Date.now()}.csv`);
    else {
      const data = await response.json();
      download(data.result, `servo-data-${Date.now()}.json`);
    }
  });
}

function saveFigure(ids, title, filename) {
  const source = ids.map($),
    canvas = document.createElement('canvas');
  canvas.width = Math.max(...source.map((c) => c.width));
  canvas.height = source.reduce((sum, c) => sum + c.height, 0) + 90;
  const ctx = canvas.getContext('2d');
  ctx.fillStyle = '#fff';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = '#202c39';
  ctx.font = '22px sans-serif';
  ctx.fillText(title, 30, 38);
  let top = 70;
  for (const chart of source) {
    ctx.drawImage(chart, 0, top);
    top += chart.height;
  }
  canvas.toBlob((blob) => {
    if (blob) download(blob, filename);
  }, 'image/png');
}

async function initialize() {
  info = await api('/api/info');
  $('arm').innerHTML = info.arms
    .map(
      (a) =>
        `<option value="${escapeHTML(a.name)}">${escapeHTML(a.name)} · ${a.role === 'leader' ? '主臂' : '从臂'}</option>`,
    )
    .join('');
  $('baudrate').innerHTML = info.baudrates
    .map((b) => `<option value="${b}">${b.toLocaleString()}</option>`)
    .join('');
  $('multi-targets').innerHTML = JOINTS.map(
    (j, i) =>
      `<label class="multi-row"><span>J${i + 1} ${j}</span><input id="multi-${j}" aria-label="电机 ${i + 1} 独立目标" type="number" step="1" disabled></label>`,
  ).join('');
  JOINTS.forEach((j) => {
    $(`multi-${j}`).addEventListener('input', (e) => {
      targets[j] = Number(e.target.value);
      if (j === active) $('target').value = e.target.value;
    });
  });
  for (const id of ['live-chart', 'error-chart', 'response-chart', 'settled-chart']) {
    measurements.set(
      id,
      new ChartMeasurement($(id), queueDraw, () => {
        if (id === 'live-chart' || id === 'error-chart') {
          paused = true;
          $('pause-display').textContent = '继续显示';
        }
      }),
    );
  }
  selectArm();
  render();
  renderParameters();
  renderResponse();
  $('connect').addEventListener('click', connectArm);
  $('disconnect').addEventListener('click', () =>
    action(async () => {
      await api('/api/disconnect', {});
      resetSession();
      selectArm();
    }, '已断开；远端已请求保持当前位置。'),
  );
  $('arm').addEventListener('change', () =>
    action(async () => {
      if (pack.connected) await api('/api/disconnect', {});
      resetSession();
      selectArm();
    }),
  );
  $('hold').addEventListener('click', () =>
    action(() => command('hold'), '已停止目标轨迹并保持当前位置。'),
  );
  $('release-all').addEventListener('click', () =>
    action(
      () => command('release', { joints: JOINTS.filter((j) => state().discovery?.[j]?.online) }),
      '已释放全部在线电机扭矩。',
    ),
  );
  $('clear-fault').addEventListener('click', () =>
    action(() => command('clear_fault'), '故障已清除；请重新检查并使能。'),
  );
  $('motion-confirmed').addEventListener('change', renderControls);
  $('enable').addEventListener('click', () =>
    action(
      () => command('enable', { joints: checked() }),
      '已在实际当前位置使能，没有发送复位动作。',
    ),
  );
  $('release').addEventListener('click', () =>
    action(() => command('release', { joints: checked() }), '已释放勾选电机扭矩。'),
  );
  for (const id of ['bound-low', 'bound-high'])
    $(id).addEventListener('input', () => {
      boundsDrafts[active] = [$('bound-low').value, $('bound-high').value];
      renderControls();
    });
  $('save-bounds').addEventListener('click', () => {
    const joint = active;
    const inputs = [$('bound-low').value, $('bound-high').value];
    const bounds = inputs.map(Number);
    if (inputs.some((v) => v === '') || !bounds.every(Number.isInteger) || bounds[0] >= bounds[1])
      return notice('范围上下限必须是整数，且下限小于上限。', 'error');
    action(async () => {
      const response = await command('bounds', { joint, bounds });
      state().bounds[joint] = response.bounds;
    }, '动作范围已设置（仅本次连接有效）。');
  });
  $('target').addEventListener('input', () => {
    targets[active] = value('target');
  });
  $('send-target').addEventListener('click', () =>
    action(
      () =>
        command('move', { targets: { [active]: value('target') }, speed_deg_s: value('speed') }),
      '已发送位置目标，远端执行。',
    ),
  );
  $('send-multi').addEventListener('click', () =>
    action(
      () =>
        command('move', {
          targets: Object.fromEntries(checked().map((j) => [j, targets[j]])),
          speed_deg_s: value('speed'),
        }),
      '已发送勾选电机目标。',
    ),
  );
  for (const [id, delta] of [
    ['jog-minus', -10],
    ['jog-plus', 10],
  ])
    $(id).addEventListener('click', () =>
      action(() => {
        const actual = state().frame.motors[active].position;
        targets[active] = actual + delta;
        return command('move', {
          targets: { [active]: actual + delta },
          speed_deg_s: value('speed'),
        });
      }),
    );
  $('start-step').addEventListener('click', () =>
    action(async () => {
      const started = await command('step', {
        joints: checked(),
        amplitude_deg: value('amplitude'),
        repeats: value('repeats'),
        dwell: value('dwell'),
      });
      result = null;
      fetchedResult = null;
      renderResponse();
      switchTab('response');
      return started;
    }, '阶跃测试已在远端开始；可随时停止并保持。'),
  );
  document
    .querySelectorAll('[data-tab]')
    .forEach((b) => b.addEventListener('click', () => switchTab(b.dataset.tab)));
  $('refresh-parameters').addEventListener('click', loadParameters);
  $('parameter-search').addEventListener('input', renderParameters);
  $('parameter-group').addEventListener('change', renderParameters);
  $('discard-edits').addEventListener('click', () => {
    edits = {};
    renderParameters();
  });
  $('review-write').addEventListener('click', reviewWrite);
  $('confirm-write').addEventListener('click', confirmWrite);
  $('export-parameters').addEventListener('click', () =>
    download(
      { version: 1, arm: pack.hello.arm, ...params[active] },
      `servo-${active}-parameters.json`,
    ),
  );
  $('import-parameters').addEventListener('change', async (event) => {
    try {
      const file = event.target.files[0];
      if (!file) return;
      if (file.size > 65536) throw new Error('参数文件不能超过 64 KiB');
      if (!params[active]) throw new Error('请先读取当前电机参数');
      const imported = JSON.parse(await file.text());
      const values = imported.values || imported.requested || imported;
      let count = 0;
      for (const r of info.schema) {
        const v = values[r.key];
        if (
          r.writable &&
          !['Goal_Position', 'Torque_Enable'].includes(r.key) &&
          v !== undefined &&
          v !== params[active].values[r.key]
        ) {
          if (!Number.isInteger(v) || v < r.minimum || v > r.maximum)
            throw new Error(`${r.label} 超出范围`);
          edits[r.key] = v;
          count++;
        }
      }
      renderParameters();
      notice(`已导入 ${count} 项待修改参数；尚未写入，请检查后提交。`);
    } catch (error) {
      notice(error.message, 'error');
    } finally {
      event.target.value = '';
    }
  });
  $('plot-field').addEventListener('change', queueDraw);
  $('response-joint').addEventListener('change', renderResponse);
  $('pause-display').addEventListener('click', () => {
    paused = !paused;
    if (!paused) for (const id of ['live-chart', 'error-chart']) measurements.get(id).reset();
    $('pause-display').textContent = paused ? '继续显示' : '暂停显示';
    queueDraw();
  });
  $('clear-display').addEventListener('click', () => {
    history = [];
    for (const id of ['live-chart', 'error-chart']) measurements.get(id).reset();
    queueDraw();
  });
  $('export-live-csv').addEventListener('click', () => downloadRemote('csv'));
  $('export-json').addEventListener('click', () => downloadRemote('json'));
  $('export-response-csv').addEventListener('click', () => downloadRemote('csv', 'response'));
  $('export-response-json').addEventListener('click', () =>
    download(result, `servo-response-${result.id}.json`),
  );
  $('export-live-png').addEventListener('click', () =>
    saveFigure(['live-chart', 'error-chart'], 'LeRobot / live servo feedback', 'servo-live.png'),
  );
  $('export-response-png').addEventListener('click', () =>
    saveFigure(
      ['response-chart', 'settled-chart'],
      `LeRobot / step response / ${$('response-joint').value}`,
      'servo-response.png',
    ),
  );
  new ResizeObserver(queueDraw).observe($('motors'));
  setInterval(() => {
    if (pack.connected && !connected()) render();
  }, 500);
  poll();
}

initialize().catch((error) => notice(`界面初始化失败：${error.message}`, 'error'));
