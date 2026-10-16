/* Deterministic chart measurements; all API requests are mocked, no motor access. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const { chromium } = require('playwright');

(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ['--no-proxy-server'],
    ...(process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH
      ? { executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH }
      : {}),
  });
  try {
    const page = await browser.newPage({
      viewport: { width: 1440, height: 1320 },
      deviceScaleFactor: 2,
    });
    const errors = [],
      commands = [];
    page.on('pageerror', (error) => errors.push(error.message));
    let seq = 0;
    const hello = { arm: 'follower', port: 'mock', baudrate: 1000000 };
    const metrics = {
      time_to_90_s: 0.45,
      rise_time_10_90_s: 0.35,
      settling_time_s: 0.8,
      steady_peak_error_ticks: 0,
      overshoot_ticks: 5,
    };
    const result = {
      id: 'fixture',
      completed: true,
      dwell: 1.2,
      amplitude_deg: 5,
      segments: ['positive', 'negative'].map((direction) => ({
        kind: 'step',
        joint: 'shoulder_pan',
        direction,
        repeat: 0,
        start: 0,
        target: 100,
        frames: [0, 40, 80, 105, 100, 100, 100].map((position, i) => ({ t: i / 5, position })),
        metrics,
      })),
    };
    await page.route('**/api/**', async (route) => {
      const url = new URL(route.request().url());
      let response;
      if (url.pathname === '/api/info')
        response = {
          arms: [{ name: 'follower', role: 'follower', port: 'mock', ids: [1, 2, 3, 4, 5, 6] }],
          schema: [],
          baudrates: [1000000],
        };
      else if (url.pathname === '/api/state')
        response = {
          connected: true,
          age_seconds: 0,
          hello,
          state: {
            discovery: { shoulder_pan: { online: true, id: 1 } },
            motion: 'idle',
            enabled: [],
            bounds: { shoulder_pan: [1953, 2181] },
            result_id: result.id,
            frame: {
              seq: ++seq,
              t: 1 + seq / 100,
              motors: {
                shoulder_pan: {
                  position: 2067,
                  goal: 2077,
                  voltage: 5.3,
                  temperature: 25,
                  torque: 0,
                  status: 0,
                },
              },
            },
          },
        };
      else if (url.pathname === '/api/result') response = result;
      else if (url.pathname === '/api/command') {
        const command = route.request().postDataJSON();
        commands.push(command);
        assert.equal(
          command.operation,
          'parameters',
          'measurement must never send motion commands',
        );
        response = { values: {}, errors: {} };
      } else throw new Error(`Unexpected API request: ${url.pathname}`);
      await route.fulfill({ json: { ok: true, result: response } });
    });
    await page.goto(process.env.SERVO_WORKBENCH_URL || 'http://127.0.0.1:8766');
    await page.waitForFunction(() => document.querySelector('.motor-check')?.disabled === false);
    await page.locator('.motor-check').first().check();
    const panel = (id) => page.locator(`.measurement[data-chart="${id}"]`);
    const mode = (id) => panel(id).locator('select');
    const readout = (id) => panel(id).locator('output');
    const delta = async (id, name) =>
      Number((await readout(id).innerText()).match(new RegExp(`${name}=(-?[\\d.]+)`))[1]);
    const coords = async (id, fx, fy) => {
      const canvas = page.locator(`#${id}`);
      await canvas.scrollIntoViewIfNeeded();
      const r = await canvas.boundingBox();
      return { x: r.x + 51 + fx * (r.width - 65), y: r.y + 15 + fy * (r.height - 43) };
    };
    const place = async (id, index, fx, fy) => {
      await panel(id).locator(`[data-cursor="${index}"]`).click();
      const p = await coords(id, fx, fy);
      await page.mouse.click(p.x, p.y);
      await page.waitForTimeout(50);
    };
    const close = (actual, expected, tolerance = 0.1) =>
      assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} must be near ${expected}`);
    await page.waitForFunction(
      () => !document.querySelector('[data-chart="live-chart"] select').disabled,
    );
    await mode('live-chart').selectOption('xy');
    await page.waitForFunction(
      () => document.querySelector('#pause-display').textContent === '继续显示',
    );
    await place('live-chart', 0, 0.2, 0.75);
    await place('live-chart', 1, 0.8, 0.25);
    close(await delta('live-chart', 'Δt'), 12000);
    close(await delta('live-chart', 'Δy'), 6.2, 0.02);
    const first = await readout('live-chart').innerText();
    await page.waitForTimeout(400);
    assert.equal(
      await readout('live-chart').innerText(),
      first,
      'live display freezes for measurement',
    );
    const start = await coords('live-chart', 0.2, 0.75),
      end = await coords('live-chart', 0.3, 0.75);
    await page.mouse.move(start.x, start.y);
    await page.mouse.down();
    await page.mouse.move(end.x, end.y, { steps: 5 });
    await page.mouse.up();
    await page.waitForTimeout(50);
    close(await delta('live-chart', 'Δt'), 10000);
    await panel('live-chart').locator('[data-cursor="0"]').click();
    await page.locator('#live-chart').press('ArrowRight');
    await page.waitForTimeout(50);
    close(await delta('live-chart', 'Δt'), 9980);
    await page.setViewportSize({ width: 1150, height: 1100 });
    await page.waitForTimeout(100);
    close(await delta('live-chart', 'Δt'), 9980);

    await panel('live-chart').locator('input').check();
    await place('live-chart', 0, 0.05, 0.9);
    await place('live-chart', 1, 0.05, 0.1);
    assert.match(await readout('live-chart').innerText(), /2067\.000 tick.*J1 实测/s);
    assert.match(await readout('live-chart').innerText(), /2077\.000 tick.*J1 目标/s);
    close(await delta('live-chart', 'Δy'), 10);
    await mode('error-chart').selectOption('y');
    await place('error-chart', 0, 0.2, 0.75);
    await place('error-chart', 1, 0.8, 0.25);
    close(await delta('error-chart', 'Δy'), 2, 0.02);
    assert.doesNotMatch(await readout('error-chart').innerText(), /Δt/);
    await page.locator('#plot-field').selectOption('voltage');
    await page.waitForFunction(() =>
      document.querySelector('[data-chart="live-chart"] output').textContent.includes(' V'),
    );
    assert.doesNotMatch(await readout('live-chart').innerText(), /2067|2077/);
    await page.locator('#pause-display').click();
    assert.equal(await mode('live-chart').inputValue(), 'off');
    assert.equal(await mode('error-chart').inputValue(), 'off');

    await page.locator('[data-tab="response"]').click();
    await mode('response-chart').selectOption('xy');
    await place('response-chart', 0, 0.25, 0.75);
    await place('response-chart', 1, 0.75, 0.25);
    close(await delta('response-chart', 'Δt'), 600);
    close(await delta('response-chart', 'Δy'), 60, 0.05);
    await mode('response-chart').selectOption('x');
    await page.waitForFunction(
      () =>
        !document.querySelector('[data-chart="response-chart"] output').textContent.includes('Δy'),
    );
    assert.doesNotMatch(await readout('response-chart').innerText(), /Δy/);
    await mode('settled-chart').selectOption('y');
    await place('settled-chart', 0, 0.25, 0.75);
    await place('settled-chart', 1, 0.75, 0.25);
    close(await delta('settled-chart', 'Δy'), 2, 0.02);
    await mode('response-chart').selectOption('xy');
    await place('response-chart', 0, 0.9, 0.1);
    close(await delta('response-chart', 'Δt'), -180);
    close(await delta('response-chart', 'Δy'), -18, 0.05);
    await place('response-chart', 0, 0.25, 0.75);
    const output = process.env.SERVO_WORKBENCH_OUTPUT || 'logs/servo-workbench-measurement';
    fs.mkdirSync(output, { recursive: true });
    await page.locator('#tab-response').screenshot({ path: path.join(output, 'measurement.png') });
    await page.setViewportSize({ width: 390, height: 844 });
    await page.waitForTimeout(100);
    close(await delta('response-chart', 'Δt'), 600);
    assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth));
    await panel('response-chart').locator('.clear-cursors').click();
    await page.waitForTimeout(50);
    close(await delta('response-chart', 'Δt'), 480);
    assert.deepEqual(errors, []);
    assert.ok(commands.every((c) => c.operation === 'parameters'));
    console.log(
      'PASS: all four charts, axis modes, signed deltas, drag, keyboard, sample snapping, frozen live display, units, reset, high-DPI resize and mobile layout; no motor commands.',
    );
  } finally {
    await browser.close();
  }
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
