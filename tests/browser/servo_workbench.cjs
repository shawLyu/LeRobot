/* Optional real-browser acceptance. Hardware execution requires an explicit flag.
   NODE_PATH must resolve Playwright; no browser library is needed by the application. */
'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const http = require('node:http');
const { chromium } = require('playwright');
const base = process.env.SERVO_WORKBENCH_URL || 'http://127.0.0.1:8765';
const output = process.env.SERVO_WORKBENCH_OUTPUT || 'logs/servo-workbench-browser';
const hardware = process.argv.includes('--hardware');
const expected = require('../../docs/measurements/native-pid-single-joint-2026-10-16/applied-settings.json');
fs.mkdirSync(output, { recursive: true });

function rawRequest(endpoint, token, body) {
  return new Promise((resolve, reject) => {
    const data = body === undefined ? null : JSON.stringify(body);
    const req = http.request(
      new URL(endpoint, base),
      {
        method: data === null ? 'GET' : 'POST',
        headers: {
          'X-Workbench-Token': token,
          'Content-Type': 'application/json',
          ...(data === null ? {} : { 'Content-Length': Buffer.byteLength(data) }),
        },
        timeout: 15000,
      },
      (res) => {
        let text = '';
        res.on('data', (chunk) => (text += chunk));
        res.on('end', () => {
          try {
            const value = JSON.parse(text);
            if (!value.ok) reject(new Error(value.error));
            else resolve(value.result);
          } catch (error) {
            reject(error);
          }
        });
      },
    );
    req.on('error', reject);
    req.on('timeout', () => req.destroy(new Error('HTTP timeout')));
    if (data !== null) req.write(data);
    req.end();
  });
}

(async () => {
  const browser = await chromium.launch({
    headless: true,
    args: ['--no-proxy-server'],
    ...(process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH
      ? { executablePath: process.env.PLAYWRIGHT_CHROMIUM_EXECUTABLE_PATH }
      : {}),
  });
  const page = await browser.newPage({
    viewport: { width: 1440, height: 1320 },
    deviceScaleFactor: 1,
  });
  const errors = [],
    report = { hardware, checks: [], started: new Date().toISOString() };
  let token,
    touched = false,
    changedP = false,
    baselineP = expected.requested.P_Coefficient;
  page.on('pageerror', (error) => errors.push(error.message));
  page.on('console', (message) => {
    if (message.type() === 'error') errors.push(message.text());
  });
  const param = (key) => page.locator(`#parameter-rows tr[data-key="${key}"] td`).nth(2);
  const readParam = async (key) => Number((await param(key).textContent()).trim());
  const waitPosition = (target) =>
    page.waitForFunction(
      (target) =>
        Math.abs(
          Number(
            document.querySelector('[data-joint="shoulder_pan"] .position-number').textContent,
          ) - target,
        ) <= 2,
      target,
      { timeout: 5000 },
    );
  const clickTab = (name) => page.locator(`[data-tab="${name}"]`).click();
  const writeP = async (value, persist) => {
    await clickTab('parameters');
    await page.locator('#parameter-rows tr[data-key="P_Coefficient"] input').fill(String(value));
    await page.locator('#persist').setChecked(persist);
    await page.locator('#review-write').click();
    await page.locator('#write-dialog').waitFor({ state: 'visible' });
    assert.match(await page.locator('#write-preview').innerText(), /P_Coefficient/);
    await page.locator('#confirm-write').click();
    await page.locator('#write-dialog').waitFor({ state: 'hidden' });
    await page.waitForFunction(
      (value) =>
        Number(document.querySelector('[data-key="P_Coefficient"] td:nth-child(3)').textContent) ===
        value,
      value,
    );
  };
  try {
    if (!hardware)
      await page.route('**/api/state', (route) =>
        route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({ ok: true, result: { connected: false, state: null } }),
        }),
      );
    await page.goto(base);
    await page.waitForFunction(() => document.querySelector('#arm')?.options.length === 2);
    token = await page.locator('meta[name="workbench-token"]').getAttribute('content');
    assert.equal(await page.locator('.motor').count(), 6);
    assert.equal(await page.locator('#enable').isDisabled(), true);
    assert.equal(await page.locator('#device-ids input').count(), 6);
    await page.locator('#arm').selectOption('arm1');
    await page.waitForFunction(() =>
      document.querySelector('#port-label').textContent.includes('5B91028324'),
    );
    await page.locator('#arm').selectOption('follower');
    await page.waitForFunction(() =>
      document.querySelector('#port-label').textContent.includes('5B8E114461'),
    );
    report.checks.push('six slots, arm selection, disabled motion before connection');
    await page.screenshot({ path: path.join(output, 'initial.png') });
    if (hardware) {
      await page.locator('#connect').click();
      await page.waitForFunction(
        () => document.querySelector('#connection-status').textContent.includes('已连接'),
        null,
        { timeout: 20000 },
      );
      await page.waitForFunction(
        () => document.querySelector('#parameter-state').textContent.includes('已读取'),
        null,
        { timeout: 20000 },
      );
      assert.equal(await page.locator('.motor:not(.offline)').count(), 1);
      assert.equal(
        await page
          .locator('[data-joint="shoulder_pan"]')
          .evaluate((e) => e.classList.contains('offline')),
        false,
      );
      assert.equal(await readParam('Torque_Enable'), 0);
      assert.equal(await readParam('Status'), 0);
      assert.equal(await readParam('ID'), expected.identity.id);
      assert.equal(await readParam('Model_Number'), 777);
      assert.ok((await page.locator('#port-label').textContent()).includes(expected.identity.port));
      for (const [key, v] of Object.entries(expected.requested))
        assert.equal(await readParam(key), v, key);
      for (const [key, v] of Object.entries(expected.identity.registers))
        assert.equal(await readParam(key), v, key);
      const start = await readParam('Present_Position');
      assert.ok(
        Math.abs(start - expected.verified_readback.Present_Position) + 114 <= 1024,
        `pose outside previously authorized +/-90 deg envelope: ${start}`,
      );
      report.initial = { position: start, native: expected.requested };
      report.checks.push('real ID1, five absent motors tolerated, full parameter readback');
      touched = true;
      changedP = true;
      await writeP(baselineP - 1, false);
      assert.equal(await readParam('Integral_Limit'), 8);
      assert.equal(await readParam('Minimum_Startup_Force'), 16);
      report.checks.push('temporary PID write and independent displayed readback');
      await writeP(baselineP, true);
      changedP = false;
      assert.equal(await readParam('Lock'), 1);
      report.checks.push('EEPROM write, relock and baseline restoration');
      await page.locator('#parameter-rows tr[data-key="P_Coefficient"] input').fill('255');
      await page.locator('#review-write').click();
      assert.equal(await page.locator('#write-dialog').isVisible(), false);
      assert.match(await page.locator('#notice').innerText(), /0\.\.254/);
      await page.locator('#discard-edits').click();
      assert.equal(await page.locator('#parameter-rows tr[data-key="Lock"] input').count(), 0);
      assert.equal(
        await page.locator('#parameter-rows tr[data-key="Maximum_Acceleration"] input').count(),
        0,
      );
      report.checks.push('invalid PID rejected; managed lock and factory registers read-only');
      await clickTab('live');
      await page.locator('#motion-confirmed').check();
      await page.locator('#enable').click();
      await page.waitForFunction(
        () =>
          document.querySelector('[data-joint="shoulder_pan"] .motor-torque').textContent ===
          '扭矩开启',
      );
      const stale = await rawRequest('/api/state', token);
      stale.age_seconds = 3;
      await page.route('**/api/state', (route) =>
        route.fulfill({
          status: 200,
          contentType: 'application/json',
          body: JSON.stringify({ ok: true, result: stale }),
        }),
      );
      await page.waitForFunction(
        () =>
          document.querySelector('#send-target').disabled &&
          !document.querySelector('#hold').disabled,
      );
      await page.waitForTimeout(1800);
      const watchdog = await rawRequest('/api/state', token);
      assert.deepEqual(watchdog.state.enabled, []);
      assert.equal(watchdog.state.motion, 'idle');
      await page.unroute('**/api/state');
      await page.waitForFunction(
        () =>
          document.querySelector('#connection-status').textContent.includes('已连接') &&
          !document.querySelector('#enable').disabled,
      );
      await page.locator('#enable').click();
      await page.waitForFunction(() => !document.querySelector('#send-target').disabled);
      report.checks.push(
        'real heartbeat timeout holds, stale display blocks motion, reenable required',
      );
      await page.locator('.multi-target summary').click();
      assert.equal(await page.locator('#multi-targets .multi-row').count(), 6);
      await page.locator('#multi-shoulder_pan').fill(String(start + 10));
      await page.locator('#send-multi').click();
      await waitPosition(start + 10);
      await page.locator('#target').fill(String(start));
      await page.waitForFunction(() => !document.querySelector('#send-target').disabled);
      await page.locator('#send-target').click();
      await waitPosition(start);
      report.checks.push(
        'six independent target inputs, checked-target +10 tick move and single-target return',
      );
      await page.waitForFunction(() => !document.querySelector('#start-step').disabled);
      await page.locator('#start-step').click();
      await page.waitForFunction(
        () => document.querySelector('#response-rows').children.length === 2,
        null,
        { timeout: 20000 },
      );
      assert.match(await page.locator('#response-description').innerText(), /采集完成/);
      assert.equal(await page.locator('#fault-banner').isVisible(), false);
      const downloadJson = page.waitForEvent('download');
      await page.locator('#export-response-json').click();
      const jsonFile = await downloadJson;
      await jsonFile.saveAs(path.join(output, 'response.json'));
      const response = JSON.parse(fs.readFileSync(path.join(output, 'response.json'), 'utf8'));
      assert.equal(response.completed, true);
      assert.equal(response.segments.filter((s) => s.kind === 'step').length, 2);
      assert.equal(response.native_parameters.shoulder_pan.P_Coefficient, baselineP);
      assert.equal(response.device_context.arm, 'follower');
      assert.equal(response.device_context.port, expected.identity.port);
      assert.equal(response.motor_identities.shoulder_pan.id, 1);
      report.device_context = response.device_context;
      report.response = response.segments
        .filter((s) => s.kind === 'step')
        .map((s) => ({ direction: s.direction, ...s.metrics }));
      report.checks.push('remote 5 degree step completed, both-direction metrics and JSON export');
      const downloadCsv = page.waitForEvent('download');
      await page.locator('#export-response-csv').click();
      await (await downloadCsv).saveAs(path.join(output, 'response.csv'));
      assert.ok(
        fs.readFileSync(path.join(output, 'response.csv'), 'utf8').split('\n').length > 100,
      );
      await page.screenshot({ path: path.join(output, 'response.png') });
      report.checks.push('response graph, settled errors, CSV export');
      await clickTab('live');
      await page.locator('#release-all').click();
      await page.waitForFunction(
        () =>
          document.querySelector('[data-joint="shoulder_pan"] .motor-torque').textContent ===
          '扭矩关闭',
      );
      await clickTab('parameters');
      await page.locator('#refresh-parameters').click();
      await page.waitForFunction(() =>
        document.querySelector('#parameter-state').textContent.includes('已读取'),
      );
      for (const [key, v] of Object.entries(expected.requested))
        assert.equal(await readParam(key), v, key);
      assert.equal(await readParam('Torque_Enable'), 0);
      assert.equal(await readParam('Lock'), 1);
      assert.equal(await readParam('Status'), 0);
      assert.ok(Math.abs((await readParam('Present_Position')) - start) <= 2);
      report.final = {
        position: await readParam('Present_Position'),
        torque: await readParam('Torque_Enable'),
        lock: await readParam('Lock'),
        status: await readParam('Status'),
      };
      await page.screenshot({ path: path.join(output, 'parameters.png') });
      report.checks.push('final native settings retained, starting pose recovered, torque off');
      await page.locator('#disconnect').click();
      await page.waitForFunction(
        () => document.querySelector('#connection-status').textContent === '未连接',
      );
      touched = false;
    }
    await page.setViewportSize({ width: 390, height: 844 });
    await page.locator('[data-tab="live"]').click();
    await page.screenshot({ path: path.join(output, 'mobile.png'), fullPage: true });
    assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth));
    report.checks.push('mobile layout without horizontal page overflow');
    assert.deepEqual(errors, []);
    report.completed = true;
  } catch (error) {
    report.completed = false;
    report.error = error.stack;
    await page.screenshot({ path: path.join(output, 'failure.png') }).catch(() => {});
    throw error;
  } finally {
    if (hardware && token) {
      try {
        if (touched) {
          await rawRequest('/api/command', token, { operation: 'hold' });
          await rawRequest('/api/command', token, {
            operation: 'release',
            payload: { joints: ['shoulder_pan'] },
          });
        }
        if (changedP) {
          const p = await rawRequest('/api/command', token, {
            operation: 'parameters',
            payload: { joint: 'shoulder_pan' },
          });
          await rawRequest('/api/command', token, {
            operation: 'write',
            payload: {
              joint: 'shoulder_pan',
              changes: { P_Coefficient: baselineP },
              expected: { P_Coefficient: p.values.P_Coefficient },
              persist: true,
            },
          });
        }
        await rawRequest('/api/disconnect', token, {});
      } catch (error) {
        report.cleanup_note = error.message;
      }
    }
    report.browser_errors = errors;
    report.finished = new Date().toISOString();
    fs.writeFileSync(
      path.join(output, 'browser-report.json'),
      JSON.stringify(report, null, 2) + '\n',
    );
    console.log(JSON.stringify(report));
    await browser.close();
  }
})().catch((error) => {
  console.error(error);
  process.exit(1);
});
