/* Range-editor regression: every API is mocked; this never operates a motor. */
'use strict';
const assert = require('node:assert/strict');
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
    const page = await browser.newPage();
    const errors = [],
      commands = [];
    page.on('pageerror', (error) => errors.push(error.message));
    const joints = ['shoulder_pan', 'shoulder_lift'];
    const state = {
      bounds: { shoulder_pan: [1953, 2181], shoulder_lift: [1953, 2181] },
      discovery: Object.fromEntries(joints.map((j, i) => [j, { id: i + 1, online: true }])),
      enabled: [],
      motion: 'idle',
      frame: {
        seq: 0,
        t: 0,
        motors: Object.fromEntries(
          joints.map((j) => [
            j,
            {
              position: 2067,
              goal: 2067,
              torque: 0,
              voltage: 5.3,
              temperature: 25,
              status: 0,
            },
          ]),
        ),
      },
    };
    let failSave = false,
      polls = 0;
    // Fail closed: unexpected API requests fail locally, never reach real hardware.
    await page.route('**/api/**', async (route) => {
      const path = new URL(route.request().url()).pathname;
      let result;
      if (path === '/api/info')
        result = {
          arms: [{ name: 'follower', role: 'follower', port: 'mock', ids: [1, 2, 3, 4, 5, 6] }],
          schema: [],
          baudrates: [1000000],
        };
      else if (path === '/api/state') {
        polls++;
        state.frame.seq++;
        state.frame.t += 0.1;
        result = {
          connected: true,
          age_seconds: 0,
          hello: { arm: 'follower', port: 'mock', baudrate: 1000000 },
          state,
        };
      } else if (path === '/api/command') {
        const command = route.request().postDataJSON();
        commands.push(command);
        if (command.operation === 'parameters') result = { values: {}, errors: {} };
        else if (command.operation === 'bounds') {
          if (failSave)
            return route.fulfill({
              status: 400,
              json: {
                ok: false,
                error: '动作范围必须处于单圈 EEPROM 位置限制内',
              },
            });
          state.bounds[command.payload.joint] = command.payload.bounds;
          result = { bounds: command.payload.bounds };
        } else throw new Error(`Unexpected motor operation: ${command.operation}`);
      } else throw new Error(`Unexpected API: ${path}`);
      await route.fulfill({ json: { ok: true, result } });
    });
    await page.goto(process.env.SERVO_WORKBENCH_URL || 'http://127.0.0.1:8765');
    await page.waitForFunction(() => document.querySelector('#bound-low').value === '1953');
    await page.locator('#motion-confirmed').check();
    await page.locator('#bound-low').fill('1800');
    await page.locator('#bound-high').focus();
    const before = polls;
    await page.waitForFunction(() => document.querySelector('#sample-rate').textContent !== '— Hz');
    const deadline = Date.now() + 5000;
    while (polls < before + 3 && Date.now() < deadline) await page.waitForTimeout(100);
    assert.ok(polls >= before + 3, 'feedback polling must continue during editing');
    assert.equal(
      await page.locator('#bound-low').inputValue(),
      '1800',
      'poll must preserve blurred draft',
    );
    await page.locator('#bound-high').fill('2300');
    await page.locator('#target').focus();
    await page.waitForTimeout(350);
    assert.equal(await page.locator('#bound-high').inputValue(), '2300');
    await page.locator('#save-bounds').click();
    await page.waitForFunction(() =>
      document.querySelector('#notice').textContent.includes('动作范围已设置'),
    );
    assert.deepEqual(commands.find((c) => c.operation === 'bounds').payload, {
      joint: 'shoulder_pan',
      bounds: [1800, 2300],
    });
    await page.waitForTimeout(350);
    assert.equal(await page.locator('#bound-low').inputValue(), '1800');
    assert.match(await page.locator('#bounds-status').textContent(), /1800.*2300/);

    await page.locator('#bound-low').fill('1700');
    await page.locator('[data-joint="shoulder_lift"] .joint-name').click();
    assert.equal(await page.locator('#bound-low').inputValue(), '1953');
    await page.locator('[data-joint="shoulder_pan"] .joint-name').click();
    assert.equal(await page.locator('#bound-low').inputValue(), '1700');
    failSave = true;
    await page.locator('#save-bounds').click();
    await page.waitForFunction(() =>
      document.querySelector('#notice').textContent.includes('EEPROM'),
    );
    await page.waitForTimeout(350);
    assert.equal(await page.locator('#bound-low').inputValue(), '1700', 'rejection retains draft');
    assert.match(await page.locator('#bounds-status').textContent(), /尚未应用/);
    await page.locator('#bound-low').fill('');
    await page.locator('#bound-high').focus();
    await page.waitForTimeout(250);
    assert.equal(await page.locator('#bound-low').inputValue(), '');
    const count = commands.length;
    await page.locator('#save-bounds').click();
    await page.waitForFunction(() =>
      document.querySelector('#notice').textContent.includes('整数'),
    );
    assert.equal(commands.length, count, 'empty bound must not become zero');

    state.enabled = ['shoulder_pan'];
    await page.waitForFunction(() => document.querySelector('#save-bounds').disabled);
    assert.match(await page.locator('#bounds-status').textContent(), /释放.*扭矩/);
    state.enabled = [];
    state.motion = 'moving';
    await page.waitForFunction(() =>
      document.querySelector('#bounds-status').textContent.includes('停止'),
    );
    state.motion = 'idle';
    await page.locator('#motion-confirmed').uncheck();
    await page.waitForFunction(() =>
      document.querySelector('#bounds-status').textContent.includes('勾选'),
    );
    assert.equal(await page.locator('#save-bounds').isDisabled(), true);
    assert.deepEqual(errors, []);
    assert.ok(commands.every((c) => ['bounds', 'parameters'].includes(c.operation)));
    console.log(
      'PASS: range drafts survive polling, blur, joint switches, save and rejection; guards explained; no hardware API used.',
    );
  } finally {
    await browser.close();
  }
})().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
