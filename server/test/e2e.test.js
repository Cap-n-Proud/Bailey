// End to end: the real server, a socket.io client like the web UI, and the
// firmware simulator (bailey_FW/test, "live" mode) standing in for the robot.
// Takes about a minute (it runs in real time, including a full autotune).
import { test, before, after } from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { io as connect } from 'socket.io-client';
import { createApp } from '../src/app.js';
import { loadConfig, SERVER_DIR } from '../src/config.js';
import { Gamepad, ABS, BTN } from '../src/gamepad.js';
import { FakeGamepad } from './fixtures/fake-gamepad.js';

const SIM_DIR = path.resolve(SERVER_DIR, '../bailey_FW/test');
const SIM = path.join(SIM_DIR, 'sim');
const quiet = { debug() {}, info() {}, warn() {}, error: console.error };

let bailey, url, logDir, pad, skip = false;
const clients = [];

before(async () => {
  try {
    // Always rebuild so the test runs the current firmware.
    execFileSync('sh', ['build.sh'], { cwd: SIM_DIR });
  } catch (err) {
    skip = `can't build the simulator: ${err.message}`;
    return;
  }
  logDir = fs.mkdtempSync(path.join(os.tmpdir(), 'bailey-'));
  const config = loadConfig(['--sim', '--no-camera']);
  config.telemetry.logDir = logDir;
  pad = new FakeGamepad();
  const gamepad = new Gamepad(config.gamepad, { log: quiet, sysDir: pad.sysDir, inputDir: pad.inputDir });
  bailey = createApp(config, { log: quiet, gamepad });
  const port = await bailey.start(0);
  url = `http://localhost:${port}`;
});

after(async () => {
  // Also the ones a failed test didn't get to close: they would keep reconnecting.
  for (const c of clients) c.disconnect();
  if (bailey) await bailey.close();
  if (pad) pad.close();
  if (logDir) fs.rmSync(logDir, { recursive: true, force: true });
});

function client() {
  const socket = connect(url, { transports: ['websocket'] });
  clients.push(socket);
  socket.latest = {};
  socket.onAny((event, data) => { socket.latest[event] = data; });
  return socket;
}

function ask(socket, event, payload) {
  return socket.timeout(3000).emitWithAck(event, payload);
}

async function waitFor(fn, ms, what) {
  const end = Date.now() + ms;
  for (;;) {
    const v = fn();
    if (v) return v;
    if (Date.now() > end) throw new Error(`timed out waiting for ${what}`);
    await new Promise((r) => setTimeout(r, 50));
  }
}

const speedOf = (t) => (t.leftMotorSpeed + t.rightMotorSpeed) / 2;

test('the robot balances, drives and stops from the web API', async (t) => {
  if (skip) return t.skip(skip);
  const ui = client();
  await waitFor(() => ui.latest.snapshot, 3000, 'snapshot');
  await waitFor(() => ui.latest.telemetry?.state === 'OFF' && ui.latest.gains && ui.latest.lqr, 6000, 'telemetry, gains and LQR gain');
  assert.equal(ui.latest.gains.gains.anglePIDConKp, 10.89);   // defaults: L 0.3 m, kappa 1.35
  assert.equal(ui.latest.gains.gains.controller, 1);              // steppers balance with LQR
  assert.equal(ui.latest.lqr.length, 0.3);
  await waitFor(() => ui.latest.snapshot && (ui.latest.firmware || ui.latest.snapshot.robot.firmware), 3000, 'firmware info');
  const fw = ui.latest.firmware || ui.latest.snapshot.robot.firmware;
  assert.equal(fw.Board, 'ESP32-S3');
  assert.equal(fw.ControlPeriodUs, '2000');
  assert.ok(ui.latest.lqr.gain[2] < -50, 'LQR pitch gain');

  assert.equal((await ask(ui, 'command', { name: 'Motors', value: 1 })).ok, true);
  await waitFor(() => ui.latest.telemetry?.state === 'BAL', 8000, 'balancing');

  // Hold the joystick forward for 3 s, the way the UI does (10 Hz).
  for (let i = 0; i < 30; i++) {
    ui.emit('drive', { throttle: 100, steer: 0 });
    await new Promise((r) => setTimeout(r, 100));
  }
  assert.ok(speedOf(ui.latest.telemetry) > 10, `drives forward (speed ${speedOf(ui.latest.telemetry)})`);

  // The browser disappears mid-drive: the robot must stop by itself.
  ui.disconnect();
  const watcher = client();
  await waitFor(() => watcher.latest.telemetry, 3000, 'telemetry');
  // Throttle drops to 0 at once; the robot then brakes and settles.
  await waitFor(() => Math.abs(speedOf(watcher.latest.telemetry)) < 3, 10000, 'the robot to stop');
  assert.equal(watcher.latest.telemetry.state, 'BAL');
  watcher.disconnect();
});

test('a PS4 controller drives the robot and stops it when out of range', async (t) => {
  if (skip) return t.skip(skip);
  const ui = client();
  await waitFor(() => ui.latest.snapshot, 3000, 'snapshot');
  assert.equal(ui.latest.snapshot.gamepad.connected, false);
  pad.connect();
  await waitFor(() => ui.latest.gamepad?.connected, 3000, 'gamepad connected');
  assert.equal(ui.latest.gamepad.name, 'Wireless Controller');

  pad.press(BTN.EAST);                                          // ○: motors off
  await waitFor(() => ui.latest.telemetry?.state === 'OFF', 3000, 'motors off');
  pad.press(BTN.SOUTH);                                         // ✕: motors on
  await waitFor(() => ui.latest.telemetry?.state === 'BAL', 8000, 'balancing');

  pad.send([1, BTN.TR, 1], [3, ABS.Y, 0]);                      // R1 + left stick fully up
  await new Promise((r) => setTimeout(r, 3000));
  assert.ok(speedOf(ui.latest.telemetry) > 10, `drives forward (speed ${speedOf(ui.latest.telemetry)})`);

  // Out of range with the stick still held: the robot must stop by itself.
  pad.silence();
  await waitFor(() => ui.latest.gamepad && !ui.latest.gamepad.connected, 2000, 'gamepad lost');
  await waitFor(() => Math.abs(speedOf(ui.latest.telemetry)) < 3, 10000, 'the robot to stop');
  assert.equal(ui.latest.telemetry.state, 'BAL');
  ui.disconnect();
});

test('gains, console, recording and HTTP endpoints', async (t) => {
  if (skip) return t.skip(skip);
  const ui = client();
  await waitFor(() => ui.latest.snapshot, 3000, 'snapshot');

  assert.equal((await ask(ui, 'set', { name: 'anglePIDConKd', value: 1.5 })).ok, true);
  await waitFor(() => ui.latest.gains?.gains.anglePIDConKd === 1.5, 3000, 'new gain echoed by the robot');

  const bad = await ask(ui, 'raw', 'SCMD Motors 1\nREAD x; reboot');
  assert.equal(bad.ok, true);                                   // newline stripped: one harmless line
  assert.equal((await ask(ui, 'raw', 'shutdown now')).ok, false);
  assert.equal((await ask(ui, 'command', { name: 'Motors\n', value: 1 })).ok, false);

  assert.equal((await ask(ui, 'logging', true)).ok, true);
  await new Promise((r) => setTimeout(r, 1500));
  const { file, rows } = (await ask(ui, 'logging', false));
  const csv = fs.readFileSync(file, 'utf8').trim().split('\n');
  assert.match(csv[0], /^time_ms,yaw,pitch,roll,.*anglePIDConKp/);
  assert.ok(rows >= 8 && csv.length === rows + 1, `${rows} rows at 10 Hz`);

  const status = await (await fetch(`${url}/api/status`)).json();
  assert.equal(status.robot.link.connected, true);
  assert.equal((await fetch(`${url}/stream.mjpg`)).status, 503);   // camera disabled
  assert.match(await (await fetch(`${url}/`)).text(), /<title>Bailey<\/title>/);
  ui.disconnect();
});

test('autotune measures the simulated robot', async (t) => {
  if (skip) return t.skip(skip);
  const ui = client();
  await waitFor(() => ui.latest.telemetry?.state === 'BAL', 8000, 'balancing');
  const phases = [];
  ui.on('autotune', (a) => phases.push(a.phase));
  assert.equal((await ask(ui, 'command', { name: 'AUTOTUNE', value: 1 })).ok, true);
  const result = await waitFor(() => ui.latest.autotune?.phase === 'IDLE' && ui.latest.autotune.result, 45000, 'autotune result');
  assert.deepEqual(phases.slice(0, 3), ['BASELINE', 'MEASURE', 'VERIFY']);
  assert.equal(result.accepted, true);
  // The simulated robot is Bailey: L = 0.31 m, kappa = 1.38 (bailey_FW/test/sim.cpp, Plant).
  assert.ok(Math.abs(result.length - 0.31) < 0.04, `L = ${result.length} (true 0.31)`);   // +/- 13 %
  assert.ok(Math.abs(result.coupling - 1.38) < 0.2, `kappa = ${result.coupling} (true 1.38)`);
  await waitFor(() => Math.abs(ui.latest.gains?.gains.calibratedZeroAngle - result.zero) < 0.02, 3000, 'zero applied');
  await waitFor(() => Math.abs(ui.latest.lqr?.length - result.length) < 0.002, 3000, 'LQR gain rescaled to the measured length');

  assert.equal((await ask(ui, 'command', { name: 'Motors', value: 0 })).ok, true);
  await waitFor(() => ui.latest.telemetry?.state === 'OFF', 3000, 'motors off');
  ui.disconnect();
});
