import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RobotProtocol, scmd, setGain, sanitizeRaw, tagOf, crc8, withChecksum, checkLine } from '../src/protocol.js';
import { DriveControl } from '../src/drive.js';
import { JpegSplitter } from '../src/camera.js';
import { loadConfig } from '../src/config.js';
import { whepProxy } from '../src/webrtc.js';
import express from 'express';
import { Gamepad, findController, driveFrom, parseEvents, encodeEvent, eventSize, ABS, BTN, EV_ABS, EV_KEY } from '../src/gamepad.js';
import { FakeGamepad } from './fixtures/fake-gamepad.js';
import path from 'node:path';

const TH = 'TH,yaw,pitch,roll,pitchSpeed,dISTE,leftMotorSpeed,rightMotorSpeed,LoopT,bal';
const PIDH = 'PIDH,speedPIDKp,speedPIDKi,speedPIDKd,anglePIDConKp,anglePIDConKi,anglePIDConKd,'
  + 'anglePIDAggKp,anglePIDAggKi,anglePIDAggKd,TriggerAngleAggressive,calibratedZeroAngle,OPT';

test('tags come from the first field only', () => {
  // The old server matched anywhere in the line, so these were all "telemetry".
  assert.equal(tagOf('SCMD Throttle 50'), 'SCMD');
  assert.equal(tagOf(PIDH), 'PIDH');
  assert.equal(tagOf('T,1,2'), 'T');
  assert.equal(tagOf('E, Unknown command'), 'E');
});

test('lines carry a CRC-8 checksum', () => {
  assert.equal(crc8('123456789'), 0xf4);                       // CRC-8 (poly 0x07) check value
  assert.equal(withChecksum('SCMD Motors 1'), `SCMD Motors 1*${crc8('SCMD Motors 1').toString(16).toUpperCase().padStart(2, '0')}`);
  assert.deepEqual(checkLine(withChecksum('EV,Ready')), { line: 'EV,Ready', ok: true });
  assert.deepEqual(checkLine('EV,Ready'), { line: 'EV,Ready', ok: true });          // none: accepted
  const bad = withChecksum('T,0.0,1.5').replace('1.5', '1.6');
  assert.equal(checkLine(bad).ok, false);
  const p = new RobotProtocol();
  assert.equal(p.parse(bad).type, 'corrupt');
  assert.equal(p.parse(withChecksum('EV,Ready')).text, 'Ready');
});

test('telemetry is mapped onto the announced header', () => {
  const p = new RobotProtocol();
  assert.equal(p.parse('T,1,2').type, 'unknown');          // no header yet
  assert.equal(p.parse(TH + '\r').type, 'header');
  const msg = p.parse('T,12.5,-1.25,0.3,4.0,0.002,10.0,11.0,1830,2\r');
  assert.equal(msg.type, 'telemetry');
  assert.deepEqual(msg.values, {
    yaw: 12.5, pitch: -1.25, roll: 0.3, pitchSpeed: 4, dISTE: 0.002,
    leftMotorSpeed: 10, rightMotorSpeed: 11, LoopT: 1830, bal: 2, state: 'BAL',
  });
});

test('gains are scaled back to real units', () => {
  const p = new RobotProtocol();
  p.parse(PIDH);
  const msg = p.parse('PID,1500,200,0,1200,1200,130,1560,1560,169,800,-150,2');
  assert.equal(msg.type, 'pid');
  assert.equal(msg.gains.speedPIDKp, 0.15);
  assert.equal(msg.gains.anglePIDConKd, 1.3);
  assert.equal(msg.gains.calibratedZeroAngle, -1.5);
  assert.equal(msg.raw.anglePIDConKp, 1200);
});

test('system values stay text', () => {
  const p = new RobotProtocol();
  p.parse('SYSH,FirmwareVersion');
  assert.equal(p.parse('SYS,2.0').values.FirmwareVersion, '2.0');
});

test('events, errors, autotune and motor tests', () => {
  const p = new RobotProtocol();
  assert.deepEqual(p.parse('EV,Balancing'), { type: 'event', text: 'Balancing' });
  assert.deepEqual(p.parse('E, autotune aborted: driving'), { type: 'error', text: 'autotune aborted: driving' });
  assert.deepEqual(p.parse('AT,MEASURE'), { type: 'autotune', phase: 'MEASURE' });
  const r = p.parse('AT,RESULT,1,0.305,1.320,1.45,0.99,0.0214,0.0006');
  assert.equal(r.result.accepted, true);
  assert.equal(r.result.length, 0.305);
  assert.equal(r.result.coupling, 1.32);
  assert.equal(r.result.zero, 1.45);
  assert.deepEqual(p.parse('TEST,40.0,39.5'), { type: 'motorTest', left: 40, right: 39.5 });
  assert.deepEqual(p.parse('LQR,-8.926,-15.944,-105.718,-11.577,0.200'),
    { type: 'lqr', gain: [-8.926, -15.944, -105.718, -11.577], length: 0.2 });
  assert.equal(p.parse('*** speedRamp = 30.0000').type, 'config');
  assert.equal(p.parse(''), null);
});

test('commands are validated and formatted for the firmware', () => {
  assert.equal(scmd('Motors', 1), 'SCMD Motors 1');
  assert.equal(scmd('Throttle', -42), 'SCMD Throttle -42');
  assert.equal(scmd('speedFilter', 0.125), 'SCMD speedFilter 0.125');
  assert.equal(setGain('anglePIDConKp', 12.345), 'SCMD anglePIDConKp 1235');
  assert.equal(setGain('speedPIDKi', 0.02), 'SCMD speedPIDKi 200');
  assert.throws(() => scmd('Motors 1\nSCMD Motors', 1));
  assert.throws(() => scmd('Motors', 'abc'));
  assert.throws(() => scmd('Motors', Infinity));
});

test('raw console input is one safe line', () => {
  assert.equal(sanitizeRaw('  SCMD Motors 1  '), 'SCMD Motors 1');
  assert.equal(sanitizeRaw('SCMD Motors 1\nSCMD Throttle 100'), 'SCMD Motors 1SCMD Throttle 100');
  assert.throws(() => sanitizeRaw('rm -rf /'));
  assert.throws(() => sanitizeRaw('SCMD ' + 'x'.repeat(80)));
});

test('drive: repeats while held, stops when released or abandoned', async (t) => {
  t.mock.timers.enable({ apis: ['setInterval', 'Date'] });
  const sent = [];
  const d = new DriveControl((l) => sent.push(l));
  d.set('a', 50, -20);
  assert.deepEqual(sent, ['SCMD Throttle 50', 'SCMD Steer -20']);
  sent.length = 0;

  // The client keeps updating: commands keep being repeated.
  for (let i = 0; i < 5; i++) {
    t.mock.timers.tick(100);
    d.set('a', 50, -20);
  }
  assert.ok(sent.filter((l) => l === 'SCMD Throttle 50').length >= 4);

  // Another client can't stop someone else's drive by disconnecting.
  d.release('b');
  assert.equal(d.throttle, 50);

  // The client stops updating (browser frozen, Wi-Fi gone): stop.
  sent.length = 0;
  t.mock.timers.tick(1000);
  assert.ok(sent.includes('SCMD Throttle 0'));
  assert.equal(d.throttle, 0);

  // Disconnect while driving: stop.
  d.set('a', 100, 0);
  sent.length = 0;
  d.release('a');
  assert.deepEqual(sent, ['SCMD Throttle 0', 'SCMD Steer 0']);
  assert.equal(d.timer, null);

  // Out-of-range input is clamped.
  d.set('a', 500, -500);
  assert.equal(d.throttle, 100);
  assert.equal(d.steer, -100);
  d.stop();
});

test('JPEG splitter finds frames across chunk boundaries', () => {
  const frames = [];
  const s = new JpegSplitter((f) => frames.push(f));
  const jpeg = (n) => Buffer.from([0xff, 0xd8, n, n, 0xff, 0x00, n, 0xff, 0xd9]);
  const stream = Buffer.concat([Buffer.from([1, 2, 3]), jpeg(1), jpeg(2), jpeg(3)]);
  for (let i = 0; i < stream.length; i += 4) s.push(stream.subarray(i, i + 4));
  assert.equal(frames.length, 3);
  assert.deepEqual(frames[1], jpeg(2));
});

test('config: flags override the file', () => {
  const c = loadConfig(['--sim', '--port', '8080', '--no-camera']);
  assert.equal(c.serial.port, 'sim');
  assert.equal(c.http.port, 8080);
  assert.equal(c.camera.enabled, false);
  assert.equal(c.serial.baudRate, 115200);
  assert.throws(() => loadConfig(['--bogus']));
});

test('camera streams MJPEG to several viewers and stops when nobody watches', async () => {
  const { createApp } = await import('../src/app.js');
  const { EventEmitter } = await import('node:events');
  const config = loadConfig(['--serial', '/dev/null-robot']);
  config.camera.mode = 'mjpeg';
  config.camera.command = new URL('./fixtures/fake-camera.js', import.meta.url).pathname;
  // No robot needed for this test.
  const link = Object.assign(new EventEmitter(), { start() {}, close() {}, write: () => false });
  const quiet = { debug() {}, info() {}, warn() {}, error() {} };
  const bailey = createApp(config, { log: quiet, link });
  const port = await bailey.start(0);
  try {
    const read = async () => {
      const res = await fetch(`http://localhost:${port}/stream.mjpg`);
      assert.match(res.headers.get('content-type'), /multipart\/x-mixed-replace; boundary=baileyframe/);
      const reader = res.body.getReader();
      let text = '';
      while ((text.match(/--baileyframe/g) || []).length < 3) {
        const { value } = await reader.read();
        text += Buffer.from(value).toString('latin1');
      }
      await reader.cancel();
      return text;
    };
    const [a, b] = await Promise.all([read(), read()]);
    for (const t of [a, b]) assert.match(t, /Content-Type: image\/jpeg\r\nContent-Length: 10\r\n\r\n\xff\xd8/);
    assert.equal(bailey.camera.status, 'running');
    const snap = await fetch(`http://localhost:${port}/snapshot.jpg`);
    assert.equal(snap.headers.get('content-type'), 'image/jpeg');
    assert.equal((await snap.arrayBuffer()).byteLength, 10);
  } finally {
    await bailey.close();
  }
});

test('gamepad: input events, sticks and device discovery', () => {
  // struct input_event is 24 bytes on 64-bit Pi OS, 16 on 32-bit.
  assert.equal(eventSize('arm64'), 24);
  assert.equal(eventSize('arm'), 16);
  for (const size of [16, 24]) {
    const buf = Buffer.concat([encodeEvent(EV_ABS, ABS.Y, 0, size), encodeEvent(EV_KEY, BTN.SOUTH, 1, size), encodeEvent(EV_ABS, ABS.RX, -3, size)]);
    assert.deepEqual(parseEvents(buf, size), [
      { type: EV_ABS, code: ABS.Y, value: 0 }, { type: EV_KEY, code: BTN.SOUTH, value: 1 }, { type: EV_ABS, code: ABS.RX, value: -3 }]);
  }

  const state = (y, rx, ...buttons) => ({ axes: { [ABS.Y]: y, [ABS.RX]: rx }, buttons: new Set(buttons) });
  assert.deepEqual(driveFrom(state(128, 128)), { throttle: 0, steer: 0 });
  assert.deepEqual(driveFrom(state(140, 118)), { throttle: 0, steer: 0 });       // dead zone
  assert.deepEqual(driveFrom(state(0, 255)), { throttle: 50, steer: 60 });       // up = forward
  assert.deepEqual(driveFrom(state(255, 0)), { throttle: -50, steer: -60 });
  assert.deepEqual(driveFrom(state(0, 128, BTN.TR)), { throttle: 100, steer: 0 }); // R1: full speed
  assert.ok(driveFrom(state(100, 128)).throttle > 0 && driveFrom(state(100, 128)).throttle < 10);

  const pad = new FakeGamepad();
  try {
    assert.deepEqual(findController(pad.sysDir, pad.inputDir), {
      name: 'Wireless Controller', address: 'a4:ae:12:34:56:78',
      main: path.join(pad.inputDir, 'event5'), sensors: path.join(pad.inputDir, 'event6'),
    });
    assert.equal(findController(path.join(pad.root, 'missing'), pad.inputDir), null);
  } finally {
    pad.close();
  }
});

test('gamepad: drives, presses buttons and stops when the controller goes quiet', async () => {
  const fake = new FakeGamepad();
  const gamepad = new Gamepad({}, { log: { info() {}, warn() {} }, sysDir: fake.sysDir, inputDir: fake.inputDir });
  const seen = [];
  for (const ev of ['drive', 'command', 'lost', 'status']) gamepad.on(ev, (d) => seen.push([ev, d]));
  const wait = (ms) => new Promise((r) => setTimeout(r, ms));
  try {
    gamepad.start();
    assert.equal(gamepad.status.connected, false);    // devices open, no reports yet
    fake.connect();
    await wait(60);
    assert.equal(gamepad.status.connected, true);

    fake.press(BTN.SOUTH);
    fake.stick(ABS.Y, 0);
    await wait(250);
    assert.deepEqual(seen.find(([e]) => e === 'command'), ['command', { name: 'Motors', value: 1 }]);
    const drives = seen.filter(([e]) => e === 'drive');
    assert.deepEqual(drives[0][1], { throttle: 50, steer: 0 });
    assert.ok(drives.length >= 2, 'repeated while the stick is held');

    fake.stick(ABS.Y, 128);
    await wait(60);
    assert.deepEqual(seen.filter(([e]) => e === 'drive').at(-1)[1], { throttle: 0, steer: 0 });

    // Out of range: the reports stop and the controller is dropped quickly.
    fake.stick(ABS.Y, 0);
    await wait(60);
    fake.silence();
    await wait(700);
    assert.equal(seen.filter(([e]) => e === 'lost').length, 1);
    assert.equal(gamepad.status.connected, false);
    assert.deepEqual(seen.filter(([e]) => e === 'drive').at(-1)[1], { throttle: 50, steer: 0 }, 'no repeats after it was lost');

    // Back in range: connected again, but the stick has to be moved again.
    const before = seen.length;
    fake.connect();
    await wait(200);
    assert.equal(gamepad.status.connected, true);
    assert.equal(seen.slice(before).filter(([e]) => e === 'drive').length, 0);

    // Bluetooth drops the device: closed, and found again when it reappears.
    fake.remove();
    gamepad.disconnect('ENODEV');
    assert.equal(gamepad.status.connected, false);
  } finally {
    gamepad.stop();
    fake.close();
  }
});

test('WebRTC: only the WHEP handshake is forwarded to MediaMTX', async () => {
  const calls = [];
  const fakeFetch = async (url, init) => {
    calls.push({ url, method: init.method, body: init.body });
    return new Response('v=0 answer', { status: 201, headers: { 'content-type': 'application/sdp', location: '/cam/whep/abc123' } });
  };
  const app = express();
  app.use('/mediamtx', whepProxy('http://127.0.0.1:8889', { log: { warn() {} }, fetchImpl: fakeFetch }));
  const server = app.listen(0);
  const base = `http://localhost:${server.address().port}/mediamtx`;
  try {
    const res = await fetch(`${base}/cam/whep`, { method: 'POST', headers: { 'content-type': 'application/sdp' }, body: 'v=0 offer' });
    assert.equal(res.status, 201);
    assert.equal(await res.text(), 'v=0 answer');
    assert.equal(res.headers.get('location'), '/mediamtx/cam/whep/abc123');   // back through this server
    assert.deepEqual(calls[0], { url: 'http://127.0.0.1:8889/cam/whep', method: 'POST', body: 'v=0 offer' });
    assert.equal((await fetch(`${base}/cam/whep/abc123`, { method: 'DELETE' })).status, 201);
    // Anything else is not forwarded: other paths, other methods.
    for (const [path, method] of [['v3/config/global/get', 'GET'], ['cam/whep', 'GET'], ['other/whep', 'POST'], ['cam/whep/a/b', 'POST']]) {
      assert.equal((await fetch(`${base}/${path}`, { method })).status, 404, `${method} ${path}`);
    }
    assert.equal(calls.length, 2);
  } finally {
    server.close();
  }
  // MediaMTX not running: a clear 502.
  const down = express();
  down.use('/mediamtx', whepProxy('http://127.0.0.1:1', { log: { warn() {} } }));
  const s2 = down.listen(0);
  try {
    const res = await fetch(`http://localhost:${s2.address().port}/mediamtx/cam/whep`, { method: 'POST', headers: { 'content-type': 'application/sdp' }, body: 'x' });
    assert.equal(res.status, 502);
  } finally {
    s2.close();
  }
});
