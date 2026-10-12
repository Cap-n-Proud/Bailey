import { test } from 'node:test';
import assert from 'node:assert/strict';
import { RobotProtocol, scmd, setGain, sanitizeRaw, tagOf } from '../src/protocol.js';
import { DriveControl } from '../src/drive.js';
import { JpegSplitter } from '../src/camera.js';
import { loadConfig } from '../src/config.js';

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
  const r = p.parse('AT,RESULT,1,0.202,0.202,0.202,1.72,0.99,0.0027,0.0002');
  assert.equal(r.result.accepted, true);
  assert.equal(r.result.length, 0.202);
  assert.equal(r.result.zero, 1.72);
  assert.deepEqual(p.parse('TEST,40.0,39.5'), { type: 'motorTest', left: 40, right: 39.5 });
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
