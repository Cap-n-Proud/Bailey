// Bailey web UI. Talks to the server over socket.io (API documented in src/app.js).
'use strict';

const $ = (id) => document.getElementById(id);
const socket = io();

let robot = { telemetry: null, gains: null, autotune: { phase: 'IDLE' }, link: {} };
let allowShutdown = false;
let cameraEnabled = true;

// ---------- helpers ----------
function toast(text, ms = 2500) {
  const el = $('toast');
  el.textContent = text;
  el.classList.add('show');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => el.classList.remove('show'), ms);
}

function emit(event, payload) {
  return new Promise((resolve) => {
    socket.timeout(3000).emit(event, payload, (err, res) => {
      if (err) res = { ok: false, error: 'no answer from the server' };
      if (!res.ok) toast(res.error);
      resolve(res);
    });
  });
}

const command = (name, value = 1) => emit('command', { name, value });
// Number with d decimals; never "-0.0".
const fmt = (v, d = 1) => {
  if (typeof v !== 'number' || !Number.isFinite(v)) return '–';
  const s = v.toFixed(d);
  return Number(s) === 0 ? (0).toFixed(d) : s;
};
const timeOf = (t) => new Date(t).toLocaleTimeString([], { hour12: false });

// ---------- header ----------
function renderLink() {
  const l = robot.link || {};
  const el = $('link');
  el.textContent = l.connected ? `robot: ${l.port === 'sim' ? 'simulator' : 'connected'}` : 'robot: offline';
  el.title = l.error || l.port || '';
  el.className = 'chip ' + (l.connected ? 'ok' : 'bad');
}

function renderState() {
  const s = robot.telemetry?.state ?? '—';
  const el = $('state');
  el.textContent = s;
  el.className = 'chip state ' + ({ BAL: 'ok', WAIT: 'warn', FALL: 'bad' }[s] || '');
  const on = s !== 'OFF' && s !== '—';
  $('motorsBtn').textContent = on ? 'Motors off' : 'Motors on';
  $('motorsBtn').classList.toggle('on', on);
  $('motorTestBtn').disabled = s !== 'OFF';
  $('autotuneBtn').disabled = s !== 'BAL' && robot.autotune.phase === 'IDLE';
  $('joystick').classList.toggle('disabled', s !== 'BAL');
}

// ---------- telemetry & charts ----------
const HISTORY_S = 20;
const history = { pitch: [], speed: [] };

function css(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

function drawChart(canvas, points, fixedRange) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth, h = canvas.clientHeight || 110;
  if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
    canvas.width = w * dpr;
    canvas.height = h * dpr;
  }
  const ctx = canvas.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, w, h);
  const now = performance.now() / 1000;
  let lo = -fixedRange, hi = fixedRange;
  for (const p of points) { lo = Math.min(lo, p.v); hi = Math.max(hi, p.v); }
  const pad = 18;
  const y = (v) => pad / 2 + (h - pad) * (1 - (v - lo) / (hi - lo));
  const x = (t) => 32 + (w - 36) * (1 - (now - t) / HISTORY_S);

  ctx.font = '11px system-ui';
  ctx.fillStyle = css('--text-3');
  ctx.strokeStyle = css('--grid');
  ctx.lineWidth = 1;
  for (const v of [lo, 0, hi]) {
    ctx.beginPath();
    ctx.moveTo(32, y(v));
    ctx.lineTo(w, y(v));
    ctx.stroke();
    ctx.fillText(v.toFixed(Math.abs(hi - lo) < 10 ? 1 : 0), 0, y(v) + 4);
  }
  ctx.strokeStyle = css('--series');
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  points.forEach((p, i) => (i ? ctx.lineTo(x(p.t), y(p.v)) : ctx.moveTo(x(p.t), y(p.v))));
  ctx.stroke();
}

function onTelemetry(t) {
  robot.telemetry = t;
  const now = performance.now() / 1000;
  const speed = (t.leftMotorSpeed + t.rightMotorSpeed) / 2;
  history.pitch.push({ t: now, v: t.pitch });
  history.speed.push({ t: now, v: speed });
  for (const k in history) while (history[k].length && history[k][0].t < now - HISTORY_S) history[k].shift();

  $('pitch').textContent = `${fmt(t.pitch, 2)}°`;
  $('speed').textContent = `${fmt(speed)} %`;
  $('loopTime').textContent = `${t.LoopT} µs`;
  // Budget: the firmware's control period (5000 µs on the Mega, 2000 µs on the ESP32-S3).
  const budget = Number(robot.firmware?.ControlPeriodUs) || 5000;
  $('loopTime').style.color = t.LoopT > budget ? css('--bad') : '';
  $('loopTime').title = `Worst control loop time since the last telemetry; must stay under ${budget} µs`;
  $('wobble').textContent = fmt(t.dISTE, 3);
  $('balAngle').textContent = typeof t.balAngle === 'number' ? `${fmt(t.balAngle, 2)}°` : '–';
  $('hudPitch').textContent = fmt(t.pitch, 1);
  $('hudHeading').textContent = fmt(t.yaw, 0);
  $('horizon').style.transform = `rotate(${-t.pitch}deg)`;
  renderState();
}

function animate() {
  drawChart($('pitchChart'), history.pitch, 2);
  drawChart($('speedChart'), history.speed, 10);
  requestAnimationFrame(animate);
}
requestAnimationFrame(animate);

// ---------- firmware ----------
function renderFirmware() {
  const f = robot.firmware;
  $('firmware').textContent = `firmware ${f.FirmwareVersion}${f.Board ? ` · ${f.Board}` : ''}`;
  renderController();
}

// ---------- controller ----------
function renderController() {
  const c = robot.gains?.controller;
  const isStepper = robot.gains?.OPT === 2;
  $('ctrlLqr').classList.toggle('on', c === 1);
  $('ctrlMpc').classList.toggle('on', c === 2);
  $('ctrlPid').classList.toggle('on', c === 0);
  $('ctrlLqr').disabled = !isStepper;
  $('ctrlMpc').disabled = !isStepper || robot.firmware?.Board !== 'ESP32-S3';
  const l = robot.lqr;
  $('lqrInfo').textContent = (c === 1 || c === 2) && l
    ? `K = ${l.gain.map((k) => k.toFixed(1)).join(', ')} for L = ${(l.length * 100).toFixed(1)} cm`
    : '';
}
for (const b of document.querySelectorAll('[data-controller]')) {
  b.onclick = () => {
    if (robot.telemetry?.state === 'BAL') { toast('Switch the controller with the motors off'); return; }
    command('controller', Number(b.dataset.controller));
  };
}

// ---------- gains ----------
function renderGains() {
  if (!robot.gains) return;
  renderController();
  for (const input of document.querySelectorAll('[data-gain]')) {
    if (document.activeElement === input) continue;
    const v = robot.gains[input.dataset.gain];
    input.value = typeof v === 'number' ? +v.toFixed(4) : '';
  }
}

for (const input of document.querySelectorAll('[data-gain]')) {
  input.type = 'number';
  input.step = 'any';
  input.addEventListener('change', async () => {
    const res = await emit('set', { name: input.dataset.gain, value: Number(input.value) });
    if (res.ok) toast(`${input.dataset.gain} = ${input.value} (not saved: use Save to EEPROM)`);
  });
}

// ---------- autotune ----------
const PHASES = { BASELINE: [0, 6], MEASURE: [6, 18], VERIFY: [24, 6] };   // start s, duration s
let phaseStarted = 0;

function renderAutotune() {
  const a = robot.autotune;
  const running = a.phase !== 'IDLE';
  $('autotuneBtn').textContent = running ? 'Stop' : 'Autotune';
  $('autotuneBtn').classList.toggle('on', running);
  const text = {
    BASELINE: 'Measuring how it balances now…',
    MEASURE: 'Wobbling gently to measure the robot…',
    VERIFY: 'Trying the new settings…',
  }[a.phase];
  if (running) {
    $('autotuneText').textContent = text;
  } else if (a.result) {
    const r = a.result;
    $('autotuneText').textContent = r.accepted
      ? `Done: pendulum length ${(r.length * 100).toFixed(1)} cm, zero ${r.zero.toFixed(2)}°. Wobble ${r.before} → ${r.after}. Save to keep it.`
      : `Rejected: the measurement wasn't consistent (${(r.lengthFromGravity * 100).toFixed(0)} vs ${(r.lengthFromAccel * 100).toFixed(0)} cm). Nothing changed.`;
    $('lengthInput').value = r.accepted ? Math.round(r.length * 1000) : $('lengthInput').value;
  }
  renderState();
}

setInterval(() => {
  const p = PHASES[robot.autotune.phase];
  const bar = $('autotuneBar');
  if (!p) { bar.style.width = robot.autotune.result ? '100%' : '0'; return; }
  const elapsed = Math.min(p[1], (Date.now() - phaseStarted) / 1000);
  bar.style.width = `${((p[0] + elapsed) / 30) * 100}%`;
}, 250);

$('autotuneBtn').onclick = () => command('AUTOTUNE', robot.autotune.phase === 'IDLE' ? 1 : 0);

// ---------- buttons ----------
$('motorsBtn').onclick = () => command('Motors', robot.telemetry?.state === 'OFF' ? 1 : 0);
$('stopBtn').onclick = () => { stopDriving(); command('Motors', 0); };
$('calibrateBtn').onclick = () => {
  if (confirm('Hold the robot still at its balance point, then press OK.')) command('CalibrateZero');
};
$('motorTestBtn').onclick = () => {
  if (confirm('Wheels off the ground? Both wheels will run forward at 30 % for 1 s.')) command('MotorTest', 30);
};
$('saveBtn').onclick = async () => { if ((await command('SaveCfg')).ok) toast('Saved to EEPROM'); };
$('defaultsBtn').onclick = () => { if (confirm('Load the default settings? (Not saved until you save.)')) command('Load_def'); };
$('lengthForm').onsubmit = (e) => {
  e.preventDefault();
  const mm = Number($('lengthInput').value);
  if (mm) command('pendulumLength', mm);
};
for (const b of document.querySelectorAll('[data-read]')) b.onclick = () => emit('read', b.dataset.read);

// ---------- video ----------
let videoOn = false;
function setVideo(on) {
  videoOn = on;
  $('video').classList.toggle('on', on);
  $('videoImg').src = on ? `stream.mjpg?t=${Date.now()}` : '';
  $('videoBtn').textContent = on ? 'Video off' : 'Video on';
}
$('videoBtn').onclick = () => setVideo(!videoOn);
$('videoImg').onerror = () => {
  if (videoOn) { setVideo(false); toast('No video: ' + ($('cameraStatus').textContent || 'camera unavailable')); }
};
function renderCamera(c) {
  $('cameraStatus').textContent = c.enabled === false ? 'camera disabled in config' : c.error || (c.status === 'running' ? '' : c.status);
}

// ---------- driving ----------
const DRIVE_HZ = 10;
let drive = { throttle: 0, steer: 0 };
let driveTimer = null;

function sendDrive() {
  socket.volatile.emit('drive', drive);
}

function setDrive(throttle, steer) {
  drive = { throttle: Math.round(throttle), steer: Math.round(steer) };
  const moving = drive.throttle !== 0 || drive.steer !== 0;
  sendDrive();
  if (moving && !driveTimer) driveTimer = setInterval(sendDrive, 1000 / DRIVE_HZ);
  if (!moving) { clearInterval(driveTimer); driveTimer = null; }
}

function stopDriving() {
  setDrive(0, 0);
  moveKnob(0, 0);
}

const pad = $('joystick');
function moveKnob(x, y) {
  $('knob').style.transform = `translate(${x}px, ${y}px)`;
}

pad.addEventListener('pointerdown', (e) => {
  pad.setPointerCapture(e.pointerId);
  pad.dataset.active = '1';
  onPointer(e);
});
pad.addEventListener('pointermove', (e) => { if (pad.dataset.active) onPointer(e); });
for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) {
  pad.addEventListener(ev, () => {
    if (!pad.dataset.active) return;
    delete pad.dataset.active;
    stopDriving();
  });
}

function onPointer(e) {
  const r = pad.getBoundingClientRect();
  const radius = r.width / 2;
  let dx = e.clientX - (r.left + radius);
  let dy = e.clientY - (r.top + radius);
  const max = radius * 0.72;
  const d = Math.hypot(dx, dy);
  if (d > max) { dx *= max / d; dy *= max / d; }
  moveKnob(dx, dy);
  // Small dead zone in the middle; up = forward.
  const dz = (v) => (Math.abs(v) < 0.08 ? 0 : v);
  setDrive(dz(-dy / max) * 100, dz(dx / max) * 100);
}

const keys = new Set();
function keyDrive() {
  const t = (keys.has('ArrowUp') ? 1 : 0) - (keys.has('ArrowDown') ? 1 : 0);
  const s = (keys.has('ArrowRight') ? 1 : 0) - (keys.has('ArrowLeft') ? 1 : 0);
  setDrive(t * 100, s * 60);
  const r = pad.clientWidth / 2 * 0.72;
  moveKnob(s * r, -t * r);
}
window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === ' ') { e.preventDefault(); $('stopBtn').click(); return; }
  if (!e.key.startsWith('Arrow')) return;
  e.preventDefault();
  if (!keys.has(e.key)) { keys.add(e.key); keyDrive(); }
});
window.addEventListener('keyup', (e) => {
  if (keys.delete(e.key)) keyDrive();
});
window.addEventListener('blur', () => { keys.clear(); stopDriving(); });
document.addEventListener('visibilitychange', () => { if (document.hidden) stopDriving(); });

// ---------- events & console ----------
function addEvent(e) {
  const li = document.createElement('li');
  li.className = e.kind;
  const time = document.createElement('time');
  time.textContent = timeOf(e.time);
  const text = document.createElement('span');
  text.textContent = e.text;
  li.append(time, text);
  const list = $('events');
  list.prepend(li);
  while (list.children.length > 100) list.lastChild.remove();
}

function addConsole(line) {
  const el = $('console');
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 4;
  const div = document.createElement('div');
  div.className = line.direction;
  div.textContent = line.text;
  el.append(div);
  while (el.children.length > 300) el.firstChild.remove();
  if (atBottom) el.scrollTop = el.scrollHeight;
}

$('consoleForm').onsubmit = async (e) => {
  e.preventDefault();
  const input = $('consoleInput');
  if ((await emit('raw', input.value)).ok) input.value = '';
};

// ---------- Pi ----------
function renderSystem(s) {
  $('sysTemp').textContent = s.temperature == null ? '–' : `${s.temperature.toFixed(1)} °C`;
  $('piTemp').textContent = s.temperature == null ? '' : `Pi ${s.temperature.toFixed(0)} °C`;
  $('sysLoad').textContent = `${s.load.toFixed(2)} / ${s.cpus}`;
  $('sysMem').textContent = `${Math.round(s.memoryUsed * 100)} %`;
  $('sysAddr').textContent = s.addresses.map((a) => a.address).join(', ') || '–';
}

function renderLogging(l) {
  $('recordBtn').textContent = l.active ? 'Stop recording' : 'Record telemetry';
  $('recordBtn').classList.toggle('on', l.active);
  $('recordStatus').textContent = l.file ? `${l.active ? 'Recording' : 'Saved'}: ${l.file.split('/').pop()} (${l.rows} rows)` : '';
}
$('recordBtn').onclick = () => emit('logging', $('recordBtn').textContent.startsWith('Record'));

async function power(action) {
  if (!confirm(`${action === 'reboot' ? 'Reboot' : 'Shut down'} the Raspberry Pi? The motors will be switched off.`)) return;
  if ((await emit('power', action)).ok) toast(`Pi ${action} in progress`);
}
$('rebootBtn').onclick = () => power('reboot');
$('shutdownBtn').onclick = () => power('shutdown');

// ---------- socket ----------
socket.on('snapshot', (s) => {
  robot = s.robot;
  allowShutdown = s.allowShutdown;
  cameraEnabled = s.camera.enabled;
  $('rebootBtn').disabled = $('shutdownBtn').disabled = !allowShutdown;
  $('videoBtn').disabled = !cameraEnabled;
  $('events').replaceChildren();
  s.events.forEach(addEvent);
  $('console').replaceChildren();
  s.console.forEach(addConsole);
  renderLink();
  renderGains();
  renderAutotune();
  renderCamera(s.camera);
  renderSystem(s.system);
  renderLogging(s.logging);
  if (robot.firmware) renderFirmware();
  if (robot.telemetry) onTelemetry(robot.telemetry);
});
socket.on('telemetry', onTelemetry);
socket.on('gains', (g) => { robot.gains = g.gains; renderGains(); });
socket.on('lqr', (l) => { robot.lqr = l; renderController(); });
socket.on('firmware', (f) => { robot.firmware = f; renderFirmware(); });
socket.on('link', (l) => { robot.link = l; renderLink(); });
socket.on('autotune', (a) => {
  if (a.phase !== robot.autotune.phase) phaseStarted = Date.now();
  robot.autotune = a;
  renderAutotune();
});
socket.on('event', (e) => {
  addEvent(e);
  if (e.kind === 'error' || e.kind === 'autotune' || e.kind === 'test') toast(e.text, 4000);
});
socket.on('console', addConsole);
socket.on('camera', (c) => renderCamera({ ...c, enabled: cameraEnabled }));
socket.on('system', renderSystem);
socket.on('logging', renderLogging);
socket.on('disconnect', () => {
  stopDriving();
  robot.link = { connected: false, error: 'lost connection to the Pi' };
  renderLink();
  $('link').textContent = 'Pi: offline';
});
