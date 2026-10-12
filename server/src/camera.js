// Camera: MJPEG from rpicam-vid (Raspberry Pi OS Bookworm; libcamera-vid on
// older releases), served to browsers as multipart/x-mixed-replace. The camera
// only runs while someone is watching. Overlays (pitch, heading, ...) are drawn
// by the browser on top of the video, so the Pi doesn't re-encode frames.
import { spawn } from 'node:child_process';
import { EventEmitter } from 'node:events';

const BOUNDARY = 'baileyframe';
const IDLE_STOP_MS = 5000;
const SOI = Buffer.from([0xff, 0xd8]);
const EOI = Buffer.from([0xff, 0xd9]);

// Splits a byte stream of concatenated JPEGs into frames.
export class JpegSplitter {
  constructor(onFrame, maxBytes = 4 * 1024 * 1024) {
    this.onFrame = onFrame;
    this.maxBytes = maxBytes;
    this.buf = Buffer.alloc(0);
  }

  push(chunk) {
    this.buf = this.buf.length ? Buffer.concat([this.buf, chunk]) : chunk;
    for (;;) {
      const start = this.buf.indexOf(SOI);
      if (start < 0) {
        // Keep a trailing 0xFF: it may be the first half of the next start marker.
        this.buf = this.buf.length && this.buf[this.buf.length - 1] === 0xff ? this.buf.subarray(-1) : Buffer.alloc(0);
        return;
      }
      const end = this.buf.indexOf(EOI, start + 2);
      if (end < 0) {
        if (start > 0) this.buf = this.buf.subarray(start);
        if (this.buf.length > this.maxBytes) this.buf = Buffer.alloc(0);   // garbage, resync
        return;
      }
      this.onFrame(Buffer.from(this.buf.subarray(start, end + 2)));
      this.buf = this.buf.subarray(end + 2);
    }
  }
}

export class Camera extends EventEmitter {
  constructor(options, { log = console } = {}) {
    super();
    this.options = options;
    this.log = log;
    this.clients = new Set();
    this.child = null;
    this.lastFrame = null;
    this.status = options.enabled ? 'off' : 'disabled';
    this.error = null;
    this.stopTimer = null;
    this.frames = 0;
  }

  setStatus(status, error = null) {
    this.status = status;
    this.error = error;
    this.emit('status', { status, error });
  }

  commands() {
    const list = [this.options.command, 'rpicam-vid', 'libcamera-vid'];
    return [...new Set(list.filter(Boolean))];
  }

  args() {
    const o = this.options;
    return ['-t', '0', '-n', '--codec', 'mjpeg', '--width', String(o.width), '--height', String(o.height),
      '--framerate', String(o.fps), '-q', String(o.quality), '--flush', '-o', '-'];
  }

  start(candidates = this.commands()) {
    if (this.child || !this.options.enabled) return;
    const [cmd, ...rest] = candidates;
    if (!cmd) {
      this.setStatus('unavailable', 'no camera program found (install rpicam-apps)');
      return;
    }
    this.setStatus('starting');
    const child = spawn(cmd, this.args(), { stdio: ['ignore', 'pipe', 'pipe'] });
    this.child = child;
    let stderr = '';
    const splitter = new JpegSplitter((frame) => this.broadcast(frame));
    child.stdout.on('data', (chunk) => splitter.push(chunk));
    child.stderr.on('data', (d) => { stderr = (stderr + d).slice(-2000); });
    child.on('error', (err) => {
      this.child = null;
      if (err.code === 'ENOENT') this.start(rest);           // try the next program
      else this.setStatus('unavailable', err.message);
    });
    child.on('exit', (code, signal) => {
      if (this.child !== child) return;
      this.child = null;
      if (signal === 'SIGTERM') {
        this.setStatus('off');
      } else {
        const last = stderr.trim().split('\n').pop() || `exit ${code}`;
        this.log.warn(`camera: ${cmd} stopped: ${last}`);
        this.setStatus('unavailable', last);
      }
    });
  }

  stop() {
    clearTimeout(this.stopTimer);
    if (this.child) this.child.kill('SIGTERM');
  }

  broadcast(frame) {
    this.lastFrame = frame;
    this.frames++;
    if (this.status !== 'running') this.setStatus('running');
    const header = `--${BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: ${frame.length}\r\n\r\n`;
    for (const res of this.clients) {
      // A slow client skips frames instead of buffering them.
      if (res.writableNeedDrain) continue;
      res.write(header);
      res.write(frame);
      res.write('\r\n');
    }
  }

  // Express handler for the MJPEG stream.
  stream(req, res) {
    if (!this.options.enabled || this.status === 'unavailable') {
      res.status(503).json({ status: this.status, error: this.error });
      return;
    }
    res.writeHead(200, {
      'Content-Type': `multipart/x-mixed-replace; boundary=${BOUNDARY}`,
      'Cache-Control': 'no-cache, no-store',
      Connection: 'close',
    });
    this.clients.add(res);
    clearTimeout(this.stopTimer);
    this.start();
    req.on('close', () => {
      this.clients.delete(res);
      if (this.clients.size === 0) this.stopTimer = setTimeout(() => this.stop(), IDLE_STOP_MS);
    });
  }

  snapshot(req, res) {
    if (!this.lastFrame) {
      res.status(503).json({ status: this.status, error: this.error || 'no frame yet: open the video first' });
      return;
    }
    res.type('image/jpeg').send(this.lastFrame);
  }
}
