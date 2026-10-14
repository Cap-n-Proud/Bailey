// Line-based link to the robot: the Arduino on a serial port, or the firmware
// simulator (bailey_FW/test, "live" mode) as a child process.
//
// Events: 'line' (string), 'status' ({ connected, port, error }).
// The serial port is reopened automatically if it disappears (USB unplugged,
// Arduino reset) or can't be opened yet.
import { EventEmitter } from 'node:events';
import { spawn } from 'node:child_process';
import readline from 'node:readline';
import path from 'node:path';
import { SerialPort, ReadlineParser } from 'serialport';

const RETRY_MS = 2000;
// Opening the port resets the Arduino; it then calibrates the gyro before it
// answers, so ask for the headers again after a while.
const INIT_DELAY_MS = 2500;

export class RobotLink extends EventEmitter {
  constructor({ port, baudRate, simCommand }, { serverDir, log = console } = {}) {
    super();
    this.portPath = port;
    this.baudRate = baudRate;
    this.simCommand = simCommand;
    this.serverDir = serverDir;
    this.log = log;
    this.connected = false;
    this.closed = false;
    this.port = null;
    this.child = null;
    this.retryTimer = null;
    this.initTimer = null;
  }

  get isSim() {
    return this.portPath === 'sim';
  }

  start() {
    if (this.isSim) this.startSim();
    else this.openSerial();
  }

  setStatus(connected, error) {
    this.connected = connected;
    this.emit('status', { connected, port: this.portPath, error: error ? String(error) : null });
  }

  scheduleInit() {
    clearTimeout(this.initTimer);
    this.initTimer = setTimeout(() => this.write('READ RemoteInit'), INIT_DELAY_MS);
  }

  openSerial() {
    if (this.closed) return;
    const port = new SerialPort({ path: this.portPath, baudRate: this.baudRate, autoOpen: false });
    const parser = port.pipe(new ReadlineParser({ delimiter: '\n' }));
    parser.on('data', (line) => this.emit('line', line.replace(/\r$/, '')));
    port.on('close', () => {
      this.port = null;
      this.setStatus(false, 'port closed');
      this.retry();
    });
    port.on('error', (err) => this.log.warn(`serial: ${err.message}`));
    port.open((err) => {
      if (err) {
        // Log once per distinct error, not every retry.
        if (this.lastError !== err.message) {
          this.log.warn(`serial: can't open ${this.portPath}: ${err.message} (retrying)`);
        }
        this.lastError = err.message;
        this.setStatus(false, err.message);
        this.retry();
        return;
      }
      this.lastError = null;
      this.port = port;
      this.log.info(`serial: connected to ${this.portPath} at ${this.baudRate}`);
      this.setStatus(true);
      this.scheduleInit();
    });
  }

  retry() {
    if (this.closed) return;
    clearTimeout(this.retryTimer);
    this.retryTimer = setTimeout(() => this.openSerial(), RETRY_MS);
  }

  startSim() {
    const [cmd, ...args] = this.simCommand.split(/\s+/);
    const exe = path.resolve(this.serverDir, cmd);
    this.log.info(`sim: starting ${exe} ${args.join(' ')}`);
    const child = spawn(exe, args, { stdio: ['pipe', 'pipe', 'inherit'] });
    this.child = child;
    readline.createInterface({ input: child.stdout }).on('line', (line) => this.emit('line', line));
    child.on('error', (err) => {
      this.log.error(`sim: ${err.message}. Build it with bailey_FW/test/build.sh`);
      this.setStatus(false, err.message);
    });
    child.on('exit', (code) => {
      this.child = null;
      this.setStatus(false, `simulator exited (${code})`);
      if (!this.closed) setTimeout(() => this.startSim(), RETRY_MS);
    });
    this.setStatus(true);
    this.scheduleInit();
  }

  write(line) {
    if (/[\r\n]/.test(line)) throw new Error('one line at a time');
    if (this.port) this.port.write(line + '\n');
    else if (this.child) this.child.stdin.write(line + '\n');
    else return false;
    return true;
  }

  close() {
    this.closed = true;
    clearTimeout(this.retryTimer);
    clearTimeout(this.initTimer);
    if (this.port) this.port.close();
    if (this.child) this.child.kill();
  }
}
