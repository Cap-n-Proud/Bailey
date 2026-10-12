// Joystick -> Throttle/Steer commands.
//
// The firmware stops driving unless a non-zero Throttle/Steer is repeated at
// least every second, so a dead Pi, network or browser can't leave the robot
// driving. This side repeats the current command every REPEAT_MS while it is
// non-zero, and drops it to zero if the client that set it stops updating
// (CLIENT_TIMEOUT_MS) or disconnects. The last client to move the joystick
// is in control.
import { scmd, clampDrive } from './protocol.js';

const REPEAT_MS = 100;
const CLIENT_TIMEOUT_MS = 600;

export class DriveControl {
  constructor(send, { now = Date.now } = {}) {
    this.send = send;
    this.now = now;
    this.throttle = 0;
    this.steer = 0;
    this.owner = null;
    this.lastUpdate = 0;
    this.timer = null;
  }

  set(clientId, throttle, steer) {
    const t = clampDrive(throttle);
    const s = clampDrive(steer);
    const changed = t !== this.throttle || s !== this.steer;
    this.owner = clientId;
    this.lastUpdate = this.now();
    this.throttle = t;
    this.steer = s;
    if (changed) this.transmit();
    if (t !== 0 || s !== 0) this.startRepeating();
    else this.stopRepeating();
  }

  // Client went away: stop if it was the one driving.
  release(clientId) {
    if (this.owner === clientId) this.stop();
  }

  stop() {
    const wasMoving = this.throttle !== 0 || this.steer !== 0;
    this.throttle = 0;
    this.steer = 0;
    this.owner = null;
    this.stopRepeating();
    if (wasMoving) this.transmit();
  }

  transmit() {
    this.send(scmd('Throttle', this.throttle));
    this.send(scmd('Steer', this.steer));
  }

  tick() {
    if (this.now() - this.lastUpdate > CLIENT_TIMEOUT_MS) this.stop();
    else this.transmit();
  }

  startRepeating() {
    if (!this.timer) this.timer = setInterval(() => this.tick(), REPEAT_MS);
  }

  stopRepeating() {
    clearInterval(this.timer);
    this.timer = null;
  }
}
