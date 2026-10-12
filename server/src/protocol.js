// The serial protocol of the Bailey firmware (see bailey_FW/comms.ino).
//
// Robot -> Pi, one line each, tagged by the first comma-separated field:
//   TH/T      telemetry header / values     PIDH/PID  gains header / values
//   SYSH/SYS  system header / values        EV        event (state change, ...)
//   AT        autotune phase / result       TEST      motor test result
//   SCMD ...  command echo                  ***       configuration dump
//   E, ...    error
// Pi -> robot: "SCMD <name> <value>" and "READ <what>".
//
// This module has no I/O, so it can be unit tested.

export const ROBOT_STATES = ['OFF', 'WAIT', 'BAL', 'FALL'];

// PID values are sent as integers; divide by these to get the gains.
export const PID_SCALE = {
  speedPIDKp: 10000,
  speedPIDKi: 10000,
  speedPIDKd: 10000,
  anglePIDConKp: 100,
  anglePIDConKi: 100,
  anglePIDConKd: 100,
  anglePIDAggKp: 100,
  anglePIDAggKi: 100,
  anglePIDAggKd: 100,
  TriggerAngleAggressive: 100,
  calibratedZeroAngle: 100,
};

const MAX_LINE = 63;   // firmware line buffer is 64 bytes

export function tagOf(line) {
  return line.split(',')[0].split(' ')[0];
}

function toNumber(s) {
  const n = Number(s);
  return Number.isFinite(n) ? n : s;
}

// Keeps the headers announced by the robot and turns lines into events.
export class RobotProtocol {
  constructor() {
    this.headers = { T: null, PID: null, SYS: null };
  }

  // Returns { type, ... } describing the line.
  parse(rawLine) {
    const line = rawLine.replace(/[\r\n]+$/, '').trim();
    if (!line) return null;
    const tag = tagOf(line);
    const fields = line.split(',');

    switch (tag) {
      case 'TH':
      case 'PIDH':
      case 'SYSH':
        this.headers[tag.slice(0, -1)] = fields.slice(1).map((s) => s.trim());
        return { type: 'header', kind: tag.slice(0, -1), names: this.headers[tag.slice(0, -1)] };

      case 'T':
      case 'PID':
      case 'SYS': {
        const names = this.headers[tag];
        if (!names) return { type: 'unknown', line, reason: `${tag} before ${tag}H` };
        const values = {};
        names.forEach((name, i) => {
          if (i + 1 >= fields.length) return;
          const text = fields[i + 1].trim();
          values[name] = tag === 'SYS' ? text : toNumber(text);   // SYS: versions etc. stay text
        });
        if (tag === 'T') {
          values.state = ROBOT_STATES[values.bal] ?? String(values.bal);
          return { type: 'telemetry', values };
        }
        if (tag === 'PID') {
          const gains = {};
          for (const [name, value] of Object.entries(values)) {
            gains[name] = PID_SCALE[name] ? value / PID_SCALE[name] : value;
          }
          return { type: 'pid', raw: values, gains };
        }
        return { type: 'system', values };
      }

      case 'EV':
        return { type: 'event', text: fields.slice(1).join(',') };

      case 'AT':
        if (fields[1] === 'RESULT') {
          const [accepted, L, L1, L2, zero, r2, before, after] = fields.slice(2).map(Number);
          return {
            type: 'autotune',
            phase: 'RESULT',
            result: { accepted: accepted === 1, length: L, lengthFromGravity: L1, lengthFromAccel: L2, zero, r2, before, after },
          };
        }
        return { type: 'autotune', phase: fields[1] };

      case 'TEST':
        return { type: 'motorTest', left: Number(fields[1]), right: Number(fields[2]) };

      case 'SCMD':
        return { type: 'echo', text: line };

      case 'E':
        return { type: 'error', text: line.replace(/^E,\s*/, '') };

      default:
        if (line.startsWith('***')) return { type: 'config', text: line.replace(/^\*+\s*/, '') };
        return { type: 'unknown', line };
    }
  }
}

// --- Commands to the robot ---

export function scmd(name, value) {
  if (!/^[A-Za-z_]{1,32}$/.test(String(name))) throw new Error(`bad command name: ${name}`);
  const v = Number(value);
  if (!Number.isFinite(v)) throw new Error(`bad value for ${name}: ${value}`);
  // The firmware parses with atof/atol: plain decimal, no exponent.
  const text = Number.isInteger(v) ? String(v) : v.toFixed(4).replace(/0+$/, '').replace(/\.$/, '');
  return `SCMD ${name} ${text}`;
}

export function read(what) {
  if (!/^[A-Za-z]{1,32}$/.test(String(what))) throw new Error(`bad READ target: ${what}`);
  return `READ ${what}`;
}

// A gain in real units -> the integer SCMD the firmware expects.
export function setGain(name, gain) {
  const scale = PID_SCALE[name] ?? 1;
  return scmd(name, Math.round(Number(gain) * scale));
}

// Free text typed in the web console: one printable line that fits the
// firmware's buffer, never a way to inject extra lines.
export function sanitizeRaw(text) {
  const line = String(text).replace(/[^\x20-\x7e]/g, '').trim();
  if (!line) throw new Error('empty command');
  if (line.length > MAX_LINE) throw new Error(`command longer than ${MAX_LINE} characters`);
  if (!/^(SCMD|READ) /.test(line)) throw new Error('commands start with SCMD or READ');
  return line;
}

export function clampDrive(v) {
  const n = Math.round(Number(v));
  if (!Number.isFinite(n)) return 0;
  return Math.max(-100, Math.min(100, n));
}
