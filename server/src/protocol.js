// The serial protocol of the Bailey firmware (see bailey_FW/comms.ino).
//
// Robot -> Pi, one line each, tagged by the first comma-separated field:
//   TH/T      telemetry header / values     PIDH/PID  gains header / values
//   SYSH/SYS  system header / values        EV        event (state change, ...)
//   AT        autotune phase / result       TEST      motor test result
//   LQR       LQR gain K (4) and the pendulum length it is for
//   SCMD ...  command echo                  ***       configuration dump
//   E, ...    error
// Pi -> robot: "SCMD <name> <value>" and "READ <what>".
//
// Every line, both ways, ends in "*HH": a CRC-8 (polynomial 0x07) of the text
// before the '*', in hex. It catches lines cut short or run together. Lines
// without one are accepted (older firmware, typing by hand).
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

const MAX_LINE = 72;   // firmware line buffer is 80 bytes, including the checksum

export function crc8(text) {
  let crc = 0;
  for (let i = 0; i < text.length; i++) {
    crc ^= text.charCodeAt(i) & 0xff;
    for (let b = 0; b < 8; b++) crc = crc & 0x80 ? ((crc << 1) ^ 0x07) & 0xff : (crc << 1) & 0xff;
  }
  return crc;
}

export function withChecksum(line) {
  return `${line}*${crc8(line).toString(16).toUpperCase().padStart(2, '0')}`;
}

// { line, ok }: the line without its checksum, and whether it matched
// (true when there was none).
export function checkLine(raw) {
  const m = /^(.*)\*([0-9A-Fa-f]{2})$/.exec(raw);
  if (!m) return { line: raw, ok: true };
  return { line: m[1], ok: crc8(m[1]) === parseInt(m[2], 16) };
}

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
    const checked = checkLine(rawLine.replace(/[\r\n]+$/, '').trim());
    const line = checked.line.trim();
    if (!checked.ok) return { type: 'corrupt', line: rawLine };
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
          // AT,RESULT,<accepted>,<L>,<kappa>,<zero>,<R²>,<wobble before>,<after>
          const [accepted, L, coupling, zero, r2, before, after] = fields.slice(2).map(Number);
          return {
            type: 'autotune',
            phase: 'RESULT',
            result: { accepted: accepted === 1, length: L, coupling, zero, r2, before, after },
          };
        }
        return { type: 'autotune', phase: fields[1] };

      case 'LQR': {
        const [k0, k1, k2, k3, length] = fields.slice(1).map(Number);
        return { type: 'lqr', gain: [k0, k1, k2, k3], length };
      }

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
