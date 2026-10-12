// Configuration: config.json, overridden by config.local.json (not in git, for
// per-robot settings), then by command-line flags.
//
//   --sim            use the simulated robot instead of the serial port
//   --port <n>       HTTP port
//   --serial <dev>   serial device, e.g. /dev/ttyUSB0
//   --no-camera      don't start the camera
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

export const SERVER_DIR = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');

function readJson(file) {
  try {
    return JSON.parse(fs.readFileSync(file, 'utf8'));
  } catch (err) {
    if (err.code === 'ENOENT') return {};
    throw new Error(`${file}: ${err.message}`);
  }
}

function merge(base, extra) {
  const out = { ...base };
  for (const [key, value] of Object.entries(extra)) {
    out[key] = value && typeof value === 'object' && !Array.isArray(value) ? merge(base[key] || {}, value) : value;
  }
  return out;
}

export function expandHome(p) {
  return p.startsWith('~') ? path.join(os.homedir(), p.slice(1)) : p;
}

export function loadConfig(argv = process.argv.slice(2), dir = SERVER_DIR) {
  let config = merge(readJson(path.join(dir, 'config.json')), readJson(path.join(dir, 'config.local.json')));
  const flags = {};
  for (let i = 0; i < argv.length; i++) {
    const a = argv[i];
    if (a === '--sim') flags.serial = { port: 'sim' };
    else if (a === '--no-camera') flags.camera = { enabled: false };
    else if (a === '--port') flags.http = { port: Number(argv[++i]) };
    else if (a === '--serial') flags.serial = { port: argv[++i] };
    else throw new Error(`unknown option ${a}`);
  }
  config = merge(config, flags);
  config.telemetry.logDir = expandHome(config.telemetry.logDir);
  return config;
}
