// Pi health (temperature, load, memory, addresses) and power control.
import fs from 'node:fs';
import os from 'node:os';
import { execFile } from 'node:child_process';

export function cpuTemperature() {
  try {
    return Number(fs.readFileSync('/sys/class/thermal/thermal_zone0/temp', 'utf8')) / 1000;
  } catch {
    return null;
  }
}

export function addresses() {
  const out = [];
  for (const [name, list] of Object.entries(os.networkInterfaces())) {
    for (const a of list || []) {
      if (a.family === 'IPv4' && !a.internal) out.push({ interface: name, address: a.address });
    }
  }
  return out;
}

export function stats() {
  return {
    temperature: cpuTemperature(),
    load: os.loadavg()[0],
    cpus: os.cpus().length,
    memoryUsed: 1 - os.freemem() / os.totalmem(),
    uptime: os.uptime(),
    hostname: os.hostname(),
    addresses: addresses(),
  };
}

// Needs the sudoers rule installed by install/install.sh (shutdown and reboot
// only, no password). -n: fail instead of prompting.
export function power(action, log = console) {
  const args = action === 'reboot' ? ['-n', '/sbin/shutdown', '-r', 'now'] : ['-n', '/sbin/shutdown', '-h', 'now'];
  return new Promise((resolve, reject) => {
    execFile('sudo', args, (err, stdout, stderr) => {
      if (err) {
        log.error(`power ${action}: ${stderr || err.message}`);
        reject(new Error(stderr.trim() || err.message));
      } else {
        resolve();
      }
    });
  });
}
