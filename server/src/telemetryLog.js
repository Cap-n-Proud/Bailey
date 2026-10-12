// Telemetry to CSV: one file per recording, one row per telemetry line, with
// the current gains appended so a recording shows what it was tuned with.
import fs from 'node:fs';
import path from 'node:path';

function stamp(d = new Date()) {
  const p = (n) => String(n).padStart(2, '0');
  return `${d.getFullYear()}-${p(d.getMonth() + 1)}-${p(d.getDate())}_${p(d.getHours())}-${p(d.getMinutes())}-${p(d.getSeconds())}`;
}

export class TelemetryLog {
  constructor(dir) {
    this.dir = dir;
    this.stream = null;
    this.file = null;
    this.columns = null;
    this.rows = 0;
  }

  get active() {
    return this.stream !== null;
  }

  start(telemetryNames, gainNames) {
    if (this.stream) return this.file;
    fs.mkdirSync(this.dir, { recursive: true });
    this.file = path.join(this.dir, `telemetry_${stamp()}.csv`);
    this.columns = { telemetry: telemetryNames, gains: gainNames };
    this.stream = fs.createWriteStream(this.file);
    this.stream.write(['time_ms', ...telemetryNames, ...gainNames].join(',') + '\n');
    this.rows = 0;
    return this.file;
  }

  add(telemetry, gains) {
    if (!this.stream) return;
    const row = [Date.now()];
    for (const name of this.columns.telemetry) row.push(telemetry[name] ?? '');
    for (const name of this.columns.gains) row.push(gains?.[name] ?? '');
    this.stream.write(row.join(',') + '\n');
    this.rows++;
  }

  stop() {
    if (!this.stream) return null;
    const file = this.file;
    this.stream.end();
    this.stream = null;
    return file;
  }
}
