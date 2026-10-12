#!/usr/bin/env node
// Bailey server entry point. See README.md for options.
import { loadConfig } from './config.js';
import { createApp } from './app.js';
import { addresses } from './system.js';

const log = {
  debug: process.env.DEBUG ? (m) => console.log(`[debug] ${m}`) : () => {},
  info: (m) => console.log(m),
  warn: (m) => console.warn(`warning: ${m}`),
  error: (m) => console.error(`error: ${m}`),
};

let config;
try {
  config = loadConfig();
} catch (err) {
  console.error(err.message);
  process.exit(2);
}

const bailey = createApp(config, { log });
const port = await bailey.start();
const urls = addresses().map((a) => `http://${a.address}:${port}`);
log.info(`Bailey server listening on ${urls.join(', ') || `port ${port}`}`);
log.info(`robot: ${config.serial.port === 'sim' ? 'simulator' : config.serial.port}, camera: ${config.camera.enabled ? config.camera.command : 'off'}`);

for (const sig of ['SIGINT', 'SIGTERM']) {
  process.on(sig, async () => {
    log.info('stopping');
    await bailey.close();
    process.exit(0);
  });
}
