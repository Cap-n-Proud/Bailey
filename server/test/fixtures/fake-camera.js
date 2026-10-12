#!/usr/bin/env node
// Stands in for rpicam-vid in tests: writes small fake JPEG frames to stdout
// at ~20 fps, ignoring the camera arguments.
let n = 0;
setInterval(() => {
  n = (n + 1) % 250;
  process.stdout.write(Buffer.from([0xff, 0xd8, 0xff, 0xe0, n, 1, 2, 3, 0xff, 0xd9]));
}, 50);
