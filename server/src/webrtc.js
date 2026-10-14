// WebRTC video (camera.mode "webrtc"): MediaMTX owns the Pi camera
// (install/mediamtx.yml) and serves it with WHEP, the WebRTC playback
// handshake. Browsers do that handshake through this server, on its own port,
// and the video itself then flows straight from MediaMTX to the browser.
//
//   POST   /mediamtx/cam/whep          SDP offer  -> 201, SDP answer, Location
//   PATCH  /mediamtx/cam/whep/<id>     ICE candidates (trickle)
//   DELETE /mediamtx/cam/whep/<id>     end the session
//
// Only these requests are forwarded, and only to MediaMTX on this machine.
import express from 'express';

const PATH = /^cam\/whep(\/[A-Za-z0-9_-]{1,64})?$/;
const TIMEOUT_MS = 8000;

export function whepProxy(base, { log = console, fetchImpl = fetch } = {}) {
  const router = express.Router();
  router.use(express.text({ type: ['application/sdp', 'application/trickle-ice-sdpfrag'], limit: '64kb' }));

  router.all('/*path', async (req, res) => {
    const path = [].concat(req.params.path).join('/');
    if (!PATH.test(path) || !['POST', 'PATCH', 'DELETE'].includes(req.method)) {
      res.status(404).end();
      return;
    }
    const headers = {};
    if (req.get('content-type')) headers['content-type'] = req.get('content-type');
    if (req.get('if-match')) headers['if-match'] = req.get('if-match');
    let upstream;
    try {
      upstream = await fetchImpl(`${base}/${path}`, {
        method: req.method,
        headers,
        body: req.method === 'DELETE' ? undefined : (typeof req.body === 'string' ? req.body : ''),
        signal: AbortSignal.timeout(TIMEOUT_MS),
      });
    } catch (err) {
      log.warn(`video: MediaMTX not reachable at ${base}: ${err.message}`);
      res.status(502).type('text/plain').send('video server (MediaMTX) not running');
      return;
    }
    res.status(upstream.status);
    for (const name of ['content-type', 'etag', 'link', 'accept-patch']) {
      const v = upstream.headers.get(name);
      if (v) res.set(name, v);
    }
    // MediaMTX answers with its own path ("/cam/whep/<id>"): point it back here.
    const location = upstream.headers.get('location');
    if (location) res.set('location', location.startsWith('/') ? `/mediamtx${location}` : location);
    res.send(Buffer.from(await upstream.arrayBuffer()));
  });
  return router;
}
