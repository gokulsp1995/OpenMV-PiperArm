#!/usr/bin/env python3
"""Live annotated camera view, served over HTTP.

Polls the camera's FRAME command over serial and serves the annotated
frames as MJPEG. Open it in a browser on your laptop:

    python3 viewer.py --port /dev/ttyACM0
    # then browse to http://<robot-pc-ip>:8000/

Endpoints:
    /          simple page with the live stream and the latest detections
    /stream    raw MJPEG (multipart)
    /capture   single JPEG
    /json      latest detection data as JSON

IMPORTANT: only one process can hold the serial port. Run this OR
run_controller.py, not both. Stop the viewer before running a press.
"""

import argparse
import base64
import glob
import http.server
import json
import threading
import time

import serial

# Base64 JPEG lines are far longer than the JSON control messages, so the
# reader below uses a much larger cap than camera_client.py's 8192.
MAX_LINE = 2_000_000


def find_port(hint="OpenMV"):
    if hint.startswith("/dev/"):
        return hint
    for path in glob.glob("/dev/serial/by-id/*"):
        if hint.lower() in path.lower():
            return path
    raise RuntimeError("No by-id entry matched %r" % hint)


class CameraFeed:
    """Owns the serial port, polls FRAME, keeps the latest result."""

    def __init__(self, port, timeout=8.0, quality=50, interval=0.1,
                 verbose=False):
        self.ser = serial.Serial(port, 115200, timeout=timeout, dsrdtr=False)
        self.ser.reset_input_buffer()
        self.timeout = timeout
        self.quality = quality
        self.interval = interval
        self.verbose = verbose

        self.frame = None        # latest JPEG bytes
        self.buttons = []        # latest detections
        self.tof_grid = None     # latest raw 8x8 ToF grid
        self.error = None
        self.lock = threading.Lock()
        self._stop = False
        self.n_ok = 0
        self.n_fail = 0

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def stop(self):
        self._stop = True

    def _log(self, msg):
        if self.verbose:
            print("    %s" % msg, flush=True)

    def _read_json_line(self):
        """Read one line and parse it, skipping non-JSON noise.

        The camera's modules still print() at init, and those land on the
        same stream -- skipping anything not starting with '{' keeps that
        from being mistaken for a reply.

        Reads in chunks rather than byte-at-a-time: a base64 JPEG is tens
        of thousands of bytes, and one read() syscall per byte is slow
        enough to be a plausible cause of timeouts on its own.
        """
        start = time.time()
        deadline = start + self.timeout
        buf = bytearray()
        total = 0

        while time.time() < deadline:
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if not chunk:
                continue
            total += len(chunk)
            buf += chunk

            while b"\n" in buf:
                raw, _, rest = bytes(buf).partition(b"\n")
                buf = bytearray(rest)
                line = raw.decode("utf-8", "ignore").strip()

                if not line:
                    continue
                if not line.startswith("{"):
                    self._log("skipped non-JSON (%d B): %r"
                              % (len(line), line[:80]))
                    continue

                elapsed = time.time() - start
                try:
                    msg = json.loads(line)
                except ValueError as e:
                    self._log("JSON parse failed after %.2fs, %d B line: %s"
                              % (elapsed, len(line), e))
                    self._log("  head: %r" % line[:120])
                    self._log("  tail: %r" % line[-120:])
                    continue

                self._log("got type=%s in %.2fs (%d B line, %d B read)"
                          % (msg.get("type"), elapsed, len(line), total))
                return msg

            if len(buf) > MAX_LINE:
                self._log("line exceeded %d B -- discarding" % MAX_LINE)
                buf = bytearray()

        self._log("TIMEOUT after %.1fs -- %d bytes read, %d buffered "
                  "with no newline" % (self.timeout, total, len(buf)))
        if buf:
            self._log("  partial head: %r" % bytes(buf)[:120].decode(
                "utf-8", "ignore"))
        return None

    def _loop(self):
        req = 0
        while not self._stop:
            req += 1
            try:
                self._log("[req %d] -> FRAME (quality=%d)"
                          % (req, self.quality))
                self.ser.reset_input_buffer()   # drop anything stale first
                cmd = json.dumps({"cmd": "FRAME",
                                   "quality": self.quality}) + "\n"
                self.ser.write(cmd.encode("utf-8"))

                msg = self._read_json_line()

                if msg is None:
                    self.n_fail += 1
                    with self.lock:
                        self.error = "no response (timeout)"

                elif msg.get("type") == "frame":
                    b64 = msg.get("jpeg_b64", "")
                    try:
                        jpeg = base64.b64decode(b64)
                    except Exception as e:
                        self.n_fail += 1
                        self._log("[req %d] base64 decode failed: %s"
                                  % (req, e))
                        with self.lock:
                            self.error = "bad base64"
                        time.sleep(self.interval)
                        continue

                    self.n_ok += 1
                    self._log("[req %d] frame ok: %d B jpeg, %d button(s)  "
                              "[ok=%d fail=%d]"
                              % (req, len(jpeg), len(msg.get("buttons", [])),
                                 self.n_ok, self.n_fail))
                    with self.lock:
                        self.frame = jpeg
                        self.buttons = msg.get("buttons", [])
                        self.tof_grid = msg.get("tof_grid")
                        self.error = None

                elif msg.get("type") == "error":
                    self.n_fail += 1
                    # This is the camera telling us what went wrong on its
                    # side -- e.g. "compress failed: memory allocation
                    # failed" would point at JPEG encoding, not transport.
                    self._log("[req %d] camera error: %s"
                              % (req, msg.get("message")))
                    with self.lock:
                        self.error = "camera: %s" % msg.get("message")

                else:
                    self.n_fail += 1
                    self._log("[req %d] unexpected type=%s"
                              % (req, msg.get("type")))
                    with self.lock:
                        self.error = "unexpected reply type=%s" % msg.get("type")

            except Exception as e:
                self.n_fail += 1
                self._log("[req %d] host-side exception: %s" % (req, e))
                with self.lock:
                    self.error = str(e)
                time.sleep(0.5)

            time.sleep(self.interval)

    def snapshot(self):
        with self.lock:
            return (self.frame, list(self.buttons), self.error,
                    self.tof_grid)


PAGE = b"""<!doctype html>
<html><head><title>OpenMV view</title>
<style>
 body{background:#111;color:#ddd;font-family:ui-monospace,monospace;
      margin:0;padding:16px;display:flex;gap:16px;align-items:flex-start;
      height:100vh;box-sizing:border-box;overflow:hidden}
 #left{flex:0 0 auto}
 img{border:1px solid #444;image-rendering:pixelated;width:480px;display:block}
 #right{flex:1 1 auto;display:flex;flex-direction:column;
        height:100%;min-width:0}
 #err{color:#e66;min-height:18px;font-size:13px;margin-bottom:6px}
 #bar{display:flex;gap:14px;align-items:center;margin-bottom:8px;
      font-size:12px;color:#888}
 button{background:#222;color:#bbb;border:1px solid #444;
        padding:3px 10px;font-family:inherit;font-size:12px;cursor:pointer}
 button:hover{background:#2c2c2c}
 #wrap{flex:1 1 auto;overflow-y:auto;border:1px solid #333}
 table{border-collapse:collapse;width:100%;font-size:12px}
 thead th{position:sticky;top:0;background:#1a1a1a;color:#888;
          font-weight:normal;text-align:left;padding:6px 8px;
          border-bottom:1px solid #333;white-space:nowrap}
 td{padding:3px 8px;border-bottom:1px solid #222;white-space:nowrap}
 tbody tr:first-child td{background:#1c2419}
 .n{color:#666}
 .d{color:#7c7}
 #tof{margin-bottom:10px}
 #tof h4{margin:0 0 4px;font-weight:normal;color:#888;font-size:12px}
 #tofgrid{border-collapse:collapse;font-size:11px;width:auto}
 #tofgrid td{padding:2px 5px;border:1px solid #2a2a2a;text-align:right;
             min-width:30px;color:#999}
 #tofgrid td.near{color:#8e8;font-weight:bold}
 #tofgrid td.bad{color:#555}
</style></head>
<body>
<div id="left"><img src="/stream"></div>
<div id="right">
  <div id="err"></div>
  <div id="bar">
    <span id="count">0 readings</span>
    <button onclick="seen.length=0;render()">clear</button>
    <label><input type="checkbox" id="pause"> pause</label>
  </div>
  <div id="tof">
    <h4>ToF grid (mm) &mdash; nearest valid highlighted</h4>
    <table id="tofgrid"><tbody id="tofrows"></tbody></table>
  </div>
  <div id="wrap">
    <table>
      <thead><tr>
        <th>#</th><th>id</th><th>centre px</th><th>bbox</th>
        <th>ToF</th><th>cam x</th><th>cam y</th><th>cam z</th>
      </tr></thead>
      <tbody id="rows"></tbody>
    </table>
  </div>
</div>
<script>
const seen = [];

function render() {
  // Newest first, so the live reading is always at the top and the list
  // never needs scrolling to follow.
  document.getElementById('rows').innerHTML = seen.map((b, i) => {
    const n = seen.length - i;
    const bb = b.bbox ? b.bbox.join(',') : '-';
    return `<tr>
      <td class="n">${n}</td>
      <td>${b.id}</td>
      <td>${b.pixel_x}, ${b.pixel_y}</td>
      <td>${bb}</td>
      <td class="d">${b.distance_mm} mm</td>
      <td>${b.cam_x}</td>
      <td>${b.cam_y}</td>
      <td>${b.cam_z}</td>
    </tr>`;
  }).join('');
  document.getElementById('count').textContent = seen.length + ' readings';
}

function key(b) {
  return [b.id, b.pixel_x, b.pixel_y, b.distance_mm].join('|');
}

// Raw 8x8 ToF grid, row-major straight from the sensor -- NO flip or scale
// applied, so this shows the sensor's own layout. Compare against where the
// button is in the image to confirm TOF_FLIP_X / TOF_FLIP_Y are right.
function renderTof(grid) {
  const el = document.getElementById('tofrows');
  if (!grid || !grid.length) { el.innerHTML = ''; return; }

  // Find the nearest valid reading, to highlight it
  let best = Infinity;
  grid.forEach(v => { if (v >= 50 && v <= 2000 && v < best) best = v; });

  let html = '';
  for (let r = 0; r < 8; r++) {
    html += '<tr>';
    for (let c = 0; c < 8; c++) {
      const v = grid[r * 8 + c];
      const valid = v >= 50 && v <= 2000;
      const cls = !valid ? 'bad' : (v === best ? 'near' : '');
      html += `<td class="${cls}">${Math.round(v)}</td>`;
    }
    html += '</tr>';
  }
  el.innerHTML = html;
}

setInterval(async () => {
  if (document.getElementById('pause').checked) return;
  try {
    const j = await (await fetch('/json')).json();
    document.getElementById('err').textContent =
      j.error ? ('ERROR: ' + j.error) : '';

    renderTof(j.tof_grid);

    let added = false;
    j.buttons.forEach(b => {
      if (b.distance_mm <= 0) return;              // ToF rejected
      if (seen.length && key(seen[0]) === key(b)) return;   // unchanged
      seen.unshift(b);                              // newest at index 0
      added = true;
    });
    if (seen.length > 500) seen.length = 500;
    if (added) render();
  } catch (e) { }
}, 300);
</script>
</body></html>"""


def make_handler(feed):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)

            elif self.path == "/stream":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                while True:
                    jpeg, _, _, _ = feed.snapshot()
                    if jpeg:
                        try:
                            self.wfile.write(
                                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                                + jpeg + b"\r\n")
                        except Exception:
                            break
                    time.sleep(0.1)

            elif self.path == "/capture":
                jpeg, _, _, _ = feed.snapshot()
                if jpeg:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.end_headers()
                    self.wfile.write(jpeg)
                else:
                    self.send_response(503)
                    self.end_headers()

            elif self.path == "/json":
                _, buttons, error, grid = feed.snapshot()
                body = json.dumps({"buttons": buttons,
                                    "error": error,
                                    "tof_grid": grid}).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            else:
                self.send_response(404)
                self.end_headers()

        def log_message(self, *a):
            pass

    return Handler


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="OpenMV")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--quality", type=int, default=50)
    ap.add_argument("--interval", type=float, default=0.1,
                     help="seconds between FRAME requests")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("-v", "--verbose", action="store_true",
                     help="log every request: timing, sizes, failures")
    args = ap.parse_args()

    port = find_port(args.port)
    print("Camera: %s" % port)

    feed = CameraFeed(port, quality=args.quality, interval=args.interval,
                       timeout=args.timeout, verbose=args.verbose)
    feed.start()

    print("Open http://<this-machine>:%d/ in a browser" % args.http_port)
    server = http.server.ThreadingHTTPServer(
        ("0.0.0.0", args.http_port), make_handler(feed))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        feed.stop()


if __name__ == "__main__":
    main()