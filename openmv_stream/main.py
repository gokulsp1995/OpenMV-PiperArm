# OpenMV AE3 — RL Data Streaming — Main Entry Point
# ===================================================
# Streams JPEG video (UDP), sensor data + AprilTags (TCP) over WiFi.
# Designed for RL policy training data capture from a robot arm camera.
#
# Architecture:
#   UDP 8471  →  chunked JPEG frames (broadcast)
#   TCP 8472  →  newline-delimited JSON sensor data + control commands

import csi
import image
import time
import json
import struct
import network
import usocket as socket
from machine import LED

import config
from tof_reader import ToFReader
from imu_reader import IMUReader

# ── LED helpers ───────────────────────────────────────────────────────────────

_led_r = LED("LED_RED")
_led_g = LED("LED_GREEN")
_led_b = LED("LED_BLUE")


def set_led(r, g, b):
    """Set RGB LED state (1 = on, 0 = off)."""
    _led_r.on() if r else _led_r.off()
    _led_g.on() if g else _led_g.off()
    _led_b.on() if b else _led_b.off()


# ── WiFi ──────────────────────────────────────────────────────────────────────

def connect_wifi():
    """Connect to the robot's WiFi network.  Blocks until connected or timeout."""
    set_led(*config.LED_ERROR)  # red while connecting
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    if wlan.isconnected():
        print("[WiFi] Already connected:", wlan.ifconfig())
        set_led(*config.LED_CONNECTED)
        return wlan

    print("[WiFi] Connecting to", config.WIFI_SSID, "...")
    wlan.connect(config.WIFI_SSID, config.WIFI_PASSWORD)

    start = time.ticks_ms()
    while not wlan.isconnected():
        if time.ticks_diff(time.ticks_ms(), start) > config.WIFI_TIMEOUT_MS:
            print("[WiFi] Connection timeout!")
            set_led(*config.LED_ERROR)
            return None
        time.sleep_ms(200)

    print("[WiFi] Connected:", wlan.ifconfig())
    set_led(*config.LED_CONNECTED)
    return wlan


# ── Camera ────────────────────────────────────────────────────────────────────

def init_camera():
    """Initialise the PAG7936 camera sensor at 1280×800 JPEG."""
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.JPEG)
    cam.framesize(csi.WVGA2)  # 1280×800
    cam.quality(config.JPEG_QUALITY)
    # Let auto-exposure settle
    cam.snapshot(time=2000)
    print("[Cam] PAG7936 initialised (1280×800, JPEG, q=%d)" % config.JPEG_QUALITY)
    return cam


# ── UDP Video Sender ──────────────────────────────────────────────────────────

class UDPVideoSender:
    """Sends JPEG frames as chunked UDP datagrams (broadcast)."""

    def __init__(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._frame_id = 0
        print("[UDP] Video sender ready (port %d, chunk %d bytes)"
              % (config.UDP_VIDEO_PORT, config.UDP_CHUNK_SIZE))

    def send_frame(self, jpeg_data, timestamp_ms):
        """Send a JPEG frame as chunked UDP packets.

        Header per chunk (16 bytes):
            magic      (4 bytes) — "OMVS"
            frame_id   (4 bytes) — uint32 LE
            timestamp  (4 bytes) — uint32 LE (ms since boot)
            chunk_idx  (2 bytes) — uint16 LE
            chunk_count(2 bytes) — uint16 LE
        """
        data = bytes(jpeg_data)
        chunk_size = config.UDP_CHUNK_SIZE
        total = len(data)
        chunk_count = (total + chunk_size - 1) // chunk_size
        dest = (config.UDP_BROADCAST_ADDR, config.UDP_VIDEO_PORT)

        for i in range(chunk_count):
            offset = i * chunk_size
            chunk = data[offset:offset + chunk_size]
            header = config.UDP_MAGIC + struct.pack("<IIhh",
                                                     self._frame_id,
                                                     timestamp_ms,
                                                     i,
                                                     chunk_count)
            try:
                self._sock.sendto(header + chunk, dest)
            except Exception as e:
                print("[UDP] Send error:", e)
                break

        self._frame_id += 1


# ── TCP Sensor / Control Server ───────────────────────────────────────────────

class TCPSensorServer:
    """TCP server for pushing sensor data and receiving control commands."""

    def __init__(self):
        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind(("0.0.0.0", config.TCP_SENSOR_PORT))
        self._server.listen(config.TCP_BACKLOG)
        self._server.setblocking(False)
        self._clients = []  # list of connected client sockets
        print("[TCP] Sensor server listening on port", config.TCP_SENSOR_PORT)

    def accept_new(self):
        """Accept any pending new connections (non-blocking)."""
        try:
            client, addr = self._server.accept()
            client.setblocking(False)
            self._clients.append(client)
            print("[TCP] Client connected:", addr)
        except OSError:
            pass  # no pending connection

    def broadcast_json(self, obj):
        """Send a JSON message to all connected clients.  Drop broken ones."""
        data = json.dumps(obj) + "\n"
        encoded = data.encode("utf-8")
        dead = []
        for c in self._clients:
            try:
                c.sendall(encoded)
            except Exception:
                dead.append(c)
        for c in dead:
            self._remove_client(c)

    def poll_commands(self):
        """Check all clients for incoming commands (non-blocking).

        Returns a list of parsed command dicts.
        """
        commands = []
        dead = []
        for c in self._clients:
            try:
                raw = c.recv(config.TCP_RECV_BUF)
                if raw:
                    # May contain multiple newline-delimited messages
                    for line in raw.decode("utf-8").strip().split("\n"):
                        line = line.strip()
                        if line:
                            try:
                                commands.append(json.loads(line))
                            except ValueError:
                                pass
                elif raw == b"":
                    dead.append(c)
            except OSError:
                pass  # no data available (EAGAIN)
        for c in dead:
            self._remove_client(c)
        return commands

    def _remove_client(self, c):
        try:
            c.close()
        except Exception:
            pass
        if c in self._clients:
            self._clients.remove(c)
            print("[TCP] Client disconnected")

    @property
    def client_count(self):
        return len(self._clients)


# ── AprilTag Detection ────────────────────────────────────────────────────────

def detect_apriltags(img):
    """Run AprilTag detection on a frame.

    Takes the full-resolution JPEG image, creates a downscaled greyscale
    copy for detection, and returns a list of tag result dicts.
    """
    tags = img.find_apriltags(
        families=image.TAG36H11,
        fx=config.FOCAL_LENGTH_PX / config.APRILTAG_SCALE,
        fy=config.FOCAL_LENGTH_PX / config.APRILTAG_SCALE,
        cx=config.PRINCIPAL_X / config.APRILTAG_SCALE,
        cy=config.PRINCIPAL_Y / config.APRILTAG_SCALE,
    )

    results = []
    for t in tags[:config.APRILTAG_MAX_TAGS]:
        # Scale corners back to full resolution
        corners = []
        for corner in t.corners():
            corners.append((corner[0] * config.APRILTAG_SCALE,
                            corner[1] * config.APRILTAG_SCALE))
        results.append({
            "id": t.id(),
            "family": t.family(),
            "cx": t.cx() * config.APRILTAG_SCALE,
            "cy": t.cy() * config.APRILTAG_SCALE,
            "rotation": round(t.rotation(), 4),
            "decision_margin": round(t.decision_margin(), 2),
            "hamming": t.hamming(),
            "goodness": round(t.goodness(), 2),
            "corners": corners,
            "x_translation": round(t.x_translation(), 4),
            "y_translation": round(t.y_translation(), 4),
            "z_translation": round(t.z_translation(), 4),
        })
    return results


# ── Command Handler ───────────────────────────────────────────────────────────

def handle_command(cmd, cam, state):
    """Process a control command and return a response dict (or None)."""
    action = cmd.get("cmd", "").upper()

    if action == "APRILTAG_ON":
        state["apriltag_enabled"] = True
        print("[Ctrl] AprilTag detection ON")
        return {"type": "ack", "cmd": "APRILTAG_ON", "status": "ok"}

    elif action == "APRILTAG_OFF":
        state["apriltag_enabled"] = False
        print("[Ctrl] AprilTag detection OFF")
        return {"type": "ack", "cmd": "APRILTAG_OFF", "status": "ok"}

    elif action == "SET_QUALITY":
        q = cmd.get("value", config.JPEG_QUALITY)
        q = max(10, min(95, int(q)))
        cam.quality(q)
        state["jpeg_quality"] = q
        print("[Ctrl] JPEG quality set to", q)
        return {"type": "ack", "cmd": "SET_QUALITY", "value": q}

    elif action == "STATUS":
        return {
            "type": "status",
            "wifi": True,
            "camera": True,
            "tof": state.get("tof_available", False),
            "imu": state.get("imu_available", False),
            "apriltag_enabled": state["apriltag_enabled"],
            "jpeg_quality": state["jpeg_quality"],
            "fps": state.get("fps", 0),
        }

    else:
        return {"type": "error", "message": "unknown command: " + action}


# ── Main Loop ─────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("  OpenMV AE3 — RL Data Streamer")
    print("=" * 50)

    # 1. Connect to WiFi
    wlan = connect_wifi()
    if wlan is None:
        print("[FATAL] Cannot connect to WiFi.  Halting.")
        while True:
            set_led(1, 0, 0)
            time.sleep_ms(500)
            set_led(0, 0, 0)
            time.sleep_ms(500)

    # 2. Initialise peripherals
    cam = init_camera()
    tof = ToFReader()
    imu = IMUReader()

    # 3. Start network servers
    udp_sender = UDPVideoSender()
    tcp_server = TCPSensorServer()

    # 4. Streaming state
    state = {
        "apriltag_enabled": config.APRILTAG_ENABLED,
        "jpeg_quality": config.JPEG_QUALITY,
        "tof_available": tof.available,
        "imu_available": imu.available,
        "fps": 0,
    }

    set_led(*config.LED_STREAMING)
    print("[Stream] Starting main loop (target %d FPS)..." % config.TARGET_FPS)

    frame_id = 0
    fps_counter = 0
    fps_timer = time.ticks_ms()
    frame_interval_ms = 1000 // config.TARGET_FPS

    # 5. Main streaming loop
    while True:
        loop_start = time.ticks_ms()

        # ── Accept new TCP clients ────────────────────────────────────────
        tcp_server.accept_new()

        # ── Process incoming commands ─────────────────────────────────────
        for cmd in tcp_server.poll_commands():
            resp = handle_command(cmd, cam, state)
            if resp:
                tcp_server.broadcast_json(resp)

        # ── Capture frame ─────────────────────────────────────────────────
        img = cam.snapshot()
        if img is None:
            continue

        timestamp_ms = time.ticks_ms()

        # ── AprilTag detection (optional) ─────────────────────────────────
        apriltags = []
        if state["apriltag_enabled"]:
            try:
                # Create a downscaled grayscale copy for detection
                det_img = img.copy(
                    x_scale=1.0 / config.APRILTAG_SCALE,
                    y_scale=1.0 / config.APRILTAG_SCALE,
                ).to_grayscale()
                apriltags = detect_apriltags(det_img)
            except Exception as e:
                print("[AT] Detection error:", e)

        # ── Read sensors ──────────────────────────────────────────────────
        tof.update()
        imu_data = imu.read()
        tof_grid = tof.get_full_grid()

        # ── Send JPEG frame via UDP ───────────────────────────────────────
        udp_sender.send_frame(img, timestamp_ms)

        # ── Send sensor data via TCP ──────────────────────────────────────
        if tcp_server.client_count > 0:
            sensor_msg = {
                "type": "sensor",
                "frame_id": frame_id,
                "timestamp_ms": timestamp_ms,
                "imu": {
                    "ax": round(imu_data[0], 4),
                    "ay": round(imu_data[1], 4),
                    "az": round(imu_data[2], 4),
                    "gx": round(imu_data[3], 2),
                    "gy": round(imu_data[4], 2),
                    "gz": round(imu_data[5], 2),
                },
                "tof": tof_grid if tof_grid else [],
                "apriltags": apriltags,
            }
            tcp_server.broadcast_json(sensor_msg)

        # ── FPS tracking ──────────────────────────────────────────────────
        frame_id += 1
        fps_counter += 1
        elapsed = time.ticks_diff(time.ticks_ms(), fps_timer)
        if elapsed >= 2000:
            state["fps"] = round(fps_counter * 1000 / elapsed, 1)
            print("[Stream] %s FPS | clients=%d | tags=%d"
                  % (state["fps"], tcp_server.client_count, len(apriltags)))
            fps_counter = 0
            fps_timer = time.ticks_ms()

        # ── Frame rate throttle ───────────────────────────────────────────
        loop_elapsed = time.ticks_diff(time.ticks_ms(), loop_start)
        if loop_elapsed < frame_interval_ms:
            time.sleep_ms(frame_interval_ms - loop_elapsed)


# ── Boot ──────────────────────────────────────────────────────────────────────
main()
