# OpenMV AE3 — Lift Button Detector — Main Entry Point
# ====================================================
# Auto-runs on boot.  Connects to WiFi, starts a TCP server, and waits for
# commands from the robot computer (192.168.123.100).
#
# Protocol (JSON, newline-delimited over TCP):
#   Robot → Camera:  {"cmd": "SCAN"}      — capture + detect + reply
#                    {"cmd": "STATUS"}    — report sensor health
#   Camera → Robot:  {"type": "detections", "buttons": [...], "timestamp": N}
#                    {"type": "status", "wifi": bool, "camera": bool, "tof": bool}
#                    {"type": "error", "message": "..."}

import csi
import time
import json
import network
import usocket as socket
from machine import LED

import config
from button_detector import ButtonDetector
from tof_reader import ToFReader

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
    """Initialise the PAG7936 camera sensor."""
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.RGB565)
    cam.framesize(csi.VGA)  # 640×480
    # Let auto-exposure settle
    cam.snapshot(time=2000)
    print("[Cam] PAG7936 initialised (VGA, RGB565)")
    return cam


# ── TCP Server ────────────────────────────────────────────────────────────────

def create_server():
    """Create and bind a TCP server socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", config.TCP_PORT))
    s.listen(config.TCP_BACKLOG)
    print("[TCP] Listening on port", config.TCP_PORT)
    return s


def send_json(client, obj):
    """Send a JSON object as a newline-terminated UTF-8 string."""
    try:
        data = json.dumps(obj) + "\n"
        client.sendall(data.encode("utf-8"))
    except Exception as e:
        print("[TCP] Send error:", e)


def recv_line(client):
    """Receive a newline-terminated line from the client.

    Returns the decoded string (without newline) or None on error/disconnect.
    """
    buf = b""
    try:
        while True:
            ch = client.recv(1)
            if not ch:
                return None  # client disconnected
            if ch == b"\n":
                break
            buf += ch
            if len(buf) > config.TCP_RECV_BUF:
                break
        return buf.decode("utf-8").strip()
    except Exception:
        return None


# ── Command Handlers ──────────────────────────────────────────────────────────

def handle_scan(client, cam, detector, tof, panel=None):
    """Capture a frame, run detection, and send results.

    Args:
        panel: Optional panel name ("panel1"/"panel2") for layout-aware
               positional identification when templates are missing.
    """
    set_led(*config.LED_SCANNING)

    # Update ToF reading
    tof.update()

    # Capture frame
    img = cam.snapshot()
    if img is None:
        send_json(client, {"type": "error", "message": "snapshot failed"})
        set_led(*config.LED_IDLE)
        return

    # Run detection pipeline (with panel layout for positional fallback)
    buttons = detector.detect(img, tof_reader=tof, panel=panel)

    # Build response
    response = {
        "type": "detections",
        "buttons": buttons,
        "timestamp": time.ticks_ms(),
    }
    send_json(client, response)
    set_led(*config.LED_IDLE)

    print("[Scan]", len(buttons), "button(s) detected")
    for b in buttons:
        print("  ", b["id"], b["state"],
              "d=%dmm" % b["distance_mm"],
              "px=(%d,%d)" % (b["pixel_x"], b["pixel_y"]))


def handle_status(client, cam, tof):
    """Report sensor health."""
    resp = {
        "type": "status",
        "wifi": True,  # if we got here, WiFi is up
        "camera": cam is not None,
        "tof": tof.available,
    }
    send_json(client, resp)


# ── Main Loop ─────────────────────────────────────────────────────────────────

def main():
    print("=" * 50)
    print("  OpenMV AE3 — Lift Button Detector")
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
    detector = ButtonDetector()

    # 3. Start TCP server
    server = create_server()
    set_led(*config.LED_IDLE)

    # 4. Accept connections and process commands
    while True:
        print("[TCP] Waiting for client...")
        client, addr = server.accept()
        print("[TCP] Client connected:", addr)
        set_led(*config.LED_CONNECTED)

        try:
            while True:
                line = recv_line(client)
                if line is None:
                    print("[TCP] Client disconnected")
                    break

                # Parse JSON command
                try:
                    msg = json.loads(line)
                except ValueError:
                    send_json(client, {"type": "error",
                                       "message": "invalid JSON"})
                    continue

                cmd = msg.get("cmd", "").upper()

                if cmd == "SCAN":
                    panel = msg.get("panel", None)  # e.g. "panel1", "panel2"
                    handle_scan(client, cam, detector, tof, panel=panel)
                elif cmd == "STATUS":
                    handle_status(client, cam, tof)
                else:
                    send_json(client, {"type": "error",
                                       "message": "unknown command: " + cmd})
        except Exception as e:
            print("[TCP] Session error:", e)
        finally:
            try:
                client.close()
            except Exception:
                pass
            set_led(*config.LED_IDLE)


# ── Boot ──────────────────────────────────────────────────────────────────────
main()
