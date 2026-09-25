# OpenMV AE3 — Lift Button Detector — Main Entry Point
# ====================================================
# Auto-runs on boot. Talks over the USB-C port's serial console.
#
# TRANSPORT NOTE: this board is the Alif Ensemble E3 port, not STM32.
# `pyb` does not exist and `machine.USB_VCP` does not exist. The USB serial
# channel is reachable as sys.stdin / sys.stdout -- the same streams the
# REPL uses. Confirmed working for raw bytes via sys.stdout.buffer.write().
#
# CONSEQUENCE: do not use print() anywhere in this file or in any module it
# imports once this is live. print() writes to the same sys.stdout that
# send_json() uses, and will inject non-JSON text into the protocol stream.
# button_detector.py and tof_reader.py both still have print() calls in
# their init paths -- those fire once at boot, before the loop starts, so
# they are survivable, but any print() during a scan will corrupt output.
# 
# Protocol (JSON, newline-delimited):
#   Robot -> Camera: {"cmd": "SCAN"} / {"cmd": "STATUS"}
#   Camera -> Robot: {"type": "detections"|"status"|"error"|"boot", ...}

import csi
import sys
import time
import json
import binascii
from machine import LED

import config
from button_detector import ButtonDetector
from tof_reader import ToFReader

# -- LED helpers ---------------------------------------------------------------

_led_r = LED("LED_RED")
_led_g = LED("LED_GREEN")
_led_b = LED("LED_BLUE")


def set_led(r, g, b):
    """Set RGB LED state (1 = on, 0 = off)."""
    _led_r.on() if r else _led_r.off()
    _led_g.on() if g else _led_g.off()
    _led_b.on() if b else _led_b.off()


# -- Camera ---------------------------------------------------------------------

def init_camera():
    """Initialise the PAG7936 camera sensor.

    GRAYSCALE + QVGA, not RGB565 + VGA: find_template() with SEARCH_EX
    raises MemoryError at VGA on this board, and a to_grayscale() copy makes
    it worse. QVGA grayscale is what worked reliably in the IDE.

    NOTE: this means colour classification no longer works -- _classify_*
    will report "dark" for everything, since a/b channels don't exist on a
    grayscale frame. press_button()'s white_lit verification is affected.
    """
    cam = csi.CSI()
    cam.reset()
    cam.pixformat(csi.GRAYSCALE)
    cam.framesize(csi.QVGA)
    cam.snapshot(time=2000)
    return cam


# -- Serial (sys.stdin / sys.stdout over USB-C) ---------------------------------

def send_json(obj):
    """Send a JSON object as a newline-terminated UTF-8 string."""
    try:
        data = json.dumps(obj) + "\n"
        sys.stdout.buffer.write(data.encode("utf-8"))
    except Exception:
        pass  # nowhere safe to report -- print() would corrupt the stream


def recv_line():
    """Block until a full line arrives on the serial console.

    Blocking is fine: the loop has nothing else to do between commands.
    Returns None or "" for blank lines, which the caller skips -- leftover
    newlines in the input buffer would otherwise parse as invalid JSON.
    """
    line = sys.stdin.readline()
    if not line:
        return None
    return line.strip()


# -- Command Handlers ------------------------------------------------------------

def _idle_led(tof):
    return config.LED_IDLE if tof.available else config.LED_DEGRADED


def handle_scan(cam, detector, tof, panel=None):
    set_led(*config.LED_SCANNING)

    tof.update()

    img = cam.snapshot()
    if img is None:
        send_json({"type": "error", "message": "snapshot failed"})
        set_led(*config.LED_ERROR)
        time.sleep_ms(300)
        set_led(*_idle_led(tof))
        return

    try:
        buttons = detector.detect(img, tof_reader=tof, panel=panel)
    except Exception as e:
        send_json({"type": "error", "message": "detect failed: %s" % e})
        set_led(*config.LED_ERROR)
        time.sleep_ms(300)
        set_led(*_idle_led(tof))
        return

    send_json({
        "type": "detections",
        "buttons": buttons,
        "timestamp": time.ticks_ms(),
    })
    set_led(*_idle_led(tof))


def handle_status(cam, tof):
    send_json({
        "type": "status",
        "serial": True,
        "camera": cam is not None,
        "tof": tof.available,
    })

def handle_frame(cam, detector, tof, panel=None, quality=50):
    """Run detection, draw the results, and return the annotated frame.
 
    Returns the same button list as SCAN, so one request gives both the
    numbers and the picture they came from
    """
    set_led(*config.LED_SCANNING)

    # Show the search region so you can see what's being scanned
    
    tof.update()
 
    img = cam.snapshot()
    if img is None:
        send_json({"type": "error", "message": "snapshot failed"})
        set_led(*_idle_led(tof))
        return
 
    try:
        buttons = detector.detect(img, tof_reader=tof, panel=panel)
    except Exception as e:
        send_json({"type": "error", "message": "detect failed: %s" % e})
        set_led(*_idle_led(tof))
        return
    if config.TEMPLATE_ROI:
        img.draw_rectangle(tuple(config.TEMPLATE_ROI), color=120)
   
 # Annotate. Grayscale frame, so 255 = white.
    for b in buttons:
        px = b.get("pixel_x", 0)
        py = b.get("pixel_y", 0)
        img.draw_cross((px, py), color=255)
        img.draw_circle((px, py, 12), color=255)
        img.draw_string((max(0, px - 20), max(0, py - 26)),
                         "%s %dmm" % (b.get("id"), b.get("distance_mm", 0)),
                         color=255)
 
    try:
        jpeg = img.compress(quality=quality).bytearray()
        b64 = binascii.b2a_base64(jpeg).decode("utf-8").strip()
    except Exception as e:
        send_json({"type": "error", "message": "compress failed: %s" % e})
        set_led(*_idle_led(tof))
        return
    
    send_json({
        "type": "frame",
        "buttons": buttons,
        "tof_grid": tof.get_full_grid(),
        "jpeg_b64": b64,
        "timestamp": time.ticks_ms(),
    })
    set_led(*_idle_led(tof))


# -- Main Loop --------------------------------------------------------------------

def main():
    cam = init_camera()
    tof = ToFReader()
    detector = ButtonDetector()

    set_led(*_idle_led(tof))

    # Machine-checkable proof of a real boot. A bare REPL cannot produce
    # this -- it would echo input back or print a Python repr with single
    # quotes. A "type" key in valid JSON means main.py is genuinely running.
    send_json({"type": "boot", "message": "main.py running"})

    while True:
        line = recv_line()
        if not line:
            continue

        try:
            msg = json.loads(line)
        except ValueError:
            send_json({"type": "error", "message": "invalid JSON"})
            continue

        cmd = msg.get("cmd", "").upper()

        if cmd == "SCAN":
            handle_scan(cam, detector, tof, panel=msg.get("panel"))

        elif cmd == "STATUS":
            handle_status(cam, tof)

        elif cmd == "FRAME":
            handle_frame(cam, detector, tof,
                         panel=msg.get("panel"),
                         quality=msg.get("quality", 50))
        else:
            send_json({"type": "error", "message": "unknown command: " + cmd})


main()