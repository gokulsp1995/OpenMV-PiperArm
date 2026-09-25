#!/usr/bin/env python3
"""Phase 1 gate: prove the serial transport works, in isolation.

No arm, no CAN, no trajectory code. Run this until it passes before
touching anything else.

    python3 test_serial.py                 # auto-resolve by /dev/serial/by-id
    python3 test_serial.py --port /dev/ttyACM4
    python3 test_serial.py --listen        # watch boot output, then reset

Each test isolates one layer, so a failure tells you where the problem is
rather than just that something is wrong.
"""

import argparse
import glob
import json
import sys
import time

import serial


def find_port(hint="OpenMV"):
    if hint.startswith("/dev/"):
        return hint
    for path in glob.glob("/dev/serial/by-id/*"):
        if hint.lower() in path.lower():
            return path
    raise RuntimeError("No by-id entry matched %r. Found: %s"
                        % (hint, glob.glob("/dev/serial/by-id/*")))


def test_0_enumeration(port):
    print("[0] Device path")
    print("    %s" % port)
    return True


def test_1_open(port, timeout):
    """Can we open the port at all? Catches permissions and contention."""
    print("[1] Open port (dsrdtr=False)")
    try:
        ser = serial.Serial(port, 115200, timeout=timeout, dsrdtr=False)
        print("    ok -- opened")
        return ser
    except Exception as e:
        print("    FAIL: %s" % e)
        print("    Check: is picocom or OpenMV IDE holding the port?")
        print("           is your user in the dialout group?")
        return None


def test_2_raw_read(ser, seconds=3.0):
    """Is anything arriving unprompted? Shows what state the camera is in."""
    print("[2] Listen %.0fs for unprompted output" % seconds)
    ser.reset_input_buffer()
    deadline = time.time() + seconds
    got = b""
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            got += chunk
    if got:
        print("    saw %d bytes:" % len(got))
        for line in got.decode("utf-8", "ignore").splitlines():
            print("      %r" % line)
    else:
        print("    (silence -- normal if main.py already booted and is waiting)")
    return got


def test_3_status(ser, timeout):
    """The real test: send a command, get valid JSON back."""
    print("[3] Send STATUS")
    ser.reset_input_buffer()
    ser.write(b'{"cmd": "STATUS"}\n')

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        ch = ser.read(1)
        if not ch:
            continue
        if ch == b"\n":
            line = buf.decode("utf-8", "ignore").strip()
            buf = b""
            if not line:
                continue
            print("    <- %r" % line)
            if line.startswith("{'"):
                print("    DIAGNOSIS: single quotes = Python repr.")
                print("    The camera is at a bare REPL, not running main.py.")
                return False
            if line == '{"cmd": "STATUS"}':
                print("    DIAGNOSIS: that's your own input echoed back.")
                print("    The camera is at a bare REPL, not running main.py.")
                return False
            if not line.startswith("{"):
                continue   # boot banner or stray print, keep reading
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("type") == "status":
                print("    PASS -- camera:%s tof:%s"
                      % (msg.get("camera"), msg.get("tof")))
                return True
            print("    (got type=%s, still waiting for status)" % msg.get("type"))
        else:
            buf += ch

    print("    FAIL: no status reply within %.0fs" % timeout)
    print("    If test 2 was also silent, main.py probably isn't running.")
    return False


def test_4_scan(ser, timeout):
    """Detection over the wire. Separate from transport -- may legitimately
    return zero buttons if the camera isn't pointed at a panel."""
    print("[4] Send SCAN")
    ser.reset_input_buffer()
    ser.write(b'{"cmd": "SCAN"}\n')

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        ch = ser.read(1)
        if not ch:
            continue
        if ch == b"\n":
            line = buf.decode("utf-8", "ignore").strip()
            buf = b""
            if not line or not line.startswith("{"):
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("type") == "detections":
                btns = msg.get("buttons", [])
                print("    PASS -- %d button(s)" % len(btns))
                for b in btns:
                    print("      %s px=(%s,%s) d=%smm cam=(%s,%s,%s)"
                          % (b.get("id"), b.get("pixel_x"), b.get("pixel_y"),
                             b.get("distance_mm"), b.get("cam_x"),
                             b.get("cam_y"), b.get("cam_z")))
                if not btns:
                    print("      (empty is fine if not aimed at a panel)")
                return True
            if msg.get("type") == "error":
                print("    camera reported: %s" % msg.get("message"))
                return False
        else:
            buf += ch

    print("    FAIL: no scan reply within %.0fs" % timeout)
    return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="OpenMV",
                     help="device path or by-id substring")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--listen", action="store_true",
                     help="just watch output -- reset the camera while this runs")
    args = ap.parse_args()

    try:
        port = find_port(args.port)
    except Exception as e:
        print("FAIL:", e)
        sys.exit(1)

    test_0_enumeration(port)
    ser = test_1_open(port, args.timeout)
    if ser is None:
        sys.exit(1)

    try:
        if args.listen:
            print("\nListening 20s -- reset the camera now (button or replug).")
            print("You want to see: {\"type\": \"boot\", ...}\n")
            test_2_raw_read(ser, 20.0)
            return

        test_2_raw_read(ser, 3.0)
        if not test_3_status(ser, args.timeout):
            print("\nStopping -- fix STATUS before testing SCAN.")
            sys.exit(1)
        test_4_scan(ser, args.timeout)
        print("\nPhase 1 passed. Transport works.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
