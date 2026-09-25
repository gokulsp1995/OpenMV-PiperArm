#!/usr/bin/env python3
"""Stage 1b — capture checkerboard images for intrinsics calibration.

Pulls frames over the existing FRAME command and saves them as JPEGs.
Press ENTER to grab; move the board (or the camera) between grabs.

    python3 capture_intrinsics.py --out calib_images/

WHAT MAKES A GOOD SET
---------------------
15-20 images, and the VARIETY matters more than the count:

  - board tilted at different angles, not just face-on. Tilt is what
    lets the solver separate focal length from distance -- a set of
    face-on shots is degenerate and gives a confident wrong answer.
  - board in different parts of the frame: centre, each corner, edges.
    Distortion is strongest at the edges, so a set that only uses the
    centre cannot estimate it.
  - a range of distances, near and far.
  - the whole board visible and in focus every time.

The camera's lens focuses by rotating the barrel and appears set for
~20cm. Check sharpness before starting -- a blurry set calibrates the
blur, not the lens.
"""

import argparse
import base64
import glob
import json
import os
import sys
import time

import serial

try:
    import cv2
    import numpy as np
    HAVE_CV = True
except ImportError:
    HAVE_CV = False


def check_corners(jpeg_bytes, pattern):
    """Detect the board in a captured frame.

    Returns (found, coverage_note). Running this at capture time is the
    whole point -- otherwise you pose the arm 20 times, then discover at
    solve time that half the shots had the board clipped or blurred, and
    have to start over.
    """
    if not HAVE_CV:
        return None, "opencv not installed -- cannot verify"

    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
    if img is None:
        return False, "could not decode frame"

    found, corners = cv2.findChessboardCorners(
        img, pattern,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if not found:
        return False, "board not fully visible, or too blurred"

    # Where in the frame did it land? Distortion is strongest at the
    # edges, so a set that only ever sees the board centred cannot
    # estimate it. Report the region so you can deliberately cover the
    # corners too.
    h, w = img.shape
    cx = float(corners[:, 0, 0].mean()) / w
    cy = float(corners[:, 0, 1].mean()) / h

    horiz = "left" if cx < 0.38 else ("right" if cx > 0.62 else "centre")
    vert = "top" if cy < 0.38 else ("bottom" if cy > 0.62 else "middle")
    region = "centre" if (horiz, vert) == ("centre", "middle") \
        else "%s-%s" % (vert, horiz)

    # Fraction of the frame the board spans. Very small means far away
    # and the corners will be imprecise; very large risks clipping.
    span_x = (corners[:, 0, 0].max() - corners[:, 0, 0].min()) / w
    span_y = (corners[:, 0, 1].max() - corners[:, 0, 1].min()) / h
    fill = max(span_x, span_y)

    note = "%s, fills %.0f%% of frame" % (region, fill * 100)
    if fill < 0.25:
        note += "  (far -- corners will be imprecise)"
    elif fill > 0.92:
        note += "  (very close -- risk of clipping)"

    return True, note


def find_port(hint="OpenMV"):
    if hint.startswith("/dev/"):
        return hint
    for path in glob.glob("/dev/serial/by-id/*"):
        if hint.lower() in path.lower():
            return path
    raise RuntimeError("No /dev/serial/by-id/ entry matched %r" % hint)


def read_json_line(ser, timeout):
    """Read one JSON line, skipping the camera's init print() output.

    Reads in chunks rather than byte-at-a-time: a base64 JPEG is tens of
    thousands of bytes and one syscall per byte is slow enough to cause
    spurious timeouts.
    """
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if not chunk:
            continue
        buf += chunk
        while b"\n" in buf:
            raw, _, rest = bytes(buf).partition(b"\n")
            buf = bytearray(rest)
            line = raw.decode("utf-8", "ignore").strip()
            if not line or not line.startswith("{"):
                continue
            try:
                return json.loads(line)
            except ValueError:
                continue
    return None


def grab(ser, quality, timeout):
    ser.reset_input_buffer()
    ser.write((json.dumps({"cmd": "FRAME", "quality": quality}) + "\n")
              .encode("utf-8"))
    msg = read_json_line(ser, timeout)
    if msg is None:
        return None, "no response"
    if msg.get("type") == "error":
        return None, msg.get("message")
    if msg.get("type") != "frame":
        return None, "unexpected type=%s" % msg.get("type")
    try:
        return base64.b64decode(msg.get("jpeg_b64", "")), None
    except Exception as e:
        return None, "bad base64: %s" % e


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="OpenMV")
    ap.add_argument("--out", default="calib_images")
    ap.add_argument("--quality", type=int, default=90,
                    help="JPEG quality -- keep HIGH for calibration, "
                         "compression artefacts blur the corners")
    ap.add_argument("--timeout", type=float, default=10.0)
    ap.add_argument("--cols", type=int, default=9,
                    help="INNER corners across (default 9)")
    ap.add_argument("--rows", type=int, default=6,
                    help="INNER corners down (default 6)")
    ap.add_argument("--keep-failed", action="store_true",
                    help="save frames where the board was not detected")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)

    os.makedirs(args.out, exist_ok=True)

    port = find_port(args.port)
    print("Camera: %s" % port)
    ser = serial.Serial(port, 115200, timeout=args.timeout, dsrdtr=False)
    ser.reset_input_buffer()

    if not HAVE_CV:
        print("\nWARNING: opencv not installed, so captures cannot be")
        print("verified here. You will not find out a shot was unusable")
        print("until you run the solver. Install it first:")
        print("  pip install --break-system-packages opencv-python numpy\n")

    existing = len(glob.glob(os.path.join(args.out, "*.jpg")))
    n = existing
    if existing:
        print("Found %d existing image(s) -- continuing from there." % existing)

    print("""
ENTER  capture and verify
q      finish

Each shot is checked immediately: it is only saved if all %dx%d corners
are found. Pose the arm in teach mode, press ENTER, move, repeat.

Vary TILT most of all. A set of face-on shots is degenerate and gives a
confident wrong focal length. Also cover the frame CORNERS, not just the
centre -- distortion is strongest at the edges.
""" % pattern)

    regions = {}

    try:
        while True:
            resp = input("[%d saved] ENTER to capture (q to finish): " % n)
            if resp.strip().lower() == "q":
                break

            jpeg, err = grab(ser, args.quality, args.timeout)
            if jpeg is None:
                print("  failed: %s" % err)
                if err == "no response":
                    print("  (reset the camera and retry -- main.py may "
                          "have stopped)")
                continue

            found, note = check_corners(jpeg, pattern)

            if found is False and not args.keep_failed:
                print("  REJECTED: %s" % note)
                print("  -> reposition so the whole board is in view, "
                      "then try again")
                continue

            n += 1
            path = os.path.join(args.out, "img_%03d.jpg" % n)
            with open(path, "wb") as f:
                f.write(jpeg)

            if found is None:
                print("  saved %s  (unverified)" % path)
            else:
                print("  OK  %s  -- %s" % (path, note))
                region = note.split(",")[0]
                regions[region] = regions.get(region, 0) + 1

            if n and n % 5 == 0 and regions:
                print("\n  --- coverage so far ---")
                for r, c in sorted(regions.items()):
                    print("    %-16s %d" % (r, c))
                if len(regions) < 3:
                    print("    only %d region(s) -- move the board around "
                          "the frame more" % len(regions))
                print()

    except (EOFError, KeyboardInterrupt):
        print()
    finally:
        ser.close()

    print("\n%d usable image(s) in %s/" % (n, args.out))
    if regions:
        print("Regions covered: %s" % ", ".join(sorted(regions)))
    if n < 15:
        print("WARNING: fewer than 15. Aim for 15-20 with varied tilt.")
    else:
        print("Now run:  python3 solve_intrinsics.py --images %s/ "
              "--square-mm <your measurement>" % args.out)


if __name__ == "__main__":
    main()