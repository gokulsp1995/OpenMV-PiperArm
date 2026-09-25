#!/usr/bin/env python3
"""Log synchronized arm pose + camera detections for calibration work.

Run with the arm in teach mode, move it to several poses while keeping the
same button in view, and press ENTER at each pose to record a sample.
Produces a JSONL file (one JSON object per line) for calib_analyse.py.

Vary ORIENTATION between poses, not just distance -- pure translation
cannot separate a rotation error in camera_transform from a translation
error, and the analysis will look fine while the rotation is still wrong.

Usage:
    python3 calib_logger.py --out calib.jsonl
    python3 calib_logger.py --out calib.jsonl --target up

NEW: reads the camera's port_hint/baudrate/timeout from config.yaml, same
as arm_controller.py and everything else -- no --camera-ip anymore, since
the camera is serial now, not a TCP server with an IP.
"""

import argparse
import json
import time
import sys

import yaml

from arm_controller import ArmController
from camera_client import CameraClient


def load_camera_from_config(config_path="config.yaml"):
    with open(config_path) as f:
        cfg = yaml.safe_load(f) or {}
    cam_cfg = cfg.get("camera", {})
    return CameraClient(
        cam_cfg.get("port_hint", "OpenMV"),
        cam_cfg.get("baudrate", 115200),
        cam_cfg.get("timeout", 5.0),
    )


def capture_sample(arm, cam, target_id, panel, label=None):
    """Grab one synchronized (arm pose, camera detection) pair."""
    pose = arm.get_current_pose()
    joints = arm.get_current_joint_angles()
    result = cam.scan(panel=panel)

    sample = {
        "t": time.time(),
        "label": label,
        "target": target_id,
        "flange": {
            "x": pose[0], "y": pose[1], "z": pose[2],
            "roll": pose[3], "pitch": pose[4], "yaw": pose[5],
        },
        "joints": list(joints),
        "scan_ok": result is not None and result.get("type") == "detections",
        "buttons": result.get("buttons", []) if result else [],
    }
    return sample


def summarize(sample, target_id):
    f = sample["flange"]
    head = "flange=(%.3f, %.3f, %.3f)" % (f["x"], f["y"], f["z"])
    btn = next((b for b in sample["buttons"] if b.get("id") == target_id), None)
    if not btn:
        return head + "  | NO DETECTION for '%s'" % target_id
    return head + "  | %s px=(%s,%s) d=%smm cam=(%.4f, %.4f, %.4f)" % (
        btn.get("id"), btn.get("pixel_x"), btn.get("pixel_y"),
        btn.get("distance_mm"),
        btn.get("cam_x", 0.0), btn.get("cam_y", 0.0), btn.get("cam_z", 0.0),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--target", default="up",
                    help="button id to track (default: up)")
    ap.add_argument("--panel", default=None)
    ap.add_argument("--out", default="calib_log.jsonl")
    ap.add_argument("--continuous", action="store_true",
                    help="log continuously instead of on keypress")
    ap.add_argument("--interval", type=float, default=0.5,
                    help="seconds between samples in continuous mode")
    args = ap.parse_args()

    arm = ArmController(config_path=args.config)

    print("Connecting to arm...")
    arm.connect_arm()

    print("Disabling motors for teach mode...")
    t0 = time.time()
    while not arm.robot.disable():
        time.sleep(0.01)
        if time.time() - t0 > 5:
            print("WARNING: could not disable all joints.")
            break
    time.sleep(0.5)

    print("Connecting to camera...")
    cam = load_camera_from_config(args.config)
    if not cam.connect():
        print("ERROR: could not open camera port -- is viewer.py or "
              "picocom holding it?")
        sys.exit(1)
    st = cam.status()
    if st is None:
        print("ERROR: camera did not answer STATUS -- is main.py running "
              "on it? (check with a reset if unsure)")
        sys.exit(1)
    print("Camera ready:", st)

    out = open(args.out, "a")   # append, so multiple runs accumulate
    n = 0

    print("\nLogging to %s (target='%s')" % (args.out, args.target))
    if args.continuous:
        print("Continuous mode -- Ctrl-C to stop.")
    else:
        print("Move the arm by hand, then press ENTER to record a pose.")
        print("VARY ORIENTATION, not just distance. Type 'q' to finish.\n")

    try:
        while True:
            label = None
            if not args.continuous:
                resp = input("[sample %d] ENTER to record (or q): " % (n + 1))
                if resp.strip().lower() == "q":
                    break
                label = resp.strip() or None

            sample = capture_sample(arm, cam, args.target, args.panel, label)
            out.write(json.dumps(sample) + "\n")
            out.flush()
            n += 1
            print("  " + summarize(sample, args.target))

            if args.continuous:
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        out.close()
        cam.disconnect()
        try:
            arm.disconnect_arm()
        except Exception:
            pass
        print("Wrote %d sample(s) to %s" % (n, args.out))
        if n < 4:
            print("WARNING: fewer than 4 samples -- collect at least 4-6, "
                 "ideally 6-10, with varied orientation, before analysing.")
        else:
            print("Now run:  python3 calib_analyse.py %s --target %s"
                 % (args.out, args.target))


if __name__ == "__main__":
    main()