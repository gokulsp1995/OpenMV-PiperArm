#!/usr/bin/env python3
"""Analyse a calibration log produced by calib_logger.py.

Runs the invariance test: if the camera_transform is correct, the same
physical button should compute to the SAME base-frame coordinate from
every arm pose. The spread across poses IS your calibration error.

Usage:
    python3 calib_analyse.py calib_run1.jsonl --target up
"""

import argparse
import json
import math

from arm_controller import ArmController


def load(path):
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                samples.append(json.loads(line))
    return samples


def mean(vals):
    return sum(vals) / len(vals) if vals else 0.0


def stdev(vals):
    if len(vals) < 2:
        return 0.0
    m = mean(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--target", required=True,
                     help="Button id to analyse (e.g. 'up')")
    args = ap.parse_args()

    samples = load(args.logfile)
    print("Loaded %d sample(s)" % len(samples))

    # ArmController is used only for its CoordinateTransform -- no arm
    # connection needed, so this runs offline on logged data.
    ctrl = ArmController()

    points = []
    for s in samples:
        btn = None
        for b in s.get("buttons", []):
            if b.get("id") == args.target:
                btn = b
                break
        if btn is None:
            continue

        f = s["flange"]
        cam_point = (btn.get("cam_x", 0.0),
                      btn.get("cam_y", 0.0),
                      btn.get("cam_z", 0.0))

        base = ctrl.transform.camera_to_base(
            cam_point,
            f["x"], f["y"], f["z"],
            math.degrees(f["roll"]),
            math.degrees(f["pitch"]),
            math.degrees(f["yaw"]),
        )
        points.append(base)
        print("  flange=(%.3f, %.3f, %.3f) -> button_base=(%.4f, %.4f, %.4f)"
              % (f["x"], f["y"], f["z"], base[0], base[1], base[2]))

    if len(points) < 2:
        print("\nNeed at least 2 samples with target '%s' to compare."
              % args.target)
        return

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]

    print("\n--- Invariance test (%d poses) ---" % len(points))
    print("  mean  = (%.4f, %.4f, %.4f) m" % (mean(xs), mean(ys), mean(zs)))
    print("  stdev = (%.4f, %.4f, %.4f) m" % (stdev(xs), stdev(ys), stdev(zs)))
    print("  spread= (%.4f, %.4f, %.4f) m" % (max(xs) - min(xs),
                                                max(ys) - min(ys),
                                                max(zs) - min(zs)))

    worst = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    print("\n  Worst-axis spread: %.1f mm" % (worst * 1000))
    print("  The button did not move between samples, so this spread is")
    print("  calibration error. Which axis dominates tells you which")
    print("  camera_transform component is most wrong.")


if __name__ == "__main__":
    main()
