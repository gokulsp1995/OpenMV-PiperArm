#!/usr/bin/env python3
"""Solve for camera_transform (T_flange_cam) from logged samples.

The button never moves, so its base-frame position is one unknown
constant. Every sample must agree on it:

    R_ee_i * (R_cam * p_cam_i + t_cam) + t_ee_i  =  p_button   (same for all i)

9 unknowns: R_cam (3 angles) + t_cam (3) + p_button (3).
Each sample gives 3 equations, so >= 3 samples is minimally determined;
more gives a real least-squares fit with a residual you can trust.

Usage:
    python3 solve_camera_transform.py calib.jsonl --target up

Input: the JSONL produced by calib_logger.py -- one JSON object per line,
each with "flange" (x,y,z,roll,pitch,yaw) and "buttons" (the camera's
detection list for that pose).

Needs: pip install numpy scipy
"""

import argparse
import json
import math
import sys

try:
    import numpy as np
    from scipy.optimize import least_squares
except ImportError:
    print("ERROR: needs numpy and scipy.\n"
          "  pip install --break-system-packages numpy scipy")
    sys.exit(1)


# -- Rotation, matching coordinate_transform.py's convention exactly ------------
# R = Rz . Ry . Rx, degrees. Copied rather than imported so this script has
# no dependency on the robot-side package and can run anywhere.

def rotation_matrix_xyz(rx_deg, ry_deg, rz_deg):
    rx, ry, rz = math.radians(rx_deg), math.radians(ry_deg), math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    return np.array([
        [cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz],
        [cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz],
        [-sy,   sx*cy,             cx*cy],
    ])


def load_samples(path, target_id):
    """Load logged samples, keeping only ones with a valid detection of
    the target button."""
    samples = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)

            btn = None
            for b in rec.get("buttons", []):
                if b.get("id") == target_id and b.get("distance_mm", 0) > 0:
                    btn = b
                    break
            if btn is None:
                continue

            fl = rec["flange"]
            samples.append({
                "ee_t": np.array([fl["x"], fl["y"], fl["z"]]),
                "ee_r": (math.degrees(fl["roll"]),
                        math.degrees(fl["pitch"]),
                        math.degrees(fl["yaw"])),
                "p_cam": np.array([btn["cam_x"], btn["cam_y"], btn["cam_z"]]),
            })
    return samples


def residuals(params, samples):
    """One residual triple per sample: predicted base position minus the
    (unknown, also being solved for) button position. The solver drives
    all of these toward zero simultaneously."""
    cam_r = params[0:3]
    cam_t = params[3:6]
    button = params[6:9]

    R_cam = rotation_matrix_xyz(*cam_r)

    out = []
    for s in samples:
        p_ee = R_cam @ s["p_cam"] + cam_t
        R_ee = rotation_matrix_xyz(*s["ee_r"])
        p_base = R_ee @ p_ee + s["ee_t"]
        out.append(p_base - button)

    return np.concatenate(out)


def orientation_spread(samples):
    """How much the samples actually vary in orientation.

    Pure translation between poses gives zero information about the
    rotation part of the transform -- the fit will converge to SOMETHING
    but it will be meaningless. This is a coarse sanity check, not a
    rigorous one: it just reports the range of each Euler angle across
    the samples, so an obviously flat set (e.g. all samples within 2
    degrees of each other) is caught before you trust the result.
    """
    angles = np.array([s["ee_r"] for s in samples])
    spread = angles.max(axis=0) - angles.min(axis=0)
    return spread   # [roll_range, pitch_range, yaw_range], degrees


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--target", default="up")
    ap.add_argument("--init-tz", type=float, default=0.05,
                    help="initial guess for tz, metres (default 0.05)")
    args = ap.parse_args()

    samples = load_samples(args.logfile, args.target)
    print("Loaded %d valid sample(s) for target '%s'"
          % (len(samples), args.target))

    if len(samples) < 4:
        print("\nNeed at least 4 samples (3 is the bare minimum, 4+ gives "
              "a real residual to judge). Collect more with calib_logger.py.")
        sys.exit(1)

    spread = orientation_spread(samples)
    print("Orientation spread across samples (deg): "
          "roll=%.1f  pitch=%.1f  yaw=%.1f" % tuple(spread))
    if max(spread) < 15:
        print("\nWARNING: less than 15 degrees of spread on every axis.")
        print("This under-determines the rotation part of the transform --")
        print("the fit below may converge to a low residual while still")
        print("being wrong. Collect samples at more varied ORIENTATIONS,")
        print("not just different distances, before trusting this result.")

    # Initial guess: zero rotation, small forward offset, button roughly
    # where the first sample's naive (unrotated) transform would put it.
    x0 = np.zeros(9)
    x0[5] = args.init_tz   # tz
    R0 = rotation_matrix_xyz(0, 0, 0)
    p0 = R0 @ samples[0]["p_cam"] + x0[3:6]
    Ree0 = rotation_matrix_xyz(*samples[0]["ee_r"])
    x0[6:9] = Ree0 @ p0 + samples[0]["ee_t"]

    result = least_squares(residuals, x0, args=(samples,), method="lm")

    cam_r = result.x[0:3]
    cam_t = result.x[3:6]
    button = result.x[6:9]

    res = residuals(result.x, samples).reshape(-1, 3)
    per_sample_error = np.linalg.norm(res, axis=1)

    print("\n" + "=" * 58)
    print("  RESULT")
    print("=" * 58)
    print("  RMS residual: %.4f m  (%.1f mm)"
          % (np.sqrt(np.mean(per_sample_error**2)),
             np.sqrt(np.mean(per_sample_error**2)) * 1000))
    print("  worst sample: %.4f m  (%.1f mm)"
          % (per_sample_error.max(), per_sample_error.max() * 1000))
    print()
    print("  per-sample residuals (mm):")
    for i, e in enumerate(per_sample_error):
        print("    sample %2d:  %6.1f mm" % (i + 1, e * 1000))

    print("\n--- judging the result ---")
    rms_mm = np.sqrt(np.mean(per_sample_error**2)) * 1000
    if rms_mm < 5:
        print("  Under 5mm RMS: good.")
    elif rms_mm < 15:
        print("  Under 15mm RMS: usable, worth tightening if time allows.")
    else:
        print("  Over 15mm RMS: SUSPECT. Check orientation spread above,")
        print("  and check for any sample with unusually high residual --")
        print("  a single bad detection can be dropped and re-solved.")

    print("\n" + "=" * 58)
    print("  PASTE INTO config.yaml's camera_transform")
    print("=" * 58)
    print("camera_transform:")
    print("  calibrated: true")
    print("  tx: %.4f" % cam_t[0])
    print("  ty: %.4f" % cam_t[1])
    print("  tz: %.4f" % cam_t[2])
    print("  rx: %.2f" % cam_r[0])
    print("  ry: %.2f" % cam_r[1])
    print("  rz: %.2f" % cam_r[2])
    print()
    print("  # solved button position in base frame, for reference:")
    print("  # (%.4f, %.4f, %.4f) m" % tuple(button))
    print()
    print("Run the existing invariance test (calib_analyse.py) as a second,")
    print("independent check before trusting this in a live press.")


if __name__ == "__main__":
    main()