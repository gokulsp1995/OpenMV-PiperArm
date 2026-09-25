#!/usr/bin/env python3
"""Stage 1c — solve for camera intrinsics.

    python3 solve_intrinsics.py --images calib_images/ --square-mm 24.5

Runs cv2.calibrateCamera over the captured checkerboard images and prints
the values to paste into the camera's config.py.

SQUARE SIZE IS THE ONE THING YOU MUST MEASURE
---------------------------------------------
--square-mm is the physical edge length of one square, measured on the
screen (or paper) you actually displayed. It is the only place real-world
scale enters the calculation.

Getting it wrong does NOT corrupt the focal length or principal point --
those are pixel quantities and come out fine. It corrupts the extrinsics,
which matters later when the same measurement discipline applies to your
AprilTag size.

Measure across several squares and divide. Measuring a single square
compounds the reading error.

Needs: pip install opencv-python
"""

import argparse
import glob
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("ERROR: needs opencv-python and numpy.\n"
          "  pip install --break-system-packages opencv-python numpy\n"
          "  (or inside your venv: pip install opencv-python numpy)")
    sys.exit(1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", default="calib_images")
    ap.add_argument("--cols", type=int, default=9,
                    help="INNER corners across (default 9)")
    ap.add_argument("--rows", type=int, default=6,
                    help="INNER corners down (default 6)")
    ap.add_argument("--square-mm", type=float, required=True,
                    help="measured square edge length, in mm")
    ap.add_argument("--show-failures", action="store_true",
                    help="list images where corners were not found")
    args = ap.parse_args()

    pattern = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0

    # Object points: the board's corners in its own frame, z=0 since the
    # board is planar. Scaled to metres so the extrinsics come out in
    # metres too -- consistent with everything else in the pipeline.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= square_m

    obj_points = []
    img_points = []
    img_shape = None
    failures = []

    paths = sorted(glob.glob(os.path.join(args.images, "*.jpg")))
    if not paths:
        print("No .jpg files in %s/" % args.images)
        sys.exit(1)

    print("Looking for %dx%d inner corners in %d image(s)...\n"
          % (args.cols, args.rows, len(paths)))

    for path in paths:
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            failures.append((path, "could not read"))
            continue

        if img_shape is None:
            img_shape = img.shape[::-1]      # (width, height)
        elif img.shape[::-1] != img_shape:
            failures.append((path, "size differs from the others"))
            continue

        found, corners = cv2.findChessboardCorners(
            img, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)

        if not found:
            failures.append((path, "corners not found"))
            continue

        # Sub-pixel refinement. Without this the corners are integer
        # pixels and the focal length comes out noticeably noisier.
        corners = cv2.cornerSubPix(
            img, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))

        obj_points.append(objp)
        img_points.append(corners)
        print("  ok   %s" % os.path.basename(path))

    print("\nUsable: %d / %d" % (len(obj_points), len(paths)))

    if failures and args.show_failures:
        print("\nFailed:")
        for path, why in failures:
            print("  %-24s %s" % (os.path.basename(path), why))

    if len(obj_points) < 5:
        print("\nToo few usable images to solve. Capture more, and check "
              "the board is fully visible and in focus.")
        sys.exit(1)
    if len(obj_points) < 12:
        print("\nWARNING: fewer than 12 usable images. The result will be "
              "usable but not well conditioned.")

    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None)

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    w, h = img_shape

    print("\n" + "=" * 58)
    print("  RESULT")
    print("=" * 58)
    print("  image size      %d x %d" % (w, h))
    print("  reprojection    %.4f px RMS" % rms)
    print()
    print("  fx = %.2f    fy = %.2f" % (fx, fy))
    print("  cx = %.2f    cy = %.2f" % (cx, cy))
    print("  distortion      %s"
          % np.array2string(dist.ravel(), precision=4, suppress_small=True))

    print("\n--- judging the result ---")
    if rms < 0.5:
        print("  RMS under 0.5 px: good.")
    elif rms < 1.0:
        print("  RMS under 1.0 px: acceptable.")
    else:
        print("  RMS above 1.0 px: SUSPECT. Usually blur, too few tilted")
        print("  views, or screen glare. Recapture before trusting it.")

    fx_fy_pct = abs(fx - fy) / max(fx, fy) * 100
    if fx_fy_pct > 5:
        print("  fx and fy differ by %.1f%% -- larger than square pixels"
              % fx_fy_pct)
        print("  would give. Check the board is rigid and the measured")
        print("  square size is right.")

    off_x = abs(cx - w / 2) / w * 100
    off_y = abs(cy - h / 2) / h * 100
    if off_x > 10 or off_y > 10:
        print("  principal point is %.0f%%/%.0f%% off centre -- large."
              % (off_x, off_y))
        print("  Possible, but worth re-checking with more edge coverage.")

    print("\n" + "=" * 58)
    print("  PASTE INTO THE CAMERA'S config.py")
    print("=" * 58)
    print("CAMERA_WIDTH = %d" % w)
    print("CAMERA_HEIGHT = %d" % h)
    print("PRINCIPAL_X = %.2f" % cx)
    print("PRINCIPAL_Y = %.2f" % cy)
    print("FOCAL_LENGTH_PX = %.2f" % ((fx + fy) / 2))
    print()
    print("# _pixel_to_camera() uses one focal length for both axes.")
    print("# The average above is the honest simplification; if fx and fy")
    print("# differ much, change that function to take both separately.")
    print()
    print("# Distortion is NOT used by _pixel_to_camera(). For a button")
    print("# near frame centre the error is small. If detections near the")
    print("# frame edges come out biased, that is where it shows up.")


if __name__ == "__main__":
    main()