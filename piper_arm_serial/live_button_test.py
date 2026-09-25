#!/usr/bin/env python3
"""Test the transform + press motion using a cam_x/y/z reading you already
have (e.g. copied from the viewer dashboard), skipping the live camera
scan entirely.

Always prints the computed poses first. Only drives the arm if you pass
--live, and even then asks for confirmation before the first move.

    # dry -- just show the numbers
    python3 live_button_test.py --cam 0.0096 0.0121 0.216

    # actually drive it, slow
    python3 live_button_test.py --cam 0.0096 0.0121 0.216 --live --speed 15

    # drive it, then return to a named look pose afterward
    python3 live_button_test.py --cam 0.0096 0.0121 0.216 --live --speed 15 \\
        --return-to yellow
"""

import argparse
import math

from arm_controller import ArmController


def to_rad(p):
    return (p[0], p[1], p[2],
            math.radians(p[3]), math.radians(p[4]), math.radians(p[5]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cam", nargs=3, type=float, required=True,
                    metavar=("X", "Y", "Z"),
                    help="cam_x cam_y cam_z, in metres, from the dashboard")
    ap.add_argument("--config", default="config.yaml")
    ap.add_argument("--speed", type=int, default=15)
    ap.add_argument("--approach-offset-mm", type=float, default=None,
                    help="override config.yaml's motion.approach_offset_mm")
    ap.add_argument("--press-depth-mm", type=float, default=None,
                    help="override config.yaml's motion.press_depth_mm")
    ap.add_argument("--clearance-mm", type=float, default=None,
                    help="override config.yaml's motion.clearance_mm")
    ap.add_argument("--live", action="store_true",
                    help="actually drive the arm. Without this, only "
                         "the computed poses are printed.")
    ap.add_argument("--return-to", default=None, metavar="POSITION",
                    help="after the press sequence, move_j back to this "
                         "recorded position (from positions/), e.g. "
                         "the look pose you tested from")
    args = ap.parse_args()

    ctrl = ArmController(config_path=args.config)
    ctrl.connect_arm()

    motion_cfg = ctrl.cfg.get("motion", {})
    approach_offset = (args.approach_offset_mm if args.approach_offset_mm is not None
                       else motion_cfg.get("approach_offset_mm", 50.0)) / 1000.0
    press_depth = (args.press_depth_mm if args.press_depth_mm is not None
                  else motion_cfg.get("press_depth_mm", 5.0)) / 1000.0
    clearance = (args.clearance_mm if args.clearance_mm is not None
                else motion_cfg.get("clearance_mm", 150.0)) / 1000.0
    tool_z = ctrl.cfg.get("tool_offset", {}).get("z", 0.145)

    cam_point = tuple(args.cam)
    print("cam point (from dashboard): %s" % (cam_point,))

    ee = ctrl.get_current_pose()
    ee_rx = math.degrees(ee[3])
    ee_ry = math.degrees(ee[4])
    ee_rz = math.degrees(ee[5])
    print("current flange pose: xyz=(%.4f, %.4f, %.4f)  rpy=(%.2f, %.2f, %.2f)deg"
          % (ee[0], ee[1], ee[2], ee_rx, ee_ry, ee_rz))

    button_base = ctrl.transform.camera_to_base(
        cam_point, ee[0], ee[1], ee[2], ee_rx, ee_ry, ee_rz)
    print("\nButton in base frame: (%.4f, %.4f, %.4f)" % button_base)

    pre, press, safe = ctrl.transform.compute_press_pose(
        button_base, ee_rx, ee_ry, ee_rz,
        approach_offset_m=approach_offset,
        press_depth_m=press_depth,
        tool_offset_z=tool_z,
        clearance_m=clearance,
    )

    print("\nComputed poses (x, y, z, rx, ry, rz):")
    print("  pre   (approach): %s" % (tuple(round(v, 4) for v in pre),))
    print("  press (contact):  %s" % (tuple(round(v, 4) for v in press),))
    print("  safe  (clear):    %s" % (tuple(round(v, 4) for v in safe),))

    if not args.live:
        print("\n[dry] Not moving -- pass --live to actually drive the arm.")
        ctrl.disconnect_arm()
        return

    print("\n" + "=" * 60)
    print("  ABOUT TO MOVE THE ARM -- speed=%d%%" % args.speed)
    print("  Sequence: approach (move_p) -> press (move_l) ->")
    print("            retract to pre (move_l) -> clear (move_l)")
    print("  Keep your hand near the E-STOP.")
    print("=" * 60)
    confirm = input("\nProceed? [y/N]: ").strip().lower()
    if confirm != "y":
        print("Cancelled.")
        ctrl.disconnect_arm()
        return

    print("\nApproach...")
    ctrl.move_to_end_pose(*to_rad(pre), mode="movej", speed=args.speed)

    print("Press (linear)...")
    ctrl.move_to_end_pose(*to_rad(press), mode="movel", speed=args.speed)
    import time
    time.sleep(0.3)

    print("Retract (linear)...")
    ctrl.move_to_end_pose(*to_rad(pre), mode="movel", speed=args.speed)

    print("Clear (linear)...")
    ctrl.move_to_end_pose(*to_rad(safe), mode="movel", speed=args.speed)

    if args.return_to:
        if args.return_to not in ctrl.positions:
            print("\nWARNING: '%s' not found in positions/ -- staying at "
                  "the clear pose. Available: %s"
                  % (args.return_to, list(ctrl.positions.keys())))
        else:
            print("\nReturning to '%s'..." % args.return_to)
            ctrl.move_to_joint_position(args.return_to, speed=args.speed)

    print("\nDone.")
    ctrl.disconnect_arm()


if __name__ == "__main__":
    main()
