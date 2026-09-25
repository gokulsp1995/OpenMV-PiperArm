#!/usr/bin/env python3
"""CLI for the Piper L lift button controller.

    # run a full sequence: approach, press, retreat
    python3 run_controller.py trajectory --name call-up

    # single press from a recorded pose
    python3 run_controller.py press --target up --start-pos up-down-panel

    # move to a recorded pose
    python3 run_controller.py move --name home --speed 15
    python3 run_controller.py move --name list

    # record a pose by hand (motors disabled)
    python3 run_controller.py record --name up-down-panel

    # arm + camera status
    python3 run_controller.py status

Add --dry-run to press/move/trajectory to log intended motion without
commanding the arm.

NOTE: the camera's serial port can only be held by one process. Stop
viewer.py before running anything that needs a scan.
"""

import argparse
import logging
import sys

from arm_controller import ArmController
from position_recorder import record_position


def setup_logging(level: str = "INFO"):
    logging.basicConfig(
        level=getattr(logging, level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")


def cmd_press(args):
    ctrl = ArmController(config_path=args.config, dry_run=args.dry_run)
    ok = ctrl.press_button(target_id=args.target,
                           start_position=args.start_pos)
    print("\n%s button '%s'." % ("Pressed" if ok else "FAILED to press",
                                 args.target))
    if not ok:
        sys.exit(1)


def cmd_trajectory(args):
    ctrl = ArmController(config_path=args.config, dry_run=args.dry_run)

    if args.name == "list":
        print("Available trajectories:")
        for n in sorted(ctrl.trajectories):
            t = ctrl.trajectories[n]
            print("  %-24s %d waypoints   %s"
                  % (n, len(t.get("trajectory", [])),
                     t.get("description", "")))
        return

    ok = ctrl.play_trajectory(args.name, speed=args.speed)
    print("\nTrajectory '%s' %s." % (args.name, "complete" if ok else "FAILED"))
    if not ok:
        sys.exit(1)


def cmd_move(args):
    ctrl = ArmController(config_path=args.config, dry_run=args.dry_run)

    if args.name == "list":
        print("Available positions:")
        for n in sorted(ctrl.positions):
            print("  %-24s %s"
                  % (n, ctrl.positions[n].get("description", "")))
        return

    ctrl.connect_arm()
    ctrl.move_to_joint_position(args.name, speed=args.speed)
    print("Moved to '%s'." % args.name)


def cmd_record(args):
    record_position(
        channel=args.channel,
        position_name=args.name,
        output_dir=args.output_dir,
        description=args.description,
        interface=args.interface,
        firmware_version=args.firmware,
    )


def cmd_status(args):
    ctrl = ArmController(config_path=args.config, dry_run=False)
    try:
        ctrl.connect_arm()
    except Exception as e:
        print("Arm connection failed:", e)

    if not args.no_camera:
        try:
            ctrl.connect_camera()
        except Exception as e:
            print("Camera connection failed:", e)

    ctrl.print_status()


def main():
    p = argparse.ArgumentParser(
        description="Piper L -- lift button controller",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="config.yaml")
    p.add_argument("--log-level", default="INFO",
                   choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    sub = p.add_subparsers(dest="command", required=True)

    sp = sub.add_parser("trajectory", help="Run a waypoint sequence.")
    sp.add_argument("--name", required=True,
                    help="trajectory name, or 'list'")
    sp.add_argument("--speed", type=int, default=None,
                    help="override move speed %% for joint moves")
    sp.add_argument("--dry-run", action="store_true")
    sp.set_defaults(func=cmd_trajectory)

    sp = sub.add_parser("press", help="Press a button from a recorded pose.")
    sp.add_argument("--target", required=True, help="button id, e.g. up")
    sp.add_argument("--start-pos", required=True)
    sp.add_argument("--dry-run", action="store_true")
    sp.set_defaults(func=cmd_press)

    sp = sub.add_parser("move", help="Move to a recorded position.")
    sp.add_argument("--name", required=True, help="position name, or 'list'")
    sp.add_argument("--speed", type=int, default=None)
    sp.add_argument("--dry-run", action="store_true")
    sp.set_defaults(func=cmd_move)

    sp = sub.add_parser("record", help="Record a pose by manual teaching.")
    sp.add_argument("--name", required=True)
    sp.add_argument("--channel", default="can0")
    sp.add_argument("--interface", default="socketcan")
    sp.add_argument("--firmware", default="default")
    sp.add_argument("--output-dir", default="positions")
    sp.add_argument("--description", default="")
    sp.set_defaults(func=cmd_record)

    sp = sub.add_parser("status", help="Show arm and camera status.")
    sp.add_argument("--no-camera", action="store_true")
    sp.set_defaults(func=cmd_status)

    args = p.parse_args()
    setup_logging(args.log_level)
    args.func(args)


if __name__ == "__main__":
    main()
