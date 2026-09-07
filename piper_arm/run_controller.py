#!/usr/bin/env python3
"""CLI entry point for the Piper L lift button controller.

Uses pyAgxArm (https://github.com/agilexrobotics/pyAgxArm).

Usage:
    # Press a button
    python run_controller.py press --target 2 --start-pos floor-select-panel1 --camera-ip 192.168.123.X

    # Press the up arrow
    python run_controller.py press --target up --start-pos call-lift --camera-ip 192.168.123.X

    # Record a new position
    python run_controller.py record --name call-lift

    # Show arm/camera status
    python run_controller.py status --camera-ip 192.168.123.X

    # Dry run (log intended commands without moving)
    python run_controller.py press --target 2 --start-pos floor-select-panel1 --dry-run
"""

import argparse
import logging
import sys

from arm_controller import ArmController
from position_recorder import record_position


def setup_logging(level: str = "INFO"):
    """Configure logging format."""
    fmt = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
    logging.basicConfig(level=getattr(logging, level.upper(), logging.INFO),
                        format=fmt)


def cmd_press(args):
    """Handle the 'press' sub-command."""
    ctrl = ArmController(config_path=args.config, dry_run=args.dry_run)
    success = ctrl.press_button(
        target_id=args.target,
        start_position=args.start_pos,
        camera_ip=args.camera_ip,
    )
    if success:
        print("\n✓ Button '%s' pressed successfully." % args.target)
    else:
        print("\n✗ Failed to press button '%s'." % args.target)
        sys.exit(1)


def cmd_record(args):
    """Handle the 'record' sub-command."""
    record_position(
        channel=args.channel,
        position_name=args.name,
        output_dir=args.output_dir,
        description=args.description,
        interface=args.interface,
        firmware_version=args.firmware,
    )


def cmd_move(args):
    """Handle the 'move' sub-command."""
    ctrl = ArmController(config_path=args.config, dry_run=args.dry_run)
    ctrl.connect_arm()

    # List available positions if requested
    if args.name == "list":
        print("Available positions:")
        for name in ctrl.positions:
            pos = ctrl.positions[name]
            desc = pos.get("description", "")
            print("  %-30s %s" % (name, desc))
        return

    if args.speed:
        # Override the config speed for this move
        ctrl.cfg.setdefault("motion", {})["approach_speed"] = args.speed

    ctrl.move_to_joint_position(args.name)
    print("✓ Moved to position '%s'." % args.name)


def cmd_status(args):
    """Handle the 'status' sub-command."""
    ctrl = ArmController(config_path=args.config, dry_run=False)
    try:
        ctrl.connect_arm()
    except Exception as e:
        print("Arm connection failed:", e)

    if args.camera_ip:
        try:
            ctrl.connect_camera(args.camera_ip)
        except Exception as e:
            print("Camera connection failed:", e)

    ctrl.print_status()


def main():
    parser = argparse.ArgumentParser(
        description="Piper L — Lift Button Controller (pyAgxArm)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--config", default="config.yaml",
                        help="Path to config.yaml (default: config.yaml)")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
                        help="Log verbosity (default: INFO)")

    subparsers = parser.add_subparsers(dest="command", required=True)

    # ── press ─────────────────────────────────────────────────────────────────
    sp_press = subparsers.add_parser("press", help="Press a lift button.")
    sp_press.add_argument("--target", required=True,
                          help="Button to press: 1, 2, 3, up, down")
    sp_press.add_argument("--start-pos", required=True,
                          help="Starting position name "
                               "(e.g. call-lift, floor-select-panel1)")
    sp_press.add_argument("--camera-ip", required=True,
                          help="OpenMV camera IP address")
    sp_press.add_argument("--dry-run", action="store_true",
                          help="Log commands without executing")
    sp_press.set_defaults(func=cmd_press)

    # ── record ────────────────────────────────────────────────────────────────
    sp_record = subparsers.add_parser("record",
                                      help="Record arm position via teaching.")
    sp_record.add_argument("--name", required=True,
                           help="Position name (e.g. call-lift)")
    sp_record.add_argument("--channel", default="can0",
                           help="CAN channel (default: can0)")
    sp_record.add_argument("--interface", default="socketcan",
                           help="CAN interface type (default: socketcan)")
    sp_record.add_argument("--firmware", default="default",
                           help="Firmware version: default, v183, v188, v189")
    sp_record.add_argument("--output-dir", default="positions",
                           help="Directory for YAML files (default: positions/)")
    sp_record.add_argument("--description", default="",
                           help="Optional position description")
    sp_record.set_defaults(func=cmd_record)

    # ── move ──────────────────────────────────────────────────────────────────
    sp_move = subparsers.add_parser("move",
                                     help="Move arm to a recorded position.")
    sp_move.add_argument("--name", required=True,
                         help="Position name (e.g. call-lift) or 'list' "
                              "to show available positions")
    sp_move.add_argument("--speed", type=int, default=None,
                         help="Speed %% override (0-100, default: from config)")
    sp_move.add_argument("--dry-run", action="store_true",
                         help="Log commands without executing")
    sp_move.set_defaults(func=cmd_move)

    # ── status ────────────────────────────────────────────────────────────────
    sp_status = subparsers.add_parser("status",
                                      help="Show arm and camera status.")
    sp_status.add_argument("--camera-ip", default="",
                           help="Camera IP (optional)")
    sp_status.set_defaults(func=cmd_status)

    args = parser.parse_args()
    setup_logging(args.log_level)
    args.func(args)


if __name__ == "__main__":
    main()
