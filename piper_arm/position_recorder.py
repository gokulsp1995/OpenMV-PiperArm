#!/usr/bin/env python3
"""Position recorder utility for the Piper L arm.

Allows the operator to manually move the arm to a desired position (with motors
disabled / in teaching mode), then record the current joint angles and end-
effector pose to a YAML file.

Uses the pyAgxArm library (https://github.com/agilexrobotics/pyAgxArm).

Usage:
    python position_recorder.py --name call-lift
    python position_recorder.py --name floor-select-panel1
    python position_recorder.py --name floor-select-panel2 --channel can0
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import yaml

try:
    from pyAgxArm import (
        create_agx_arm_config,
        AgxArmFactory,
        ArmModel,
        PiperFW,
    )
except ImportError as e:
    import traceback
    traceback.print_exc()
    print(
        "\nERROR: pyAgxArm not installed or import failed.\n"
        "  Detail: %s\n"
        "  Python: %s\n"
        "  Run: python3 -m pip install \"git+https://github.com/agilexrobotics/pyAgxArm.git\""
        % (e, sys.executable)
    )
    sys.exit(1)


# ── Main ──────────────────────────────────────────────────────────────────────

def record_position(channel: str, position_name: str, output_dir: str,
                    description: str = "", interface: str = "socketcan",
                    firmware_version: str = "default"):
    """Record the arm's current position and save to YAML.

    Steps:
        1. Connect to arm
        2. Disable motors for manual positioning (teaching mode)
        3. Wait for user to move arm and press Enter
        4. Read joint angles + flange pose
        5. Save to YAML
    """
    print("=" * 60)
    print("  Piper L — Position Recorder (pyAgxArm)")
    print("=" * 60)
    print()

    # Map firmware string to PiperFW constant
    fw_map = {
        "default": PiperFW.DEFAULT,
        "v183": PiperFW.V183,
        "v188": PiperFW.V188,
        "v189": PiperFW.V189,
    }
    fw = fw_map.get(firmware_version, PiperFW.DEFAULT)

    # --- Connect to arm ---
    print("[1/5] Connecting to arm on '%s' (interface=%s)..." %
          (channel, interface))

    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER_L,
        firmeware_version=fw,
        interface=interface,
        channel=channel,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    # Wait for communication
    start_t = time.monotonic()
    while not robot.is_ok():
        if time.monotonic() - start_t > 10.0:
            print("ERROR: Arm communication timeout. Is CAN bus active?")
            sys.exit(1)
        time.sleep(0.1)
    print("  Connected.")

    # --- Disable motors for manual positioning ---
    print("[2/5] Disabling motors for manual positioning...")
    start_t = time.monotonic()
    while not robot.disable():
        time.sleep(0.01)
        if time.monotonic() - start_t > 5.0:
            print("WARNING: Could not disable all joints in 5s. Continuing...")
            break
    time.sleep(0.5)

    print()
    print("  ╔══════════════════════════════════════════════════╗")
    print("  ║  Motors are now DISABLED.                        ║")
    print("  ║  Manually move the arm to the desired position.  ║")
    print("  ║                                                  ║")
    print("  ║  Press ENTER when the arm is in position...      ║")
    print("  ╚══════════════════════════════════════════════════╝")
    print()
    input("  >>> ")

    # --- Read current position ---
    print("[3/5] Reading joint angles and flange pose...")
    time.sleep(0.2)  # let feedback settle

    # Joint angles (radians)
    ja = robot.get_joint_angles()
    if ja is None:
        print("ERROR: Could not read joint angles.")
        sys.exit(1)

    joint_angles_rad = list(ja.msg)  # [j1..j6] in radians
    joints_dict = {
        "joint_1": round(joint_angles_rad[0], 6),
        "joint_2": round(joint_angles_rad[1], 6),
        "joint_3": round(joint_angles_rad[2], 6),
        "joint_4": round(joint_angles_rad[3], 6),
        "joint_5": round(joint_angles_rad[4], 6),
        "joint_6": round(joint_angles_rad[5], 6),
    }

    # Flange pose (metres + radians)
    fp = robot.get_flange_pose()
    if fp is None:
        print("ERROR: Could not read flange pose.")
        sys.exit(1)

    flange_msg = list(fp.msg)  # [x, y, z, roll, pitch, yaw] m/rad
    pose_dict = {
        "x": round(flange_msg[0], 6),   # metres
        "y": round(flange_msg[1], 6),
        "z": round(flange_msg[2], 6),
        "roll": round(flange_msg[3], 6),   # radians
        "pitch": round(flange_msg[4], 6),
        "yaw": round(flange_msg[5], 6),
    }

    # Also store human-readable values
    pose_readable = {
        "x_mm": round(flange_msg[0] * 1000.0, 2),
        "y_mm": round(flange_msg[1] * 1000.0, 2),
        "z_mm": round(flange_msg[2] * 1000.0, 2),
        "roll_deg": round(math.degrees(flange_msg[3]), 2),
        "pitch_deg": round(math.degrees(flange_msg[4]), 2),
        "yaw_deg": round(math.degrees(flange_msg[5]), 2),
    }

    joints_deg = {
        "joint_1_deg": round(math.degrees(joint_angles_rad[0]), 2),
        "joint_2_deg": round(math.degrees(joint_angles_rad[1]), 2),
        "joint_3_deg": round(math.degrees(joint_angles_rad[2]), 2),
        "joint_4_deg": round(math.degrees(joint_angles_rad[3]), 2),
        "joint_5_deg": round(math.degrees(joint_angles_rad[4]), 2),
        "joint_6_deg": round(math.degrees(joint_angles_rad[5]), 2),
    }

    # --- Build YAML document ---
    data = {
        "name": position_name,
        "description": description or "Recorded position: %s" % position_name,
        "timestamp": datetime.now().isoformat(),
        "sdk": "pyAgxArm",
        "joints": joints_dict,         # radians (used by move_j)
        "end_pose": pose_dict,          # metres + radians
        "gripper": 0.0,                 # default: gripper closed
        "readable": {
            "joints_deg": joints_deg,
            "end_pose_mm_deg": pose_readable,
        },
    }

    # --- Save to file ---
    os.makedirs(output_dir, exist_ok=True)
    filepath = os.path.join(output_dir, "%s.yaml" % position_name)

    print("[4/5] Saving to: %s" % filepath)
    with open(filepath, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print("[5/5] Done!  Position '%s' saved." % position_name)
    print()
    print("  Joints (rad):", joints_dict)
    print("  Flange pose (m/rad):", pose_dict)
    print()

    # --- Live preview (optional) ---
    preview = input("Show live position feedback? [y/N] ").strip().lower()
    if preview == "y":
        print("  Streaming position... press Ctrl+C to stop.")
        try:
            while True:
                ja = robot.get_joint_angles()
                fp = robot.get_flange_pose()
                if ja is not None and fp is not None:
                    j = ja.msg
                    p = fp.msg
                    print(
                        "  J: %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f  |  "
                        "P: X=%+.4f Y=%+.4f Z=%+.4f R=%+.3f P=%+.3f Y=%+.3f"
                        % (
                            j[0], j[1], j[2], j[3], j[4], j[5],
                            p[0], p[1], p[2], p[3], p[4], p[5],
                        ),
                        end="\r",
                    )
                time.sleep(0.05)
        except KeyboardInterrupt:
            print()

    # Clean up
    robot.disconnect()

    return filepath


# ── CLI ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Record Piper L arm positions by manual teaching."
    )
    parser.add_argument(
        "--name", required=True,
        help="Position name (e.g. 'call-lift', 'floor-select-panel1')."
    )
    parser.add_argument(
        "--channel", default="can0",
        help="CAN channel name (default: can0)."
    )
    parser.add_argument(
        "--interface", default="socketcan",
        help="CAN interface type (default: socketcan)."
    )
    parser.add_argument(
        "--firmware", default="default",
        help="Firmware version: default, v183, v188, v189 (default: default)."
    )
    parser.add_argument(
        "--output-dir", default="positions",
        help="Directory to save YAML files (default: positions/)."
    )
    parser.add_argument(
        "--description", default="",
        help="Optional description for the recorded position."
    )
    args = parser.parse_args()

    record_position(
        channel=args.channel,
        position_name=args.name,
        output_dir=args.output_dir,
        description=args.description,
        interface=args.interface,
        firmware_version=args.firmware,
    )


if __name__ == "__main__":
    main()
