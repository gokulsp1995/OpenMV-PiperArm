#!/usr/bin/env python3
"""Interactive Trajectory Recorder for Piper L Arm.

Allows manual positioning of the arm (Teach Mode).
Automatically creates and saves YAML output to a 'trajectory/' subfolder.
Allows custom naming or auto-numbering of output files.
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
except ImportError:
    print(
        "ERROR: pyAgxArm not installed.\n"
        "  Run: pip3 install \"git+https://github.com/agilexrobotics/pyAgxArm.git\""
    )
    sys.exit(1)


def get_next_default_filepath(output_dir: str) -> str:
    """Finds the next available auto-numbered file path (e.g. trajectory/trajectory_1.yaml)."""
    os.makedirs(output_dir, exist_ok=True)
    counter = 1
    while True:
        filename = f"trajectory_{counter}.yaml"
        filepath = os.path.join(output_dir, filename)
        if not os.path.exists(filepath):
            return filepath
        counter += 1


def record_trajectory(channel: str = "can0", interface: str = "socketcan", 
                      firmware_version: str = "default", output_dir: str = "trajectory"):
    print("=" * 60)
    print("  Piper L — Multi-Point Trajectory Recorder (pyAgxArm)")
    print("=" * 60)
    print()

    # Determine default trajectory output file
    default_filepath = get_next_default_filepath(output_dir)
    default_filename = os.path.basename(default_filepath)

    user_file = input(f"Enter file name [{default_filename}]: ").strip()
    if not user_file:
        target_filepath = default_filepath
    else:
        if not (user_file.endswith(".yaml") or user_file.endswith(".yml")):
            user_file += ".yaml"
        target_filepath = os.path.join(output_dir, user_file)

    print(f"\n[1/4] Target file path: {target_filepath}")

    # Map firmware string to PiperFW constant
    fw_map = {
        "default": PiperFW.DEFAULT,
        "v183": PiperFW.V183,
        "v188": PiperFW.V188,
        "v189": PiperFW.V189,
    }
    fw = fw_map.get(firmware_version, PiperFW.DEFAULT)

    # --- Connect to arm ---
    print(f"[2/4] Connecting to arm on '{channel}' (interface={interface})...")
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER_L,
        firmeware_version=fw,
        interface=interface,
        channel=channel,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    start_t = time.monotonic()
    while not robot.is_ok():
        if time.monotonic() - start_t > 10.0:
            print("ERROR: Arm communication timeout. Is CAN bus active?")
            sys.exit(1)
        time.sleep(0.1)
    print("  Connected.")

    # --- Disable motors for manual positioning ---
    print("[3/4] Disabling motors for manual teaching mode...")
    start_t = time.monotonic()
    while not robot.disable():
        time.sleep(0.01)
        if time.monotonic() - start_t > 5.0:
            print("WARNING: Could not disable all joints in 5s. Continuing...")
            break
    time.sleep(0.5)

    print()
    print("  ╔══════════════════════════════════════════════════════════════════╗")
    print("  ║  Motors are now DISABLED. Manually move the arm by hand.        ║")
    print("  ║                                                                  ║")
    print("  ║  - Press ENTER to record current point using default auto-name.  ║")
    print("  ║  - Type a name + ENTER to explicitly label a point.              ║")
    print("  ║  - Type 'done' or 'q' to stop recording and save the file.       ║")
    print("  ╚══════════════════════════════════════════════════════════════════╝")
    print()

    positions_dict = {}
    trajectory_sequence = []
    point_counter = 1

    # --- Interactive Recording Loop ---
    try:
        while True:
            default_point_name = f"point_{point_counter}"
            user_input = input(f"  Position name [{default_point_name}] (or 'q' to stop): ").strip()

            if user_input.lower() in ["q", "quit", "done", "exit"]:
                break

            point_name = user_input if user_input else default_point_name

            time.sleep(0.1)  # Let feedback settle
            ja = robot.get_joint_angles()
            fp = robot.get_flange_pose()

            if ja is None or fp is None:
                print("  ERROR: Could not read joint angles or flange pose. Skipping point...")
                continue

            joint_angles_rad = list(ja.msg)
            flange_msg = list(fp.msg)

            # Standard units (radians and metres) for robot driver control
            joints_dict = {
                "joint_1": round(joint_angles_rad[0], 6),
                "joint_2": round(joint_angles_rad[1], 6),
                "joint_3": round(joint_angles_rad[2], 6),
                "joint_4": round(joint_angles_rad[3], 6),
                "joint_5": round(joint_angles_rad[4], 6),
                "joint_6": round(joint_angles_rad[5], 6),
            }

            pose_dict = {
                "x": round(flange_msg[0], 6),
                "y": round(flange_msg[1], 6),
                "z": round(flange_msg[2], 6),
                "roll": round(flange_msg[3], 6),
                "pitch": round(flange_msg[4], 6),
                "yaw": round(flange_msg[5], 6),
            }

            # Human readable units
            joints_deg = {
                "joint_1_deg": round(math.degrees(joint_angles_rad[0]), 2),
                "joint_2_deg": round(math.degrees(joint_angles_rad[1]), 2),
                "joint_3_deg": round(math.degrees(joint_angles_rad[2]), 2),
                "joint_4_deg": round(math.degrees(joint_angles_rad[4]), 2),
                "joint_5_deg": round(math.degrees(joint_angles_rad[4]), 2),
                "joint_6_deg": round(math.degrees(joint_angles_rad[5]), 2),
            }

            pose_readable = {
                "x_mm": round(flange_msg[0] * 1000.0, 2),
                "y_mm": round(flange_msg[1] * 1000.0, 2),
                "z_mm": round(flange_msg[2] * 1000.0, 2),
                "roll_deg": round(math.degrees(flange_msg[3]), 2),
                "pitch_deg": round(math.degrees(flange_msg[4]), 2),
                "yaw_deg": round(math.degrees(flange_msg[5]), 2),
            }

            # Store waypoint entry
            positions_dict[point_name] = {
                "joints": joints_dict,
                "end_pose": pose_dict,
                "readable": {
                    "joints_deg": joints_deg,
                    "end_pose_mm_deg": pose_readable,
                },
            }
            trajectory_sequence.append(point_name)

            print(f"  ✓ Saved '{point_name}': J1..J6 (rad)={[joints_dict['joint_1'], joints_dict['joint_2'], joints_dict['joint_3'], joints_dict['joint_4'], joints_dict['joint_5'], joints_dict['joint_6']]}")
            point_counter += 1

    except KeyboardInterrupt:
        print("\n  Recording stopped by user.")

    # --- Save Trajectory File ---
    if trajectory_sequence:
        data = {
            "title": os.path.splitext(os.path.basename(target_filepath))[0],
            "timestamp": datetime.now().isoformat(),
            "sdk": "pyAgxArm",
            "settings": {
                "default_speed": 30,
                "settle_time": 0.5,
                "move_timeout": 10.0,
            },
            "positions": positions_dict,
            "trajectory": trajectory_sequence,
        }

        os.makedirs(os.path.dirname(target_filepath), exist_ok=True)
        print(f"\n[4/4] Writing output to '{target_filepath}'...")
        with open(target_filepath, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        print("  Trajectory recorded successfully!")
    else:
        print("\n[4/4] No waypoints captured. File was not written.")

    robot.disconnect()


def main():
    parser = argparse.ArgumentParser(description="Record Piper L trajectory paths by manual teaching.")
    parser.add_argument("--channel", default="can0", help="CAN channel name (default: can0).")
    parser.add_argument("--interface", default="socketcan", help="CAN interface type (default: socketcan).")
    parser.add_argument("--firmware", default="default", help="Firmware version (default: default).")
    parser.add_argument("--output-dir", default="trajectory", help="Directory to save trajectory YAML files.")
    args = parser.parse_args()

    record_trajectory(
        channel=args.channel,
        interface=args.interface,
        firmware_version=args.firmware,
        output_dir=args.output_dir,
    )


if __name__ == "__main__":
    main()
