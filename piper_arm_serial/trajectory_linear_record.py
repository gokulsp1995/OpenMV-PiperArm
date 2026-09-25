#!/usr/bin/env python3
"""Record trajectory waypoints with Joint Angles and End-Effector (end_pose) data.

Saves YAML files compatible with play_linear.py for Piper L (pyAgxArm).
"""

import argparse
import datetime
import math
import os
import sys
import time
import yaml

try:
    from pyAgxArm import (
        create_agx_arm_config,
        AgxArmFactory,
        ArmModel,
        PiperFW,
    )
except ImportError:
    print("\nERROR: pyAgxArm not installed.")
    sys.exit(1)


def get_next_auto_filename(directory: str = "trajectory") -> str:
    """Finds the next available auto-numbered file name: trajectory_1.yaml, trajectory_2.yaml, etc."""
    os.makedirs(directory, exist_ok=True)
    idx = 1
    while True:
        candidate = os.path.join(directory, f"linear_trajectory_{idx}.yaml")
        if not os.path.exists(candidate):
            return candidate
        idx += 1


def format_output_path(filename_input: str, directory: str = "trajectory") -> str:
    """Formats the filename and ensures it resides in the target directory."""
    if not filename_input:
        return get_next_auto_filename(directory)

    # Add .yaml extension if missing
    if not filename_input.endswith(".yaml") and not filename_input.endswith(".yml"):
        filename_input += ".yaml"

    # If user provided a raw filename without folder, put it in directory
    if not os.path.dirname(filename_input):
        output_path = os.path.join(directory, filename_input)
    else:
        output_path = filename_input

    return output_path


def record_trajectory(output_file: str, can_channel: str = "can0"):
    output_file = format_output_path(output_file)

    print("Connecting to Piper L on channel '%s'..." % can_channel)
    cfg = create_agx_arm_config(
        robot=ArmModel.PIPER_L,
        firmeware_version=PiperFW.DEFAULT,
        interface="socketcan",
        channel=can_channel,
        bitrate=1000000,
    )
    robot = AgxArmFactory.create_arm(cfg)
    robot.connect()

    start_t = time.monotonic()
    while not robot.is_ok():
        if time.monotonic() - start_t > 10.0:
            raise RuntimeError("Timeout connecting to arm.")
        time.sleep(0.1)

    print("Enabling Drag-Teach Mode (Passive arm movement)...")
    try:
        robot.enable_drag_teach_mode()
    except Exception as e:
        print("Note on enabling teach mode:", e)

    positions = {}
    sequence = []
    pt_count = 1

    print("\n" + "=" * 65)
    print("  RECORDING MODE: Move arm by hand to desired location.")
    print("  Press ENTER to record a point | Type 'q' and ENTER to save & exit")
    print("=" * 65 + "\n")

    try:
        while True:
            cmd = input(f"Press ENTER to record Waypoint {pt_count} (or 'q' to finish): ").strip().lower()
            if cmd == 'q':
                break

            # Read raw joint positions
            ja_msg = robot.get_joint_angles()
            if ja_msg is None or not hasattr(ja_msg, "msg"):
                print("Failed to read joint positions. Try again.")
                continue
            joints_rad = list(ja_msg.msg)

            # Read end-effector pose [X, Y, Z, Roll, Pitch, Yaw]
            ee_pose = robot.get_end_effector_pose()
            if ee_pose is None:
                print("Failed to read EE pose. Try again.")
                continue

            pt_name = f"point_{pt_count}"
            
            # Select motion type for playback
            m_type = input("  Motion type for this point? [j = Joint (default), l = Linear straight]: ").strip().lower()
            motion_type = "linear" if m_type == 'l' else "joint"

            positions[pt_name] = {
                "motion_type": motion_type,
                "joints": {
                    f"joint_{i+1}": float(ang) for i, ang in enumerate(joints_rad)
                },
                "end_pose": {
                    "x": float(ee_pose[0]),
                    "y": float(ee_pose[1]),
                    "z": float(ee_pose[2]),
                    "roll": float(ee_pose[3]),
                    "pitch": float(ee_pose[4]),
                    "yaw": float(ee_pose[5]),
                },
                "readable": {
                    "joints_deg": {f"joint_{i+1}_deg": round(math.degrees(ang), 2) for i, ang in enumerate(joints_rad)},
                    "end_pose_mm_deg": {
                        "x_mm": round(ee_pose[0] * 1000.0, 2),
                        "y_mm": round(ee_pose[1] * 1000.0, 2),
                        "z_mm": round(ee_pose[2] * 1000.0, 2),
                        "roll_deg": round(math.degrees(ee_pose[3]), 2),
                        "pitch_deg": round(math.degrees(ee_pose[4]), 2),
                        "yaw_deg": round(math.degrees(ee_pose[5]), 2),
                    }
                }
            }
            sequence.append(pt_name)
            print(f"  Saved '{pt_name}' as [{motion_type.upper()}] | Pose Z: {ee_pose[2]:.3f}m\n")
            pt_count += 1

    finally:
        print("Disabling drag-teach mode...")
        try:
            robot.disable_drag_teach_mode()
        except Exception:
            pass
        robot.disconnect()

    if not sequence:
        print("No points recorded. File not created.")
        return

    output_data = {
        "title": os.path.splitext(os.path.basename(output_file))[0],
        "timestamp": datetime.datetime.now().isoformat(),
        "sdk": "pyAgxArm",
        "settings": {
            "default_speed": 15,
            "settle_time": 0.5,
            "move_timeout": 10.0,
        },
        "positions": positions,
        "trajectory": sequence,
    }

    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    with open(output_file, "w") as f:
        yaml.dump(output_data, f, default_flow_style=False, sort_keys=False)

    print(f"\nSuccessfully written {len(sequence)} points to: {output_file}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record EE Trajectory for Piper L")
    parser.add_argument("--file", "-f", default=None, help="Output YAML filename or path (e.g. trajectory_1 or button_press.yaml)")
    parser.add_argument("--can", default="can0", help="CAN channel name")
    args = parser.parse_args()

    record_trajectory(args.file, args.can)
