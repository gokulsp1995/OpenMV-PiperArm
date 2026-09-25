#!/usr/bin/env python3
"""Trajectory Player for Piper L Arm (pyAgxArm).

Compatible with YAML files recorded by trajectory_recorder.py.
Executes move_j by default for safe joint motion, and move_l if specified.
"""

import argparse
import logging
import math
import os
import sys
import time
from typing import Any, Dict, List

import yaml

try:
    from pyAgxArm import (
        AgxArmFactory,
        ArmModel,
        PiperFW,
        create_agx_arm_config,
    )
except ImportError:
    print(
        "ERROR: pyAgxArm not installed.\n"
        "  Run: pip3 install \"git+https://github.com/agilexrobotics/pyAgxArm.git\""
    )
    sys.exit(1)

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s"
)
logger = logging.getLogger("TrajectoryPlay")


def resolve_file_path(filename: str) -> str:
    """Resolves path whether passed as 'trajectory_6', 'trajectory_6.yaml', or 'trajectory/trajectory_6.yaml'."""
    if os.path.exists(filename):
        return filename

    if not filename.endswith((".yaml", ".yml")):
        filename += ".yaml"

    candidate = os.path.join("trajectory", os.path.basename(filename))
    if os.path.exists(candidate):
        return candidate

    return filename


class TrajectoryPlayer:
    def __init__(
        self,
        trajectory_file: str,
        speed_pct: int = None,
        can_channel: str = "can0",
        force_mode: str = "auto",
    ):
        self.trajectory_file = resolve_file_path(trajectory_file)
        self.can_channel = can_channel
        self.force_mode = force_mode.lower()  # 'auto', 'joint', or 'linear'
        self.robot = None

        self.settings: Dict[str, Any] = {}
        self.positions: Dict[str, Dict[str, Any]] = {}
        self.sequence: List[str] = []

        self._load_trajectory()

        # Command-line speed overrides YAML default speed if provided
        if speed_pct is not None:
            self.speed_pct = max(1, min(speed_pct, 100))
        else:
            self.speed_pct = int(self.settings.get("default_speed", 15))

    def _load_trajectory(self):
        """Parse YAML structure from trajectory_recorder.py."""
        if not os.path.exists(self.trajectory_file):
            raise FileNotFoundError(f"Trajectory file not found: {self.trajectory_file}")

        with open(self.trajectory_file, "r") as f:
            data = yaml.safe_load(f)

        self.settings = data.get("settings", {})
        self.sequence = data.get("trajectory", [])
        raw_positions = data.get("positions", {})

        for pos_name, pos_data in raw_positions.items():
            joints = pos_data.get("joints", {})
            joint_list = [
                float(joints.get("joint_1", 0.0)),
                float(joints.get("joint_2", 0.0)),
                float(joints.get("joint_3", 0.0)),
                float(joints.get("joint_4", 0.0)),
                float(joints.get("joint_5", 0.0)),
                float(joints.get("joint_6", 0.0)),
            ]

            end_pose = pos_data.get("end_pose", {})
            pose_list = [
                float(end_pose.get("x", 0.0)),
                float(end_pose.get("y", 0.0)),
                float(end_pose.get("z", 0.0)),
                float(end_pose.get("roll", 0.0)),
                float(end_pose.get("pitch", 0.0)),
                float(end_pose.get("yaw", 0.0)),
            ]

            # Determine motion mode
            recorded_mode = pos_data.get("motion_type", "joint")
            if self.force_mode != "auto":
                mode = self.force_mode
            else:
                mode = recorded_mode

            self.positions[pos_name] = {
                "joints": joint_list,
                "end_pose": pose_list,
                "motion_type": mode,
            }

        logger.info(
            "Loaded '%s': %d total points in execution sequence.",
            self.trajectory_file,
            len(self.sequence),
        )

    def connect(self):
        """Connect to CAN interface and enable arm motors."""
        logger.info("Connecting to Piper L on '%s'...", self.can_channel)
        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=PiperFW.DEFAULT,
            interface="socketcan",
            channel=self.can_channel,
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

        start_t = time.monotonic()
        while not self.robot.is_ok():
            if time.monotonic() - start_t > 10.0:
                raise RuntimeError("Timeout connecting to arm over CAN bus.")
            time.sleep(0.1)

        logger.info("Enabling arm motors...")
        start_t = time.monotonic()
        while not self.robot.enable():
            if time.monotonic() - start_t > 5.0:
                raise RuntimeError("Failed to enable motor drivers.")
            time.sleep(0.1)

        logger.info("Motors enabled and ready.")

    def disconnect(self):
        """Safely disconnect from the arm."""
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.robot = None
            logger.info("Arm interface closed safely.")

    def _get_max_joint_error_deg(self, target_joints: List[float]) -> float:
        """Calculates maximum error across all 6 joints in degrees."""
        ja = self.robot.get_joint_angles()
        if ja is None:
            return 999.0
        
        # Extract raw iterable array from pyAgxArm message
        current = list(ja.msg) if hasattr(ja, "msg") else list(ja)
        return max([abs(math.degrees(t - c)) for t, c in zip(target_joints, current)])

    def execute(self):
        if not self.sequence:
            logger.warning("No waypoints found in trajectory sequence.")
            return

        settle_time = float(self.settings.get("settle_time", 0.5))
        timeout = float(self.settings.get("move_timeout", 10.0))
        total_points = len(self.sequence)

        print("\n" + "=" * 70)
        print("  SAFETY CHECK — READY TO EXECUTE TRAJECTORY")
        print(f"  File: {self.trajectory_file}")
        print(f"  Waypoints: {total_points}")
        print(f"  Execution Speed: {self.speed_pct}%")
        print("  Keep your hand near the E-STOP / Power switch!")
        print("=" * 70)

        confirm = input("\nStart execution? [y/N]: ").strip().lower()
        if confirm != "y":
            logger.info("Execution cancelled by operator.")
            return

        print("\n" + "-" * 70)
        print("  STARTING MOTION PLAYBACK")
        print("-" * 70)

        for idx, pos_name in enumerate(self.sequence, start=1):
            if pos_name not in self.positions:
                logger.error("Point '%s' missing from position definitions.", pos_name)
                break

            pt = self.positions[pos_name]
            target_joints = pt["joints"]
            target_pose = pt["end_pose"]
            motion_mode = pt["motion_type"]

            joints_deg = [round(math.degrees(rad), 1) for rad in target_joints]
            print(f"\n▶ Waypoint [{idx}/{total_points}]: '{pos_name}' | Mode: {motion_mode.upper()}")
            print(f"  Target Joints (deg): {joints_deg}")

            self.robot.set_speed_percent(self.speed_pct)

            # Execution route based on motion mode
            if motion_mode == "linear":
                self.robot.move_l(target_pose)
            else:
                self.robot.move_j(target_joints)

            start_t = time.monotonic()
            time.sleep(0.1)

            # Monitoring loop
            while True:
                elapsed = time.monotonic() - start_t
                max_err_deg = self._get_max_joint_error_deg(target_joints)

                status_code = "UNKNOWN"
                try:
                    status = self.robot.get_arm_status()
                    if status is not None and hasattr(status, "msg"):
                        status_code = str(getattr(status.msg, "motion_status", "UNKNOWN"))
                except Exception:
                    pass

                print(
                    f"\r  Status: MOVING | Elapsed: {elapsed:.1f}s | Max Joint Err: {max_err_deg:.2f}° | Code: {status_code}   ",
                    end="",
                    flush=True,
                )

                # Check if waypoint reached (< 2.0 degrees max error)
                if max_err_deg < 2.0:
                    print(f"\r  Status: REACHED | Elapsed: {elapsed:.1f}s | Max Joint Err: {max_err_deg:.2f}°           ", flush=True)
                    break

                if elapsed > timeout:
                    print(f"\n  WARNING: Timeout of {timeout:.1f}s reached waiting for point '{pos_name}'.")
                    break

                time.sleep(0.05)

            if settle_time > 0:
                print(f"  Settling for {settle_time}s...", end="", flush=True)
                time.sleep(settle_time)
                print(" Done.")

        print("\n" + "=" * 70)
        logger.info("Trajectory execution completed successfully.")
        print("=" * 70 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Piper L Trajectory Player")
    parser.add_argument("--file", "-f", required=True, help="YAML trajectory file name or path")
    parser.add_argument("--speed", "-s", type=int, default=None, help="Speed percentage override (1-100)")
    parser.add_argument("--can", default="can0", help="CAN channel name (default: can0)")
    parser.add_argument(
        "--mode",
        choices=["auto", "joint", "linear"],
        default="auto",
        help="Force execution mode: 'joint' (safest), 'linear', or 'auto' (default)",
    )
    args = parser.parse_args()

    player = TrajectoryPlayer(
        trajectory_file=args.file,
        speed_pct=args.speed,
        can_channel=args.can,
        force_mode=args.mode,
    )

    try:
        player.connect()
        player.execute()
    except KeyboardInterrupt:
        print("\n\n!!! MOTION INTERRUPTED BY OPERATOR (Ctrl+C) !!!")
    finally:
        player.disconnect()


if __name__ == "__main__":
    main()

