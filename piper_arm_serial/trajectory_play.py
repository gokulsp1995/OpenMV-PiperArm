#!/usr/bin/env python3
"""Safe Trajectory Player for Piper L Arm (pyAgxArm).

Executes a YAML trajectory sequence with real-time terminal feedback 
showing waypoint progress, joint angle targets, and motion state.
"""

import argparse
import math
import logging
import os
import sys
import time
from typing import Dict, List

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
        "\nERROR: pyAgxArm not installed.\n"
        "  Run: pip3 install \"git+https://github.com/agilexrobotics/pyAgxArm.git\""
    )
    sys.exit(1)

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("TrajectoryPlayer")


class SafeTrajectoryPlayer:
    def __init__(self, trajectory_file: str, speed_pct: int = 15, can_channel: str = "can0", dry_run: bool = False):
        self.trajectory_file = trajectory_file
        self.speed_pct = max(1, min(speed_pct, 100))
        self.can_channel = can_channel
        self.dry_run = dry_run
        self.robot = None

        self.settings = {}
        self.positions: Dict[str, List[float]] = {}
        self.sequence: List[str] = []

        self._load_trajectory()

    def _load_trajectory(self):
        """Parse and validate the trajectory YAML configuration file."""
        if not os.path.exists(self.trajectory_file):
            raise FileNotFoundError(f"Trajectory file missing: {self.trajectory_file}")

        with open(self.trajectory_file, "r") as f:
            data = yaml.safe_load(f)

        self.settings = data.get("settings", {})
        self.sequence = data.get("trajectory", [])
        raw_positions = data.get("positions", {})

        for pos_name, pos_data in raw_positions.items():
            joints = pos_data.get("joints", {})
            self.positions[pos_name] = [
                float(joints.get("joint_1", 0.0)),
                float(joints.get("joint_2", 0.0)),
                float(joints.get("joint_3", 0.0)),
                float(joints.get("joint_4", 0.0)),
                float(joints.get("joint_5", 0.0)),
                float(joints.get("joint_6", 0.0)),
            ]

        logger.info("Loaded '%s': %d total points in execution sequence.", 
                    self.trajectory_file, len(self.sequence))

    def connect(self):
        """Establish CAN bus connection and enable motor drives."""
        if self.dry_run:
            logger.info("[DRY-RUN] Simulating connection on channel '%s'.", self.can_channel)
            return

        logger.info("Connecting to Piper L on '%s'...", self.can_channel)
        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=PiperFW.DEFAULT,
            interface="socketcan",
            channel=self.can_channel,
            bitrate=1000000,
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
        """Safely disconnect from arm."""
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.robot = None
            logger.info("Arm interface closed safely.")

    def _get_max_joint_error(self, target: List[float]) -> float:
        """Calculate the maximum absolute angular error across all 6 joints in degrees."""
        if self.robot is None:
            return 0.0
        ja = self.robot.get_joint_angles()
        if ja is None or not hasattr(ja, "msg"):
            return 999.0
        current = list(ja.msg)
        errors_deg = [abs(math.degrees(t - c)) for t, c in zip(target, current)]
        return max(errors_deg)

    def execute(self):
        """Run the loaded trajectory sequence safely with terminal status updates."""
        if not self.sequence:
            logger.warning("No waypoints found in trajectory file.")
            return

        settle_time = float(self.settings.get("settle_time", 0.5))
        timeout = float(self.settings.get("move_timeout", 12.0))

        print("\n" + "=" * 70)
        print("  SAFETY CHECK — READY TO EXECUTE TRAJECTORY")
        print(f"  File: {self.trajectory_file}")
        print(f"  Waypoints: {len(self.sequence)}")
        print(f"  Override Speed: {self.speed_pct}%")
        print("  Keep your hand near the E-STOP / Power switch!")
        print("=" * 70)

        if not self.dry_run:
            confirm = input("\nStart execution? [y/N]: ").strip().lower()
            if confirm != "y":
                logger.info("Execution cancelled by operator.")
                return

        print("\n" + "-" * 70)
        print("  STARTING MOTION PLAYBACK")
        print("-" * 70)

        total_points = len(self.sequence)

        for idx, pos_name in enumerate(self.sequence, start=1):
            if pos_name not in self.positions:
                logger.error("Point '%s' missing from position definitions. Aborting.", pos_name)
                break

            target_rad = self.positions[pos_name]
            target_deg = [round(math.degrees(a), 1) for a in target_rad]
            
            # Print waypoint header summary
            print(f"\n▶ Waypoint [{idx}/{total_points}]: '{pos_name}'")
            print(f"  Target Joints (deg): {target_deg}")

            if self.dry_run:
                print("  Status: [DRY-RUN SIMULATION] -> Reached")
                time.sleep(0.5)
                continue

            # Issue move command
            self.robot.set_speed_percent(self.speed_pct)
            self.robot.move_j(target_rad)

            start_t = time.monotonic()
            time.sleep(0.1)  # Brief delay to allow trajectory command to initiate

            # Live terminal update loop during movement
            while True:
                elapsed = time.monotonic() - start_t
                max_err_deg = self._get_max_joint_error(target_rad)

                # Read motion status code
                status_code = -1
                try:
                    status = self.robot.get_arm_status()
                    if status is not None and hasattr(status, "msg"):
                        status_code = getattr(status.msg, "motion_status", -1)
                except Exception:
                    pass

                # Live status string updated on single line
                print(
                    f"\r  Status: MOVING | Elapsed: {elapsed:.1f}s | Max Joint Err: {max_err_deg:.2f}° | Code: {status_code}",
                    end="",
                    flush=True
                )

                # Exit loop when motion reports complete (motion_status == 0 and joint error < 1.0 deg)
                if status_code == 0 and max_err_deg < 1.0:
                    print(
                        f"\r  Status: ARRIVED | Elapsed: {elapsed:.1f}s | Max Joint Err: {max_err_deg:.2f}° | Code: 0   ",
                        flush=True
                    )
                    break

                if elapsed > timeout:
                    print(f"\n  WARNING: Timeout of {timeout}s reached waiting for point '{pos_name}'.")
                    break

                time.sleep(0.05)

            # Settle time feedback
            if settle_time > 0:
                print(f"  Settling for {settle_time}s...", end="", flush=True)
                time.sleep(settle_time)
                print(" Done.")

        print("\n" + "=" * 70)
        logger.info("Trajectory execution completed successfully.")
        print("=" * 70 + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Piper L Safe Trajectory Player")
    parser.add_argument("--file", "-f", required=True, help="Path to YAML trajectory file")
    parser.add_argument("--speed", "-s", type=int, default=15, help="Speed percentage 1-100 (Default: 15%% for testing)")
    parser.add_argument("--can", default="can0", help="CAN channel name (default: can0)")
    parser.add_argument("--dry-run", action="store_true", help="Simulate without physically driving arm")

    args = parser.parse_args()

    player = SafeTrajectoryPlayer(
        trajectory_file=args.file,
        speed_pct=args.speed,
        can_channel=args.can,
        dry_run=args.dry_run,
    )

    try:
        player.connect()
        player.execute()
    except KeyboardInterrupt:
        print("\n\n!!! MOTION INTERRUPTED BY OPERATOR (Ctrl+C) !!!")
    finally:
        player.disconnect()
