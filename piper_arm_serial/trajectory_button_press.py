#!/usr/bin/env python3
"""Safe Trajectory Player for Piper L Arm (pyAgxArm).

Executes a YAML trajectory sequence with real-time terminal feedback
showing waypoint progress, joint angle targets, and motion state.

NEW: a waypoint may be an action dict instead of a point name:

    trajectory:
    - point_1
    - point_2
    - {action: press, target: up}
    - {action: pause, seconds: 1.0}
    - point_1

'press' detects the button from wherever the arm currently is (so the
waypoint before it must be the look pose), computes its position via the
camera_transform in config.yaml, and drives approach -> press -> retract
-> clear, all as straight-line (move_l) motion near the panel.

camera_transform, tool_offset, and camera connection settings are read
from config.yaml rather than duplicated into every trajectory file --
they describe the hardware, not this particular sequence.
"""

import argparse
import math
import logging
import os
import statistics
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

from camera_client import CameraClient
from coordinate_transform import CoordinateTransform

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("TrajectoryPlayer")


class SafeTrajectoryPlayer:
    def __init__(self, trajectory_file: str, speed_pct: int = 15, can_channel: str = "can0",
                 dry_run: bool = False, config_path: str = "config.yaml",
                 home_point: str = None):
        self.trajectory_file = trajectory_file
        self.speed_pct = max(1, min(speed_pct, 100))
        self.can_channel = can_channel
        self.dry_run = dry_run
        self.robot = None
        self.home_point = home_point   # override for _go_home_on_failure;
                                        # None -> first point name in sequence

        self.settings = {}
        self.positions: Dict[str, List[float]] = {}
        self.sequence: List = []   # strings (point names) or action dicts

        # Camera / press support -- populated by _load_config()
        self.camera = None
        self.transform = None
        self.cam_cfg = {}
        self.motion_cfg = {}
        self.tool_z = 0.145

        self._load_config(config_path)
        self._load_trajectory()

    # -- Loading ------------------------------------------------------------------

    def _load_config(self, config_path: str):
        """Load camera_transform, tool_offset, and camera settings.

        These are hardware calibration constants shared across every
        trajectory, so they live in one place rather than being copied
        into each trajectory_N.yaml.
        """
        if not os.path.exists(config_path):
            logger.warning(
                "No %s found. Point-only trajectories still work; any "
                "'press' action will fail without camera_transform.",
                config_path)
            return

        with open(config_path) as f:
            cfg = yaml.safe_load(f) or {}

        self.cam_cfg = cfg.get("camera", {})
        self.motion_cfg = cfg.get("motion", {})
        self.tool_z = cfg.get("tool_offset", {}).get("z", 0.145)

        ct = cfg.get("camera_transform", {})
        if not ct.get("calibrated"):
            logger.warning(
                "camera_transform.calibrated is not true in %s -- any "
                "'press' action will use PLACEHOLDER values and compute "
                "the wrong position.", config_path)

        self.transform = CoordinateTransform(
            cam_tx=ct.get("tx", 0.0), cam_ty=ct.get("ty", 0.0),
            cam_tz=ct.get("tz", 0.05),
            cam_rx=ct.get("rx", 0.0), cam_ry=ct.get("ry", 0.0),
            cam_rz=ct.get("rz", 0.0),
        )

    def _load_trajectory(self):
        """Parse and validate the trajectory YAML configuration file."""
        if not os.path.exists(self.trajectory_file):
            raise FileNotFoundError(f"Trajectory file missing: {self.trajectory_file}")

        with open(self.trajectory_file, "r") as f:
            data = yaml.safe_load(f)

        self.settings = data.get("settings", {})
        self.sequence = data.get("trajectory", [])   # dicts pass through as-is
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

    def _needs_camera(self) -> bool:
        return any(isinstance(s, dict) and s.get("action") == "press"
                   for s in self.sequence)

    # -- Connection ----------------------------------------------------------------

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

    def _connect_camera(self):
        """Open the serial link to the camera. Only called if a 'press'
        action is actually in the sequence."""
        if self.camera is not None or self.dry_run:
            return

        port_hint = self.cam_cfg.get("port_hint", "OpenMV")
        baudrate = self.cam_cfg.get("baudrate", 115200)
        timeout = self.cam_cfg.get("timeout", 5.0)

        logger.info("Connecting to camera (port_hint=%s)...", port_hint)
        self.camera = CameraClient(port_hint, baudrate, timeout)
        if not self.camera.connect():
            raise RuntimeError(
                "Could not open camera port (port_hint=%s). Is viewer.py "
                "or picocom holding it?" % port_hint)

        st = self.camera.status()
        if st is None:
            raise RuntimeError(
                "Camera port opened but did not answer STATUS -- main.py "
                "is probably not running on it. Reset the camera and retry.")
        logger.info("Camera ready (camera=%s tof=%s)",
                    st.get("camera"), st.get("tof"))

    def disconnect(self):
        """Safely disconnect from arm and camera."""
        if self.camera is not None:
            self.camera.disconnect()
            self.camera = None
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.robot = None
            logger.info("Arm interface closed safely.")

    # -- Motion helpers --------------------------------------------------------------

    def _get_max_joint_error(self, target: List[float]) -> float:
        """Max absolute angular error across all 6 joints, in degrees."""
        if self.robot is None:
            return 0.0
        ja = self.robot.get_joint_angles()
        if ja is None or not hasattr(ja, "msg"):
            return 999.0
        current = list(ja.msg)
        errors_deg = [abs(math.degrees(t - c)) for t, c in zip(target, current)]
        return max(errors_deg)

    def _get_current_pose(self):
        """Flange pose (x, y, z, roll, pitch, yaw), metres and radians."""
        if self.dry_run or self.robot is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        fp = self.robot.get_flange_pose()
        if fp is None:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        return tuple(fp.msg)

    def _wait_for_motion(self, timeout: float = None):
        """Block until motion_status reports complete."""
        if self.dry_run:
            return
        if timeout is None:
            timeout = float(self.settings.get("move_timeout", 10.0))
        settle = float(self.settings.get("settle_time", 0.5))

        time.sleep(0.2)
        start = time.monotonic()
        while True:
            try:
                status = self.robot.get_arm_status()
                if status is not None and getattr(status.msg, "motion_status", None) == 0:
                    break
            except Exception:
                pass
            if time.monotonic() - start > timeout:
                logger.warning("Motion wait timed out after %.1fs.", timeout)
                break
            time.sleep(0.05)
        time.sleep(settle)

    # -- Press action -----------------------------------------------------------------

    def _sample_button(self, target_id: str, panel=None):
        """Scan several times, return the median detection.

        A single scan is usually stable to a few mm but occasionally jumps
        50mm+ when a ToF zone catches the background instead of the button.
        The median ignores that; a mean would be dragged by it.
        Returns None if too few scans found the button, or if the samples
        disagree by more than scan_max_spread_mm.
        """
        samples = self.motion_cfg.get("scan_samples", 5)
        min_hits = self.motion_cfg.get("scan_min_hits", 3)
        max_spread = self.motion_cfg.get("scan_max_spread_mm", 25)

        hits = []
        for _ in range(samples):
            result = self.camera.scan(panel=panel)
            if result is None or result.get("type") != "detections":
                continue
            for btn in result.get("buttons", []):
                if btn.get("id") == target_id and btn.get("distance_mm", 0) > 0:
                    hits.append(btn)
                    break
            time.sleep(0.05)

        if len(hits) < min_hits:
            logger.warning("Only %d/%d valid samples for '%s' (need %d).",
                           len(hits), samples, target_id, min_hits)
            return None

        dists = [h["distance_mm"] for h in hits]
        spread = max(dists) - min(dists)
        if spread > max_spread:
            logger.warning("Distance spread %.0fmm exceeds %.0fmm -- "
                           "readings disagree, not pressing.",
                           spread, max_spread)
            return None

        med = {
            "distance_mm": statistics.median(dists),
            "cam_x": statistics.median([h["cam_x"] for h in hits]),
            "cam_y": statistics.median([h["cam_y"] for h in hits]),
            "cam_z": statistics.median([h["cam_z"] for h in hits]),
        }
        logger.info("Button '%s': %d/%d samples, d=%.0fmm (spread %.0fmm), "
                    "cam=(%.4f, %.4f, %.4f)",
                    target_id, len(hits), samples, med["distance_mm"], spread,
                    med["cam_x"], med["cam_y"], med["cam_z"])
        return med

    def _press(self, target_id: str, panel=None) -> bool:
        """Detect and press from the arm's CURRENT pose.

        Assumes the previous waypoint already put the arm at the look pose.
        Motion: approach (move_p) -> press (move_l) -> retract (move_l) ->
        clear (move_l). The final clear exists because the next waypoint is
        likely a joint-space move_j, whose path through real space is not a
        straight line and could sweep the tool across the panel.
        """
        logger.info("--- press action: target='%s' ---", target_id)

        if self.dry_run:
            logger.info("[DRY-RUN] Would detect and press '%s'.", target_id)
            time.sleep(0.5)
            return True

        if self.transform is None:
            logger.error("No camera_transform loaded -- cannot press. "
                        "Check config.yaml.")
            return False

        self._connect_camera()

        m = self.motion_cfg
        approach_offset = m.get("approach_offset_mm", 50.0) / 1000.0
        press_depth = m.get("press_depth_mm", 5.0) / 1000.0
        clearance = m.get("clearance_mm", 150.0) / 1000.0
        approach_speed = m.get("approach_speed", self.speed_pct)
        press_speed = m.get("press_speed", 20)
        retract_speed = m.get("retract_speed", 40)
        max_retries = m.get("max_retries", 3)

        def to_rad(p):
            return (p[0], p[1], p[2],
                    math.radians(p[3]), math.radians(p[4]), math.radians(p[5]))

        for attempt in range(1, max_retries + 1):
            logger.info("Press attempt %d/%d", attempt, max_retries)

            btn = self._sample_button(target_id, panel=panel)
            if btn is None:
                time.sleep(0.5)
                continue

            ee = self._get_current_pose()
            ee_rx = math.degrees(ee[3])
            ee_ry = math.degrees(ee[4])
            ee_rz = math.degrees(ee[5])

            button_base = self.transform.camera_to_base(
                (btn["cam_x"], btn["cam_y"], btn["cam_z"]),
                ee[0], ee[1], ee[2], ee_rx, ee_ry, ee_rz,
            )
            logger.info("Button in base frame: (%.4f, %.4f, %.4f) m", *button_base)

            pre, press, safe = self.transform.compute_press_pose(
                button_base, ee_rx, ee_ry, ee_rz,
                approach_offset_m=approach_offset,
                press_depth_m=press_depth,
                tool_offset_z=self.tool_z,
                clearance_m=clearance,
            )

            self.robot.set_speed_percent(approach_speed)
            self.robot.move_p(list(to_rad(pre)))
            self._wait_for_motion()

            self.robot.set_speed_percent(press_speed)
            self.robot.move_l(list(to_rad(press)))
            self._wait_for_motion()
            time.sleep(0.3)   # let the switch actuate

            self.robot.set_speed_percent(retract_speed)
            self.robot.move_l(list(to_rad(pre)))
            self._wait_for_motion()

            self.robot.move_l(list(to_rad(safe)))
            self._wait_for_motion()

            logger.info("Press sequence complete for '%s'.", target_id)
            return True

        logger.error("FAILED to press '%s' after %d attempts.",
                     target_id, max_retries)
        return False

    def _go_home_on_failure(self):
        """Best-effort return to a safe position after a press fails.

        Uses the FIRST point name in this trajectory's own sequence as
        "home" -- the convention used throughout this project is that a
        trajectory starts at its safe/rest position. Override with
        --home-point if a given file doesn't follow that convention.

        Deliberately slow and deliberately best-effort: if the arm is in
        an awkward pose after a failed detection, a fast or careless move
        home is exactly the kind of motion that could sweep the tool
        across the panel. A failure here is logged, not raised -- the
        operator should still see the original press failure as the
        primary error, not a secondary move failure.
        """
        home_name = self.home_point
        if home_name is None:
            for item in self.sequence:
                if isinstance(item, str):
                    home_name = item
                    break

        if home_name is None or home_name not in self.positions:
            logger.error("No home point available to retreat to -- "
                        "arm left at failure pose. Check manually.")
            return

        logger.info("Retreating to '%s' after press failure...", home_name)
        try:
            pt = self.positions[home_name]
            safe_speed = min(self.speed_pct, 15)
            self.robot.set_speed_percent(safe_speed)
            self.robot.move_j(pt["joints"])

            start_t = time.monotonic()
            timeout = float(self.settings.get("move_timeout", 12.0))
            while True:
                err = self._get_max_joint_error_deg(pt["joints"])
                if err < 2.0:
                    logger.info("Reached '%s'.", home_name)
                    break
                if time.monotonic() - start_t > timeout:
                    logger.warning("Timeout retreating to '%s' -- check "
                                   "the arm manually.", home_name)
                    break
                time.sleep(0.05)
        except Exception as e:
            logger.error("Retreat to '%s' failed: %s -- check the arm "
                        "manually.", home_name, e)

    # -- Execution -------------------------------------------------------------------

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
        if self._needs_camera():
            print("  Includes camera-driven press action(s).")
        print("  Keep your hand near the E-STOP / Power switch!")
        print("=" * 70)

        if not self.dry_run:
            confirm = input("\nStart execution? [y/N]: ").strip().lower()
            if confirm != "y":
                logger.info("Execution cancelled by operator.")
                return

        if self._needs_camera() and not self.dry_run:
            self._connect_camera()

        print("\n" + "-" * 70)
        print("  STARTING MOTION PLAYBACK")
        print("-" * 70)

        total_points = len(self.sequence)

        for idx, item in enumerate(self.sequence, start=1):

            # --- action waypoint ---
            if isinstance(item, dict):
                action = item.get("action")

                if action == "press":
                    target = item.get("target")
                    print(f"\n▶ Waypoint [{idx}/{total_points}]: press '{target}'")
                    if not self._press(target, panel=item.get("panel")):
                        logger.error("Press failed. Aborting trajectory.")
                        self._go_home_on_failure()
                        break
                    continue

                if action == "pause":
                    secs = float(item.get("seconds", 1.0))
                    print(f"\n▶ Waypoint [{idx}/{total_points}]: pause {secs}s")
                    if not self.dry_run:
                        time.sleep(secs)
                    continue

                logger.warning("[%d/%d] unknown action %r -- skipping.",
                               idx, total_points, action)
                continue

            # --- point waypoint (original behaviour, unchanged) ---
            pos_name = item
            if pos_name not in self.positions:
                logger.error("Point '%s' missing from position definitions. Aborting.", pos_name)
                break

            target_rad = self.positions[pos_name]
            target_deg = [round(math.degrees(a), 1) for a in target_rad]

            print(f"\n▶ Waypoint [{idx}/{total_points}]: '{pos_name}'")
            print(f"  Target Joints (deg): {target_deg}")

            if self.dry_run:
                print("  Status: [DRY-RUN SIMULATION] -> Reached")
                time.sleep(0.5)
                continue

            self.robot.set_speed_percent(self.speed_pct)
            self.robot.move_j(target_rad)

            start_t = time.monotonic()
            time.sleep(0.1)

            while True:
                elapsed = time.monotonic() - start_t
                max_err_deg = self._get_max_joint_error(target_rad)

                status_code = -1
                try:
                    status = self.robot.get_arm_status()
                    if status is not None and hasattr(status, "msg"):
                        status_code = getattr(status.msg, "motion_status", -1)
                except Exception:
                    pass

                print(
                    f"\r  Status: MOVING | Elapsed: {elapsed:.1f}s | Max Joint Err: {max_err_deg:.2f}° | Code: {status_code}",
                    end="",
                    flush=True
                )

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

            if settle_time > 0:
                print(f"  Settling for {settle_time}s...", end="", flush=True)
                time.sleep(settle_time)
                print(" Done.")

        print("\n" + "=" * 70)
        logger.info("Trajectory execution finished.")
        print("=" * 70 + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Piper L Safe Trajectory Player")
    parser.add_argument("--file", "-f", required=True, help="Path to YAML trajectory file")
    parser.add_argument("--speed", "-s", type=int, default=15, help="Speed percentage 1-100 (Default: 15%% for testing)")
    parser.add_argument("--can", default="can0", help="CAN channel name (default: can0)")
    parser.add_argument("--config", default="config.yaml",
                        help="Path to config.yaml, for camera_transform etc. (default: config.yaml)")
    parser.add_argument("--dry-run", action="store_true", help="Simulate without physically driving arm")
    parser.add_argument("--home-point", default=None,
                        help="Point name to retreat to if a press fails. "
                             "Default: the first point in this file's own "
                             "trajectory sequence.")

    args = parser.parse_args()

    player = SafeTrajectoryPlayer(
        trajectory_file=args.file,
        speed_pct=args.speed,
        can_channel=args.can,
        dry_run=args.dry_run,
        config_path=args.config,
        home_point=args.home_point,
    )

    try:
        player.connect()
        player.execute()
    except KeyboardInterrupt:
        print("\n\n!!! MOTION INTERRUPTED BY OPERATOR (Ctrl+C) !!!")
    finally:
        player.disconnect()
