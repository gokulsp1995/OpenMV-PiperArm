#!/usr/bin/env python3
"""Piper L arm controller for lift button pressing.

Orchestrates the full workflow:
  1. Move arm to a starting position (loaded from YAML)
  2. Trigger an OpenMV camera scan
  3. Transform detected button position to arm base frame
  4. Plan approach → press → retract motion
  5. Verify button state (white = success)
  6. Retry up to N times if unsuccessful
  7. Return to starting position

Uses the pyAgxArm library (https://github.com/agilexrobotics/pyAgxArm).
"""

import glob
import logging
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

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

from camera_client import CameraClient
from coordinate_transform import CoordinateTransform

logger = logging.getLogger(__name__)


# ══════════════════════════════════════════════════════════════════════════════
# ArmController
# ══════════════════════════════════════════════════════════════════════════════


class ArmController:
    """High-level controller for the Piper L arm + OpenMV camera system.

    Uses pyAgxArm which works in SI units natively:
      - Positions: metres
      - Angles: radians
      - Speed: percent (0-100)
    """

    def __init__(self, config_path: str = "config.yaml", dry_run: bool = False):
        """
        Args:
            config_path: Path to the YAML configuration file.
            dry_run: If True, log intended commands without sending them.
        """
        self.dry_run = dry_run
        self.cfg = self._load_config(config_path)
        self.positions: Dict[str, dict] = {}
        self.robot = None  # pyAgxArm driver instance
        self.camera: Optional[CameraClient] = None
        self.transform: Optional[CoordinateTransform] = None

        self._init_transform()
        self._load_positions()

    # ── Initialisation ────────────────────────────────────────────────────────

    @staticmethod
    def _load_config(path: str) -> dict:
        with open(path, "r") as f:
            return yaml.safe_load(f)

    def _init_transform(self):
        ct = self.cfg.get("camera_transform", {})
        self.transform = CoordinateTransform(
            cam_tx=ct.get("tx", 0.0),
            cam_ty=ct.get("ty", 0.0),
            cam_tz=ct.get("tz", 0.05),
            cam_rx=ct.get("rx", 0.0),
            cam_ry=ct.get("ry", 0.0),
            cam_rz=ct.get("rz", 0.0),
        )

    def _load_positions(self):
        """Load all YAML position files from the configured directory."""
        pos_dir = self.cfg.get("positions_dir", "positions/")
        pattern = os.path.join(pos_dir, "*.yaml")
        for filepath in glob.glob(pattern):
            try:
                with open(filepath, "r") as f:
                    data = yaml.safe_load(f)
                name = data.get("name", os.path.splitext(
                    os.path.basename(filepath))[0])
                self.positions[name] = data
                logger.info("Loaded position: %s (%s)", name, filepath)
            except Exception as e:
                logger.warning("Failed to load %s: %s", filepath, e)

        if self.positions:
            logger.info("Loaded %d position(s): %s",
                        len(self.positions), list(self.positions.keys()))
        else:
            logger.warning("No positions loaded from %s", pos_dir)

    # ── Arm Connection ────────────────────────────────────────────────────────

    def connect_arm(self):
        """Connect to the Piper L arm via CAN bus and enable it.

        Uses pyAgxArm's factory pattern:
          1. create_agx_arm_config() with ArmModel.PIPER_L
          2. AgxArmFactory.create_arm()
          3. robot.connect()
          4. robot.enable()
        """
        can_cfg = self.cfg.get("can", {})
        channel = can_cfg.get("channel", "can0")
        interface = can_cfg.get("interface", "socketcan")
        bitrate = can_cfg.get("bitrate", 1000000)
        fw_str = can_cfg.get("firmware_version", "default")

        # Map firmware string to PiperFW constant
        fw_map = {
            "default": PiperFW.DEFAULT,
            "v183": PiperFW.V183,
            "v188": PiperFW.V188,
            "v189": PiperFW.V189,
        }
        fw = fw_map.get(fw_str, PiperFW.DEFAULT)

        logger.info("Connecting to Piper L on '%s' (interface=%s, fw=%s)...",
                     channel, interface, fw_str)

        if self.dry_run:
            logger.info("[DRY RUN] Would connect to CAN '%s'", channel)
            return

        cfg = create_agx_arm_config(
            robot=ArmModel.PIPER_L,
            firmeware_version=fw,
            interface=interface,
            channel=channel,
            bitrate=bitrate,
        )
        self.robot = AgxArmFactory.create_arm(cfg)
        self.robot.connect()

        # Wait for communication to be established
        logger.info("Waiting for arm communication...")
        start_t = time.monotonic()
        while not self.robot.is_ok():
            if time.monotonic() - start_t > 10.0:
                raise RuntimeError(
                    "Arm communication timeout — is the CAN bus active?")
            time.sleep(0.1)

        # Enable all joints
        logger.info("Enabling arm motors...")
        start_t = time.monotonic()
        while not self.robot.enable():
            time.sleep(0.01)
            if time.monotonic() - start_t > 5.0:
                raise RuntimeError("Failed to enable arm after 5 seconds")

        logger.info("Arm enabled and ready.")

    def disconnect_arm(self):
        """Disconnect from the arm and release resources."""
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.robot = None

    def connect_camera(self, camera_ip: str):
        """Connect to the OpenMV camera TCP server."""
        cam_cfg = self.cfg.get("camera", {})
        port = cam_cfg.get("port", 8470)
        timeout = cam_cfg.get("timeout", 5.0)

        logger.info("Connecting to camera at %s:%d ...", camera_ip, port)
        self.camera = CameraClient(camera_ip, port, timeout)
        if not self.camera.connect():
            raise ConnectionError(
                "Cannot connect to camera at %s:%d" % (camera_ip, port))
        logger.info("Camera connected.")

    # ── Motion Primitives ─────────────────────────────────────────────────────

    def get_current_pose(self) -> Tuple[float, float, float,
                                         float, float, float]:
        """Return the current flange pose (x, y, z, roll, pitch, yaw).

        Returns:
            Tuple of (x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad).
        """
        if self.dry_run:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        fp = self.robot.get_flange_pose()
        if fp is None:
            logger.warning("get_flange_pose() returned None")
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        # fp.msg is [x, y, z, roll, pitch, yaw] in metres/radians
        msg = fp.msg
        return (msg[0], msg[1], msg[2], msg[3], msg[4], msg[5])

    def get_current_joint_angles(self) -> List[float]:
        """Return the current joint angles in radians.

        Returns:
            List of 6 floats [j1, j2, j3, j4, j5, j6] in radians.
        """
        if self.dry_run:
            return [0.0] * 6

        ja = self.robot.get_joint_angles()
        if ja is None:
            logger.warning("get_joint_angles() returned None")
            return [0.0] * 6

        return list(ja.msg)

    def move_to_joint_position(self, position_name: str):
        """Move to a named position using joint interpolation (MoveJ).

        Reads joint angles in radians from the position YAML file.
        """
        if position_name not in self.positions:
            raise ValueError(
                "Position '%s' not found. Available: %s"
                % (position_name, list(self.positions.keys())))

        pos = self.positions[position_name]

        # Read joint angles in radians from the YAML
        joints = pos.get("joints", {})
        joint_angles = [
            joints.get("joint_1", 0.0),
            joints.get("joint_2", 0.0),
            joints.get("joint_3", 0.0),
            joints.get("joint_4", 0.0),
            joints.get("joint_5", 0.0),
            joints.get("joint_6", 0.0),
        ]

        speed = self.cfg.get("motion", {}).get("approach_speed", 50)

        logger.info("Moving to position '%s' (MoveJ, %d%% speed)...",
                    position_name, speed)

        if self.dry_run:
            logger.info("[DRY RUN] move_j(%s)", joint_angles)
            return

        self.robot.set_speed_percent(speed)
        self.robot.move_j(joint_angles)

        # Wait for move to complete
        self._wait_for_motion(timeout=self.cfg["motion"].get(
            "move_timeout", 10.0))

    def move_to_end_pose(self, x_m: float, y_m: float, z_m: float,
                         roll_rad: float, pitch_rad: float, yaw_rad: float,
                         mode: str = "movej", speed: int = 50):
        """Move to an end-effector pose.

        Args:
            x_m, y_m, z_m: Position in metres.
            roll_rad, pitch_rad, yaw_rad: Orientation in radians.
            mode: "movej" (joint) or "movel" (linear).
            speed: Speed percentage (0-100).
        """
        pose = [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]

        logger.info("EndPose(%s, %d%%) → [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
                    mode.upper(), speed, *pose)

        if self.dry_run:
            logger.info("[DRY RUN] Skipping actual motion.")
            return

        self.robot.set_speed_percent(speed)

        if mode == "movel":
            self.robot.move_l(pose)
        else:
            # move_p is point-to-point Cartesian (fastest path)
            self.robot.move_p(pose)

        self._wait_for_motion(timeout=self.cfg["motion"].get(
            "move_timeout", 10.0))

    def _wait_for_motion(self, timeout: float = 10.0):
        """Wait for the arm to reach its target (blocking).

        Polls `get_arm_status().msg.motion_status == 0` to detect completion.
        Falls back to time-based wait on timeout.
        """
        if self.dry_run:
            return

        settle = self.cfg.get("motion", {}).get("settle_time", 1.5)

        # Wait for motion_status to indicate target reached
        time.sleep(0.5)  # initial delay for motion to start
        start = time.monotonic()
        while True:
            try:
                status = self.robot.get_arm_status()
                if (status is not None and
                        getattr(status.msg, "motion_status", None) == 0):
                    logger.debug("Motion complete (motion_status=0)")
                    break
            except Exception:
                pass

            if time.monotonic() - start > timeout:
                logger.warning("Motion wait timeout (%.1fs)", timeout)
                break

            time.sleep(0.05)

        # Additional settle time to ensure the arm has stopped vibrating
        time.sleep(settle)

    # ── Button Pressing Workflow ──────────────────────────────────────────────

    def press_button(self, target_id: str, start_position: str,
                     camera_ip: str) -> bool:
        """Execute the full button-pressing workflow.

        Args:
            target_id: Button to press ("1", "2", "3", "up", "down").
            start_position: Name of the starting position YAML.
            camera_ip: IP address of the OpenMV camera.

        Returns:
            True if the button was successfully pressed (white_lit), False otherwise.
        """
        motion_cfg = self.cfg.get("motion", {})
        max_retries = motion_cfg.get("max_retries", 3)
        approach_offset = motion_cfg.get("approach_offset_mm", 50.0) / 1000.0
        press_depth = motion_cfg.get("press_depth_mm", 5.0) / 1000.0
        approach_speed = motion_cfg.get("approach_speed", 50)
        press_speed = motion_cfg.get("press_speed", 20)
        retract_speed = motion_cfg.get("retract_speed", 40)
        tool_z = self.cfg.get("tool_offset", {}).get("z", 0.145)

        # Look up the panel layout for this starting position
        panel_mapping = self.cfg.get("panel_mapping", {})
        panel = panel_mapping.get(start_position, None)

        # --- Step 1: Connect to arm and camera ---
        logger.info("═" * 60)
        logger.info("  PRESS BUTTON '%s'  (start: '%s')", target_id,
                    start_position)
        logger.info("═" * 60)

        self.connect_arm()
        self.connect_camera(camera_ip)

        # --- Step 2: Move to starting position ---
        logger.info("Moving to start position '%s'...", start_position)
        self.move_to_joint_position(start_position)

        # --- Step 3: Scan and find button ---
        for attempt in range(1, max_retries + 1):
            logger.info("─── Attempt %d/%d ───", attempt, max_retries)

            # Scan (with panel layout for position-based identification)
            logger.info("Requesting camera scan (panel=%s)...", panel)
            result = self.camera.scan(panel=panel)
            if result is None or result.get("type") == "error":
                logger.error("Camera scan failed: %s", result)
                continue

            buttons = result.get("buttons", [])
            logger.info("Detected %d button(s)", len(buttons))

            # Find target button
            target_btn = None
            for btn in buttons:
                if btn["id"] == target_id:
                    target_btn = btn
                    break

            if target_btn is None:
                logger.warning("Target button '%s' not found in scan results.",
                               target_id)
                # Move back and retry
                time.sleep(1.0)
                continue

            logger.info("Found button '%s': state=%s, distance=%d mm, "
                        "cam=(%s, %s, %s)",
                        target_btn["id"], target_btn["state"],
                        target_btn["distance_mm"],
                        target_btn["cam_x"], target_btn["cam_y"],
                        target_btn["cam_z"])

            # Check if already pressed (white)
            if target_btn["state"] == "white_lit":
                logger.info("Button '%s' is already lit white — no press "
                            "needed.", target_id)
                self.move_to_joint_position(start_position)
                return True

            # --- Step 4: Transform to base frame ---
            cam_point = (
                target_btn["cam_x"],
                target_btn["cam_y"],
                target_btn["cam_z"],
            )

            # Get current EE pose for the transform
            ee_pose = self.get_current_pose()
            ee_x, ee_y, ee_z = ee_pose[0], ee_pose[1], ee_pose[2]
            # pyAgxArm returns radians — convert to degrees for
            # CoordinateTransform which expects degrees
            ee_rx = math.degrees(ee_pose[3])
            ee_ry = math.degrees(ee_pose[4])
            ee_rz = math.degrees(ee_pose[5])

            button_base = self.transform.camera_to_base(
                cam_point, ee_x, ee_y, ee_z, ee_rx, ee_ry, ee_rz,
            )

            logger.info("Button in base frame: (%.4f, %.4f, %.4f) m",
                        *button_base)

            # --- Step 5: Compute approach and press poses ---
            pre_pose, press_pose = self.transform.compute_press_pose(
                button_base,
                ee_rx, ee_ry, ee_rz,
                approach_offset_m=approach_offset,
                press_depth_m=press_depth,
                tool_offset_z=tool_z,
            )

            # Convert the orientation from degrees (from CoordinateTransform)
            # back to radians for pyAgxArm
            pre_pose_rad = (
                pre_pose[0], pre_pose[1], pre_pose[2],
                math.radians(pre_pose[3]),
                math.radians(pre_pose[4]),
                math.radians(pre_pose[5]),
            )
            press_pose_rad = (
                press_pose[0], press_pose[1], press_pose[2],
                math.radians(press_pose[3]),
                math.radians(press_pose[4]),
                math.radians(press_pose[5]),
            )

            # --- Step 6: Execute motion sequence ---
            # 6a. Move to pre-press (MoveP/MoveJ for speed)
            logger.info("Moving to pre-press position...")
            self.move_to_end_pose(
                *pre_pose_rad,
                mode="movej", speed=approach_speed,
            )

            # 6b. Linear approach to button (MoveL for precision)
            logger.info("Pressing button (MoveL, %d%% speed)...", press_speed)
            self.move_to_end_pose(
                *press_pose_rad,
                mode="movel", speed=press_speed,
            )

            # Brief hold for mechanical activation
            time.sleep(0.3)

            # 6c. Retract to pre-press
            logger.info("Retracting...")
            self.move_to_end_pose(
                *pre_pose_rad,
                mode="movel", speed=retract_speed,
            )

            # --- Step 7: Verify ---
            logger.info("Verifying button state...")
            time.sleep(0.5)  # let button illumination settle
            verify = self.camera.scan(panel=panel)
            if verify and verify.get("type") == "detections":
                for btn in verify.get("buttons", []):
                    if btn["id"] == target_id:
                        if btn["state"] == "white_lit":
                            logger.info("✓ Button '%s' confirmed WHITE — "
                                        "SUCCESS!", target_id)
                            # Return to start
                            self.move_to_joint_position(start_position)
                            return True
                        else:
                            logger.warning("✗ Button '%s' state: %s — "
                                           "retrying...",
                                           target_id, btn["state"])
                            break

            # Move back to start before retrying
            logger.info("Returning to start position for retry...")
            self.move_to_joint_position(start_position)

        # All retries exhausted
        logger.error("FAILED: Could not press button '%s' after %d attempts.",
                     target_id, max_retries)
        return False

    # ── Status ────────────────────────────────────────────────────────────────

    def print_status(self):
        """Print arm and camera status."""
        print("─── Arm Status ───")
        if self.robot is not None:
            try:
                print("  Connected: %s" % self.robot.is_ok())
                print("  FPS: %.1f Hz" % self.robot.get_fps())

                status = self.robot.get_arm_status()
                if status is not None:
                    print("  Ctrl mode: %s" % status.msg.ctrl_mode)
                    print("  Arm status: %s" % status.msg.arm_status)
                    print("  Motion status: %s" % status.msg.motion_status)

                pose = self.get_current_pose()
                print("  Flange pose (m/rad): X=%.4f Y=%.4f Z=%.4f "
                      "R=%.4f P=%.4f Y=%.4f" % pose)

                joints = self.get_current_joint_angles()
                print("  Joint angles (rad): %s" %
                      ["%.4f" % j for j in joints])
            except Exception as e:
                print("  Error:", e)
        else:
            print("  Not connected")

        print("─── Camera Status ───")
        if self.camera is not None and self.camera.connected:
            result = self.camera.status()
            if result:
                print("  WiFi:", result.get("wifi"))
                print("  Camera:", result.get("camera"))
                print("  ToF:", result.get("tof"))
            else:
                print("  No response")
        else:
            print("  Not connected")

        print("─── Loaded Positions ───")
        for name in self.positions:
            pos = self.positions[name]
            ep = pos.get("end_pose", {})
            print("  %s: X=%.4f Y=%.4f Z=%.4f" % (
                name, ep.get("x", 0), ep.get("y", 0), ep.get("z", 0)))
