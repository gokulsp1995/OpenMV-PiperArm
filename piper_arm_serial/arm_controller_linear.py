#!/usr/bin/env python3
"""Piper L arm controller for lift button pressing.

Owns the CAN connection, the camera connection, and the coordinate
transform. Everything else in the package is either a tool this uses or a
CLI that drives it.

Trajectory waypoints (see play_trajectory) can be:

    - home                                   plain joint move (move_j)
    - {name: up-down-panel}                  same, explicit form
    - {name: up-down-panel, mode: linear}     straight-line move (move_l)
    - {action: press, target: up}            camera-driven press
    - {action: pause, seconds: 1.0}          wait

Uses pyAgxArm (https://github.com/agilexrobotics/pyAgxArm).
"""

import glob
import logging
import math
import os
import statistics
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


class ArmController:
    """High-level controller for the Piper L arm + OpenMV camera.

    pyAgxArm works in SI units natively: metres, radians, speed percent.
    CoordinateTransform works in DEGREES, so orientations are converted at
    each boundary -- watch for that when reading press_at_current_pose().
    """

    def __init__(self, config_path: str = "config.yaml", dry_run: bool = False):
        self.dry_run = dry_run
        self.cfg = self._load_config(config_path)
        self.positions: Dict[str, dict] = {}
        self.trajectories: Dict[str, dict] = {}
        self.robot = None
        self.camera: Optional[CameraClient] = None
        self.transform: Optional[CoordinateTransform] = None

        self._init_transform()
        self._load_positions()
        self._load_trajectories()

    # -- Initialisation ----------------------------------------------------------

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
        if ct.get("calibrated") is not True:
            logger.warning(
                "camera_transform is NOT calibrated -- using placeholders. "
                "Computed button positions will be wrong. Run jog_axes.py, "
                "then calib_logger.py + calib_analyse.py, then set "
                "camera_transform.calibrated: true in config.yaml.")

    def _load_positions(self):
        """Load positions/*.yaml. Stores both joints and end_pose, since
        linear-move waypoints need end_pose and joint-move waypoints need
        joints."""
        pos_dir = self.cfg.get("positions_dir", "positions/")
        for filepath in glob.glob(os.path.join(pos_dir, "*.yaml")):
            try:
                with open(filepath, "r") as f:
                    data = yaml.safe_load(f)
                name = data.get("name", os.path.splitext(
                    os.path.basename(filepath))[0])

                joints = data.get("joints", {})
                joint_list = [joints.get("joint_%d" % i, 0.0)
                             for i in range(1, 7)]

                ep = data.get("end_pose", {})
                pose_list = [
                    ep.get("x", 0.0), ep.get("y", 0.0), ep.get("z", 0.0),
                    ep.get("roll", 0.0), ep.get("pitch", 0.0), ep.get("yaw", 0.0),
                ]

                self.positions[name] = {
                    "joints": joint_list,
                    "end_pose": pose_list,
                    "description": data.get("description", ""),
                }
            except Exception as e:
                logger.warning("Failed to load %s: %s", filepath, e)
        logger.info("Loaded %d position(s): %s",
                    len(self.positions), list(self.positions.keys()))

    def _load_trajectories(self):
        traj_dir = self.cfg.get("trajectories_dir", "trajectories/")
        if not os.path.isdir(traj_dir):
            return
        for filepath in glob.glob(os.path.join(traj_dir, "*.yaml")):
            try:
                with open(filepath, "r") as f:
                    data = yaml.safe_load(f)
                name = data.get("name", os.path.splitext(
                    os.path.basename(filepath))[0])
                self.trajectories[name] = data
            except Exception as e:
                logger.warning("Failed to load %s: %s", filepath, e)
        if self.trajectories:
            logger.info("Loaded %d trajectory(ies): %s",
                        len(self.trajectories), list(self.trajectories.keys()))

    # -- Connections -------------------------------------------------------------

    def connect_arm(self):
        """Connect over CAN and enable the motors."""
        can_cfg = self.cfg.get("can", {})
        channel = can_cfg.get("channel", "can0")
        interface = can_cfg.get("interface", "socketcan")
        bitrate = can_cfg.get("bitrate", 1000000)
        fw_str = can_cfg.get("firmware_version", "default")

        fw = {
            "default": PiperFW.DEFAULT,
            "v183": PiperFW.V183,
            "v188": PiperFW.V188,
            "v189": PiperFW.V189,
        }.get(fw_str, PiperFW.DEFAULT)

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

        start_t = time.monotonic()
        while not self.robot.is_ok():
            if time.monotonic() - start_t > 10.0:
                raise RuntimeError(
                    "Arm communication timeout -- is the CAN bus up? "
                    "Check: ip link show can0 / candump can0")
            time.sleep(0.1)

        logger.info("Enabling arm motors...")
        start_t = time.monotonic()
        while not self.robot.enable():
            time.sleep(0.01)
            if time.monotonic() - start_t > 5.0:
                raise RuntimeError("Failed to enable arm after 5 seconds")

        logger.info("Arm enabled and ready.")

    def disconnect_arm(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass
            self.robot = None

    def connect_camera(self):
        """Open the serial link to the OpenMV camera.

        NOTE this only confirms the port opened -- not that main.py is
        running on the camera. status() is the real liveness check, which
        is why it's called straight after.
        """
        cam_cfg = self.cfg.get("camera", {})
        port_hint = cam_cfg.get("port_hint", "OpenMV")
        baudrate = cam_cfg.get("baudrate", 115200)
        timeout = cam_cfg.get("timeout", 5.0)

        logger.info("Connecting to camera (port_hint=%s)...", port_hint)
        self.camera = CameraClient(port_hint, baudrate, timeout)
        if not self.camera.connect():
            raise ConnectionError(
                "Cannot open camera port (port_hint=%s). Is viewer.py or "
                "picocom holding it?" % port_hint)

        st = self.camera.status()
        if st is None:
            raise ConnectionError(
                "Camera port opened but STATUS got no reply -- main.py is "
                "probably not running on the camera. Reset it and retry.")
        logger.info("Camera ready (camera=%s tof=%s)",
                    st.get("camera"), st.get("tof"))
        if not st.get("tof"):
            logger.warning("ToF reports unavailable -- distances will be 0 "
                           "and no press can be planned.")

    def disconnect_camera(self):
        if self.camera is not None:
            self.camera.disconnect()
            self.camera = None

    # -- Motion primitives -------------------------------------------------------

    def get_current_pose(self) -> Tuple[float, float, float,
                                        float, float, float]:
        """Flange pose (x, y, z, roll, pitch, yaw) in metres and RADIANS."""
        if self.dry_run:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        fp = self.robot.get_flange_pose()
        if fp is None:
            logger.warning("get_flange_pose() returned None")
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        m = fp.msg
        return (m[0], m[1], m[2], m[3], m[4], m[5])

    def get_current_joint_angles(self) -> List[float]:
        """Six joint angles in radians."""
        if self.dry_run:
            return [0.0] * 6
        ja = self.robot.get_joint_angles()
        if ja is None:
            logger.warning("get_joint_angles() returned None")
            return [0.0] * 6
        return list(ja.msg)

    def move_to_joint_position(self, position_name: str, speed: int = None):
        """Move to a recorded position by joint interpolation (move_j)."""
        if position_name not in self.positions:
            raise ValueError("Position '%s' not found. Available: %s"
                             % (position_name, list(self.positions.keys())))

        angles = self.positions[position_name]["joints"]

        if speed is None:
            speed = self.cfg.get("motion", {}).get("approach_speed", 50)

        logger.info("MoveJ -> '%s' (%d%%)", position_name, speed)

        if self.dry_run:
            logger.info("[DRY RUN] move_j(%s)", angles)
            return

        self.robot.set_speed_percent(speed)
        self.robot.move_j(angles)
        self._wait_for_motion(target_joints=angles)

    def move_to_named_pose_linear(self, position_name: str, speed: int = None):
        """Move to a recorded position's Cartesian pose by straight line
        (move_l), rather than joint interpolation. Needs end_pose recorded
        for this position -- position_recorder.py already saves that."""
        if position_name not in self.positions:
            raise ValueError("Position '%s' not found. Available: %s"
                             % (position_name, list(self.positions.keys())))

        pose = self.positions[position_name]["end_pose"]

        if speed is None:
            speed = self.cfg.get("motion", {}).get("approach_speed", 50)

        logger.info("MoveL -> '%s' (%d%%)", position_name, speed)

        if self.dry_run:
            logger.info("[DRY RUN] move_l(%s)", pose)
            return

        self.robot.set_speed_percent(speed)
        self.robot.move_l(pose)
        self._wait_for_motion()

    def move_to_end_pose(self, x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad,
                         mode: str = "movej", speed: int = 50):
        """Move the flange to a Cartesian pose.

        mode="movel" -> move_l, a straight line in space. Use it wherever a
        collision matters; it is the only mode whose path is predictable.
        mode="movej" -> move_p, point-to-point Cartesian.
        """
        pose = [x_m, y_m, z_m, roll_rad, pitch_rad, yaw_rad]
        logger.info("%s (%d%%) -> [%.4f, %.4f, %.4f | %.3f, %.3f, %.3f]",
                    mode.upper(), speed, *pose)

        if self.dry_run:
            logger.info("[DRY RUN] skipping motion")
            return

        self.robot.set_speed_percent(speed)
        if mode == "movel":
            self.robot.move_l(pose)
        else:
            self.robot.move_p(pose)
        self._wait_for_motion()

    def _wait_for_motion(self, timeout: float = None, target_joints=None):
        """Block until the arm reports the move complete.

        Checks motion_status AND, when target_joints is given, that the
        joints actually reached the target within a degree. motion_status
        alone has been seen to report complete while the arm was still
        settling.
        """
        if self.dry_run:
            return

        motion_cfg = self.cfg.get("motion", {})
        if timeout is None:
            timeout = motion_cfg.get("move_timeout", 10.0)
        settle = motion_cfg.get("settle_time", 1.5)
        tol_deg = motion_cfg.get("joint_tolerance_deg", 1.0)

        time.sleep(0.3)
        start = time.monotonic()

        while True:
            status_ok = False
            try:
                st = self.robot.get_arm_status()
                status_ok = (st is not None and
                             getattr(st.msg, "motion_status", None) == 0)
            except Exception:
                pass

            joints_ok = True
            if target_joints is not None:
                try:
                    cur = self.get_current_joint_angles()
                    err = max(abs(math.degrees(t - c))
                              for t, c in zip(target_joints, cur))
                    joints_ok = err < tol_deg
                except Exception:
                    joints_ok = True

            if status_ok and joints_ok:
                break

            if time.monotonic() - start > timeout:
                logger.warning("Motion wait timed out after %.1fs", timeout)
                break

            time.sleep(0.05)

        time.sleep(settle)

    # -- Vision ------------------------------------------------------------------

    def sample_button(self, target_id: str, panel=None, samples: int = None):
        """Scan several times and return the median detection.

        A single scan is not trustworthy: readings are usually stable to a
        few mm, but the occasional outlier jumps 50mm+ when a ToF zone
        catches the background instead of the button. The median ignores
        those; a mean would be dragged by them.
        """
        motion_cfg = self.cfg.get("motion", {})
        if samples is None:
            samples = motion_cfg.get("scan_samples", 5)
        min_hits = motion_cfg.get("scan_min_hits", 3)
        max_spread = motion_cfg.get("scan_max_spread_mm", 25)

        hits = []
        for i in range(samples):
            result = self.camera.scan(panel=panel)
            if result is None or result.get("type") != "detections":
                continue
            for btn in result.get("buttons", []):
                if btn.get("id") != target_id:
                    continue
                if btn.get("distance_mm", 0) <= 0:
                    continue
                hits.append(btn)
                break
            time.sleep(0.05)

        if len(hits) < min_hits:
            logger.warning("Only %d/%d valid samples for '%s' (need %d)",
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
            "id": target_id,
            "pixel_x": statistics.median([h["pixel_x"] for h in hits]),
            "pixel_y": statistics.median([h["pixel_y"] for h in hits]),
            "distance_mm": statistics.median(dists),
            "cam_x": statistics.median([h["cam_x"] for h in hits]),
            "cam_y": statistics.median([h["cam_y"] for h in hits]),
            "cam_z": statistics.median([h["cam_z"] for h in hits]),
            "state": hits[-1].get("state", "unknown"),
            "spread_mm": spread,
            "samples": len(hits),
        }
        logger.info("Button '%s': %d/%d samples, d=%.0fmm (spread %.0fmm), "
                    "px=(%.0f, %.0f), cam=(%.4f, %.4f, %.4f)",
                    target_id, len(hits), samples, med["distance_mm"], spread,
                    med["pixel_x"], med["pixel_y"],
                    med["cam_x"], med["cam_y"], med["cam_z"])
        return med

    # -- Press -------------------------------------------------------------------

    def press_at_current_pose(self, target_id: str, panel=None) -> bool:
        """Detect and press, assuming the arm is ALREADY looking at the panel.

        Motion is approach (move_p) -> press (move_l) -> retract (move_l)
        -> clear (move_l).
        """
        motion_cfg = self.cfg.get("motion", {})
        max_retries = motion_cfg.get("max_retries", 3)
        approach_offset = motion_cfg.get("approach_offset_mm", 50.0) / 1000.0
        press_depth = motion_cfg.get("press_depth_mm", 5.0) / 1000.0
        clearance = motion_cfg.get("clearance_mm", 150.0) / 1000.0
        approach_speed = motion_cfg.get("approach_speed", 50)
        press_speed = motion_cfg.get("press_speed", 20)
        retract_speed = motion_cfg.get("retract_speed", 40)
        tool_z = self.cfg.get("tool_offset", {}).get("z", 0.145)
        verify = motion_cfg.get("verify_press", False)

        look_pose = self.get_current_pose()

        for attempt in range(1, max_retries + 1):
            logger.info("--- press '%s', attempt %d/%d ---",
                        target_id, attempt, max_retries)

            btn = self.sample_button(target_id, panel=panel)
            if btn is None:
                logger.warning("No reliable detection; retrying.")
                time.sleep(0.5)
                continue

            ee = self.get_current_pose()
            ee_rx = math.degrees(ee[3])
            ee_ry = math.degrees(ee[4])
            ee_rz = math.degrees(ee[5])

            button_base = self.transform.camera_to_base(
                (btn["cam_x"], btn["cam_y"], btn["cam_z"]),
                ee[0], ee[1], ee[2], ee_rx, ee_ry, ee_rz,
            )
            logger.info("Button in base frame: (%.4f, %.4f, %.4f) m",
                        *button_base)

            pre, press, safe = self.transform.compute_press_pose(
                button_base, ee_rx, ee_ry, ee_rz,
                approach_offset_m=approach_offset,
                press_depth_m=press_depth,
                tool_offset_z=tool_z,
                clearance_m=clearance,
            )

            def to_rad(p):
                return (p[0], p[1], p[2],
                        math.radians(p[3]), math.radians(p[4]),
                        math.radians(p[5]))

            logger.info("Approach to pre-press...")
            self.move_to_end_pose(*to_rad(pre), mode="movej",
                                  speed=approach_speed)

            logger.info("Press (linear)...")
            self.move_to_end_pose(*to_rad(press), mode="movel",
                                  speed=press_speed)
            time.sleep(0.3)

            logger.info("Retract (linear)...")
            self.move_to_end_pose(*to_rad(pre), mode="movel",
                                  speed=retract_speed)

            logger.info("Clear (linear)...")
            self.move_to_end_pose(*to_rad(safe), mode="movel",
                                  speed=retract_speed)

            if not verify:
                logger.info("Press sequence complete (verification disabled).")
                return True

            logger.info("Verifying...")
            self.move_to_end_pose(*look_pose[:3],
                                  look_pose[3], look_pose[4], look_pose[5],
                                  mode="movej", speed=approach_speed)
            time.sleep(0.5)
            check = self.sample_button(target_id, panel=panel)
            if check and check.get("state") == "white_lit":
                logger.info("Button '%s' confirmed lit.", target_id)
                return True
            logger.warning("Not confirmed lit; retrying.")

        logger.error("FAILED: could not press '%s' after %d attempts.",
                     target_id, max_retries)
        return False

    def press_button(self, target_id: str, start_position: str) -> bool:
        """Connect, move to a recorded pose, then press. Standalone entry."""
        logger.info("=" * 60)
        logger.info("  PRESS '%s'  (start: '%s')", target_id, start_position)
        logger.info("=" * 60)

        self.connect_arm()
        self.connect_camera()

        self.move_to_joint_position(start_position)

        panel = self.cfg.get("panel_mapping", {}).get(start_position)
        return self.press_at_current_pose(target_id, panel=panel)

    # -- Trajectories ------------------------------------------------------------

    def _needs_camera(self, waypoints) -> bool:
        return any(isinstance(w, dict) and w.get("action") == "press"
                   for w in waypoints)

    def play_trajectory(self, name: str, speed: int = None) -> bool:
        """Run a waypoint sequence.

        Each waypoint is one of:
            "home"                                   plain joint move
            {name: "home"}                           same, explicit
            {name: "home", mode: "linear"}            straight-line move
            {action: "press", target: "up"}           camera-driven press
            {action: "pause", seconds: 1.0}          wait

        Returns False if any press fails; motion errors raise.
        """
        if name not in self.trajectories:
            raise ValueError("Trajectory '%s' not found. Available: %s"
                             % (name, list(self.trajectories.keys())))

        traj = self.trajectories[name]
        waypoints = traj.get("trajectory", [])
        if not waypoints:
            logger.warning("Trajectory '%s' has no waypoints.", name)
            return True

        logger.info("=" * 60)
        logger.info("  TRAJECTORY '%s'  (%d waypoints)", name, len(waypoints))
        logger.info("=" * 60)

        if self.robot is None and not self.dry_run:
            self.connect_arm()
        if self._needs_camera(waypoints) and self.camera is None and not self.dry_run:
            self.connect_camera()

        ok = True
        for i, wp in enumerate(waypoints, 1):

            # -- plain joint move: bare string --
            if isinstance(wp, str):
                logger.info("[%d/%d] move_j -> %s", i, len(waypoints), wp)
                self.move_to_joint_position(wp, speed=speed)
                continue

            if not isinstance(wp, dict):
                logger.warning("[%d/%d] skipping unrecognised waypoint: %r",
                               i, len(waypoints), wp)
                continue

            # -- named position, optionally linear --
            if "name" in wp:
                pos_name = wp["name"]
                if wp.get("mode") == "linear":
                    logger.info("[%d/%d] move_l -> %s",
                                i, len(waypoints), pos_name)
                    self.move_to_named_pose_linear(pos_name, speed=speed)
                else:
                    logger.info("[%d/%d] move_j -> %s",
                                i, len(waypoints), pos_name)
                    self.move_to_joint_position(pos_name, speed=speed)
                continue

            # -- action waypoints --
            action = wp.get("action")

            if action == "press":
                target = wp.get("target")
                panel = wp.get("panel")
                logger.info("[%d/%d] press -> %s", i, len(waypoints), target)
                if not self.press_at_current_pose(target, panel=panel):
                    logger.error("Press failed; aborting trajectory.")
                    ok = False
                    break

            elif action == "pause":
                secs = float(wp.get("seconds", 1.0))
                logger.info("[%d/%d] pause %.1fs", i, len(waypoints), secs)
                if not self.dry_run:
                    time.sleep(secs)

            else:
                logger.warning("[%d/%d] unknown waypoint: %r",
                               i, len(waypoints), wp)

        logger.info("Trajectory '%s' %s.", name,
                    "complete" if ok else "aborted")
        return ok

    # -- Status ------------------------------------------------------------------

    def print_status(self):
        print("--- Arm ---")
        if self.robot is not None:
            try:
                print("  connected: %s" % self.robot.is_ok())
                print("  fps: %.1f Hz" % self.robot.get_fps())
                st = self.robot.get_arm_status()
                if st is not None:
                    print("  ctrl mode: %s" % st.msg.ctrl_mode)
                    print("  arm status: %s" % st.msg.arm_status)
                    print("  motion status: %s" % st.msg.motion_status)
                p = self.get_current_pose()
                print("  flange: x=%.4f y=%.4f z=%.4f  r=%.3f p=%.3f y=%.3f"
                      % p)
                print("  joints (deg): %s"
                      % ["%.1f" % math.degrees(j)
                         for j in self.get_current_joint_angles()])
            except Exception as e:
                print("  error:", e)
        else:
            print("  not connected")

        print("--- Camera ---")
        if self.camera is not None and self.camera.connected:
            st = self.camera.status()
            if st:
                print("  serial: %s  camera: %s  tof: %s"
                      % (st.get("serial"), st.get("camera"), st.get("tof")))
            else:
                print("  no response -- is main.py running?")
        else:
            print("  not connected")

        ct = self.cfg.get("camera_transform", {})
        print("--- camera_transform ---")
        print("  t=(%.4f, %.4f, %.4f) m   r=(%.1f, %.1f, %.1f) deg   %s"
              % (ct.get("tx", 0), ct.get("ty", 0), ct.get("tz", 0),
                 ct.get("rx", 0), ct.get("ry", 0), ct.get("rz", 0),
                 "CALIBRATED" if ct.get("calibrated") else "PLACEHOLDER"))

        print("--- positions (%d) ---" % len(self.positions))
        for n in sorted(self.positions):
            print("  %s" % n)
        print("--- trajectories (%d) ---" % len(self.trajectories))
        for n in sorted(self.trajectories):
            wps = self.trajectories[n].get("trajectory", [])
            print("  %s (%d waypoints)" % (n, len(wps)))
