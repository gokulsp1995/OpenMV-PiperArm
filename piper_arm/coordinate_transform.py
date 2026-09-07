#!/usr/bin/env python3
"""Coordinate transforms between camera frame, end-effector frame, and arm
base frame.

The camera is rigidly mounted on (or near) the Piper L end-effector.
The chain is:

    button_in_camera  →  camera_to_ee  →  ee_to_base  →  button_in_base

The arm SDK reports the end-effector pose in the base frame, so we compose
the camera-to-EE static transform with the current EE pose to get the full
camera-to-base transform.
"""

import math
from typing import Tuple


def rotation_matrix_xyz(rx_deg: float, ry_deg: float, rz_deg: float):
    """Build a 3×3 rotation matrix from XYZ Euler angles (degrees).

    Rotation order: R = Rz · Ry · Rx  (same convention as Piper SDK).
    Returns a row-major 3×3 list-of-lists.
    """
    rx = math.radians(rx_deg)
    ry = math.radians(ry_deg)
    rz = math.radians(rz_deg)

    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    return [
        [cy * cz, sx * sy * cz - cx * sz, cx * sy * cz + sx * sz],
        [cy * sz, sx * sy * sz + cx * cz, cx * sy * sz - sx * cz],
        [-sy,     sx * cy,                cx * cy],
    ]


def mat_vec_mul(R, v):
    """Multiply 3×3 matrix R by 3-vector v."""
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def mat_mat_mul(A, B):
    """Multiply two 3×3 matrices."""
    C = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                C[i][j] += A[i][k] * B[k][j]
    return C


class CoordinateTransform:
    """Handles camera → base frame coordinate conversions."""

    def __init__(self, cam_tx: float, cam_ty: float, cam_tz: float,
                 cam_rx: float, cam_ry: float, cam_rz: float):
        """
        Args:
            cam_tx, cam_ty, cam_tz: Camera offset from J6 flange (metres).
            cam_rx, cam_ry, cam_rz: Camera rotation from J6 flange (degrees).
        """
        # Static camera-to-EE transform
        self._cam_t = [cam_tx, cam_ty, cam_tz]
        self._cam_R = rotation_matrix_xyz(cam_rx, cam_ry, cam_rz)

    def camera_to_base(
        self,
        cam_point: Tuple[float, float, float],
        ee_x: float, ee_y: float, ee_z: float,
        ee_rx: float, ee_ry: float, ee_rz: float,
    ) -> Tuple[float, float, float]:
        """Transform a point from camera frame to arm base frame.

        Args:
            cam_point: (x, y, z) in camera frame, metres.
            ee_x, ee_y, ee_z: Current EE position from SDK, in metres.
            ee_rx, ee_ry, ee_rz: Current EE orientation from SDK, in degrees.

        Returns:
            (x, y, z) in arm base frame, metres.
        """
        # Step 1: camera frame → EE frame
        #   p_ee = R_cam · p_cam + t_cam
        p_ee = mat_vec_mul(self._cam_R, list(cam_point))
        p_ee = [p_ee[i] + self._cam_t[i] for i in range(3)]

        # Step 2: EE frame → base frame
        #   p_base = R_ee · p_ee + t_ee
        R_ee = rotation_matrix_xyz(ee_rx, ee_ry, ee_rz)
        p_base = mat_vec_mul(R_ee, p_ee)
        p_base = [
            p_base[0] + ee_x,
            p_base[1] + ee_y,
            p_base[2] + ee_z,
        ]

        return tuple(p_base)

    def compute_press_pose(
        self,
        button_base: Tuple[float, float, float],
        ee_rx: float, ee_ry: float, ee_rz: float,
        approach_offset_m: float = 0.05,
        press_depth_m: float = 0.005,
        tool_offset_z: float = 0.06,
    ):
        """Compute pre-press and press end-effector poses for a button.

        Assumes the press direction is along the camera Z axis (forward into
        the button panel).  The EE orientation is kept constant — only the
        position changes along the approach vector.

        Args:
            button_base: Button position in base frame (metres).
            ee_rx, ee_ry, ee_rz: Desired EE orientation for pressing (degrees).
            approach_offset_m: Standoff distance before the button (metres).
            press_depth_m: How far to push past the button surface (metres).
            tool_offset_z: Tool tip offset from flange along Z (metres).

        Returns:
            (pre_press_pose, press_pose) — each is (x, y, z, rx, ry, rz)
            with x/y/z in metres and rx/ry/rz in degrees.
        """
        bx, by, bz = button_base

        # The press direction is the EE's local Z axis in base frame
        R_ee = rotation_matrix_xyz(ee_rx, ee_ry, ee_rz)
        # EE Z axis in base frame (3rd column of rotation matrix)
        z_axis = [R_ee[0][2], R_ee[1][2], R_ee[2][2]]

        # Normalise (should already be unit, but be safe)
        mag = math.sqrt(sum(a * a for a in z_axis))
        if mag > 1e-6:
            z_axis = [a / mag for a in z_axis]

        # Pre-press: button position minus (approach_offset + tool_offset)
        # along the press direction
        total_offset = approach_offset_m + tool_offset_z
        pre = (
            bx - z_axis[0] * total_offset,
            by - z_axis[1] * total_offset,
            bz - z_axis[2] * total_offset,
            ee_rx, ee_ry, ee_rz,
        )

        # Press: button position plus press_depth minus tool_offset
        press_offset = tool_offset_z - press_depth_m
        press = (
            bx - z_axis[0] * press_offset,
            by - z_axis[1] * press_offset,
            bz - z_axis[2] * press_offset,
            ee_rx, ee_ry, ee_rz,
        )

        return pre, press
