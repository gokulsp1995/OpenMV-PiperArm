#!/usr/bin/env python3
"""Coordinate transforms: camera frame -> flange frame -> arm base frame.

The camera is rigidly mounted on the Piper L flange, so:

    p_base = T_base_flange(joints) . T_flange_cam . p_cam

T_flange_cam is the static camera_transform from config.yaml -- the one
link in the chain that is measured rather than reported by the arm.
T_base_flange comes from the arm's own forward kinematics and is accurate.

Rotation convention: R = Rz . Ry . Rx from XYZ Euler angles in DEGREES.
Verified offline against known cases (90 deg about each axis in turn).
Whether it matches what pyAgxArm means by roll/pitch/yaw is NOT verified
by that test -- the invariance test on real hardware is what settles it.
"""

import math
from typing import Tuple


def rotation_matrix_xyz(rx_deg: float, ry_deg: float, rz_deg: float):
    """3x3 rotation matrix from XYZ Euler angles (degrees). R = Rz.Ry.Rx."""
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
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


def mat_mat_mul(A, B):
    C = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            for k in range(3):
                C[i][j] += A[i][k] * B[k][j]
    return C


class CoordinateTransform:
    """Camera -> base frame conversions, and press pose planning."""

    def __init__(self, cam_tx: float, cam_ty: float, cam_tz: float,
                 cam_rx: float, cam_ry: float, cam_rz: float):
        """
        Args:
            cam_tx, cam_ty, cam_tz: camera offset from the J6 flange, metres.
            cam_rx, cam_ry, cam_rz: camera rotation from the flange, degrees.
        """
        self._cam_t = [cam_tx, cam_ty, cam_tz]
        self._cam_R = rotation_matrix_xyz(cam_rx, cam_ry, cam_rz)

    # -- Frame conversion --------------------------------------------------------

    def camera_to_base(
        self,
        cam_point: Tuple[float, float, float],
        ee_x: float, ee_y: float, ee_z: float,
        ee_rx: float, ee_ry: float, ee_rz: float,
    ) -> Tuple[float, float, float]:
        """Transform a point from camera frame to arm base frame.

        Args:
            cam_point: (x, y, z) in the camera frame, metres.
            ee_x/y/z: current flange position from the SDK, metres.
            ee_rx/ry/rz: current flange orientation, DEGREES.

        Returns (x, y, z) in the base frame, metres.
        """
        # camera frame -> flange frame
        p_ee = mat_vec_mul(self._cam_R, list(cam_point))
        p_ee = [p_ee[i] + self._cam_t[i] for i in range(3)]

        # flange frame -> base frame
        R_ee = rotation_matrix_xyz(ee_rx, ee_ry, ee_rz)
        p_base = mat_vec_mul(R_ee, p_ee)
        return (p_base[0] + ee_x, p_base[1] + ee_y, p_base[2] + ee_z)

    # -- Press planning ----------------------------------------------------------

    @staticmethod
    def approach_axis(ee_rx: float, ee_ry: float, ee_rz: float):
        """Unit vector of the flange's local Z in the base frame.

        This is the press direction. NOTE it comes from the arm's current
        orientation, not from the panel -- so the press goes in square only
        if the look pose faces the panel square. The 8x8 ToF could give the
        real panel normal via a plane fit; that's a future improvement.
        """
        R = rotation_matrix_xyz(ee_rx, ee_ry, ee_rz)
        z = [R[0][2], R[1][2], R[2][2]]
        mag = math.sqrt(sum(a * a for a in z))
        if mag > 1e-6:
            z = [a / mag for a in z]
        return z

    def compute_press_pose(
        self,
        button_base: Tuple[float, float, float],
        ee_rx: float, ee_ry: float, ee_rz: float,
        approach_offset_m: float = 0.05,
        press_depth_m: float = 0.005,
        tool_offset_z: float = 0.06,
        clearance_m: float = 0.15,
    ):
        """Plan the three poses of a press.

        All three keep the flange orientation fixed -- only position moves,
        along the approach axis.

        Returns (pre_pose, press_pose, safe_pose), each
        (x, y, z, rx, ry, rz) with position in metres and orientation in
        DEGREES.

            safe_pose  further back than pre_pose. Move here before any
                       joint-space motion: move_j interpolates in joint
                       space, so the tool's path through real space is not
                       a straight line and could sweep across the panel.
                       pre_pose alone is only approach_offset clear, which
                       is not much next to a 145mm tool.
        """
        bx, by, bz = button_base
        z = self.approach_axis(ee_rx, ee_ry, ee_rz)

        def along(dist):
            return (bx - z[0] * dist, by - z[1] * dist, bz - z[2] * dist,
                    ee_rx, ee_ry, ee_rz)

        # The flange must sit tool_offset_z back from where the TIP goes.
        pre = along(approach_offset_m + tool_offset_z)
        press = along(tool_offset_z - press_depth_m)
        safe = along(approach_offset_m + tool_offset_z + clearance_m)

        return pre, press, safe
