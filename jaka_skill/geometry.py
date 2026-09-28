# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Pixel -> arm base_link XY via a 2D hand-eye homography.

Only the calibrated geometry lives here — no detection, no arm motion. The
matrix comes from deployment calibration (same kind of 3x3 the agilex
grasp_pose service uses): it maps an undistorted image pixel (u, v) to a
planar (x, y) in the arm's base frame, assuming the object sits at a known
table height (supplied separately as the grasp z). Not configured -> callers
get a clear "geometry unavailable" signal rather than a guessed pose.
"""
from __future__ import annotations

import logging
import math
from typing import Optional, Sequence, Tuple

log = logging.getLogger("jaka_skill.geometry")


class Homography:
    """2D homography: undistorted pixel -> arm base_link XY (metres).

    `matrix` is a 3x3 nested list/array (deployment hand-eye calibration).
    When unset, `to_arm_xy()` raises ValueError so the orchestrator can
    report "geometry not calibrated" instead of driving the arm blind.
    """

    def __init__(self, matrix: Optional[Sequence[Sequence[float]]] = None) -> None:
        self._m = matrix

    @property
    def configured(self) -> bool:
        return self._m is not None

    def to_arm_xy(self, u: float, v: float) -> Tuple[float, float]:
        """Pixel (u, v) -> arm base (x, y) in metres. Raises ValueError when
        the matrix is missing or the projection degenerates."""
        h = self._m
        if h is None:
            raise ValueError("homography matrix is not configured")
        try:
            h00, h01, h02 = (float(x) for x in h[0])
            h10, h11, h12 = (float(x) for x in h[1])
            h20, h21, h22 = (float(x) for x in h[2])
        except (TypeError, ValueError, IndexError) as e:
            raise ValueError(f"bad homography matrix: {e}") from e
        x = h00 * u + h01 * v + h02
        y = h10 * u + h11 * v + h12
        w = h20 * u + h21 * v + h22
        if abs(w) < 1e-12:
            raise ValueError("homography projection has near-zero scale")
        return x / w, y / w

    def bbox_center_to_arm_xy(
            self, bbox: Sequence[float]) -> Optional[Tuple[float, float]]:
        """Project the bbox centre to arm base XY; None on a bad bbox or a
        failed projection (never raises — the caller decides how to report)."""
        if len(bbox) < 4:
            return None
        u = 0.5 * (float(bbox[0]) + float(bbox[2]))
        v = 0.5 * (float(bbox[1]) + float(bbox[3]))
        try:
            return self.to_arm_xy(u, v)
        except ValueError:
            return None


def long_edge_yaw_rad(
        u: float, v: float, w: float, h: float, rot_deg: float) -> float:
    """Gripper yaw (rad, in [-pi/2, pi/2]) for a rotated bbox.

    roboarm's long-edge rule (same math as the agilex grasp_pose service's
    `_gripper_angle_by_longer`, numpy-free): rebuild the 4 rotated corners,
    take the LONGER edge, order its endpoints left-to-right, and return
    `pi/2 + atan2(dy, dx)` wrapped into [-pi/2, pi/2] — the gripper closes
    ACROSS the object's long edge. `rot_deg` follows the OpenCV `boxPoints`
    convention (what the yolo obb backend emits as bbox_2d[4]).
    """
    theta = math.radians(float(rot_deg))
    c, s = math.cos(theta), math.sin(theta)
    half_w, half_h = float(w) / 2.0, float(h) / 2.0
    local = ((-half_w, -half_h), (half_w, -half_h),
             (half_w, half_h), (-half_w, half_h))
    pts = [(x * c - y * s + float(u), x * s + y * c + float(v))
           for x, y in local]

    if math.dist(pts[0], pts[1]) > math.dist(pts[1], pts[2]):
        a, b = pts[0], pts[1]
    else:
        a, b = pts[1], pts[2]
    if b[0] < a[0]:
        a, b = b, a

    yaw = math.pi / 2 + math.atan2(b[1] - a[1], b[0] - a[0])
    if yaw > math.pi / 2:
        yaw -= math.pi
    return yaw


def bbox_yaw_rad(bbox: Sequence[float]) -> Optional[float]:
    """Yaw for a 5-element bbox [x_min, y_min, x_max, y_max, rot_deg];
    None for a plain 4-element bbox (no rotation information)."""
    if len(bbox) < 5:
        return None
    try:
        x_min, y_min, x_max, y_max, rot = (float(x) for x in bbox[:5])
    except (TypeError, ValueError):
        return None
    w, h = abs(x_max - x_min), abs(y_max - y_min)
    if w <= 0.0 or h <= 0.0:
        return None
    return long_edge_yaw_rad(0.5 * (x_min + x_max), 0.5 * (y_min + y_max),
                             w, h, rot)
