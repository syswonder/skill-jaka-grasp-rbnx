# SPDX-License-Identifier: Apache-2.0
"""Unit tests for place's target precedence (no ROS traffic, no hardware).

`place` takes four floats, but the framework has no optional MCP
arguments, so 0/0 spells "this pair is not given". These tests pin the
precedence — pixel (u, v) / metres (x, y) (mutually exclusive) >
configured place_pose > in place — that negative coordinates are real
targets, and that a pixel goes through undistort + hand-eye homography.
"""
from __future__ import annotations

import unittest

from jaka_skill import atlas_bridge
from jaka_skill.config import SkillConfig
from jaka_skill.geometry import Homography
from jaka_skill.motion import MotionError

DESKTOP = 0.125
TRAVEL = 0.06

# Deploy values (robonix_manifest.local.yaml), so the pixel path is tested
# against the calibration that actually runs on this cell.
REAL_K = [[606.7899169921875, 0.0, 639.11328125],
          [0.0, 606.9068603515625, 358.1773681640625],
          [0.0, 0.0, 1.0]]
REAL_D = [-0.03052361309528351, 0.03451476991176605, 0.00041998623055405915,
          0.0002993865346070379, -0.01212006900459528, 0.0, 0.0, 0.0]
REAL_H = [[-0.0012014607200399041, 1.2334616258158349e-05, 0.9363192319869995],
          [-1.7852291421149857e-05, 0.0010392841650173068, 0.10209435969591141],
          [0.0001262559526367113, -0.0004248072800692171, 1.0]]


class FakeArm:
    """The slice of Arm that _run_place touches; records every waypoint."""

    def __init__(self, now=None):
        self.now = now
        self.waypoints = []
        self.gripper = []

    def wait_feedback_ready(self, stage, timeout_s):
        return None

    def end_pose_now(self):
        return (list(self.now), None) if self.now is not None else (None, None)

    def move_pose(self, xyz, rpy, *, stage, timeout_s):
        self.waypoints.append((list(xyz), list(rpy), stage))

    def set_gripper(self, width, *, stage):
        self.gripper.append(width)

    def settle_gripper(self, *, expect_open, stage):
        return True


def make_cfg(**overrides) -> SkillConfig:
    cfg = SkillConfig(desktop_height=DESKTOP, travel_offset=TRAVEL)
    for key, value in overrides.items():
        setattr(cfg, key, value)
    return cfg


class PlaceTarget(unittest.TestCase):
    def test_commanded_xy_is_used(self):
        arm = FakeArm(now=[9.0, 9.0, 9.0])
        msg = atlas_bridge._run_place(arm, make_cfg(), 0.15, -0.10)
        self.assertEqual(arm.waypoints[0][0], [0.15, -0.10, DESKTOP + TRAVEL])
        self.assertEqual(arm.waypoints[1][0], [0.15, -0.10, DESKTOP])
        self.assertEqual(arm.waypoints[2][0], [0.15, -0.10, DESKTOP + TRAVEL])
        self.assertEqual([w[2] for w in arm.waypoints],
                         ["place/approach", "place/descend", "place/lift"])
        self.assertIn("commanded xy", msg)

    def test_commanded_xy_beats_configured_place_pose(self):
        arm = FakeArm()
        cfg = make_cfg(place_pose=[0.2, 0.1, 0.125, 3.14159265, 0.0, 0.0])
        atlas_bridge._run_place(arm, cfg, 0.18, 0.05)
        self.assertEqual(arm.waypoints[1][0], [0.18, 0.05, DESKTOP])

    def test_negative_coordinates_are_a_target(self):
        arm = FakeArm()
        msg = atlas_bridge._run_place(arm, make_cfg(), -0.15, 0.18)
        self.assertEqual(arm.waypoints[1][0], [-0.15, 0.18, DESKTOP])
        self.assertIn("commanded xy", msg)

    def test_zero_target_falls_back_to_place_pose(self):
        arm = FakeArm(now=[9.0, 9.0, 9.0])
        cfg = make_cfg(place_pose=[-0.15, 0.18, 0.125, 3.14159265, 0.0, 0.0])
        msg = atlas_bridge._run_place(arm, cfg, 0.0, 0.0)
        self.assertEqual(arm.waypoints[0][0], [-0.15, 0.18, DESKTOP + TRAVEL])
        self.assertEqual(arm.waypoints[1][0], [-0.15, 0.18, DESKTOP])
        self.assertIn("place_pose", msg)

    def test_zero_target_without_place_pose_places_in_place(self):
        arm = FakeArm(now=[-0.1285, 0.2383, 0.185])
        msg = atlas_bridge._run_place(arm, make_cfg(), 0.0, 0.0)
        self.assertEqual(arm.waypoints[1][0], [-0.1285, 0.2383, DESKTOP])
        self.assertIn("in place", msg)

    def test_in_place_without_end_pose_is_an_error(self):
        with self.assertRaises(MotionError):
            atlas_bridge._run_place(FakeArm(), make_cfg(), 0.0, 0.0)

    def test_gripper_opened_with_configured_width(self):
        arm = FakeArm()
        atlas_bridge._run_place(arm, make_cfg(gripper_open_m=0.098), 0.2, 0.0)
        self.assertEqual(arm.gripper, [0.098])


class PlacePixelTarget(unittest.TestCase):
    """Pixel targets: undistort, then the hand-eye homography."""

    def test_pixel_maps_through_homography(self):
        # Zero distortion + a 1 mm/px homography makes the expectation exact.
        cfg = make_cfg(camera_matrix=REAL_K, dist_coeffs=[0.0] * 5)
        hom = Homography([[0.001, 0.0, 0.0], [0.0, 0.001, 0.0], [0.0, 0.0, 1.0]])
        arm = FakeArm()
        msg = atlas_bridge._run_place(arm, cfg, 0.0, 0.0, 100.0, 200.0,
                                      homography=hom)
        self.assertAlmostEqual(arm.waypoints[1][0][0], 0.100, places=6)
        self.assertAlmostEqual(arm.waypoints[1][0][1], 0.200, places=6)
        self.assertEqual(arm.waypoints[1][0][2], DESKTOP)
        self.assertIn("pixel", msg)

    def test_pixel_uses_deploy_calibration(self):
        # Raw (distorted) pixel of the taught place point, derived
        # independently of the code under test: invert the homography for
        # the undistorted pixel, then apply the lens distortion with
        # projectPoints (the forward camera model, i.e. the +distortion
        # direction — the inverse of undistortPoints). If place skipped the
        # undistort or ran it the wrong way, this lands ~3 mm off.
        import cv2
        import numpy as np

        h_inv = np.linalg.inv(np.asarray(REAL_H, dtype=np.float64))
        p = h_inv @ np.array([-0.15, 0.18, 1.0])
        p /= p[2]
        k = np.asarray(REAL_K, dtype=np.float64)
        d = np.asarray(REAL_D, dtype=np.float64)
        ray = np.linalg.solve(k, np.array([p[0], p[1], 1.0])) * 0.2
        raw = cv2.projectPoints(ray.reshape(1, 1, 3), np.zeros(3),
                                np.zeros(3), k, d)[0].reshape(2)

        arm = FakeArm()
        atlas_bridge._run_place(
            arm, make_cfg(camera_matrix=REAL_K, dist_coeffs=REAL_D),
            0.0, 0.0, float(raw[0]), float(raw[1]),
            homography=Homography(REAL_H))
        self.assertAlmostEqual(arm.waypoints[1][0][0], -0.150, delta=0.001)
        self.assertAlmostEqual(arm.waypoints[1][0][1], 0.180, delta=0.001)

    def test_pixel_and_metres_together_is_an_error(self):
        arm = FakeArm()
        with self.assertRaises(MotionError) as ctx:
            atlas_bridge._run_place(arm, make_cfg(), 0.1, 0.1, 500.0, 100.0,
                                    homography=Homography(REAL_H))
        self.assertIn("ambiguous", str(ctx.exception))
        self.assertEqual(arm.waypoints, [])

    def test_pixel_without_homography_is_an_error(self):
        cfg = make_cfg(camera_matrix=REAL_K, dist_coeffs=REAL_D)
        with self.assertRaises(MotionError) as ctx:
            atlas_bridge._run_place(FakeArm(), cfg, 0.0, 0.0, 640.0, 100.0,
                                    homography=Homography(None))
        self.assertIn("homography_matrix", str(ctx.exception))

    def test_pixel_without_intrinsics_is_an_error(self):
        with self.assertRaises(MotionError) as ctx:
            atlas_bridge._run_place(FakeArm(), make_cfg(), 0.0, 0.0,
                                    640.0, 100.0,
                                    homography=Homography(REAL_H))
        self.assertIn("camera_matrix", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
