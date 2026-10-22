# SPDX-License-Identifier: Apache-2.0
"""Unit tests for the explicit dual end-effector feedback translation (no ROS)."""
from __future__ import annotations

import math
import unittest

from jaka_skill import feedback

ARM = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HAND = ["thumb_cmc_pitch", "thumb_cmc_yaw", "index_mcp_pitch",
        "middle_mcp_pitch", "ring_mcp_pitch", "pinky_mcp_pitch"]


def hand_pose(thumb_open_raw=229, index_raw=229, middle_raw=229,
              ring_raw=254, pinky_raw=254, yaw_raw=50):
    """Build hand joints (rad) from raw O6 register values (255 = extended)."""
    raws = [thumb_open_raw, yaw_raw, index_raw, middle_raw, ring_raw, pinky_raw]
    uppers = [0.52, 1.54, 1.57, 1.57, 1.57, 1.57]
    return [(1.0 - raw / 255.0) * upper for raw, upper in zip(raws, uppers)]


class GripperModeTests(unittest.TestCase):
    def test_gripper_width_used_as_is(self):
        names = ARM + ["gripper_joint"]
        positions = [0.0] * 6 + [0.073]
        self.assertAlmostEqual(
            feedback.width_from_joint_state(names, positions, "gripper"), 0.073)

    def test_gripper_mode_ignores_hand_joints(self):
        names = ARM + HAND
        positions = [0.0] * 6 + hand_pose()
        self.assertIsNone(
            feedback.width_from_joint_state(names, positions, "gripper"))

    def test_non_finite_gripper_returns_none(self):
        names = ARM + ["gripper_joint"]
        positions = [0.0] * 6 + [math.nan]
        self.assertIsNone(
            feedback.width_from_joint_state(names, positions, "gripper"))


class HandModeTests(unittest.TestCase):
    def test_hand_open_pose_reconstructs_full_width(self):
        names = ARM + HAND
        positions = [0.0] * 6 + hand_pose()
        width = feedback.width_from_joint_state(names, positions, "hand")
        self.assertIsNotNone(width)
        self.assertAlmostEqual(width, feedback.GRIPPER_STROKE_M, places=3)

    def test_hand_empty_close_anchor_is_zero(self):
        self.assertAlmostEqual(feedback.hand_width_m(
            feedback.HAND_EMPTY_CLOSE_OPENNESS), 0.0)

    def test_held_object_maps_between(self):
        width = feedback.hand_width_m(0.5)
        self.assertGreater(width, 0.0)
        self.assertLess(width, feedback.GRIPPER_STROKE_M)

    def test_hand_mode_ignores_gripper_joint(self):
        names = ARM + ["gripper_joint"] + HAND
        positions = [0.0] * 6 + [0.02] + hand_pose()
        width = feedback.width_from_joint_state(names, positions, "hand")
        self.assertAlmostEqual(width, feedback.GRIPPER_STROKE_M, places=3)

    def test_missing_grasp_finger_returns_none(self):
        names = ARM + [n for n in HAND if n != "ring_mcp_pitch"]
        positions = [0.0] * 6 + hand_pose()[:4]
        self.assertIsNone(
            feedback.width_from_joint_state(names, positions, "hand"))

    def test_openness_clamps_to_width_scale(self):
        self.assertEqual(feedback.hand_width_m(1.0),
                         feedback.GRIPPER_STROKE_M)
        self.assertEqual(feedback.hand_width_m(0.0), 0.0)


class SharedTests(unittest.TestCase):
    def test_arm_only_feedback_returns_none(self):
        self.assertIsNone(
            feedback.width_from_joint_state(ARM, [0.0] * 6, "gripper"))
        self.assertIsNone(
            feedback.width_from_joint_state(ARM, [0.0] * 6, "hand"))

    def test_unknown_mode_returns_none(self):
        names = ARM + ["gripper_joint"] + HAND
        positions = [0.0] * 6 + [0.05] + hand_pose()
        for mode in ("auto", "", "none", "Gripper"):
            self.assertIsNone(
                feedback.width_from_joint_state(names, positions, mode))


if __name__ == "__main__":
    unittest.main()
