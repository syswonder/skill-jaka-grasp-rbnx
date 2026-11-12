# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Unit tests for the gripper/state grasp verdict (no ROS traffic).

The close verdict prefers the device's own contact report from
`robonix/primitive/gripper/state` and falls back to the historical width
heuristic. These tests pin the mapping, the freshness gate (a round stamped
before the close cannot judge it) and the fallback contract.
"""
from __future__ import annotations

import time
import unittest
from types import SimpleNamespace

from jaka_skill import atlas_bridge
from jaka_skill.feedback import (GRASP_HOLDING, GRASP_NOT_HOLDING,
                                 GRASP_SLIPPING, GRASP_UNKNOWN,
                                 grasp_verdict_from_state)
from jaka_skill.motion import GRIPPER_STATE_TIMEOUT_S, Arm


class TestVerdictMapping(unittest.TestCase):
    def test_holding(self):
        kind, detail = grasp_verdict_from_state(GRASP_HOLDING, 0.031)
        self.assertEqual(kind, "holding")
        self.assertIn("established grip", detail)

    def test_slipping(self):
        self.assertEqual(
            grasp_verdict_from_state(GRASP_SLIPPING, 0.031)[0], "slipping")

    def test_not_holding_and_fully_closed_is_empty(self):
        kind, detail = grasp_verdict_from_state(GRASP_NOT_HOLDING, 0.0)
        self.assertEqual(kind, "empty")
        self.assertIn("nothing between the fingers", detail)

    def test_not_holding_with_a_wide_opening_is_ambiguous(self):
        kind, detail = grasp_verdict_from_state(GRASP_NOT_HOLDING, 0.031)
        self.assertEqual(kind, "ambiguous")
        self.assertIn("31.0 mm", detail)

    def test_not_holding_without_a_width_is_ambiguous(self):
        kind, _ = grasp_verdict_from_state(GRASP_NOT_HOLDING, None)
        self.assertEqual(kind, "ambiguous")

    def test_unknown_register_has_no_verdict(self):
        for value in (GRASP_UNKNOWN, None, 9):
            self.assertEqual(grasp_verdict_from_state(value, 0.02)[0], "unknown")


class StubArm:
    """The slice of Arm that `_close_verdict` touches."""

    def __init__(self, state=None, width=None, available=True):
        self._state = state
        self._width = width
        self.gripper_state_available = available
        self.settled = []
        self.width_waits = []
        self.state_wait = None

    def wait_gripper_state(self, *, since, timeout_s):
        self.state_wait = (since, timeout_s)
        return self._state

    def settle_gripper(self, *, expect_open, stage):
        self.settled.append((expect_open, stage))
        return ""

    def wait_gripper_settled(self, *, since):
        self.width_waits.append(since)
        return self._width


class TestCloseVerdict(unittest.TestCase):
    def test_holding_is_success_without_a_width_wait(self):
        arm = StubArm(state={"grasp_state": GRASP_HOLDING, "width_m": 0.03})
        self.assertEqual(atlas_bridge._close_verdict(arm, "grasp", 10.0),
                         (None, ""))
        self.assertEqual(arm.width_waits, [])
        self.assertEqual(arm.state_wait, (10.0, GRIPPER_STATE_TIMEOUT_S))

    def test_slipping_counts_as_grasped_but_is_flagged(self):
        arm = StubArm(state={"grasp_state": GRASP_SLIPPING, "width_m": 0.03})
        problem, note = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIsNone(problem)
        self.assertIn("slipping", note)

    def test_empty_close_drives_the_retry_path(self):
        arm = StubArm(state={"grasp_state": GRASP_NOT_HOLDING, "width_m": 0.0})
        problem, note = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIn("nothing between the fingers", problem)
        self.assertEqual(note, "")

    def test_not_holding_with_a_wide_opening_keeps_assume_grasped(self):
        arm = StubArm(state={"grasp_state": GRASP_NOT_HOLDING, "width_m": 0.03})
        problem, note = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIsNone(problem)
        self.assertIn("unverified", note)

    def test_no_state_falls_back_to_the_width_rule(self):
        arm = StubArm(state=None, width=0.0)
        problem, _ = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIn("fully closed", problem)
        self.assertEqual(arm.settled, [(False, "grasp/close")])
        self.assertEqual(arm.width_waits, [10.0])

    def test_no_state_and_no_width_is_unverified_not_failure(self):
        arm = StubArm(state=None, width=None)
        problem, note = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIsNone(problem)
        self.assertIn("no settled gripper feedback", note)

    def test_hand_body_never_waits_for_state(self):
        arm = StubArm(state={"grasp_state": GRASP_HOLDING}, width=0.0,
                      available=False)
        problem, _ = atlas_bridge._close_verdict(arm, "grasp", 10.0)
        self.assertIn("fully closed", problem)
        self.assertIsNone(arm.state_wait)


class TestFreshnessGate(unittest.TestCase):
    def _arm(self):
        return Arm(SimpleNamespace(end_effector="gripper", poll_s=0.005))

    def test_no_stream_reports_unavailable(self):
        self.assertFalse(self._arm().gripper_state_available)

    def test_sample_after_the_command_is_returned(self):
        arm = self._arm()
        arm._t_gripper_state = ("/jaka/gripper/state", "reliable")
        arm._gripper_state = {"grasp_state": GRASP_HOLDING, "motion_state": 2,
                              "width_m": 0.03}
        arm._gripper_state_mono = 100.0
        self.assertTrue(arm.gripper_state_available)
        self.assertEqual(
            arm.wait_gripper_state(since=99.0, timeout_s=0.05)["grasp_state"],
            GRASP_HOLDING)

    def test_sample_before_the_command_is_rejected(self):
        arm = self._arm()
        arm._gripper_state = {"grasp_state": GRASP_HOLDING, "motion_state": 2,
                              "width_m": 0.03}
        arm._gripper_state_mono = 100.0
        self.assertIsNone(arm.wait_gripper_state(since=101.0, timeout_s=0.02))

    def test_stamp_is_converted_to_the_monotonic_clock(self):
        arm = self._arm()
        wall = time.time() - 2.5  # acquired 2.5 s ago
        msg = SimpleNamespace(
            stamp=SimpleNamespace(sec=int(wall),
                                  nanosec=int((wall - int(wall)) * 1e9)),
            opening_width_valid=True, opening_width_m=0.031,
            grasp_state=GRASP_HOLDING, motion_state=2)
        arm._on_gripper_state(msg)
        state, mono = arm.gripper_state_now()
        self.assertEqual(state["grasp_state"], GRASP_HOLDING)
        self.assertAlmostEqual(time.monotonic() - mono, 2.5, delta=0.2)

    def test_invalid_width_is_stored_as_none(self):
        arm = self._arm()
        msg = SimpleNamespace(
            stamp=SimpleNamespace(sec=0, nanosec=0),
            opening_width_valid=False, opening_width_m=0.0,
            grasp_state=GRASP_NOT_HOLDING, motion_state=0)
        arm._on_gripper_state(msg)
        self.assertIsNone(arm.gripper_state_now()[0]["width_m"])


if __name__ == "__main__":
    unittest.main()
