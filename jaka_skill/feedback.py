# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""End-effector feedback translation for the JAKA skill.

The same skill package runs against two body deployments:

  * JAKA Mini + TG-9801 gripper — the primitive publishes a real
    `gripper_joint` position (metres) in `joint_states`;
  * JAKA Mini + LinkerHand O6 hand — the primitive publishes the six O6
    hand joints (rad, URDF convention) and no `gripper_joint`.

`width_from_joint_state` is told which body it is running against by the
deploy manifest's required `end_effector: gripper|hand` skill config and
returns a width-like opening in metres, so every existing grasp-verdict
threshold keeps its meaning without a per-deployment skill branch. There is
no auto-detection: an explicit mode makes a wrong manifest loud (the skill
reports a config error) instead of silently reading the wrong feedback.

The hand reconstruction is deliberately hardware-specific: it uses the
LinkerHand O6 joint names and URDF upper limits, plus anchors measured on
the real hand (2026-10-16, two identical empty cycles). A different hand
needs its own mapping here.
"""
from __future__ import annotations

import math

GRIPPER_STROKE_M = 0.098  # TG-9801 stroke, also the hand's width-like scale

GRIPPER_JOINT_NAME = "gripper_joint"

# O6 grasp fingers used for the width-like opening: thumb flexion +
# index/middle/ring flexion (the AIIT grasp口径; pinky excluded by their
# calibration). Each entry is (joint name, URDF upper limit in rad). The
# provider writes hand joints as `(1 - raw/255) * upper`, so openness
# (1 = straight, 0 = fully flexed) is `1 - rad/upper`.
HAND_GRASP_JOINTS = (
    ("thumb_cmc_pitch", 0.52),
    ("index_mcp_pitch", 1.57),
    ("middle_mcp_pitch", 1.57),
    ("ring_mcp_pitch", 1.57),
)

# Measured on the real O6 (2026-10-16, two identical empty cycles): the open
# pose reads mean openness 0.923, an empty close settles at 0.119. The
# affine map anchors empty close at 0 m and full open at the stroke, so
# `GRASP_CLOSED_THRESHOLD_M` and the open/close command split keep working
# unchanged. Re-measure if the hand mechanics or the URDF limits change.
HAND_OPEN_OPENNESS = 0.923
HAND_EMPTY_CLOSE_OPENNESS = 0.119


def _finite(value) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def hand_openness(names, positions):
    """Mean O6 grasp-finger openness from a joint_states sample.

    1 = straight/extended, 0 = fully flexed. Returns None when any grasp
    finger is missing or non-finite (the caller keeps the previous width
    rather than fabricating one).
    """
    idx = {name: i for i, name in enumerate(names)}
    fractions = []
    for name, upper in HAND_GRASP_JOINTS:
        i = idx.get(name)
        if i is None or i >= len(positions):
            return None
        value = positions[i]
        if not _finite(value):
            return None
        fractions.append(1.0 - float(value) / upper)
    return sum(fractions) / len(fractions)


def hand_width_m(openness: float, stroke_m: float = GRIPPER_STROKE_M) -> float:
    """Map hand openness to the TG-9801 width scale in metres."""
    span = HAND_OPEN_OPENNESS - HAND_EMPTY_CLOSE_OPENNESS
    if span <= 0.0:
        return 0.0
    widened = (float(openness) - HAND_EMPTY_CLOSE_OPENNESS) / span
    return float(stroke_m) * min(max(widened, 0.0), 1.0)


def width_from_joint_state(
    names, positions, end_effector: str, stroke_m: float = GRIPPER_STROKE_M
):
    """Width-like opening (m) from one joint_states sample, or None.

    `end_effector` is the deploy manifest's explicit `gripper` or `hand`:

    * `gripper` — the `gripper_joint` position (m) as-is; hand joints are
      ignored (a wrong manifest yields no width rather than a false one);
    * `hand` — the O6 grasp fingers reconstructed through `hand_openness` +
      `hand_width_m`; a stray `gripper_joint` is ignored.

    Anything else (including arm-only feedback) returns None, so the caller
    keeps its last width. The configured mode is validated at config parse
    time; this function only refuses unknown values.
    """
    idx = {name: i for i, name in enumerate(names)}
    if end_effector == "gripper":
        gripper_i = idx.get(GRIPPER_JOINT_NAME)
        if gripper_i is None or gripper_i >= len(positions):
            return None
        value = positions[gripper_i]
        return float(value) if _finite(value) else None
    if end_effector == "hand":
        openness = hand_openness(names, positions)
        if openness is None:
            return None
        return hand_width_m(openness, stroke_m)
    return None
