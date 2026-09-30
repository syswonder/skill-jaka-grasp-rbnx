# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Config parsing for the jaka grab/release skill.

Philosophy (same as the piper pick skill's config.py): a bad config value
must never fail CMD_INIT — every reader warns and falls back to a default.
The one exception is the two poses (hover/grasp): a missing or degenerate
pose has no sane default, so it is collected into `SkillConfig.config_error`
and surfaced on every tool call (atlas_bridge._require_active) instead of
failing the lifecycle (an Err from on_activate is a one-way door to ERROR).
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Any, Optional

LOG_PREFIX = "[jaka_skill]"

# Must match primitive-jaka-rbnx/jaka_arm/driver.py constants.
GRIPPER_STROKE_M = 0.098
GRIPPER_OPEN_THRESHOLD_M = 0.5 * GRIPPER_STROKE_M


def _num(cfg: dict, key: str, default: float,
         *, minimum: Optional[float] = None,
         maximum: Optional[float] = None) -> float:
    raw = cfg.get(key, default)
    try:
        value = float(raw)
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: {raw!r} not numeric; "
              f"using {default}", flush=True)
        return float(default)
    if not math.isfinite(value):
        print(f"{LOG_PREFIX} config {key}: non-finite; using {default}",
              flush=True)
        return float(default)
    if minimum is not None and value < minimum:
        print(f"{LOG_PREFIX} config {key}: {value} < {minimum}; "
              f"clamping to {minimum}", flush=True)
        return float(minimum)
    if maximum is not None and value > maximum:
        print(f"{LOG_PREFIX} config {key}: {value} > {maximum}; "
              f"clamping to {maximum}", flush=True)
        return float(maximum)
    return value


def _int(cfg: dict, key: str, default: int, *, minimum: int = 1) -> int:
    raw = cfg.get(key, default)
    try:
        value = int(raw)
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: {raw!r} not an integer; "
              f"using {default}", flush=True)
        return default
    if value < minimum:
        print(f"{LOG_PREFIX} config {key}: {value} < {minimum}; "
              f"clamping to {minimum}", flush=True)
        return minimum
    return value


def _flag(cfg: dict, key: str, default: bool) -> bool:
    raw = cfg.get(key, default)
    if isinstance(raw, bool):
        return raw
    if isinstance(raw, str):
        return raw.strip().lower() in ("1", "true", "yes", "on")
    return bool(raw)


def _text(cfg: dict, key: str, default: str) -> str:
    raw = cfg.get(key, default)
    return str(raw).strip() or default


def _matrix(cfg: dict, key: str, rows: int = 3, cols: int = 3
            ) -> Optional[list[list[float]]]:
    """rows×cols float matrix (homography / camera intrinsics). None on any
    malformation (wrong shape or non-numeric)."""
    raw = cfg.get(key)
    if raw is None:
        return None
    try:
        parsed = [[float(x) for x in row] for row in raw]
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: not a {rows}x{cols} matrix",
              flush=True)
        return None
    if len(parsed) != rows or any(len(r) != cols for r in parsed):
        print(f"{LOG_PREFIX} config {key}: not a {rows}x{cols} matrix",
              flush=True)
        return None
    return parsed


def _floats(cfg: dict, key: str) -> Optional[list[float]]:
    """Float list (distortion coeffs). None on any malformation."""
    raw = cfg.get(key)
    if raw is None:
        return None
    try:
        return [float(x) for x in raw]
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: not a list of numbers", flush=True)
        return None


def _choice(cfg: dict, key: str, allowed: tuple, default: str) -> str:
    """One of `allowed` (case-insensitive); anything else warns -> default."""
    raw = cfg.get(key, default)
    value = str(raw).strip().lower()
    if value not in allowed:
        print(f"{LOG_PREFIX} config {key}: {raw!r} not one of {list(allowed)}; "
              f"using {default!r}", flush=True)
        return default
    return value


def _aliases(cfg: dict, key: str) -> dict:
    """str -> str map (user phrasing -> yolo class name). Bad entries are
    dropped with a warning, never fatal."""
    raw = cfg.get(key)
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        print(f"{LOG_PREFIX} config {key}: expected a map "
              f"(e.g. {{胡萝卜: carrot}}); ignored", flush=True)
        return {}
    out: dict = {}
    for k, v in raw.items():
        ks, vs = str(k).strip(), str(v).strip()
        if ks and vs:
            out[ks] = vs
    return out


def _rpy3(cfg: dict, key: str, default: list[float]) -> list[float]:
    """[rx, ry, rz] radians. Falls back to `default` on any malformation."""
    raw = cfg.get(key)
    if raw is None:
        return list(default)
    if not isinstance(raw, (list, tuple)) or len(raw) != 3:
        print(f"{LOG_PREFIX} config {key}: expected [rx, ry, rz]; "
              f"using {default}", flush=True)
        return list(default)
    try:
        vals = [float(v) for v in raw]
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: non-numeric; using {default}",
              flush=True)
        return list(default)
    if not all(math.isfinite(v) for v in vals):
        print(f"{LOG_PREFIX} config {key}: non-finite; using {default}",
              flush=True)
        return list(default)
    return vals


def _pose6(cfg: dict, key: str) -> Optional[list[float]]:
    """[x, y, z, rx, ry, rz] — metres + RADIANS. None on any malformation."""
    raw = cfg.get(key)
    if raw is None:
        return None
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        print(f"{LOG_PREFIX} config {key}: expected [x, y, z, rx, ry, rz] "
              f"(6 numbers); got {raw!r}", flush=True)
        return None
    vals: list[float] = []
    for v in raw:
        try:
            vals.append(float(v))
        except (TypeError, ValueError):
            print(f"{LOG_PREFIX} config {key}: element {v!r} not numeric",
                  flush=True)
            return None
    if not all(math.isfinite(v) for v in vals):
        print(f"{LOG_PREFIX} config {key}: non-finite elements", flush=True)
        return None
    for name, v in zip(("rx", "ry", "rz"), vals[3:]):
        if abs(v) > 2 * math.pi + 0.1:
            print(f"{LOG_PREFIX} config {key}: {name}={v} looks like "
                  f"DEGREES — poses take radians", flush=True)
    return vals


def _angles6(cfg: dict, key: str, default: list[float]) -> list[float]:
    """Six joint angles in RADIANS. Falls back to `default` on any
    malformation."""
    raw = cfg.get(key)
    if raw is None:
        return list(default)
    if not isinstance(raw, (list, tuple)) or len(raw) != 6:
        print(f"{LOG_PREFIX} config {key}: expected 6 joint angles "
              f"(radians); using {default}", flush=True)
        return list(default)
    try:
        vals = [float(v) for v in raw]
    except (TypeError, ValueError):
        print(f"{LOG_PREFIX} config {key}: non-numeric; using {default}",
              flush=True)
        return list(default)
    if not all(math.isfinite(v) for v in vals):
        print(f"{LOG_PREFIX} config {key}: non-finite; using {default}",
              flush=True)
        return list(default)
    return vals


@dataclass
class SkillConfig:
    arm_provider_id: str = "jaka_arm"
    hover_pose: Optional[list[float]] = None
    grasp_pose: Optional[list[float]] = None
    gripper_open_m: float = 0.098
    gripper_close_m: float = 0.0
    open_before_approach: bool = True
    gripper_settle_s: float = 1.5
    gripper_repeats: int = 5
    gripper_publish_period_s: float = 0.1
    feedback_timeout_s: float = 2.0
    reach_tolerance_m: float = 0.002
    pose_timeout_s: float = 20.0
    poll_s: float = 0.05
    min_grasp_descent_m: float = 0.005
    topics: dict = field(default_factory=dict)
    # ── home (复位): joint-space target + reach check ──
    home_joints: list[float] = field(default_factory=lambda: [0.0] * 6)
    joint_reach_tolerance_rad: float = 0.02
    # ── place (放置): where a held object is set down. Optional — unset
    # means `place` sets the object down IN PLACE (current XY at
    # desktop_height, grasp_rpy); set to carry it to a configured pose
    # instead (NOT a config_error either way: grab/release/home/pick are
    # unaffected). ──
    place_pose: Optional[list[float]] = None
    # ── fixed_grasp (demo): fixed-pose pick-and-place. Optional like
    # pick's calibration — unset disables only `fixed_grasp` (checked at
    # tool-call time, NOT a config_error: grab/release/pick must keep
    # working). ──
    fixed_grasp_pose: Optional[list[float]] = None
    fixed_place_pose: Optional[list[float]] = None
    # ── pick (perception) — separate from grab so grab works uncalibrated ──
    camera_provider_id: str = "orbbec_camera"
    camera_topic: str = ""
    camera_matrix: Optional[list[list[float]]] = None
    dist_coeffs: Optional[list[float]] = None
    homography_matrix: Optional[list[list[float]]] = None
    desktop_height: float = 0.125
    travel_offset: float = 0.15
    grasp_rpy: list[float] = field(default_factory=lambda: [math.pi, 0.0, 0.0])
    # ── detection backend: "vlm" (open-vocabulary, axis-aligned bbox) or
    # "yolo" (in-process yolo11-obb; 5th bbox element carries rotation) ──
    detect_scheme: str = "vlm"
    yolo_model_path: str = ""
    yolo_conf: float = 0.6
    yolo_iou: float = 0.45
    yolo_two_frame_interval_s: float = 0.7
    yolo_two_frame_iou: float = 0.3
    yolo_class_aliases: dict = field(default_factory=dict)
    # rz = radians(grasp_yaw_offset_deg - degrees(yaw)) for rotated targets
    # (AIIT jaka parity: their jaka_catch_rot_offset_deg=-90, same rig)
    grasp_yaw_offset_deg: float = -90.0
    vlm_base_url: str = ""
    vlm_api_key: str = ""
    vlm_model: str = ""
    vlm_timeout_s: float = 60.0
    frame_max_age_s: float = 1.0
    frame_wait_s: float = 5.0
    config_error: str = ""

    @property
    def pick_error(self) -> str:
        """Why `pick` is unavailable (empty = ready). Kept distinct from
        config_error: grab needs neither calibration nor a detector, so a
        pick-only gap must not block grab."""
        missing = []
        if self.homography_matrix is None:
            missing.append("homography_matrix (2D hand-eye calibration)")
        if self.detect_scheme == "yolo":
            path = os.path.expanduser(self.yolo_model_path) \
                if self.yolo_model_path else ""
            if not path:
                missing.append("yolo_model_path (YOLO obb weights)")
            elif not os.path.isfile(path):
                missing.append(f"yolo_model_path not found: {path}")
        elif not (self.vlm_base_url and self.vlm_api_key and self.vlm_model):
            missing.append("vlm_base_url/vlm_api_key/vlm_model")
        return "; ".join(missing)

    @property
    def fixed_grasp_error(self) -> str:
        """Why `fixed_grasp` is unavailable (empty = ready). Same pattern
        as pick_error: a fixed_grasp-only gap must not block
        grab/release/pick."""
        missing = []
        if self.fixed_grasp_pose is None:
            missing.append(
                "fixed_grasp_pose (where the object is picked up)")
        if self.fixed_place_pose is None:
            missing.append(
                "fixed_place_pose (where the object is set down)")
        return "; ".join(missing)

    @classmethod
    def from_cfg(cls, cfg: dict) -> "SkillConfig":
        topics_raw = cfg.get("topics")
        topics = {}
        if isinstance(topics_raw, dict):
            for key in ("joint_states", "end_pose",
                        "joint_command", "pos_command"):
                if topics_raw.get(key):
                    topics[key] = str(topics_raw[key])
        elif topics_raw is not None:
            print(f"{LOG_PREFIX} config topics: expected a map; ignored",
                  flush=True)

        parsed = cls(
            arm_provider_id=_text(cfg, "arm_provider_id", "jaka_arm"),
            hover_pose=_pose6(cfg, "hover_pose"),
            grasp_pose=_pose6(cfg, "grasp_pose"),
            gripper_open_m=_num(cfg, "gripper_open_m", 0.098,
                                minimum=0.0, maximum=GRIPPER_STROKE_M),
            gripper_close_m=_num(cfg, "gripper_close_m", 0.0,
                                 minimum=0.0, maximum=GRIPPER_STROKE_M),
            open_before_approach=_flag(cfg, "open_before_approach", True),
            gripper_settle_s=_num(cfg, "gripper_settle_s", 1.5, minimum=0.0),
            gripper_repeats=_int(cfg, "gripper_repeats", 5),
            gripper_publish_period_s=_num(
                cfg, "gripper_publish_period_s", 0.1, minimum=0.01),
            feedback_timeout_s=_num(cfg, "feedback_timeout_s", 2.0,
                                    minimum=0.1),
            reach_tolerance_m=_num(cfg, "reach_tolerance_m", 0.002,
                                   minimum=0.0005),
            pose_timeout_s=_num(cfg, "pose_timeout_s", 20.0, minimum=1.0),
            poll_s=_num(cfg, "poll_s", 0.05, minimum=0.01),
            min_grasp_descent_m=_num(cfg, "min_grasp_descent_m", 0.005,
                                     minimum=0.001),
            topics=topics,
            home_joints=_angles6(cfg, "home_joints", [0.0] * 6),
            joint_reach_tolerance_rad=_num(
                cfg, "joint_reach_tolerance_rad", 0.02, minimum=0.001),
            place_pose=_pose6(cfg, "place_pose"),
            fixed_grasp_pose=_pose6(cfg, "fixed_grasp_pose"),
            fixed_place_pose=_pose6(cfg, "fixed_place_pose"),
            camera_provider_id=_text(cfg, "camera_provider_id",
                                     "orbbec_camera"),
            camera_topic=_text(cfg, "camera_topic", ""),
            camera_matrix=_matrix(cfg, "camera_matrix"),
            dist_coeffs=_floats(cfg, "dist_coeffs"),
            homography_matrix=_matrix(cfg, "homography_matrix"),
            desktop_height=_num(cfg, "desktop_height", 0.125),
            travel_offset=_num(cfg, "travel_offset", 0.15, minimum=0.0),
            grasp_rpy=_rpy3(cfg, "grasp_rpy", [math.pi, 0.0, 0.0]),
            detect_scheme=_choice(cfg, "detect_scheme", ("vlm", "yolo"), "vlm"),
            yolo_model_path=_text(cfg, "yolo_model_path", ""),
            yolo_conf=_num(cfg, "yolo_conf", 0.6, minimum=0.01, maximum=1.0),
            yolo_iou=_num(cfg, "yolo_iou", 0.45, minimum=0.01, maximum=1.0),
            yolo_two_frame_interval_s=_num(
                cfg, "yolo_two_frame_interval_s", 0.7, minimum=0.0),
            yolo_two_frame_iou=_num(
                cfg, "yolo_two_frame_iou", 0.3, minimum=0.0, maximum=1.0),
            yolo_class_aliases=_aliases(cfg, "yolo_class_aliases"),
            grasp_yaw_offset_deg=_num(cfg, "grasp_yaw_offset_deg", -90.0),
            vlm_base_url=_text(cfg, "vlm_base_url", ""),
            vlm_api_key=_text(cfg, "vlm_api_key", ""),
            vlm_model=_text(cfg, "vlm_model", ""),
            vlm_timeout_s=_num(cfg, "vlm_timeout_s", 60.0, minimum=1.0),
            frame_max_age_s=_num(cfg, "frame_max_age_s", 1.0, minimum=0.1),
            frame_wait_s=_num(cfg, "frame_wait_s", 5.0, minimum=0.0),
        )

        errors: list[str] = []
        if parsed.hover_pose is None:
            errors.append("hover_pose missing or not [x, y, z, rx, ry, rz]")
        if parsed.grasp_pose is None:
            errors.append("grasp_pose missing or not [x, y, z, rx, ry, rz]")
        if parsed.hover_pose is not None and parsed.grasp_pose is not None:
            dist = math.dist(parsed.hover_pose, parsed.grasp_pose)
            if dist < parsed.min_grasp_descent_m:
                errors.append(
                    f"hover_pose and grasp_pose are {dist * 1000:.1f} mm "
                    f"apart (< min_grasp_descent_m "
                    f"{parsed.min_grasp_descent_m * 1000:.0f} mm)")
            if parsed.grasp_pose[2] >= parsed.hover_pose[2]:
                print(f"{LOG_PREFIX} config: grasp z ({parsed.grasp_pose[2]:.3f}) "
                      f">= hover z ({parsed.hover_pose[2]:.3f}) — descending "
                      f"flow expected", flush=True)
        if parsed.gripper_open_m < GRIPPER_OPEN_THRESHOLD_M:
            errors.append(
                f"gripper_open_m {parsed.gripper_open_m} is below the binary "
                f"open threshold {GRIPPER_OPEN_THRESHOLD_M} — the provider "
                f"would CLOSE the gripper on release")
        parsed.config_error = "; ".join(errors)
        return parsed
