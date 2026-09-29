#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""jaka_skill — minimal grab/release skill for the JAKA Mini.

Owns `robonix/skill/jaka/*`. No perception, no upstream gRPC services:
both tools drive the `jaka_arm` primitive through its ROS 2 contracts
(pos_command / joint_command), resolved by topic name via atlas — the
skill hard-codes nothing.

Lifecycle (per Robonix developer guide §5 — skills are different):
    on_init       — parse config, build Arm; never touch atlas or rclpy.
                    Config problems are collected, NOT fatal (an Err from
                    on_activate is a one-way door to ERROR).
    on_activate   — resolve the four arm topics (bounded retry, then
                    config `topics:` override, then /jaka/... defaults),
                    start the ROS bridge. The executor sends this on the
                    FIRST tool call.
    on_deactivate — stop the bridge, drop resolutions.
"""
from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from typing import Any, Optional

from robonix_api import ATLAS, Err, Ok, Skill

from jaka_mcp import (Egg_Request, Egg_Response, Grab_Request, Grab_Response,
                      Pick_Request, Pick_Response, Release_Request,
                      Release_Response)

from .camera import CameraFrames
from .config import SkillConfig
from .detect import DetectConfig, Detector
from .geometry import Homography, bbox_yaw_rad
from .motion import Arm, MotionError

logging.basicConfig(
    level=os.environ.get("JAKA_SKILL_LOG_LEVEL", "INFO"),
    format="[jaka_skill] %(message)s",
)
log = logging.getLogger("jaka_skill")

# Provider id MUST match the deploy manifest's `skill: - name:` entry
# (rbnx passes it as RBNX_INSTANCE_NAME, same as jaka_arm).
jaka_skill = Skill(
    id=os.environ.get("RBNX_INSTANCE_NAME", "jaka"),
    namespace="robonix/skill/jaka",
)

_PRIMITIVE_TOPICS = {
    "joint_states": "robonix/primitive/arm/joint_states",
    "end_pose": "robonix/primitive/arm/end_pose",
    "joint_command": "robonix/primitive/arm/joint_command",
    "pos_command": "robonix/primitive/arm/pos_command",
}
_DEFAULT_TOPICS = {
    "joint_states": "/jaka/joint_states",
    "end_pose": "/jaka/end_pose",
    "joint_command": "/jaka/joint_command",
    "pos_command": "/jaka/pos_command",
}
# jaka_arm declares every topic qos="reliable" (jaka_arm/main.py).
_FALLBACK_QOS = "reliable"

# Camera RGB contract (pick path). Resolved via atlas, pinned to
# camera_provider_id; config `camera_topic` overrides; no default topic —
# unresolved means pick reports "camera unavailable" rather than guessing.
_CAMERA_RGB_CONTRACT = "robonix/primitive/camera/rgb"

_state_lock = threading.Lock()
_config: Optional[SkillConfig] = None
_arm: Optional[Arm] = None
_frames: Optional[CameraFrames] = None
_detector: Optional[Detector] = None
_homography: Optional[Homography] = None
_resolved = False  # on_activate completed


# ── guards / resolution ─────────────────────────────────────────────────────

def _require_active() -> SkillConfig:
    if not _resolved or _arm is None or _config is None:
        raise RuntimeError(
            "jaka skill not active — the executor activates it on the first "
            "tool call; if this persists, check `rbnx logs -t jaka`")
    if _config.config_error:
        raise RuntimeError(
            f"jaka skill config invalid: {_config.config_error} — fix the "
            f"deploy manifest's `skill: - name: jaka` config block and "
            f"reboot")
    return _config


def _connect_primitive(contract_id: str, provider_id: str):
    """Resolve one arm contract via atlas (pinned provider). None on any
    failure — resolution falls back, it must never be fatal."""
    try:
        cap = ATLAS.find_unique_capability(
            contract_id=contract_id, transport="ros2",
            provider_id=provider_id)
    except Exception as exc:  # noqa: BLE001
        log.debug("contract %s not resolvable via atlas: %s",
                  contract_id, exc)
        return None
    try:
        return jaka_skill.connect_capability(cap, contract_id, "ros2")
    except Exception as exc:  # noqa: BLE001
        log.warning("connect %s failed: %s", contract_id, exc)
        return None


def _topic_of(channel, fallback_topic: str):
    """(topic, qos) from a ros2 channel; configured fallback when None."""
    if channel is None:
        return fallback_topic, _FALLBACK_QOS
    qos = getattr(getattr(channel, "params", None), "qos_profile", "") \
        or _FALLBACK_QOS
    return str(channel.endpoint or fallback_topic), str(qos)


def _resolve_topics(cfg: SkillConfig) -> dict:
    """topic key → (topic, qos). Priority: explicit config `topics:`
    override → atlas resolution (bounded retry) → /jaka/... default name."""
    resolved: dict = {}
    pending = []
    for key, contract_id in _PRIMITIVE_TOPICS.items():
        override = (cfg.topics or {}).get(key)
        if override:
            resolved[key] = (override, _FALLBACK_QOS)
        else:
            pending.append((key, contract_id))
    deadline = time.monotonic() + 30.0
    while pending and time.monotonic() < deadline:
        for key, contract_id in list(pending):
            ch = _connect_primitive(contract_id, cfg.arm_provider_id)
            if ch is not None:
                resolved[key] = _topic_of(ch, _DEFAULT_TOPICS[key])
                pending.remove((key, contract_id))
        if pending:
            time.sleep(2.0)
    for key, contract_id in pending:
        log.warning("contract %s not resolvable via atlas — falling back "
                    "to default topic %s", contract_id, _DEFAULT_TOPICS[key])
        resolved[key] = (_DEFAULT_TOPICS[key], _FALLBACK_QOS)
    return resolved


# ── lifecycle ───────────────────────────────────────────────────────────────

@jaka_skill.on_init
def init(cfg: dict):
    global _config, _arm, _frames, _detector, _homography
    cfg = cfg or {}
    if isinstance(cfg, str):
        try:
            cfg = json.loads(cfg) if cfg else {}
        except json.JSONDecodeError as exc:
            return Err(f"bad config_json: {exc}")
    if not isinstance(cfg, dict):
        return Err(f"config must be an object, got {type(cfg).__name__}")
    _config = SkillConfig.from_cfg(cfg)
    # Pick (perception) wiring — light: no atlas/ROS here. The camera topic is
    # resolved and subscribed in on_activate; construction only.
    _frames = CameraFrames(camera_matrix=_config.camera_matrix,
                           dist_coeffs=_config.dist_coeffs)
    _detector = Detector(_frames, DetectConfig(
        detect_scheme=_config.detect_scheme,
        yolo_model_path=_config.yolo_model_path,
        yolo_conf=_config.yolo_conf,
        yolo_iou=_config.yolo_iou,
        yolo_two_frame_interval_s=_config.yolo_two_frame_interval_s,
        yolo_two_frame_iou=_config.yolo_two_frame_iou,
        yolo_class_aliases=_config.yolo_class_aliases,
        vlm_base_url=_config.vlm_base_url,
        vlm_api_key=_config.vlm_api_key,
        vlm_model=_config.vlm_model,
        vlm_timeout_s=_config.vlm_timeout_s,
        frame_max_age_s=_config.frame_max_age_s,
        frame_wait_s=_config.frame_wait_s,
    ))
    _homography = Homography(_config.homography_matrix)
    _arm = Arm(_config, frames=_frames)
    if _config.config_error:
        log.warning("config errors deferred to tool-call time: %s",
                    _config.config_error)
    if _config.pick_error:
        log.warning("pick unavailable until calibrated: %s",
                    _config.pick_error)
    log.info("CMD_INIT ok (arm_provider_id=%s, hover=%s, grasp=%s)",
             _config.arm_provider_id, _config.hover_pose, _config.grasp_pose)
    return Ok()


def _resolve_camera(cfg: SkillConfig) -> None:
    """Resolve the RGB topic (config override > atlas, pinned provider) and
    hand it to the frame cache. Never fatal — a missing camera only disables
    pick, and pick already guards on pick_error."""
    if cfg.camera_topic:
        _frames.set_topic(cfg.camera_topic, _FALLBACK_QOS)
        log.info("camera topic from config: %s", cfg.camera_topic)
        return
    ch = _connect_primitive(_CAMERA_RGB_CONTRACT, cfg.camera_provider_id)
    if ch is not None:
        topic, qos = _topic_of(ch, "")
        if topic:
            _frames.set_topic(topic, qos)
            log.info("camera topic from atlas: %s", topic)
            return
    log.warning("camera/rgb unresolved (provider %s) — pick will report no "
                "frames until a camera is available", cfg.camera_provider_id)


@jaka_skill.on_activate
def activate():
    global _resolved
    with _state_lock:
        if _resolved:
            return Ok()
        if _arm is None or _config is None:
            return Err("init never completed")
        topics = _resolve_topics(_config)
        _arm.set_topics(
            joint_states=topics.get("joint_states"),
            end_pose=topics.get("end_pose"),
            joint_command=topics.get("joint_command"),
            pos_command=topics.get("pos_command"),
        )
        _resolve_camera(_config)  # before start(): the bridge subscribes it
        _arm.start()
        _resolved = True
        log.info("CMD_ACTIVATE ok: %s",
                 {k: v[0] for k, v in topics.items()})
        return Ok()


@jaka_skill.on_deactivate
def deactivate():
    global _resolved
    with _state_lock:
        if _arm is not None:
            _arm.stop()
        if _frames is not None:
            _frames.clear()
        _resolved = False
    log.info("CMD_DEACTIVATE ok")
    return Ok()


# ── flows ───────────────────────────────────────────────────────────────────

def _grasp_segment(arm: Arm, cfg: SkillConfig, hover_pose: list[float],
                   grasp_pose: list[float]) -> str:
    """Shared grasp segment: open -> hover -> descend -> close -> lift.

    hover_pose / grasp_pose are [x, y, z, rx, ry, rz]. Used by both `grab`
    (poses from config) and `pick` (grasp XY computed from perception, hover
    lifted by travel_offset). A failed lift is non-fatal (object already held).
    """
    hover_xyz, hover_rpy = hover_pose[:3], hover_pose[3:]
    grasp_xyz, grasp_rpy = grasp_pose[:3], grasp_pose[3:]
    if cfg.open_before_approach:
        arm.set_gripper(cfg.gripper_open_m, stage="grasp/open")
        arm.settle_gripper(expect_open=True, stage="grasp/open")
    arm.move_pose(hover_xyz, hover_rpy, stage="approach",
                  timeout_s=cfg.pose_timeout_s)
    arm.move_pose(grasp_xyz, grasp_rpy, stage="descend",
                  timeout_s=cfg.pose_timeout_s)
    arm.set_gripper(cfg.gripper_close_m, stage="grasp/close")
    arm.settle_gripper(expect_open=False, stage="grasp/close")
    try:
        arm.move_pose(hover_xyz, hover_rpy, stage="lift",
                      timeout_s=cfg.pose_timeout_s)
    except MotionError as exc:
        # The object is already in the fingers: a failed lift is a warning,
        # not a failure (the arm stays at the grasp pose, holding).
        return f"grabbed (held at grasp pose); lift failed: {exc}"
    return "grabbed and lifted back to hover"


def _run_grab(arm: Arm, cfg: SkillConfig) -> str:
    arm.wait_feedback_ready("preflight", cfg.feedback_timeout_s)
    return _grasp_segment(arm, cfg, cfg.hover_pose, cfg.grasp_pose)


def _run_pick(arm: Arm, cfg: SkillConfig, object_name: str) -> str:
    """Locate `object_name` with the camera, map its bbox centre to arm base
    XY via the 2D homography, and grasp at (x, y, desktop_height) with the
    configured vertical-down RPY. Hover is that pose lifted by travel_offset.

    A rotated bbox (yolo obb path, 5-element bbox_2d) additionally aligns the
    gripper yaw to the object's long edge: rz = grasp_yaw_offset_deg - yaw.
    Offset derivation (AIIT JAKA parity, same rig): AIIT commands
    R = Ry(180)·Rz(deg(yaw) - 90); this skill commands R = Rz(rz)·Rx(pi);
    the two are equal iff rz = 180 - (deg(yaw) - 90) == -90 - deg(yaw)
    (mod 360, absorbed by j6) — hence the default offset of -90."""
    arm.wait_feedback_ready("preflight", cfg.feedback_timeout_s)
    if _detector is None or _homography is None:
        raise MotionError("pick not available (perception not initialised)")
    result = _detector.detect(object_name)  # raises RuntimeError on no-detect
    bbox = result.get("bbox_2d") or []
    arm_xy = _homography.bbox_center_to_arm_xy(bbox)
    if arm_xy is None:
        raise MotionError(
            f"geometry failed: could not project bbox {bbox} to arm base "
            f"(check homography_matrix calibration)")
    x, y = arm_xy
    rpy = list(cfg.grasp_rpy)
    yaw = bbox_yaw_rad(bbox)  # None on the 4-element (vlm) bbox
    if yaw is not None:
        rpy[2] = math.radians(cfg.grasp_yaw_offset_deg - math.degrees(yaw))
    grasp_pose = [x, y, cfg.desktop_height, *rpy]
    hover_pose = [x, y, cfg.desktop_height + cfg.travel_offset, *rpy]
    log.info("pick '%s' -> bbox %s -> arm xy (%.3f, %.3f), grasp z %.3f, "
             "yaw %s", object_name, bbox, x, y, cfg.desktop_height,
             f"{yaw:+.3f} rad" if yaw is not None else "n/a (fixed rpy)")
    return _grasp_segment(arm, cfg, hover_pose, grasp_pose)


def _run_release(arm: Arm, cfg: SkillConfig) -> str:
    arm.wait_feedback_ready("preflight", cfg.feedback_timeout_s)
    arm.set_gripper(cfg.gripper_open_m, stage="release/open")
    return arm.settle_gripper(expect_open=True, stage="release/open")


def _run_egg(arm: Arm, cfg: SkillConfig) -> str:
    """Fixed-pose egg pick-and-place. The grasp half reuses the grab
    segment (open -> hover -> descend -> close -> settle -> lift[non-fatal]
    ) at the configured egg_grasp_pose; the place half is hover above
    egg_place_pose -> descend -> open -> settle -> lift (non-fatal — the
    egg is already down). Both hovers are z + travel_offset (pick's height
    model). A failed post-grasp lift is carried in the message, not fatal:
    the egg is in the fingers and the next waypoint is reachable from
    wherever the arm is."""
    arm.wait_feedback_ready("preflight", cfg.feedback_timeout_s)
    g_xyz, g_rpy = cfg.egg_grasp_pose[:3], cfg.egg_grasp_pose[3:]
    g_hover = [g_xyz[0], g_xyz[1], g_xyz[2] + cfg.travel_offset, *g_rpy]
    grasp_msg = _grasp_segment(arm, cfg, g_hover, cfg.egg_grasp_pose)
    p_xyz, p_rpy = cfg.egg_place_pose[:3], cfg.egg_place_pose[3:]
    p_hover = [p_xyz[0], p_xyz[1], p_xyz[2] + cfg.travel_offset, *p_rpy]
    arm.move_pose(p_hover[:3], p_rpy, stage="egg/approach",
                  timeout_s=cfg.pose_timeout_s)
    arm.move_pose(p_xyz, p_rpy, stage="egg/descend",
                  timeout_s=cfg.pose_timeout_s)
    arm.set_gripper(cfg.gripper_open_m, stage="egg/open")
    arm.settle_gripper(expect_open=True, stage="egg/open")
    try:
        arm.move_pose(p_hover[:3], p_rpy, stage="egg/lift",
                      timeout_s=cfg.pose_timeout_s)
    except MotionError as exc:
        # The egg is already on the table: a failed lift is a warning.
        return (f"{grasp_msg}; placed (arm stayed at the place pose); "
                f"lift failed: {exc}")
    return f"{grasp_msg}; placed and lifted back to hover"


# ── MCP tools ───────────────────────────────────────────────────────────────

@jaka_skill.mcp("robonix/skill/jaka/grab")
def grab(_req: Grab_Request) -> Grab_Response:
    """Grab whatever sits between the gripper fingers at the configured
    grasp pose. NO camera, NO object detection: the hover and grasp poses
    come from the deployment config. Use this when the user says "grab it
    / 抓" and the object is already positioned at the configured grasp
    pose. Opens the gripper, moves to hover, descends to grasp, closes the
    gripper, waits for it to settle, then lifts back to hover and holds.
    Blocks until done. Takes no arguments.
    """
    cfg = _require_active()
    arm = _arm
    t0 = time.monotonic()
    try:
        message = _run_grab(arm, cfg)
    except MotionError as exc:
        raise RuntimeError(f"grab failed: {exc}") from exc
    return Grab_Response(success=True, message=message,
                         elapsed_s=time.monotonic() - t0)


@jaka_skill.mcp("robonix/skill/jaka/release")
def release(_req: Release_Request) -> Release_Response:
    """Open the gripper WHERE THE ARM IS — zero arm motion. Use this when
    the user says "release / let go / 松开 / 放下" and the arm must not
    move. Because the gripper's RS485 position reads are broken on this
    arm, the opening is NOT verifiable by software — observe the fingers.
    Blocks until the settle wait completes. Takes no arguments.
    """
    cfg = _require_active()
    arm = _arm
    t0 = time.monotonic()
    try:
        message = _run_release(arm, cfg)
    except MotionError as exc:
        raise RuntimeError(f"release failed: {exc}") from exc
    return Release_Response(success=True, message=message,
                            elapsed_s=time.monotonic() - t0)


@jaka_skill.mcp("robonix/skill/jaka/egg")
def egg(_req: Egg_Request) -> Egg_Response:
    """Pick an egg up from the fixed configured pickup position and set it
    down at the fixed configured place position. Use this when the user
    says "抓鸡蛋 / 拾蛋 / egg" and both positions are the pre-calibrated
    fixed ones — no perception and no arguments. The flow: open gripper ->
    hover over the egg -> descend -> grip -> lift -> hover over the place
    position -> descend -> open -> lift. Grip force comes from jaka_arm's
    `gripper_force` config (applied once at connect) — it must be LOW for
    eggs. Blocks until done. Takes no arguments.
    """
    cfg = _require_active()
    if cfg.egg_error:
        raise RuntimeError(
            f"egg unavailable: {cfg.egg_error} — set both poses ([x, y, z, "
            f"rx, ry, rz], metres + radians, same frame/units as "
            f"hover_pose) in the deploy manifest's `skill: - name: jaka` "
            f"config block")
    arm = _arm
    t0 = time.monotonic()
    try:
        message = _run_egg(arm, cfg)
    except MotionError as exc:
        raise RuntimeError(f"egg failed: {exc}") from exc
    return Egg_Response(success=True, message=message,
                        elapsed_s=time.monotonic() - t0)


@jaka_skill.mcp("robonix/skill/jaka/pick")
def pick(req: Pick_Request) -> Pick_Response:
    """Locate a named object with the camera and grasp it. Use this when the
    user asks to pick up a specific object ("pick up the red block / 抓起螺丝
    刀") and its position is NOT known in advance — unlike `grab`, the grasp
    location comes from perception. Pass the user's target description as
    `object`; a camera frame is captured, a vision model locates the object,
    the pixel is mapped to an arm position via the calibrated 2D homography,
    and the arm descends onto it, closes, and lifts. Requires a calibrated
    homography + a configured VLM (returns a clear error otherwise). Blocks
    until done.
    """
    cfg = _require_active()
    if cfg.pick_error:
        raise RuntimeError(
            f"pick unavailable: {cfg.pick_error} — calibrate the "
            f"homography_matrix and set the VLM endpoint in the deploy "
            f"manifest's `skill: - name: jaka` config block")
    object_name = str(getattr(req, "object", "") or "").strip()
    if not object_name:
        raise RuntimeError("pick requires a non-empty `object` (what to pick)")
    arm = _arm
    t0 = time.monotonic()
    try:
        message = _run_pick(arm, cfg, object_name)
    except MotionError as exc:
        raise RuntimeError(f"pick failed: {exc}") from exc
    return Pick_Response(success=True, message=message,
                         elapsed_s=time.monotonic() - t0)


def main() -> int:
    jaka_skill.run()
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
