# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Arm command/feedback half of the jaka grab/release skill.

Talks to the `jaka_arm` primitive ONLY through its ROS 2 contracts
(pos_command / joint_command / end_pose / joint_states), resolved by
atlas. No gRPC, no arm_status polling: jaka_arm executes motion
synchronously inside its own command worker, so "motion done" is detected
by watching /jaka/end_pose converge on the commanded pose — the same 2 mm
reach check the provider itself uses
(primitive-jaka-rbnx/jaka_arm/driver.py `_wait_pose_reached`).
"""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Optional

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

log = logging.getLogger("jaka_skill")

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
GRIPPER_JOINT_NAME = "gripper_joint"
GRIPPER_STROKE_M = 0.098        # TG-9801 stroke, matches the provider
GRIPPER_OPEN_THRESHOLD_M = 0.5 * GRIPPER_STROKE_M

# Grasp verification (after grasp/close). A settled width below
# GRASP_CLOSED_THRESHOLD_M means the fingers closed on air (nothing between
# them); at/above GRIPPER_OPEN_THRESHOLD_M they never closed at all.
# The provider republishes its cached reading (<=2 s cache, ~1.3 s per RS485
# read), so one sample can still carry a pre-command or mid-travel value —
# only trust two samples at least GRIPPER_SETTLE_GAP_S apart that agree
# within GRIPPER_SETTLE_TOL_M, which spans at least one fresh read. The gap
# also covers a read that had to retry (2 s cache + 2x read).
GRASP_CLOSED_THRESHOLD_M = 0.005
GRIPPER_SETTLE_GAP_S = 5.0
GRIPPER_SETTLE_TOL_M = 0.002
# Deadline for the verdict, measured from the wait's start (after the 1.5 s
# settle): first post-command read (<=3.3 s) + gap + one read that landed
# mid-travel (<=3.3 s) + slack.
GRASP_VERIFY_TIMEOUT_S = 13.0


class MotionError(RuntimeError):
    """A motion step failed; `str(e)` is the operator/LLM-facing text."""


def qos_from_profile(profile: str):
    """Translate the atlas-declared QoS profile name into an rclpy profile.

    jaka_arm declares every topic `reliable`, and a reliable subscriber
    never receives from a best-effort publisher — the caller-side fallback
    must therefore be reliable (piper's best-effort default would break
    silently here).
    """
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    best_effort = str(profile or "").strip().lower() in (
        "best_effort", "besteffort")
    return QoSProfile(
        depth=5,
        history=HistoryPolicy.KEEP_LAST,
        reliability=(ReliabilityPolicy.BEST_EFFORT if best_effort
                     else ReliabilityPolicy.RELIABLE),
    )


def rpy_to_quat(rx: float, ry: float, rz: float):
    """Rz·Ry·Rx rotation (== scipy Rotation.from_euler("xyz", ...)) as an
    (x, y, z, w) quaternion. Exact inverse of the provider's quat→RPY
    conversion (jaka_arm/main.py `_exec_pos_command`), so configured RPY
    round-trips through the controller."""
    cr, sr = math.cos(rx / 2), math.sin(rx / 2)
    cp, sp = math.cos(ry / 2), math.sin(ry / 2)
    cy, sy = math.cos(rz / 2), math.sin(rz / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Arm:
    """Feedback cache + command publishers for the four arm contracts.

    The ROS node lives in a dedicated daemon thread (own rclpy init/context)
    so blocking tool handlers on the FastMCP loop never starve the
    subscriptions. Topic names and QoS come from atlas resolution
    (see atlas_bridge), never from this class.
    """

    def __init__(self, cfg, frames=None) -> None:
        self.cfg = cfg
        # Optional camera frame cache (pick path). Subscribed on the same ROS
        # bridge node so frames and arm feedback share one executor.
        self.frames = frames
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._node = None
        self._pub_joint_command = None
        self._pub_pos_command = None
        # (topic, qos) pairs, set before start()
        self._t_joint_states: Optional[tuple] = None
        self._t_end_pose: Optional[tuple] = None
        self._t_joint_command: Optional[tuple] = None
        self._t_pos_command: Optional[tuple] = None
        # feedback cache (monotonic timestamp per entry)
        self._end_pose = None
        self._end_pose_mono: Optional[float] = None
        self._joints = None
        self._joints_mono: Optional[float] = None
        self._gripper_m: Optional[float] = None
        self._gripper_mono: Optional[float] = None

    # ── lifecycle ─────────────────────────────────────────────────────────

    def set_topics(self, *, joint_states=None, end_pose=None,
                   joint_command=None, pos_command=None) -> None:
        self._t_joint_states = joint_states
        self._t_end_pose = end_pose
        self._t_joint_command = joint_command
        self._t_pos_command = pos_command

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._runner, name="jaka-skill-ros", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        self._thread = None

    def _runner(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
        except ImportError:
            log.warning("rclpy unavailable; ROS bridge not started "
                        "(arm commands will fail)")
            return
        arm = self

        class _RosBridge(Node):
            def __init__(self):
                super().__init__("jaka_skill_ros_bridge")
                if arm._t_joint_states is not None:
                    topic, qos = arm._t_joint_states
                    self.create_subscription(
                        JointState, topic, arm._on_joint_state,
                        qos_from_profile(qos))
                else:
                    log.error("joint_states topic unresolved — arm "
                              "feedback unavailable")
                if arm._t_end_pose is not None:
                    topic, qos = arm._t_end_pose
                    self.create_subscription(
                        Pose, topic, arm._on_end_pose, qos_from_profile(qos))
                else:
                    log.error("end_pose topic unresolved — pose feedback "
                              "unavailable")
                if arm._t_joint_command is not None:
                    topic, qos = arm._t_joint_command
                    arm._pub_joint_command = self.create_publisher(
                        JointState, topic, qos_from_profile(qos))
                if arm._t_pos_command is not None:
                    topic, qos = arm._t_pos_command
                    arm._pub_pos_command = self.create_publisher(
                        Pose, topic, qos_from_profile(qos))
                # Camera (pick path): subscribe the RGB frame cache on this
                # same node when one was provided and its topic is resolved.
                if arm.frames is not None and arm.frames.topic:
                    arm.frames.attach(self, qos_from_profile)

        if not rclpy.ok():
            rclpy.init(args=None)
        node = _RosBridge()
        arm._node = node
        try:
            while rclpy.ok() and not arm._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
        finally:
            arm._node = None
            arm._pub_joint_command = None
            arm._pub_pos_command = None
            node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001
                pass

    # ── feedback callbacks (ROS thread) ───────────────────────────────────

    def _on_end_pose(self, msg) -> None:
        with self._lock:
            self._end_pose = (msg.position.x, msg.position.y, msg.position.z)
            self._end_pose_mono = time.monotonic()

    def _on_joint_state(self, msg) -> None:
        idx = {name: i for i, name in enumerate(msg.name)}
        joints: list[float] = []
        for name in JOINT_NAMES:
            if name not in idx:
                joints = []
                break
            v = msg.position[idx[name]]
            if not math.isfinite(v):
                joints = []
                break
            joints.append(float(v))
        with self._lock:
            if joints:
                self._joints = joints
                self._joints_mono = time.monotonic()
            if GRIPPER_JOINT_NAME in idx:
                g = msg.position[idx[GRIPPER_JOINT_NAME]]
                if math.isfinite(g):
                    self._gripper_m = float(g)
                    self._gripper_mono = time.monotonic()

    # ── feedback accessors ────────────────────────────────────────────────

    def end_pose_now(self):
        with self._lock:
            return self._end_pose, self._end_pose_mono

    def arm_joints_now(self):
        with self._lock:
            return self._joints, self._joints_mono

    def gripper_width_now(self):
        with self._lock:
            return self._gripper_m, self._gripper_mono

    def arm_joints_fresh(self, max_age_s: float):
        joints, mono = self.arm_joints_now()
        if joints is None or mono is None:
            return None
        if time.monotonic() - mono > max_age_s:
            return None
        return joints

    def wait_feedback_ready(self, stage: str, timeout_s: float) -> None:
        """Wait for the bridge node if it is still starting, then require
        one end_pose AND one joint_states sample inside the wait window —
        proof the ROS bridge and the provider are alive before any command
        is published."""
        # The node is created asynchronously by the daemon thread (rclpy
        # import + init + subscriptions ≈ 1s), while the executor invokes
        # the tool moments after on_activate's _arm.start() — so "no node
        # YET" must poll here, only "never appeared" is fatal.
        start = time.monotonic()
        while self._node is None:
            if time.monotonic() - start >= timeout_s:
                raise MotionError(f"{stage} failed: ROS bridge down")
            time.sleep(self.cfg.poll_s)
        start = time.monotonic()
        deadline = start + timeout_s
        have_pose = have_joints = False
        while time.monotonic() < deadline:
            if self._node is None:  # runner exited mid-wait
                raise MotionError(f"{stage} failed: ROS bridge down")
            _, pose_mono = self.end_pose_now()
            _, joints_mono = self.arm_joints_now()
            have_pose = pose_mono is not None and pose_mono >= start
            have_joints = joints_mono is not None and joints_mono >= start
            if have_pose and have_joints:
                return
            time.sleep(self.cfg.poll_s)
        missing = []
        if not have_pose:
            missing.append("end_pose")
        if not have_joints:
            missing.append("joint_states")
        raise MotionError(
            f"{stage} failed: no fresh feedback ({', '.join(missing)}) within "
            f"{timeout_s:.1f}s — is the jaka_arm primitive ACTIVE?")

    # ── commands ──────────────────────────────────────────────────────────

    def move_pose(self, xyz, rpy, *, stage: str, timeout_s: float) -> float:
        """Publish a Cartesian target on pos_command and wait for end_pose
        to converge on it. Returns the final distance (m); raises MotionError
        on timeout."""
        pub = self._pub_pos_command
        if pub is None or self._node is None:
            raise MotionError(
                f"{stage} failed: pos_command publisher not ready "
                f"(ROS bridge down or topic unresolved)")
        x, y, z = (float(v) for v in xyz)
        qx, qy, qz, qw = rpy_to_quat(*(float(v) for v in rpy))
        msg = Pose()
        msg.position.x, msg.position.y, msg.position.z = x, y, z
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        t_pub = time.monotonic()
        pub.publish(msg)

        deadline = t_pub + timeout_s
        last = None
        last_d = float("inf")
        while time.monotonic() < deadline:
            posed, seen = self.end_pose_now()
            # FRESH-ONLY: ignore samples that predate the publish, or a
            # tolerance-satisfying prior pose would read as instant success.
            if posed is not None and seen is not None and seen >= t_pub:
                last = posed
                last_d = math.dist(posed, (x, y, z))
                if last_d <= self.cfg.reach_tolerance_m:
                    return last_d
            time.sleep(self.cfg.poll_s)
        closest = (f"{last_d * 1000:.1f} mm" if last is not None
                   else "no fresh sample")
        raise MotionError(
            f"{stage} failed: end_pose did not reach "
            f"({x:.4f}, {y:.4f}, {z:.4f}) within {timeout_s:.1f}s "
            f"(closest {closest}). If the arm never moved at all, jaka_arm "
            f"is silently rejecting motion — check `motion_enabled: true` "
            f"in the deploy manifest's jaka_arm config (rejections are "
            f"logged there as 'motion command rejected').")

    def move_joints(self, target_rad, *, stage: str, timeout_s: float) -> float:
        """Publish a joint-space target (the six arm joints alone) on
        joint_command and wait for joint_states to converge on it. The
        message carries no `gripper_joint`, so the provider leaves the
        gripper untouched. Returns the final worst per-joint error (rad);
        raises MotionError on timeout."""
        pub = self._pub_joint_command
        node = self._node
        if pub is None or node is None:
            raise MotionError(
                f"{stage} failed: joint_command publisher not ready "
                f"(ROS bridge down or topic unresolved)")
        target = [float(v) for v in target_rad]
        if self.arm_joints_fresh(self.cfg.feedback_timeout_s) is None:
            raise MotionError(
                f"{stage} failed: no fresh arm joint feedback on "
                f"{self._t_joint_states[0] if self._t_joint_states else '?'} "
                f"within {self.cfg.feedback_timeout_s:.1f}s — refusing to "
                f"command the arm blind")
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = target
        t_pub = time.monotonic()
        pub.publish(msg)

        deadline = t_pub + timeout_s
        last = None
        last_e = float("inf")
        while time.monotonic() < deadline:
            joints, seen = self.arm_joints_now()
            # FRESH-ONLY: ignore samples that predate the publish (same
            # rationale as move_pose).
            if joints is not None and seen is not None and seen >= t_pub:
                last = joints
                last_e = max(abs(c - t) for c, t in zip(joints, target,
                                                        strict=True))
                if last_e <= self.cfg.joint_reach_tolerance_rad:
                    return last_e
            time.sleep(self.cfg.poll_s)
        closest = (f"worst joint {math.degrees(last_e):.2f} deg away"
                   if last is not None else "no fresh sample")
        raise MotionError(
            f"{stage} failed: joints did not reach the target within "
            f"{timeout_s:.1f}s ({closest}). If the arm never moved at all, "
            f"jaka_arm is silently rejecting motion — check "
            f"`motion_enabled: true` in the deploy manifest's jaka_arm "
            f"config (rejections are logged there as 'motion command "
            f"rejected').")

    def set_gripper(self, open_m: float, *, stage: str) -> float:
        """Command the gripper by publishing `gripper_joint` ALONE.

        The provider back-fills the six arm joints from its own live
        feedback (jaka_arm/main.py `_exec_joint_command`), so the commanded
        arm target IS the current pose — zero arm motion by construction.
        Returns the time of the last publish: gripper samples received after
        it are the ones that can reflect this command.
        """
        pub = self._pub_joint_command
        node = self._node
        if pub is None or node is None:
            raise MotionError(
                f"{stage} failed: joint_command publisher not ready "
                f"(ROS bridge down or topic unresolved)")
        if self.arm_joints_fresh(self.cfg.feedback_timeout_s) is None:
            raise MotionError(
                f"{stage} failed: no fresh arm joint feedback on "
                f"{self._t_joint_states[0] if self._t_joint_states else '?'} "
                f"within {self.cfg.feedback_timeout_s:.1f}s — refusing to "
                f"command the arm blind")
        value = float(min(max(open_m, 0.0), GRIPPER_STROKE_M))
        for _ in range(self.cfg.gripper_repeats):
            msg = JointState()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = [GRIPPER_JOINT_NAME]
            msg.position = [value]
            pub.publish(msg)
            time.sleep(self.cfg.gripper_publish_period_s)
        return time.monotonic()

    def wait_gripper_settled(self, *, since: float) -> Optional[float]:
        """Width (m) once the gripper has stopped moving, else None (timeout).

        The provider republishes its cached reading (<=2 s cache, ~1.3 s per
        RS485 read), so one fresh sample can still carry a pre-command or
        mid-travel value. A width is only returned when two samples at least
        GRIPPER_SETTLE_GAP_S apart agree within GRIPPER_SETTLE_TOL_M — that
        window spans at least one new read, so a moving gripper cannot pass.
        Samples received before `since` (the command's publish time) are
        ignored; a width that never settles returns None, not a verdict.
        Rarely, two reads sampled just before the command's write lands
        agree on the pre-command width: the verdict then reads as that
        width. Callers treat it as "not closed" and retry — a safe,
        bounded cost (Monte-Carlo: ~0.1% of grasps, never a false
        "grasped").
        """
        deadline = time.monotonic() + GRASP_VERIFY_TIMEOUT_S
        anchors: list[tuple[float, float]] = []  # (width, receipt time)
        while time.monotonic() < deadline:
            width, mono = self.gripper_width_now()
            if width is not None and mono is not None and mono > since:
                # Compare this sample against every anchor that is now old
                # enough (each anchor is compared exactly once, then dropped:
                # a mid-travel value must not linger as a candidate).
                while anchors and mono - anchors[0][1] >= GRIPPER_SETTLE_GAP_S:
                    old_width, _ = anchors.pop(0)
                    if abs(width - old_width) <= GRIPPER_SETTLE_TOL_M:
                        return (width + old_width) / 2.0
                anchors.append((width, mono))
            time.sleep(self.cfg.poll_s)
        return None

    def settle_gripper(self, *, expect_open: bool, stage: str) -> str:
        """Fixed wait after a gripper command. A TG-9801 position read takes
        ~1.3 s (single-entry RS485 path) and refreshes on a slow cadence, so
        normally this is just a timer; if a fresh gripper_joint reading
        happens to appear and already matches the expectation, return early
        (free upside, never a dependency)."""
        deadline = time.monotonic() + self.cfg.gripper_settle_s
        while time.monotonic() < deadline:
            width, mono = self.gripper_width_now()
            if (width is not None and mono is not None
                    and time.monotonic() - mono <= self.cfg.feedback_timeout_s
                    and (width >= GRIPPER_OPEN_THRESHOLD_M) == expect_open):
                return (f"gripper feedback confirms "
                        f"{'open' if expect_open else 'closed'} "
                        f"(width {width:.4f} m)")
            time.sleep(0.05)
        return (f"gripper {'open' if expect_open else 'close'} command "
                f"issued; settled {self.cfg.gripper_settle_s:.1f}s — no fresh "
                f"gripper feedback matched within the wait, observe the fingers")
