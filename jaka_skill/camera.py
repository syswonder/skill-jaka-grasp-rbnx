# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Camera frame cache: subscribe, buffer the latest frame, undistort.

Single responsibility — "hand back one usable image":
  - cache frames delivered by the ROS image callback, timestamp each;
  - judge freshness;
  - undistort with the calibrated intrinsics.

No detection, no object reasoning, no arm — detection lives in detect.py,
orchestration in __init__.py. The topic name is NEVER hard-coded: it is
resolved from `robonix/primitive/camera/rgb` via atlas (falling back to a
config `camera_topic` only when the operator sets one explicitly).

Undistortion lives here (not in detect) because the hand-eye homography is
calibrated on undistorted pixels — binding "undistort" to "get a frame"
means no detection path can forget it (corner error can reach ~30px / ~3cm).
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Optional, Tuple

log = logging.getLogger("jaka_skill.camera")


class CameraFrames:
    """Cache of the latest frame. The ROS subscription callback calls
    `update()`; the detector reads via `frame()`."""

    def __init__(self, camera_matrix=None, dist_coeffs=None) -> None:
        self._lock = threading.Lock()
        self._frame = None
        self._seen_mono: float = 0.0
        self._camera_matrix = camera_matrix
        self._dist_coeffs = dist_coeffs
        self._topic = ""
        self._qos = "best_effort"
        self._cb_count = 0
        if camera_matrix is None or dist_coeffs is None:
            log.warning("camera intrinsics not configured — frames stay "
                        "undistorted (hand-eye homography assumes undistorted "
                        "pixels; corner error can reach ~30px otherwise)")

    def set_topic(self, topic: str, qos: str = "best_effort") -> None:
        """Set the image topic + QoS to subscribe (call before attach())."""
        self._topic = str(topic or "").strip()
        self._qos = qos or "best_effort"

    @property
    def topic(self) -> str:
        return self._topic

    # ── subscription (callback feeds frames in) ──────────────────────────
    def attach(self, node, qos_from_profile) -> bool:
        """Subscribe to the configured image topic on the given rclpy node.
        The callback converts each message to a bgr8 frame and calls
        `update()`. Returns whether the subscription was created."""
        if not self._topic:
            log.warning("camera topic not set — not subscribing")
            return False
        try:
            from cv_bridge import CvBridge
            from sensor_msgs.msg import Image
        except Exception as e:  # noqa: BLE001
            log.warning("camera subscriber unavailable: %s", e)
            return False
        bridge = CvBridge()

        def _on_image(msg) -> None:
            try:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:  # noqa: BLE001
                return
            self.update(frame)
            if self._cb_count % 100 == 0:
                log.info("camera frame #%d %dx%d", self._cb_count,
                         frame.shape[1], frame.shape[0])
            self._cb_count += 1

        node.create_subscription(Image, self._topic, _on_image,
                                 qos_from_profile(self._qos))
        log.info("camera subscribed: %s", self._topic)
        return True

    # ── write (ROS callback) ──────────────────────────────────────────────
    def update(self, frame) -> None:
        with self._lock:
            self._frame = frame
            self._seen_mono = time.monotonic()

    # ── read ──────────────────────────────────────────────────────────────
    def latest(self) -> Tuple[Optional[object], float]:
        with self._lock:
            frame, seen = self._frame, self._seen_mono
        age = (time.monotonic() - seen) if frame is not None else 0.0
        return frame, age

    def clear(self) -> None:
        with self._lock:
            self._frame = None
            self._seen_mono = 0.0

    def undistort(self, frame):
        """Undistort with the calibrated intrinsics; identity (returns the
        raw frame) when unset or on failure — never raises."""
        if self._camera_matrix is None or self._dist_coeffs is None:
            return frame
        try:
            import cv2
            import numpy as np
            return cv2.undistort(
                frame,
                np.asarray(self._camera_matrix, dtype=np.float64),
                np.asarray(self._dist_coeffs, dtype=np.float64),
            )
        except Exception as e:  # noqa: BLE001
            log.warning("undistort failed (%s) — using raw frame", e)
            return frame

    def frame(self, max_age_s: float, wait_s: float = 0.0):
        """Return one usable (undistorted) frame:
        - wait up to `wait_s` for the first frame (skill just activated);
        - a frame older than `max_age_s` is stale -> unusable;
        - got one -> undistort and return; timed out -> None."""
        deadline = time.monotonic() + max(0.0, wait_s)
        while True:
            frame, age = self.latest()
            if frame is not None and age <= max_age_s:
                return self.undistort(frame)
            if time.monotonic() >= deadline:
                return None
            time.sleep(0.1)
