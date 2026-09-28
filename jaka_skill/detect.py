# SPDX-License-Identifier: Apache-2.0
# -*- coding: utf-8 -*-
"""Object detection for the jaka pick skill — two backends behind one entry.

detect(object_name) dispatches on `detect_scheme`:

  yolo  in-process yolo11-obb (the AIIT/agilex fruit model). Full detection
        on the undistorted frame -> two-frame consistency (flicker filter)
        -> class match (substring / alias map). Each hit carries the OBB
        rotation: bbox_2d = [x_min, y_min, x_max, y_max, rotation_deg]
        (the ecosystem seam shape — same 5th element the agilex grasp_pose
        service consumes), which the pick flow turns into gripper yaw.
  vlm   one open-vocabulary multimodal call -> pixel bbox (no rotation,
        bbox_2d has 4 elements; the pick keeps the fixed grasp_rpy).

Returns a plain dict {success, bbox_2d, confidence, message}; the caller
(pick flow) maps bbox_2d -> arm XY through geometry.Homography. Detection
failures raise RuntimeError("detection_failed: ...") — the prefix is part
of the contract (pilot relays it to the user).
"""
from __future__ import annotations

import base64
import json
import logging
import math
import threading
import urllib.request
from dataclasses import dataclass
from typing import Optional, Sequence

log = logging.getLogger("jaka_skill.detect")

# In-process YOLO model cache: one load per process, guarded because
# ultralytics predict is not guaranteed re-entrant across threads.
_MODEL = None
_MODEL_PATH = ""
_MODEL_LOCK = threading.Lock()


@dataclass
class DetectConfig:
    """Detector parameters (filled from the skill config by atlas_bridge)."""

    detect_scheme: str = "vlm"
    yolo_model_path: str = ""
    yolo_conf: float = 0.6
    yolo_iou: float = 0.45
    yolo_two_frame_interval_s: float = 0.7
    yolo_two_frame_iou: float = 0.3
    yolo_class_aliases: dict = None  # user phrasing -> class name

    vlm_base_url: str = ""
    vlm_api_key: str = ""
    vlm_model: str = ""
    vlm_timeout_s: float = 60.0
    frame_max_age_s: float = 1.0
    frame_wait_s: float = 5.0

    def __post_init__(self) -> None:
        if self.yolo_class_aliases is None:
            self.yolo_class_aliases = {}

    @property
    def ready(self) -> bool:
        """Scheme-appropriate endpoint readiness (model file / VLM keys)."""
        if self.detect_scheme == "yolo":
            return bool(self.yolo_model_path)
        return bool(self.vlm_base_url and self.vlm_api_key and self.vlm_model)


class Detector:
    """Locate a named object in the latest camera frame (yolo or vlm)."""

    def __init__(self, frames, cfg: DetectConfig) -> None:
        self._frames = frames
        self.cfg = cfg

    def _frame(self):
        frame = self._frames.frame(self.cfg.frame_max_age_s,
                                   wait_s=self.cfg.frame_wait_s)
        if frame is None:
            raise RuntimeError(
                "detection_failed: no fresh camera frame "
                f"(topic {self._frames.topic or '?'} — is the camera "
                f"primitive ACTIVE and streaming?)")
        return frame

    def detect(self, object_name: str) -> dict:
        """Locate `object_name`. Returns {success, bbox_2d, confidence,
        message}; raises RuntimeError with a stage-tagged reason on failure.

        bbox_2d is 5 elements on the yolo path (rotation in degrees as the
        5th) and 4 on the vlm path."""
        if self.cfg.detect_scheme == "yolo":
            return self._detect_via_yolo(object_name)
        return self._detect_via_vlm(object_name)

    # ── yolo path ─────────────────────────────────────────────────────────

    def _detect_via_yolo(self, object_name: str) -> dict:
        dets = self._yolo_detect_all(self._frame())
        dets = self._two_frame_consistency(dets)
        if not dets:
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(nothing found on the table)")
        matched = self._match_class(object_name, dets)
        if matched is None:
            found = ", ".join(sorted({str(d.get("object_name"))
                                      for d in dets}))
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(table has: {found})")
        log.info("match: user '%s' -> %s (conf=%.2f)",
                 object_name, matched.get("object_name"),
                 float(matched.get("confidence", 0.0)))
        return {
            "success": True,
            "bbox_2d": list(matched.get("bbox_2d") or []),
            "confidence": float(matched.get("confidence", 0.0)),
            "message": f"yolo located {matched.get('object_name')}",
        }

    def _load_model(self):
        """Load the obb model once per process (first pick pays ~seconds)."""
        global _MODEL, _MODEL_PATH
        import os

        path = os.path.expanduser(self.cfg.yolo_model_path)
        if not path or not os.path.isfile(path):
            raise RuntimeError(
                "detection_failed: yolo backend unavailable "
                f"(model file not found: {path!r} — set yolo_model_path)")
        with _MODEL_LOCK:
            if _MODEL is None or _MODEL_PATH != path:
                try:
                    from ultralytics import YOLO
                except Exception as e:  # noqa: BLE001
                    raise RuntimeError(
                        "detection_failed: yolo backend unavailable "
                        f"(ultralytics import failed: {e})") from e
                log.info("loading yolo obb model: %s", path)
                _MODEL = YOLO(path)
                _MODEL_PATH = path
                log.info("yolo classes: %s",
                         {int(k): str(v)
                          for k, v in (_MODEL.names or {}).items()})
            return _MODEL

    def _yolo_detect_all(self, frame) -> list:
        """All obb detections on one frame, highest confidence first. Same
        pathway as the AIIT/agilex stack: results.obb.xywhr/conf/cls, each
        emitted as the seam shape bbox_2d = [xmin, ymin, xmax, ymax,
        deg(r)] (center/w/h preserved for the long-edge yaw rule)."""
        model = self._load_model()
        with _MODEL_LOCK:
            results = model(frame, conf=self.cfg.yolo_conf,
                            iou=self.cfg.yolo_iou, verbose=False)[0]
        obb = getattr(results, "obb", None)
        if obb is None or obb.xywhr is None or len(obb.xywhr) == 0:
            return []
        xywhr = obb.xywhr.cpu().numpy()
        scores = obb.conf.cpu().numpy()
        class_ids = obb.cls.cpu().numpy().astype(int)
        names = {int(k): str(v) for k, v in (model.names or {}).items()}

        out: list = []
        for (u, v, w, h, r), score, cid in zip(xywhr, scores, class_ids):
            u, v, w, h, r = float(u), float(v), float(w), float(h), float(r)
            out.append({
                "object_name": names.get(int(cid), str(int(cid))),
                "confidence": float(score),
                "bbox_2d": [u - w / 2.0, v - h / 2.0,
                            u + w / 2.0, v + h / 2.0, math.degrees(r)],
            })
        out.sort(key=lambda d: d["confidence"], reverse=True)
        return out

    def _two_frame_consistency(self, dets_a: list) -> list:
        """Keep only detections that appear in BOTH of two frames taken
        `yolo_two_frame_interval_s` apart, same class and IoU above
        `yolo_two_frame_iou` (real objects are stable, flicker is not).
        Second frame / call missing -> single-frame result (warn only)."""
        interval = self.cfg.yolo_two_frame_interval_s
        if interval <= 0.0 or not dets_a:
            return dets_a
        import time
        time.sleep(interval)
        frame_b, _age = self._frames.latest()
        if frame_b is None:
            log.warning("2-frame: no second frame — single frame result")
            return dets_a
        dets_b = self._yolo_detect_all(self._frames.undistort(frame_b))
        if dets_b is None:
            log.warning("2-frame: second yolo call failed — single frame "
                        "result")
            return dets_a
        kept = []
        for d in dets_a:
            name = str(d.get("object_name", "")).strip().lower()
            for e in dets_b:
                if (str(e.get("object_name", "")).strip().lower() == name
                        and self._bbox_iou(list(d.get("bbox_2d") or []),
                                           list(e.get("bbox_2d") or []))
                        > self.cfg.yolo_two_frame_iou):
                    kept.append(d)
                    break
        log.info("2-frame consistency: %d/%d kept", len(kept), len(dets_a))
        return kept

    @staticmethod
    def _bbox_iou(a: Sequence[float], b: Sequence[float]) -> float:
        """IoU of two [x_min, y_min, x_max, y_max, ...] boxes (5th element
        ignored)."""
        try:
            ax1, ay1, ax2, ay2 = (float(v) for v in a[:4])
            bx1, by1, bx2, by2 = (float(v) for v in b[:4])
        except (TypeError, ValueError, IndexError):
            return 0.0
        ix1, iy1 = max(ax1, bx1), max(ay1, by1)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
        if inter <= 0.0:
            return 0.0
        area_a = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
        area_b = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
        return inter / (area_a + area_b - inter)

    def _match_class(self, user_request: str, detections: list) -> Optional[dict]:
        """Map the user's phrasing to a detected class name: alias map first
        (e.g. 胡萝卜 -> carrot), then case-insensitive substring either way.
        Detections are confidence-sorted, so the first match is the best."""
        key = str(user_request).strip().lower()
        alias = str(self.cfg.yolo_class_aliases.get(user_request.strip())
                    or self.cfg.yolo_class_aliases.get(key)
                    or "").strip().lower()
        for d in detections:
            name = str(d.get("object_name", "")).strip().lower()
            if alias and (name == alias or alias in name or name in alias):
                return d
        for d in detections:
            name = str(d.get("object_name", "")).strip().lower()
            if name and (name == key or key in name or name in key):
                return d
        return None

    # ── vlm path ──────────────────────────────────────────────────────────

    def _detect_via_vlm(self, object_name: str) -> dict:
        """One open-vocabulary multimodal call: current frame + the user's
        request go to a vision LLM, which returns a pixel bbox."""
        import cv2

        cfg = self.cfg
        if not cfg.ready:
            raise RuntimeError(
                "detection_failed: VLM not configured "
                "(set vlm_base_url / vlm_api_key / vlm_model in the skill "
                "config block)")

        frame = self._frame()
        ok, enc = cv2.imencode(".jpg", frame)
        if not ok:
            raise RuntimeError("detection_failed: could not JPEG-encode frame")
        b64 = base64.b64encode(enc.tobytes()).decode("ascii")
        h, w = frame.shape[:2]
        prompt = (
            "你是抓取目标定位器。图片是一张桌面俯视图,宽%d像素,高%d像素。\n"
            f"用户请求: {object_name}\n"
            "在图中定位用户请求的物体,只输出JSON: "
            "{\"present\": true, \"bbox\": [x_min, y_min, x_max, y_max]} 或 "
            "{\"present\": false}。bbox 为像素坐标,必须在图像范围内,"
            "恰好框住目标;图中没有该物体时输出 {\"present\": false}。"
        ) % (w, h)
        payload = json.dumps({
            "model": cfg.vlm_model,
            "messages": [{"role": "user", "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{b64}"}},
            ]}],
            "temperature": 0.0,
            "response_format": {"type": "json_object"},
        }).encode("utf-8")
        url = cfg.vlm_base_url.rstrip("/") + "/chat/completions"
        req = urllib.request.Request(
            url, data=payload, method="POST",
            headers={"Content-Type": "application/json",
                     "Authorization": f"Bearer {cfg.vlm_api_key}"})
        try:
            with urllib.request.urlopen(req, timeout=cfg.vlm_timeout_s) as r:
                body = json.loads(r.read().decode("utf-8"))
            parsed = json.loads(str(body["choices"][0]["message"]["content"]))
        except Exception as e:  # noqa: BLE001
            log.warning("pick VLM failed: %s", e)
            raise RuntimeError(f"detection_failed: VLM call failed: {e}")

        if not parsed.get("present", False):
            raise RuntimeError(
                f"detection_failed: object '{object_name}' not detected "
                f"(VLM found nothing)")
        try:
            bbox = [float(x) for x in parsed["bbox"]]
            assert len(bbox) == 4
        except (KeyError, TypeError, ValueError, AssertionError) as e:
            log.warning("pick VLM bad bbox: %r (%s)", parsed.get("bbox"), e)
            raise RuntimeError(f"detection_failed: VLM returned bad bbox: {e}")

        log.info("vlm locate: user '%s' -> bbox %s", object_name, bbox)
        return {
            "success": True,
            "bbox_2d": bbox,
            "confidence": 1.0,
            "message": f"vlm located {object_name}",
        }
