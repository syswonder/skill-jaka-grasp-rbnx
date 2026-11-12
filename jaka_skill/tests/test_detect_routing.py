# SPDX-License-Identifier: Apache-2.0
"""Unit tests for detect_scheme=auto routing (no camera, no ultralytics)."""
from __future__ import annotations

import unittest

from jaka_skill import config, detect

ALIASES = {"西红柿": "tomato", "土豆": "potato"}
HOM = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


class FakeModel:
    """Just the bit of the ultralytics model routing reads."""

    names = {0: "carrot", 1: "potato", 2: "tomato"}


def make_detector(scheme="auto", *, weights=True, vlm=True):
    cfg = detect.DetectConfig(
        detect_scheme=scheme,
        yolo_model_path="/tmp/best.pt" if weights else "",
        yolo_class_aliases=dict(ALIASES),
        vlm_base_url="https://example.invalid" if vlm else "",
        vlm_api_key="key" if vlm else "",
        vlm_model="model" if vlm else "",
    )
    det = detect.Detector(None, cfg)  # routing never touches the frame source
    if weights:
        det._load_model = lambda: FakeModel()
    else:
        def missing():
            raise RuntimeError("detection_failed: yolo backend unavailable "
                               "(model file not found)")
        det._load_model = missing
    return det


class AutoRouting(unittest.TestCase):
    def test_alias_resolves_to_weights_class(self):
        self.assertEqual(make_detector()._yolo_class_for("西红柿"), "tomato")

    def test_weights_class_name_itself_routes_to_yolo(self):
        self.assertEqual(make_detector()._yolo_class_for("tomato"), "tomato")

    def test_substring_of_weights_class_routes_to_yolo(self):
        self.assertEqual(make_detector()._yolo_class_for("potato chips"),
                         "potato")

    def test_name_outside_weights_routes_to_vlm(self):
        self.assertIsNone(make_detector()._yolo_class_for("苹果"))

    def test_missing_weights_routes_to_vlm(self):
        self.assertIsNone(make_detector(weights=False)._yolo_class_for("西红柿"))

    def test_auto_sends_covered_name_to_yolo(self):
        det = make_detector()
        det._detect_via_yolo = lambda name: {"message": f"yolo {name}"}
        det._detect_via_vlm = lambda name: {"message": f"vlm {name}"}
        self.assertEqual(det.detect("西红柿")["message"], "yolo 西红柿")
        self.assertEqual(det.detect("苹果")["message"], "vlm 苹果")

    def test_explicit_schemes_bypass_routing(self):
        det = make_detector("yolo")
        det._detect_via_yolo = lambda name: {"message": "yolo"}
        det._detect_via_vlm = lambda name: {"message": "vlm"}
        self.assertEqual(det.detect("苹果")["message"], "yolo")

        det = make_detector("vlm")
        det._detect_via_yolo = lambda name: {"message": "yolo"}
        det._detect_via_vlm = lambda name: {"message": "vlm"}
        self.assertEqual(det.detect("西红柿")["message"], "vlm")

    def test_auto_does_not_hide_a_missing_vlm(self):
        det = make_detector(vlm=False)
        with self.assertRaises(RuntimeError) as ctx:
            det.detect("苹果")
        self.assertIn("VLM not configured", str(ctx.exception))


class AutoConfig(unittest.TestCase):
    def test_scheme_is_accepted(self):
        cfg = config.SkillConfig.from_cfg({"detect_scheme": "auto"})
        self.assertEqual(cfg.detect_scheme, "auto")

    def test_unknown_scheme_still_defaults_to_vlm(self):
        cfg = config.SkillConfig.from_cfg({"detect_scheme": "nope"})
        self.assertEqual(cfg.detect_scheme, "vlm")

    def test_either_backend_alone_keeps_pick_ready(self):
        vlm_only = config.SkillConfig.from_cfg({
            "detect_scheme": "auto",
            "homography_matrix": HOM,
            "vlm_base_url": "https://example.invalid",
            "vlm_api_key": "key",
            "vlm_model": "model",
        })
        self.assertEqual(vlm_only.pick_error, "")

    def test_neither_backend_names_both(self):
        none = config.SkillConfig.from_cfg({
            "detect_scheme": "auto",
            "homography_matrix": HOM,
        })
        self.assertIn("yolo_model_path", none.pick_error)
        self.assertIn("vlm_base_url", none.pick_error)

    def test_explicit_yolo_scheme_still_demands_weights(self):
        cfg = config.SkillConfig.from_cfg({
            "detect_scheme": "yolo",
            "homography_matrix": HOM,
            "vlm_base_url": "https://example.invalid",
            "vlm_api_key": "key",
            "vlm_model": "model",
        })
        self.assertIn("yolo_model_path", cfg.pick_error)


if __name__ == "__main__":
    unittest.main()
