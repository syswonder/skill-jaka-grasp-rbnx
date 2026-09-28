---
description: JAKA Mini grab/release/pick skill. grab closes the gripper at the configured grasp pose and lifts to hover; release opens the gripper in place with zero arm motion; pick locates a named object with the camera (yolo11-obb or VLM backend) and grasps it via a calibrated 2D hand-eye homography; the yolo path also aligns gripper yaw to the object's long edge. grab/release need no perception; pick needs a calibrated homography + a detector (yolo weights or VLM endpoint).
---

# JAKA grab/release/pick skill

Provider id `jaka`, namespace `robonix/skill/jaka`. LLM tool names:
`jaka_grab` / `jaka_release` / `jaka_pick`. The skill owns no hardware: arm
motion goes through the `jaka_arm` primitive's `pos_command` / `joint_command`
contracts and the camera through `orbbec_camera`'s `camera/rgb`, all resolved
by topic name via atlas — nothing is hard-coded.

## Frames and units

- The two configured poses (`hover_pose` / `grasp_pose`) are
  `[x, y, z, rx, ry, rz]` — metres + **radians** — in the same base frame
  and units as `jaka_arm`'s `end_pose` / `pos_command` (the controller's
  configured TCP). RPY is the controller's fixed-XYZ convention.
- Gripper values are metres of opening: `0.098` = fully open, `0.0` =
  closed. The TG-9801 adapter is binary: `>= 0.049` releases, below grips
  adaptively with the force configured in `jaka_arm`.

## Tools

| contract | tool (pilot) | behaviour |
|---|---|---|
| `robonix/skill/jaka/grab` | `jaka_grab` | Open gripper in place → move to hover → descend to grasp → close gripper → settle → lift back to hover and hold. A failed lift is non-fatal (the object is already in the fingers). Poses from config; takes no arguments. |
| `robonix/skill/jaka/release` | `jaka_release` | Open the gripper where the arm is — zero arm motion by construction (only `gripper_joint` is commanded; the provider back-fills the arm joints from its live feedback). Takes no arguments. |
| `robonix/skill/jaka/pick` | `jaka_pick` | Locate the named `object` with the camera (yolo11-obb or VLM backend, `detect_scheme`), map the bbox centre to arm base_link XY via the calibrated 2D homography, then run the same grasp segment as grab at `(x, y, desktop_height)` — on the yolo path with the gripper yaw aligned to the object's long edge. Takes one argument: `object` (the target description). |

All tools are synchronous (block until done). `grab` / `release` take no
arguments; `pick` takes `object`.

## pick — perception path and calibration

`pick` reuses grab's grasp segment but computes the grasp XY from the camera
instead of config. Detection runs through one of two backends
(`detect_scheme`):

- **yolo** (default in this deploy's manifest): in-process yolo11-obb on
  the undistorted frame → two-frame consistency (flicker filter) → class
  match (substring + `yolo_class_aliases`, e.g. 胡萝卜→carrot). Each hit
  carries the OBB rotation (5th `bbox_2d` element) and the grasp aligns the
  gripper yaw to the object's long edge:
  `rz = radians(grasp_yaw_offset_deg - deg(yaw))`.
- **vlm**: one open-vocabulary multimodal call returns a pixel bbox (no
  rotation — the grasp keeps the fixed `grasp_rpy`).

Shared tail of both paths:

1. resolve `robonix/primitive/camera/rgb` via atlas (pinned to
   `camera_provider_id`, default `orbbec_camera`);
2. grab one undistorted frame;
3. detect `object` (backend above);
4. the 2D `homography_matrix` maps the bbox centre to arm `base_link` XY;
5. grasp at `(x, y, desktop_height)` vertical-down; hover is that pose
   lifted by `travel_offset`.

`pick` is **unavailable until calibrated**: it returns a clear error naming
what is missing (`homography_matrix` and, per scheme, the VLM endpoint or
the `yolo_model_path` weights file) until set in the deploy config.
`grab` / `release` are unaffected and work without any of this. Calibrate
`homography_matrix` on the real cell at the operating resolution (1280×720)
— a wrong matrix or `desktop_height` sends the arm to the wrong place. See
`config.spec` for every key.

## Known limitations (operator must know)

- **No grasp verification.** The TG-9801's RS485 position reads are invalid
  on this arm, so the skill cannot tell whether anything is held or whether
  the fingers actually opened. It waits a fixed settle time and the response
  message says "command sent", never "verified". Observe the fingers.
- **No perception.** `grab` assumes the object is already positioned at the
  configured grasp pose. Detection/grasp-pose services are future work.
- **The `motion_enabled` safety gate also blocks gripper commands.** While
  `jaka_arm`'s `motion_enabled: false`, every command is silently rejected
  (only a rate-limited provider log line) — `release` would report success
  without the gripper moving. Pose waypoints time out with an explicit
  `motion_enabled` hint; gripper commands cannot be checked this way.
- Near the j5 singularity the provider's `pos_command` falls back to a
  joint-space IK move, so a "straight vertical" descent is not guaranteed
  on that path — keep the configured RPY constant and away from j5 limits.
