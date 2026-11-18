---
description: JAKA Mini grab/release/home/place/pick skill. grab closes the gripper at the configured grasp pose and lifts to hover; release opens the gripper in place with zero arm motion; home returns the six arm joints to the configured home target (default all-zeros) without touching the gripper; place sets a held object down — by default in place, right below where the arm holds it (hover, descend, open, lift), or at a configured place pose when one is calibrated; pick locates a named object with the camera (yolo11-obb or VLM backend) and grasps it via a calibrated 2D hand-eye homography; the yolo path also aligns gripper yaw to the object's long edge. grab/release/home need no perception and work out of the box; place in-place needs only desktop_height; pick needs a calibrated homography + a detector (yolo weights or VLM endpoint).
---

# JAKA grab/release/home/place/pick skill

Provider id `jaka`, namespace `robonix/skill/jaka`. LLM tool names:
`jaka_grab` / `jaka_release` / `jaka_home` / `jaka_place` / `jaka_pick`. The skill owns no hardware: arm
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
- The skill runs against two bodies with this same interface. The deploy
  manifest MUST set `end_effector: gripper|hand` (required; no auto mode).
  `gripper` reads the `gripper_joint` position directly; `hand` maps the
  LinkerHand O6 hand joints onto the same width scale (see
  `jaka_skill/feedback.py`). The O6 open pose reads 0.923 mean grasp-finger
  openness and an empty close 0.119; those anchors make the existing
  thresholds meaningful without a per-body skill branch. With `hand`, a
  stray `gripper_joint` is ignored; with `gripper`, hand joints are ignored.
  A missing/invalid `end_effector` is a config error surfaced on tool calls.
  Commands are unchanged: `gripper_joint` publishes route to the primitive,
  which maps them to release/grip (gripper) or open/full-grasp (hand).
- O6 caveat: a lost RS485 answer reads as fully flexed. The verdict samples
  must be two settled readings, but a lost-answer cluster can still look
  like an empty close; do not treat one sweep as proof.

## Tools

| contract | tool (pilot) | behaviour |
|---|---|---|
| `robonix/skill/jaka/grab` | `jaka_grab` | Open gripper in place → move to hover → descend to grasp → close gripper → settle → lift back to hover and hold. A failed lift is non-fatal (the object is already in the fingers). Poses from config; takes no arguments. |
| `robonix/skill/jaka/release` | `jaka_release` | Open the gripper where the arm is — zero arm motion by construction (only `gripper_joint` is commanded; the provider back-fills the arm joints from its live feedback). Takes no arguments. |
| `robonix/skill/jaka/home` | `jaka_home` | Return the arm to its initial posture — a joint-space move of the six arm joints to the configured `home_joints` (default all-zeros, the JAKA reference posture) via `joint_command`. The gripper is NOT commanded, so anything held stays held. Takes no arguments. |
| `robonix/skill/jaka/place` | `jaka_place` | Set a held object down. Target, exactly one of: `x`, `y` (metres, arm base frame) — carried there and set down at `desktop_height` with `grasp_rpy`; or `u`, `v` (pixel in the camera's 1280×720 raw RGB frame) — undistorted with `camera_matrix`/`dist_coeffs` and mapped through `homography_matrix` to base XY; or neither (all four 0 = NO target), which falls back to the configured `place_pose`, then to IN PLACE (原地放下) at the arm's current XY. Giving both a metres and a pixel pair is an error. Hover above the target (z + `travel_offset`), descend, open the gripper, lift back to hover. The gripper keeps holding until the bottom (unlike `release`, which lets go mid-air where the arm is). |
| `robonix/skill/jaka/pick` | `jaka_pick` | Locate the named `object` with the camera (yolo11-obb or VLM backend, `detect_scheme`), map the bbox centre to arm base_link XY via the calibrated 2D homography, then run the same grasp segment as grab at `(x, y, desktop_height)` — on the yolo path with the gripper yaw aligned to the object's long edge. Takes one argument: `object` (the target description). |

All tools are synchronous (block until done). `grab` / `release` / `home`
take no arguments; `pick` takes `object`; `place` takes `x`,
`y` (metres) and `u`, `v` (camera pixels) — exactly one pair, 0/0 for the
pair you are not using; all four 0 = no target. `place` works uncalibrated
(in place, `desktop_height`); setting a `place_pose` in the deploy config
gives the no-target case a destination, and `homography_matrix` +
`camera_matrix` + `dist_coeffs` enable the pixel pair.

## pick — perception path and calibration

`pick` reuses grab's grasp segment but computes the grasp XY from the camera
instead of config. Detection runs through one of three backends
(`detect_scheme`):

- **yolo**: in-process yolo11-obb on
  the undistorted frame → two-frame consistency (flicker filter) → class
  match (substring + `yolo_class_aliases`, e.g. 胡萝卜→carrot). Each hit
  carries the OBB rotation (5th `bbox_2d` element) and the grasp aligns the
  gripper yaw to the object's long edge:
  `rz = radians(grasp_yaw_offset_deg - deg(yaw))`.
- **vlm**: one open-vocabulary multimodal call returns a pixel bbox (no
  rotation — the grasp keeps the fixed `grasp_rpy`).
- **auto** (default in this deploy's manifest): routes each request by name
  — a name the loaded weights cover (alias map + the weights' own class
  names, substring either way) runs the yolo path above, anything else runs
  the vlm path. A yolo miss is **not** retried through the VLM, so
  "nothing on the table" is reported as such instead of becoming a
  hallucinated box; weights that fail to load degrade to the VLM.

Shared tail of all paths:

1. resolve `robonix/primitive/camera/rgb` via atlas (pinned to
   `camera_provider_id`, default `orbbec_camera`);
2. grab one undistorted frame;
3. detect `object` (backend above);
4. the 2D `homography_matrix` maps the bbox centre to arm `base_link` XY;
5. grasp at `(x, y, desktop_height)` vertical-down; hover is that pose
   lifted by `travel_offset`.

`pick` is **unavailable until calibrated**: it returns a clear error naming
what is missing (`homography_matrix` and, per scheme, the VLM endpoint, the
`yolo_model_path` weights file, or on `auto` either backend) until set in
the deploy config.
`grab` / `release` are unaffected and work without any of this. Calibrate
`homography_matrix` on the real cell at the operating resolution (1280×720)
— a wrong matrix or `desktop_height` sends the arm to the wrong place. See
`config.spec` for every key.

## Known limitations (operator must know)

- **The grasp verdict is device-reported when the stream is there.** On a
  `gripper` body (TG-9801) the skill waits for one fresh
  `robonix/primitive/gripper/state` round after the close (a round costs
  ~2.6 s, so the verdict lands ~3-6 s after the close) and takes the device's
  own contact report: `HOLDING`/`SLIPPING` count as grasped (slipping is
  flagged in the message), `NOT_HOLDING` with the jaws fully closed is an
  empty close (one retry, then failure), `NOT_HOLDING` with a wider opening
  is reported as unverified rather than as success. Without that stream (hand
  body, or a `jaka_arm` without the standard gripper contracts) it falls back
  to the settled `gripper_joint` width — two samples ≥5 s apart — which still
  catches an empty close but cannot separate a gripped object from a close
  that never executed.
- **`motion_state` is not an arrival signal.** The TG-9801 firmware keeps
  reporting "moving" for as long as the closing force is applied, even at the
  mechanical stop; the skill does not gate on it.
- **No perception.** `grab` assumes the object is already positioned at the
  configured grasp pose. Detection/grasp-pose services are future work.
- **The `motion_enabled` safety gate also blocks gripper commands.** While
  `jaka_arm`'s `motion_enabled: false`, every command is silently rejected
  (only a rate-limited provider log line) — `release` would report success
  without the gripper moving. Pose waypoints time out with an explicit
  `motion_enabled` hint; gripper commands cannot be checked this way.
- **`home` sweeps through the workspace.** It is a joint-space move: each
  joint interpolates independently toward `home_joints`, so the arm passes
  through whatever lies between its current pose and home. The gripper
  keeps its state (a held object travels with the arm) — clear the path
  before homing.
- Near the j5 singularity the provider's `pos_command` falls back to a
  joint-space IK move, so a "straight vertical" descent is not guaranteed
  on that path — keep the configured RPY constant and away from j5 limits.
