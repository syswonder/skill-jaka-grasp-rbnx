config:
  # string, default: jaka_arm.
  # Provider id pinned when resolving the four arm contracts via atlas.
  # find_unique_capability raises on >1 match, so pinning makes the skill
  # immune to a second arm appearing in the manifest.
  arm_provider_id: jaka_arm

  # ── end-effector feedback (no config key) ──────────────────────────────
  # The same skill runs with a TG-9801 gripper body or a LinkerHand O6 hand
  # body. Feedback is selected per `joint_states` sample:
  #   - a `gripper_joint` position (m) is used as the width directly;
  #   - otherwise the O6 hand joints are mapped onto the same 0.098 m
  #     width scale from thumb/index/middle/ring flexion (measured open
  #     0.923 / empty-close 0.119 anchors), so every verdict threshold
  #     below keeps its meaning.
  # The heights/poses are BODY-SPECIFIC and live in each deploy manifest:
  # TG-9801 0.125 m grasp / 0.275 m hover; O6 0.14 m grasp / 0.29 m hover
  # (pick travel 0.08). The defaults below are the TG-9801 values.

  # list of 6 floats, REQUIRED, no default. [x, y, z, rx, ry, rz].
  # METRES + RADIANS, in the same base frame and units as jaka_arm's
  # end_pose / pos_command. RPY uses the controller's fixed-XYZ convention
  # (the skill converts to a quaternion with the exact inverse of the
  # provider's quat->RPY step).
  # Height model defaults (TG-9801 body, from AIIT-Roboarm/config.yaml, same
  # controller and base frame):
  #   hover z  = default_desktop_height + catch_raise_height = 0.125 + 0.15
  #   grasp z  = default_desktop_height = 0.125
  # The O6 hand body overrides these in its deploy manifest (0.29 / 0.14,
  # pick travel 0.08) because its palm is longer and reaches lower.
  hover_pose: [0.20, 0.00, 0.275, 3.14159265, 0.0, 0.0]
  grasp_pose: [0.20, 0.00, 0.125, 3.14159265, 0.0, 0.0]

  # float, default: 0.098. metres. Gripper opening command for release and
  # the pre-approach open. MUST be >= 0.049 — the TG-9801 is binary in this
  # adapter (>= half stroke = release, below = adaptive grip); a lower value
  # would silently CLOSE the gripper and is rejected as a config error.
  gripper_open_m: 0.098

  # float, default: 0.0. metres. Gripper close command for grab (< 0.049).
  gripper_close_m: 0.0

  # bool, default: true. Open the gripper in place before approaching.
  open_before_approach: true

  # float, seconds, default: 1.5. Fixed wait after every gripper command.
  # RS485 reads are invalid on this arm, so there is no feedback path;
  # if a fresh gripper_joint reading does appear and matches expectation,
  # the wait returns early (free upside).
  gripper_settle_s: 1.5

  # int, default: 5. Number of repeated gripper command publishes.
  gripper_repeats: 5

  # float, seconds, default: 0.1. Period between gripper command repeats.
  gripper_publish_period_s: 0.1

  # float, seconds, default: 2.0. Freshness window for joint_states used as
  # the liveness precondition before any command.
  feedback_timeout_s: 2.0

  # float, metres, default: 0.002. Position tolerance for waypoint reach,
  # mirrors the provider's own 2 mm check (driver.py _wait_pose_reached).
  reach_tolerance_m: 0.002

  # float, seconds, default: 20.0. Per-waypoint timeout. Must exceed
  # jaka_arm's move_timeout_s (10.0) — the provider may run linear_move
  # plus an IK-fallback joint move back to back inside one command.
  pose_timeout_s: 20.0

  # float, seconds, default: 0.05. Poll period for feedback waits.
  poll_s: 0.05

  # float, metres, default: 0.005. hover_pose and grasp_pose closer than
  # this total distance are rejected as a config error.
  min_grasp_descent_m: 0.005

  # map, optional. Overrides for the four ROS 2 topic names. Keys:
  # joint_states / end_pose / joint_command / pos_command. Topics are
  # resolved via atlas first (pinned to arm_provider_id); an explicit
  # override here wins and skips atlas resolution for that topic.
  # topics: { joint_states: /jaka/joint_states }

  # ─────────────────────── home (复位) ───────────────────────────────
  # Only the `home` tool uses the keys below. The move is JOINT-space, so
  # it works from any arm posture with no IK/reach/singularity concerns;
  # the gripper is never commanded (anything held stays held).

  # list of 6 floats, radians, default: [0, 0, 0, 0, 0, 0]. Joint target
  # of the home move. All-zeros is the JAKA reference posture (arm
  # upright); override per-deploy if a different parked posture is wanted.
  # Values must sit inside the controller's joint limits — the provider
  # rejects the move otherwise (skill-side symptom: home/move timeout).
  # home_joints: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

  # float, radians, default: 0.02 (~1.1 deg). Max per-joint error for the
  # home move to count as reached (mirrors the provider's ~1 deg reach
  # check). The wait itself reuses pose_timeout_s.
  joint_reach_tolerance_rad: 0.02

  # ─────────────────────── place (放置) ────────────────────────────────
  # Only the `place` tool uses the key below. The flow is grab's mirror:
  # hover above the place pose -> descend -> open the gripper -> lift;
  # the gripper keeps holding until the bottom.
  #
  # DEFAULT (key unset): place in place (原地放下) — the object is set down
  # right below where the arm holds it: current XY from end_pose at
  # desktop_height, orientation grasp_rpy. No calibration needed beyond
  # what pick already requires (desktop_height).

  # list of 6 floats, OPTIONAL, default: unset (in-place place). [x, y, z,
  # rx, ry, rz] — metres + RADIANS, same frame/units as hover_pose/
  # grasp_pose: carry the held object to this pose and set it down there.
  # Hover is derived as z + travel_offset (the pick height model — mind
  # travel_offset's reach note: on this arm the hover must stay inside the
  # reachable envelope at that radial).
  # place_pose: [0.20, 0.10, 0.125, 3.14159265, 0.0, 0.0]
  # ──────────────── fixed_grasp (demo, fixed poses) ────────────────
  # Only the `fixed_grasp` tool uses the keys below. Both REQUIRED for
  # fixed_grasp; until set, `fixed_grasp` reports unavailable
  # (grab/release/pick unaffected). Each hover is derived as z +
  # travel_offset (pick's height model — mind the reach note there).
  # THE DEMO OBJECT IS FRAGILE (a raw egg): grip force is jaka_arm's
  # one-time gripper_force config (0-100), LOWER it on-site and verify on
  # the real object; arm speeds likewise come from jaka_arm.

  # list of 6 floats, metres + radians. Where the object is picked up.
  # fixed_grasp_pose: [0.20, -0.10, 0.125, 3.14159265, 0.0, 0.0]

  # list of 6 floats, metres + radians. Where the object is set down.
  # fixed_place_pose: [0.20, 0.10, 0.125, 3.14159265, 0.0, 0.0]

  # ─────────────────────── pick (perception) ───────────────────────────
  # Only the `pick` tool uses the keys below. grab/release need NONE of
  # them — if pick is left uncalibrated, grab/release still work; `pick`
  # returns a clear "unavailable" error naming what is missing.

  # string, default: orbbec_camera. Provider id pinned when resolving the
  # camera/rgb contract via atlas.
  camera_provider_id: orbbec_camera

  # string, optional. Explicit RGB image topic. Empty = resolve
  # robonix/primitive/camera/rgb via atlas (pinned to camera_provider_id).
  # camera_topic: /camera/color/image_raw

  # 3x3 matrix, REQUIRED for pick, no default. 2D hand-eye homography that
  # maps an undistorted image pixel (u, v) to arm base_link (x, y) in
  # metres. Deploy-specific — CALIBRATE ON THE REAL CELL at the operating
  # resolution (1280x720), same kind of matrix as agilex grasp_pose. Until
  # set, `pick` reports unavailable.
  # homography_matrix:
  #   - [ ..., ..., ... ]
  #   - [ ..., ..., ... ]
  #   - [ ..., ..., ... ]

  # 3x3 matrix + list, optional. Camera intrinsics for undistortion. The
  # homography is calibrated on undistorted pixels, so supplying these
  # keeps corner accuracy (~30px error otherwise). Omit = frames used raw.
  # camera_matrix:
  #   - [fx, 0, cx]
  #   - [0, fy, cy]
  #   - [0, 0, 1]
  # dist_coeffs: [k1, k2, p1, p2, k3]

  # float, metres, default: 0.125. Table height in arm base_link — the
  # grasp z for every pick. MUST match the real mounting (same frame as
  # grab's grasp_pose z). A wrong value = empty grasps or table collisions.
  desktop_height: 0.125

  # float, metres, default: 0.15. Hover/lift height above desktop_height
  # (hover z = desktop_height + travel_offset).
  travel_offset: 0.15

  # list of 3 floats, radians, default: [pi, 0, 0] (vertical top-down).
  # Fixed end-effector orientation for the grasp, controller fixed-XYZ RPY.
  # On the yolo path rz is overridden per-pick by the gripper-yaw alignment
  # (see grasp_yaw_offset_deg); rx/ry always come from here.
  # grasp_rpy: [3.14159265, 0.0, 0.0]

  # ─────────────── detection backend ───────────────
  # string, "vlm" | "yolo", default: vlm. Which detector `pick` uses.
  detect_scheme: vlm

  # string, REQUIRED for pick when detect_scheme=yolo. Path to a trained
  # yolo11-obb .pt (in-process ultralytics; classes come from the weights).
  # The deploy reuses the agilex fruit model best.pt
  # (carrot/potato/tomato). Until the file exists, `pick` reports
  # unavailable naming the path.
  # yolo_model_path: /path/to/best.pt

  # float, default: 0.6 (range 0.01-1.0). YOLO confidence threshold.
  yolo_conf: 0.6

  # float, default: 0.45 (range 0.01-1.0). YOLO NMS IoU threshold.
  yolo_iou: 0.45

  # float, seconds, default: 0.7. YOLO two-frame consistency: second frame
  # is taken this long after the first; 0 disables the filter.
  yolo_two_frame_interval_s: 0.7

  # float, default: 0.3. Min IoU between the two frames for a detection to
  # survive (same class; filters flicker false positives).
  yolo_two_frame_iou: 0.3

  # map, optional. User phrasing -> yolo class name, tried before substring
  # matching (English class names match by substring already).
  # yolo_class_aliases: {胡萝卜: carrot}

  # float, degrees, default: -90. Gripper-yaw alignment for rotated (yolo
  # obb) detections: rz = radians(grasp_yaw_offset_deg - deg(yaw)), where
  # yaw comes from the bbox long edge. -90 equals AIIT's tuned JAKA value
  # (jaka_catch_rot_offset_deg) in this skill's euler family; if the fingers
  # land 90/180 deg off the object's long edge, tune by ±90 and reboot.
  grasp_yaw_offset_deg: -90

  # VLM open-vocabulary detector (REQUIRED for pick when detect_scheme=vlm).
  # One multimodal call returns a pixel bbox for the requested object (no
  # rotation — grasp keeps the fixed grasp_rpy). Reuse the deploy's VLM key
  # (see start.sh / .vlmkey) or a separate vision endpoint.
  # vlm_base_url: https://api.example.com/v1
  # vlm_api_key: <YOUR_VLM_API_KEY>
  # vlm_model: <a MULTIMODAL model — must accept image input>
  vlm_timeout_s: 60.0

  # float, seconds, default: 1.0. A camera frame older than this is stale.
  frame_max_age_s: 1.0

  # float, seconds, default: 5.0. Wait up to this for the first frame after
  # activation before a pick gives up.
  frame_wait_s: 5.0
