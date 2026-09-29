# skill-jaka-rbnx — JAKA Mini grab/release/home/place/pick skill

Robonix skill for the fixed-base JAKA Mini + TG-9801 gripper. Five
synchronous MCP tools:

- `jaka_grab` — open gripper → hover → descend to the configured grasp pose
  → close gripper → settle → lift back to hover and hold. No perception;
  poses come from the deploy config.
- `jaka_release` — open the gripper in place, zero arm motion.
- `jaka_home` — return the arm to its initial posture (复位): a joint-space
  move of the six arm joints to the configured `home_joints` (default
  all-zeros, the JAKA reference posture). The gripper is not commanded.
- `jaka_place` — set a held object down (放置). Default is in place
  (原地放下): the arm's current XY at `desktop_height` — hover, descend,
  open the gripper, lift. The gripper keeps holding until the bottom
  (unlike `release`); calibrate a `place_pose` to carry the object to a
  configured spot instead.
- `jaka_pick` — locate a named object with the camera (yolo11-obb or
  open-vocabulary VLM backend, `detect_scheme`) and grasp it: 2D hand-eye
  homography maps the detection to arm base_link XY. Requires on-site
  calibration (`homography_matrix`, detector config — see config.spec);
  reports a clear config error until then.

Everything is driven through the `jaka_arm` primitive's ROS 2 contracts
(`pos_command` / `joint_command`), resolved via atlas. See CAPABILITY.md for
frames, units and known limitations.

## Build

```bash
rbnx build -p ../skill-jaka-rbnx        # from the deploy repo
# or standalone (needs grpcio-tools on the codegen interpreter):
RBNX_CODEGEN_PYTHON=/home/nvidia/robot-jaka-mini/.venv/bin/python \
  rbnx build -p /home/nvidia/skill-jaka-rbnx
```

Outputs `rbnx-build/codegen/{proto_gen, robonix_mcp_types, ros2_idl}`.

## Boot

The deploy manifest declares the skill in its `skill:` section; soma stage 2
sends only CMD_INIT (skill stays INACTIVE) and the executor activates it on
the first tool call. Static checks after `bash start.sh`:

```bash
rbnx caps -v                              # jaka [INACTIVE] until first call
rbnx tools                                # jaka_grab / jaka_release / jaka_home / jaka_place / jaka_pick listed
rbnx contracts -p robonix/skill/jaka
rbnx describe --provider jaka
```

## Staged acceptance

1. **Read the current TCP pose** for calibration:
   `ros2 topic echo /jaka/end_pose --once`
   (ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_zenoh_cpp).
2. **release first** (gripper moves, arm must not): `rbnx ask "open the
   gripper"` — sample `end_pose` before/after; position delta must be
   ≤ 2 mm. Observe the fingers opening.
3. **grab with small descent steps**: set `grasp_pose` x/y/rpy from step 1,
   hover z = 0.275, then lower grasp z stepwise 0.20 → 0.175 → 0.15 → 0.125
   (AIIT verified desktop height, config.yaml:34), rebooting between edits.
4. Write the calibrated `hover_pose` / `grasp_pose` back into the deploy
   manifest.

## Safety notes

- The skill assumes `jaka_arm.config.motion_enabled: true`. With it false
  every command — including the gripper — is silently rejected; pose
  waypoints time out with an explicit hint, gripper commands do not (no
  feedback path exists).
- Pose-timeout error text always names `motion_enabled` as the first
  suspect when the arm never moved.
