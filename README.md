# skill-jaka-grasp-rbnx — JAKA Mini grab/release/home/place/pick skill (+ 固定抓取)

Robonix skill for the fixed-base JAKA Mini, usable with **either** body
deployment — JAKA Mini + TG-9801 gripper or JAKA Mini + LinkerHand O6 hand —
from this one package. Each deploy manifest declares its body with the
required skill config `end_effector: gripper|hand`; the skill then reads the
matching feedback (gripper_joint width, or a width reconstructed from the O6
hand joints). Commands stay `gripper_joint` in both cases; the primitive
translates them. Six synchronous MCP tools:

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
- `jaka_fixed_grasp` — fixed-pose pick-and-place (固定抓取):
  grasp at the calibrated `fixed_grasp_pose`, set down at the calibrated
  `fixed_place_pose`. No perception, no arguments; both hovers are
  z + `travel_offset`.
- `jaka_pick` — locate a named object with the camera (yolo11-obb or
  open-vocabulary VLM backend, `detect_scheme`) and grasp it: 2D hand-eye
  homography maps the detection to arm base_link XY. Requires on-site
  calibration (`homography_matrix`, detector config — see config.spec);
  reports a clear config error until then.

Everything is driven through the `jaka_arm` primitive's ROS 2 contracts
(`pos_command` / `joint_command`), resolved via atlas. See CAPABILITY.md for
frames, units and known limitations.

## End-effector compatibility

Two body deployments, one skill, selected by the **required** deploy-manifest
config key:

```yaml
skill:
  - name: jaka
    config:
      end_effector: hand      # gripper | hand — required, no auto mode
```

| `end_effector` | body | `joint_states` feedback | skill behaviour |
|---|---|---|---|
| `gripper` | JAKA Mini + TG-9801 | `gripper_joint` (m) | width read directly; empty-grasp / open verdicts use the TG stroke (0.098 m) |
| `hand` | JAKA Mini + LinkerHand O6 | six O6 hand joints (rad), no `gripper_joint` | width reconstructed from thumb/index/middle/ring flexion, anchored on the measured O6 open/empty-close poses (see `feedback.py`); the same 0.098 m scale and verdict thresholds apply |

There is deliberately no auto-detection: a missing or invalid
`end_effector` is a config error (tools report it) instead of silently
reading the wrong feedback. In `gripper` mode hand joints are ignored; in
`hand` mode a stray `gripper_joint` is ignored. Everything else is still
deployment config: the poses/heights are body-specific (TG 0.125/0.275 m,
O6 0.14/0.29 m in the respective deploy manifests), and `gripper_settle_s`
is longer for the O6.

Caveat: the O6 link drops answers that read as fully flexed, and the
reconstruction cannot distinguish that from a real close in a single
sample. The verdict already requires two settled samples
(`GRASP_VERIFY_TIMEOUT_S`), but a lost-answer cluster can still look like an
empty close; observe the hand for critical grasps.

## Running the 固定抓取 demo

1. This repo checked out on `demo` (the deploy's local manifest points at
   `../skill-jaka-grasp-rbnx`, so whatever branch is checked out is what boots).
2. Lower the grip for the fragile demo object (a raw egg) in the deploy
   manifest's `jaka_arm` config: `gripper_force: 20` (0-100 scale, applied
   once at connect — the skill cannot change it per call; verify on the
   real object, adjust down/up).
3. Calibrate the two fixed poses (teach the arm there, then
   `ros2 topic echo /jaka/end_pose --once`, ROS_DOMAIN_ID=0
   RMW_IMPLEMENTATION=rmw_zenoh_cpp) and set them in the deploy manifest's
   `skill: - name: jaka` config block:
   `fixed_grasp_pose` / `fixed_place_pose` (`[x, y, z, rx, ry, rz]`, metres +
   radians, vertical-down `[π, 0, 0]` typical). Mind reach: each hover is
   z + `travel_offset` and must stay inside the arm's envelope at that
   radial.
4. Rebuild + reboot the deploy (`bash build.sh && bash start.sh`), then
   `rbnx ask "固定抓取"`.

## Build

```bash
rbnx build -p ../skill-jaka-grasp-rbnx        # from the deploy repo
# or standalone:
bash scripts/build.sh
```

`build.sh` provisions a package-local `.venv` (runtime + codegen deps, see
`requirements.txt`) and uses its interpreter for codegen, so the generated
stubs and the runtime protobuf always agree. `start.sh` runs the venv
interpreter by absolute path (`"$PKG/.venv/bin/python"`) and does not depend
on `PATH` — under `robonix-console` boots a deployment-venv path on `PATH`
is not dependable. Outputs `rbnx-build/codegen/{proto_gen, robonix_mcp_types, ros2_idl}`.

## Boot

The deploy manifest declares the skill in its `skill:` section; soma stage 2
sends only CMD_INIT (skill stays INACTIVE) and the executor activates it on
the first tool call. Static checks after `bash start.sh`:

```bash
rbnx caps -v                              # jaka [INACTIVE] until first call
rbnx tools                                # jaka_grab / jaka_release / jaka_home / jaka_place / jaka_fixed_grasp / jaka_pick listed
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
