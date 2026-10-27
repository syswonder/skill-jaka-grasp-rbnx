#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# Build phase: package-local venv + rbnx codegen — no vendored ROS packages,
# no colcon workspace of our own. The skill is pure Python and talks to the
# jaka_arm primitive exclusively through ROS 2 topics it resolves via atlas.
#
# Self-contained by design (mirrors the jaka arm primitives' scripts/build.sh):
# the venv is created here with --system-site-packages (rclpy / cv2 / numpy
# from the system installs stay visible) and doubles as the codegen
# interpreter, so the generated stubs and the runtime protobuf always agree.
# start.sh execs "$PKG/.venv/bin/python" by absolute path — a deployment-
# level venv on PATH is not dependable (under `robonix-console` boots the
# manifest's "${PWD}/.venv" expands against the console's PWD, not the
# deployment directory).
#
# Outputs (mirrors the jaka arm primitives' scripts/build.sh):
#   proto_gen / robonix_mcp_types  — framework protos + jaka_mcp dataclasses
#   codegen/ros2_idl (colcon)      — sensor_msgs / geometry_msgs overlay;
#   on this host /opt/ros/humble alone does not put those on the python
#   path, so start.sh sources this overlay.

VENV="$PKG/.venv"
if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install -q -r "$PKG/requirements.txt"

set +u
source "$ROS_SETUP"
set -u
rbnx codegen -p "$PKG" --mcp --ros2 --python "$VENV/bin/python"
cd "$PKG/rbnx-build/codegen/ros2_idl"
colcon build

echo "[jaka_skill] build done"
