#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# Build phase: rbnx codegen only — no vendored ROS packages, no colcon
# workspace of our own. The skill is pure Python and talks to the jaka_arm
# primitive exclusively through ROS 2 topics it resolves via atlas.
#
# Outputs (mirrors primitive-jaka-rbnx/scripts/build.sh):
#   proto_gen / robonix_mcp_types  — framework protos + jaka_mcp dataclasses
#   codegen/ros2_idl (colcon)      — sensor_msgs / geometry_msgs overlay;
#   on this host /opt/ros/humble alone does not put those on the python
#   path, so start.sh sources this overlay.
#
# Codegen interpreter: RBNX_CODEGEN_PYTHON is exported by the deploy repo's
# build.sh (deploy venv with grpcio-tools); falls back to plain python3.
set +u
source "$ROS_SETUP"
set -u
rbnx codegen -p "$PKG" --mcp --ros2 --python "${RBNX_CODEGEN_PYTHON:-python3}"
cd "$PKG/rbnx-build/codegen/ros2_idl"
colcon build

echo "[jaka_skill] build done"
