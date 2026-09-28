#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
ROS_SETUP="${RBNX_ROS_SETUP:-/opt/ros/humble/setup.bash}"

# ROS setup.bash references unset variables under `set -u`; relax it around
# the source calls. The ros2_idl overlay carries the rosidl-generated
# sensor_msgs / geometry_msgs modules (see scripts/build.sh).
set +u
source "$ROS_SETUP"
source "$PKG_ROOT/rbnx-build/codegen/ros2_idl/install/setup.bash"
set -u
cd "$PKG_ROOT"

if [[ ! -d "$PKG_ROOT/rbnx-build/codegen/proto_gen" || ! -d "$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types" ]]; then
  echo "[jaka_skill/start] ERR: codegen output missing — run scripts/build.sh first" >&2
  exit 2
fi

export PYTHONPATH="$PKG_ROOT/rbnx-build/codegen/proto_gen:$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

# python3 resolves to the deploy venv interpreter (manifest env: PATH).
exec python3 -u -m jaka_skill.atlas_bridge
