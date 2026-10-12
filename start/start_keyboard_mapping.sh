#!/usr/bin/env bash
# Gazebo viewer + keyboard mapping, without Qt. Does not arm automatically.
set -euo pipefail
export DRONE_SHOW_GAZEBO="${DRONE_SHOW_GAZEBO:-1}"
export DRONE_TELEOP_MODE=keyboard
exec bash "$(dirname "${BASH_SOURCE[0]}")/start_mapping_teleop.sh" "$@"
