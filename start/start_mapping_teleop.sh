#!/usr/bin/env bash
# Simulation mapping with terminal controls; optional Gazebo viewer, no Qt.
set -euo pipefail
mapping_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
mapping_file="${1:-$mapping_root/start/maps/inspection_building.dmap}"
[[ $# -le 1 ]] || { echo 'Usage: bash start/start_mapping_teleop.sh [output-map.dmap]'; exit 2; }
source /opt/ros/noetic/setup.bash
source "$mapping_root/devel/setup.bash"
export DRONE_SIM_PROFILE=building DRONE_SHOW_GAZEBO="${DRONE_SHOW_GAZEBO:-0}" DRONE_SHOW_QT=false DRONE_OPTIONAL_MANAGERS=false
export ROS_MASTER_URI="${DRONE_ROS_MASTER_URI:-http://127.0.0.1:11311}" ROS_HOSTNAME=127.0.0.1
export ROS_HOME="${DRONE_ROS_HOME:-/tmp/drone_ros_home}" OPENBLAS_NUM_THREADS=1
mapping_log="$mapping_root/start/logs/building_mapping_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$mapping_log"
mapping_children=()
cleanup(){ for pid in "${mapping_children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
DRONE_LOG_DIR="$mapping_log/world" bash "$mapping_root/start/start_gazebo.sh" > "$mapping_log/world_entry.log" 2>&1 &
mapping_children+=("$!")
python3 - <<'PY'
import rosgraph,time
for _ in range(150):
    try:
        if rosgraph.Master('/mapping_start').getParam('/drone/split_world_launcher'):break
    except Exception:pass
    time.sleep(.2)
else:raise SystemExit('ROS master未启动，请查看world_entry.log；本地TCP/UDP需要运行环境允许')
PY
DRONE_LOG_DIR="$mapping_log/navigation" bash "$mapping_root/start/start_navigation.sh" mapping > "$mapping_log/navigation_entry.log" 2>&1 &
mapping_children+=("$!")
echo "遥控建图记录：$mapping_log；finish需要先落地锁定，并验证两层连通"
teleop_script=teleop_building_mapping.py
[[ "${DRONE_TELEOP_MODE:-commands}" != keyboard ]] || teleop_script=keyboard_building_mapping.py
python3 "$mapping_root/src/drone_stack/scripts/$teleop_script" --map "$mapping_file" --log "$mapping_log/commands.jsonl"
