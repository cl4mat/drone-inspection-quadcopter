#!/usr/bin/env bash
# Entire stored building mission, no Qt/gzclient. Never creates a truth map.
set -euo pipefail
coverage_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
coverage_map="${1:-$coverage_root/start/maps/inspection_building.dmap}"
[[ $# -le 1 ]] || { echo 'Usage: bash start/run_building_coverage.sh [measured-map.dmap]'; exit 2; }
[[ -f "$coverage_map" && -f "$coverage_map.inspection.json" ]] || {
  echo '缺少新场景实际建图地图/整栋路线。先建图覆盖两层，落地锁定后完成建图生成任务；不会使用旧地图或Gazebo真值替代。'; exit 1;
}
coverage_map="$(realpath "$coverage_map")"
source /opt/ros/noetic/setup.bash
source "$coverage_root/devel/setup.bash"
export DRONE_SIM_PROFILE=building DRONE_SHOW_GAZEBO=0 DRONE_SHOW_QT=false
export ROS_MASTER_URI="${DRONE_ROS_MASTER_URI:-http://127.0.0.1:11311}" ROS_HOSTNAME=127.0.0.1
export ROS_HOME="${DRONE_ROS_HOME:-/tmp/drone_ros_home}" OPENBLAS_NUM_THREADS=1
coverage_log="$coverage_root/start/logs/building_coverage_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$coverage_log"
coverage_children=()
cleanup(){ for pid in "${coverage_children[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done; wait || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
DRONE_LOG_DIR="$coverage_log/world" bash "$coverage_root/start/start_gazebo.sh" > "$coverage_log/world_entry.log" 2>&1 &
coverage_children+=("$!")
python3 - <<'PY'
import rosgraph,time
for _ in range(150):
    try:
        if rosgraph.Master('/coverage_start').getParam('/drone/split_world_launcher'):break
    except Exception:pass
    time.sleep(.2)
else:raise SystemExit('仿真ROS master未启动，请查看world_entry.log')
PY
DRONE_LOG_DIR="$coverage_log/navigation" bash "$coverage_root/start/start_navigation.sh" inspection "$coverage_map" > "$coverage_log/navigation_entry.log" 2>&1 &
coverage_children+=("$!")
echo "完整巡检验收日志：$coverage_log"
python3 "$coverage_root/src/drone_stack/scripts/run_building_coverage.py" --map "$coverage_map" --output "$coverage_log/acceptance"
