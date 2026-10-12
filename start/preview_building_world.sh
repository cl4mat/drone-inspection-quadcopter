#!/usr/bin/env bash
# Environment preview only: no PX4, localization, flight manager or keyboard flight.
source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
set +u
source /usr/share/gazebo/setup.sh
set -u
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH="$project_root/src/drone_stack/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}"
preview_dir="$project_root/start/logs/world_preview_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$preview_dir"
setsid gzserver --verbose "$project_root/src/drone_stack/worlds/inspection_building.world" > "$preview_dir/server.log" 2>&1 &
process_groups+=("$!")
setsid gzclient --verbose > "$preview_dir/client.log" 2>&1 &
preview_client=$!;process_groups+=("$preview_client")
echo "已启动开放式双层场景预览，无飞行任务。日志：$preview_dir"
echo '关闭Gazebo窗口或Ctrl-C结束预览；之后可启动start_keyboard_mapping.sh建图。'
wait "$preview_client"
