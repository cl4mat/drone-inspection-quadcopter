#!/usr/bin/env bash
# Actual flight -> CSV, PNG, PDF, ZIP. Truth is read-only evaluation.
set -euo pipefail
patrol_root="$(cd "$(dirname "$0")/.." && pwd)"
patrol_map="$patrol_root/start/maps/inspection_building.dmap"
if (( $# )); then patrol_map="$1"; fi
patrol_map="$(realpath "$patrol_map")"
[[ -f "$patrol_map.inspection.json" ]] || { echo '缺少已生成的实测地图巡检任务'; exit 2; }
source /opt/ros/noetic/setup.bash
source "$patrol_root/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_HOSTNAME=127.0.0.1
export ROS_HOME=/tmp/drone_ros_home OPENBLAS_NUM_THREADS=1 DRONE_SIM_AGGRESSIVE=true
patrol_out="$patrol_root/reports/two_floor_patrol_$(date +%Y%m%d_%H%M%S)"
if (( $# > 1 )); then patrol_out="$(realpath -m "$2")"; fi
mkdir -p "$patrol_out"
printf '%s\n' "$patrol_out" > "$patrol_root/start/ACTIVE_PATROL_EXPERIMENT"
patrol_owned=0
patrol_reuse=""
if python3 - <<'PY'
import rosgraph
try:
    m=rosgraph.Master('/patrol_existing_stack')
    assert m.getParam('/drone/simulation_profile')=='building'
except Exception:
    raise SystemExit(1)
PY
then
  patrol_reuse="--reuse-running"
else
  DRONE_SIM_PROFILE=building DRONE_SHOW_GAZEBO=0 DRONE_LOG_DIR="$patrol_out/world" \
    bash "$patrol_root/start/start_gazebo.sh" > "$patrol_out/world.log" 2>&1 &
  patrol_owned=1
  python3 - <<'PY'
import rosgraph,time
for _ in range(300):
    try:
        if rosgraph.Master('/patrol_world_wait').getParam('/drone/split_world_launcher'):break
    except Exception:pass
    time.sleep(.2)
else:raise SystemExit('世界入口启动超时')
PY
  DRONE_SHOW_QT=false DRONE_OPTIONAL_MANAGERS=false DRONE_LOG_DIR="$patrol_out/navigation" \
    bash "$patrol_root/start/start_navigation.sh" inspection "$patrol_map" > "$patrol_out/navigation.log" 2>&1 &
fi
python3 -u "$patrol_root/src/drone_stack/scripts/patrol_metrics.py" \
  --map "$patrol_map" --output "$patrol_out" \
  --world "$patrol_root/src/drone_stack/worlds/inspection_building.world" \
  > "$patrol_out/metrics.log" 2>&1 &
patrol_recorder=$!
echo "巡检及报告目录：$patrol_out"
patrol_result=0
# patrol_reuse is either empty or the fixed flag above.
python3 -u "$patrol_root/src/drone_stack/scripts/run_building_coverage.py" \
  --map "$patrol_map" --output "$patrol_out/execution" $patrol_reuse \
  > "$patrol_out/execution.log" 2>&1 || patrol_result=$?
if [[ "$patrol_result" != 0 ]]; then
  python3 - <<'PY'
import rospy,time
from std_srvs.srv import Trigger
from mavros_msgs.msg import State,ExtendedState
rospy.init_node('patrol_failure_finish')
state=rospy.wait_for_message('/mavros/state',State,timeout=5)
if state.armed:rospy.ServiceProxy('/drone/land',Trigger)()
deadline=time.monotonic()+180
while time.monotonic()<deadline:
    s=rospy.wait_for_message('/mavros/state',State,timeout=5)
    e=rospy.wait_for_message('/mavros/extended_state',ExtendedState,timeout=5)
    if not s.armed and e.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:break
else:raise RuntimeError('未确认落地，保留仿真和安全控制进程')
PY
  kill -INT "$patrol_recorder" 2>/dev/null || true
fi
wait "$patrol_recorder"
echo "报告包：$patrol_out.zip"
if [[ "$patrol_owned" != 0 ]]; then
  echo '仿真保留在地面，可使用 start/stop_all.sh 结束。'
fi
exit "$patrol_result"
