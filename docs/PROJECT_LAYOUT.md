# 项目目录与模块职责

## 运行组件

| 路径 | 用途 | 运行入口/关键文件 |
|---|---|---|
| `src/drone_stack` | ROS 集成层：定位桥接、共享地图、飞行与健康管理、Qt、巡检、AI、仿真模型 | `launch/stack.launch`、`scripts/flight_manager.py`、`src/operator_gui.cpp` |
| `src/ego_planner` | EGO 局部优化、三维体素地图和全局参考路线消息/服务 | `plan_manage`、`plan_env` |
| `src/faster_lio_main` | 点云去畸变、IMU传播、点云匹配与 LIO 位姿输出 | `launch/`、`config/`、`src/` |
| `src/livox_ros_driver2` | Livox 消息定义与驱动依赖 | ROS1 驱动配置 |
| `external/PX4-Autopilot` | PX4 SITL、Gazebo Classic 仿真插件与固件 | `start_px4_sim.sh` 调用的 PX4 启动流程 |
| `src/drone_stack/worlds` | 双层楼道与较小的诊断场景 | 默认 `inspection_building.world` |
| `start` | 仿真、导航、建图、建筑覆盖和进程清理入口 | `start_gazebo.sh`、`start_navigation.sh`、`stop_all.sh` |

## 已移除的旧模块

`src/controller` 是早期地面机器人控制器，包含与当前无人机无关的 ROS 控制、视觉脚本和模型权重；它不属于当前 catkin 工作区。旧 `start/startall.sh` 仍引用该包、旧 project1 路径和已弃用的启动方式，已删除。当前启动统一使用 `start/` 下的入口脚本。

## 本地生成数据

以下内容与机器、地图或单次实验绑定，不应作为源码提交：

- `build/`、`devel/`、Gazebo/PX4 构建输出。
- `start/logs/`、`start/maps/`、`start/inspection_records/`、`start/inspection_regions/`。
- 完整 rosbag、逐次运行日志、未筛选原始遥测和一次性日志；报告需要的汇总 CSV/JSON 保留在 `reports/`。
- `start/private/` 中的密钥文件。

`reports/` 只保留可引用的摘要、指标表、图表、关键逐组 JSON 和复现脚本。大体量飞行包、临时源码快照和失败重试记录不纳入发布。

## 开发注意事项

- ROS 坐标使用 ENU/FLU。MAVROS 在 PX4 边界完成 NED/FRD 转换，ROS 发布端不要再次手动转换。
- `/mavros/odometry/out` 只把 LIO 位姿交给 PX4 外部视觉融合；不融合其中的速度字段。PX4 IMU仍用于飞控状态传播和速度估计。
- A*、EGO、飞行管理器、轨迹安全审核共用实时三维占据/空闲地图；Qt 上的路径显示来自同一条控制路线。
- `start/stop_all.sh` 是进程清理工具，不会执行降落。结束实验时先降落并确认上锁。
