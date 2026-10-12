# 四旋翼自主巡检与避障

ROS Noetic / PX4 SITL / Gazebo Classic 工作区，实现基于 MID360 点云的 Faster-LIO 定位、三维体素地图、A* 全局参考路线、EGO-Planner 局部轨迹、飞行安全管理和 Qt 操作台。

当前以仿真为主。本文所列仿真数据不构成实机验收；新加入的区域巡检、AI文本控制和双层楼道流程仍需按各功能文档完成端到端验证。

## 当前功能

- Gazebo 四旋翼仿真，包含前倾 MID360、独立雷达 IMU、前视相机和下视相机。仿真模型不提供 GPS 或气压计定位来源。
- Faster-LIO 输出位姿；桥接按传感器外参换算机体中心，并向 MAVROS 发布 ENU 外部里程计。PX4 EKF 不融合外部速度，飞控 IMU仍用于飞控状态传播和速度估计。
- 共享三维占据、空闲和膨胀体素地图；在线建图或加载 `.dmap` 地图。
- 三维 A* 生成实际参与控制的平滑全局参考，EGO-Planner 滚动生成局部 B 样条；曲线经地图和空闲空间审核后才执行。
- Qt 综合操作台支持目标队列、地图与路径显示、状态监视、处理后相机画面、授权/ARM/起飞/HOLD/降落等操作。
- 预建地图下的区域巡检、已知区域保存，以及受限的 DeepSeek 文字任务入口。

详见：[系统架构](docs/ARCHITECTURE.md)、[运行时规划流程](docs/PLANNING_RUNTIME.md)、[区域巡检](docs/INSPECTION.md)、[AI文本控制](docs/AI_CONTROL.md)、[目录说明](docs/PROJECT_LAYOUT.md)、[实机验收清单](src/drone_stack/HARDWARE_ACCEPTANCE.md)。

## 环境与编译

推荐环境为 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 和 PX4 SITL。依赖需包含 catkin、MAVROS、Qt/RViz、PCL、Eigen、OpenCV、Livox ROS 驱动及 Gazebo 插件。

```bash
git clone --recursive https://github.com/cl4mat/drone-inspection-quadcopter.git project0
cd project0
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l1
source devel/setup.bash
```

PX4 仿真源码在 `external/PX4-Autopilot`。首次构建可能由仿真启动器执行，耗时取决于本机依赖和构建缓存。

## 仿真启动

建议分两个终端，先启动仿真世界，再启动导航和 Qt。

终端 A：

```bash
cd /path/to/project0
bash start/start_gazebo.sh
```

默认使用双层开放楼道场景，Gazebo 图形窗口默认关闭以节约资源。需要显示时：

```bash
DRONE_SHOW_GAZEBO=1 bash start/start_gazebo.sh
```

终端 B（二选一）：

```bash
cd /path/to/project0

# 在线建图，不加载已有地图
bash start/start_navigation.sh mapping

# 加载本地预建地图进行导航/巡检
bash start/start_navigation.sh inspection /path/to/map.dmap
```

地图模式下需要在 Qt 中确认初始位置和朝向。启动器进行话题和节点就绪检查，但不会自动授权、解锁或起飞。无地图或错误地图时，不要依赖演示地图替代真实建图结果。

停止前先让无人机降落并确认已上锁，再运行：

```bash
bash start/stop_all.sh
```

键盘遥控建图入口会同时启动仿真与导航：

```bash
bash start/start_keyboard_mapping.sh /path/to/output.dmap
```

## 验证摘要

当前可引用的精简仿真结果位于 [`reports/`](reports/README.md)：包含建图与巡检摘要、地图与 ICP 图、动态避障的逐组汇总和原始审计指标。双层楼道动态避障本轮按“新增障碍被感知且无碰撞”判据为 10/10；其中 9/10 到达目标并回归原路径。第 7 组 EGO 因候选路径触及障碍膨胀安全边界而拒绝发布，飞机保持安全悬停但没有完成导航目标。有限仿真结果不表示任意环境无碰撞，也不代表实机通过。

Qt端到端状态时延和人工接管响应时延尚无正式测量值；文档不以界面刷新周期替代实测延迟。双层楼道自动覆盖、AI云端联调和实机飞行仍需进一步验收。

## 仓库内容

```text
src/drone_stack/       系统集成、Qt、飞行管理、地图会话、巡检和仿真工具
src/ego_planner/       EGO-Planner 及其全局参考路线接口改造
src/faster_lio_main/   Faster-LIO
src/livox_ros_driver2/ Livox 驱动消息与依赖
external/PX4-Autopilot PX4 仿真固件与 Gazebo Classic 插件
start/                 当前仿真、导航、建图和清理入口
src/drone_stack/worlds/仿真世界
reports/               精简后的实验摘要和可引用图表
```

本地地图、日志、录包、Qt私有配置和构建输出不属于源码发布内容。DeepSeek 密钥通过 `start/private/deepseek.local` 从本机读取，不应提交。
