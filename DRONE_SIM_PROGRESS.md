# 项目当前状态

最后整理：2026-10-12。本文是当前状态摘要，不再记录逐轮终端日志；详细架构与接口见 [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)，导航运行过程见 [`docs/PLANNING_RUNTIME.md`](docs/PLANNING_RUNTIME.md)，精简实验依据见 [`reports/README.md`](reports/README.md)。

## 当前系统

- ROS Noetic、PX4 SITL、Gazebo Classic；默认双层开放楼道世界，MID360 模拟雷达前倾 30°，有独立雷达 IMU、前视和下视相机，不配置 GPS/气压计作为定位源。
- Faster-LIO 输出机体位姿；通过 MAVROS 外部视觉里程计融合，外部速度字段不融合。飞控 IMU仍参与PX4状态传播和速度估计。
- 一套实时三维占据/空闲/膨胀体素地图，支持在线建图与 `.dmap` 加载。
- 三维 A* 平滑全局参考、EGO 滚动局部规划、轨迹安全审核、目标队列、朝向跟随、终点减速和健康保护。
- Qt 综合页、区域巡检页、AI文本页；支持地图、路径、两路处理画面、状态监控及授权/ARM/起飞/HOLD/降落。

## 关键仿真结果

- 双层楼道建图与巡航曾完成一轮多区域仿真飞行，结果与图表整理在 `reports/two_floor_patrol_20261009_153401/`。该结果不是严格全覆盖的独立证明。
- 0.50 m 水平膨胀配置的动态障碍实验共10组。按“障碍已感知且全程无碰撞”的约定，10/10无碰撞；导航到达目标并回归原路线分别为9/10。第7组 EGO 拒绝了触及膨胀障碍边界的候选路径，无碰撞但未完成目标。
- 点云/地图重合分析对象是0.10 m占据存档体素中心，不是原始Faster-LIO全局PCD。Qt端到端延迟尚无正式测量数据。
- 所有结果只说明记录的仿真场景与条件，不表示任意场景或实机通过。

## 尚需验收

1. 新版分离启动入口、地图出生位姿选择、区域巡检页面完成整套ROS/Gazebo交互验收。
2. 双层场景自动巡检路线与返航按统一、可复核的覆盖判据完成验收。
3. AI云端解析、执行授权、取消/超时行为完成联调；云密钥只存在本机忽略文件。
4. 补测Qt状态回传和HOLD接管的端到端墙钟延迟。
5. 实机先完成传感器外参、时间同步、PX4参数、失效保护和人工接管检查，再开展受控飞行。详细项目见 [`src/drone_stack/HARDWARE_ACCEPTANCE.md`](src/drone_stack/HARDWARE_ACCEPTANCE.md)。

## 当前入口

```bash
# 终端 A：仿真世界、PX4、Gazebo
bash start/start_gazebo.sh

# 终端 B：在线建图
bash start/start_navigation.sh mapping

# 或加载预建地图
bash start/start_navigation.sh inspection /path/to/map.dmap

# 落地并确认上锁后停止
bash start/stop_all.sh
```
