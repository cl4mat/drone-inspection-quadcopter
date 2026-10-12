# `drone_stack` ROS 包

本包是无人机项目的系统集成层。定位算法、EGO规划器和PX4固件分别位于工作区其他目录。

## 主要职责

- 仿真机体、传感器和 PX4 启动：`scripts/start_px4_sim.sh`、`scripts/generate_model.py`、`worlds/`。
- MID360仿真输入适配：`src/sim_livox_native_adapter.cpp`。
- Faster-LIO坐标换算、外部里程计和点云地图输入：`scripts/lio_bridge.py`、`scripts/map_session.py`。
- 飞行控制、目标队列与健康保护：`scripts/flight_manager.py`。
- Qt操作台：`src/operator_gui.cpp`。
- 单区域与双层建筑巡检：`scripts/inspection_manager.py`、`scripts/building_manager.py`。
- AI文字指令：`scripts/drone_ai_node.py`；密钥路径由 `config/ai.yaml` 指向本机私有文件，不得写入配置或版本库。
- 启动数据检查：`scripts/check_sim_startup.py`。

## 构建与运行

从工作区根目录构建：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -j1 -l1
source devel/setup.bash
```

分离启动方式及地图模式见根目录 [`README.md`](../../README.md)。参数、坐标系、接口、规划周期和已知限制见 [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)。

## 相关资料

- [`docs/PLANNING_RUNTIME.md`](../../docs/PLANNING_RUNTIME.md)：运行时规划和路径显示流程。
- [`docs/INSPECTION.md`](../../docs/INSPECTION.md)：区域巡检算法与操作。
- [`docs/AI_CONTROL.md`](../../docs/AI_CONTROL.md)：AI指令解析和权限边界。
- [`HARDWARE_ACCEPTANCE.md`](HARDWARE_ACCEPTANCE.md)：实机上电、定位、保护和飞行验收清单。
