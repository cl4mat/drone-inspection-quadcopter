# 双层楼道仿真场景与整栋巡检

更新：2026-10-12。本文只描述当前生成器和保存的实验摘要；历史场景布局与旧终端运行记录已清理。

## 当前 Gazebo 场景

默认 `building` profile 加载 [`inspection_building.world`](../src/drone_stack/worlds/inspection_building.world)，由 [`generate_building_world.py`](../src/drone_stack/scripts/generate_building_world.py) 生成。场景没有天花板，也没有无限地面；无人机只能在大厅、走廊、楼梯和实际楼板上方飞行。

- 一楼和二楼大厅各为 16×12 m。
- 一楼左右各有约20 m长、1.8 m宽的走廊。
- 二楼相对一楼向 +Y 错开，楼层高差3.6 m；每侧有净宽约1.5 m的直行楼梯段连接两层，并有扶手/井道墙体限制横向逸出。
- 两层走廊布置交错固定障碍物；场景构件同时有渲染和碰撞几何。
- 世界原点位于一楼大厅中心附近；仿真启动时默认请求在地图原点生成机体。

这是验证定位、地图更新和局部避障的仿真几何，不是现实建筑模型，也不包含检测算法的真实目标。

## 建图与地图文件

从仓库根目录启动遥控建图：

```bash
bash start/start_keyboard_mapping.sh "$PWD/start/maps/inspection_building.dmap"
```

或使用文字命令入口：

```bash
bash start/start_mapping_teleop.sh "$PWD/start/maps/inspection_building.dmap"
```

建图必须使用真实模拟雷达扫描和 Faster-LIO/地图会话；不得把 Gazebo 几何导出直接当成实测地图。完成两层扫描后，按入口提示落地、确认上锁并执行保存/结束步骤。地图按 `DRONE_GRID_V2` 保存，并包含坐标变换及占据/自由证据；地图本身位于被 Git 忽略的 `start/maps/`，不会随源码仓库发布。

有实测地图和对应整栋路线文件后，可用：

```bash
bash start/run_building_coverage.sh "$PWD/start/maps/inspection_building.dmap"
```

该脚本会启动无图形 Gazebo 与巡检导航，并按已保存路线执行；没有地图或 sidecar 路线文件时立即退出。常规导航也可分开启动：先 `bash start/start_gazebo.sh`，再 `bash start/start_navigation.sh inspection /path/to/map.dmap`。

## 巡检路线生成与运行

`building_route.py` 从实际地图的占据、自由观测和楼面支撑关系中识别可通行区域、走廊骨架、大厅区域和跨层连接。它是几何启发式，不是语义房间识别。未观测空间、没有支撑的高度通道或断开的楼层连接不能当作已知巡检路线。

整栋路线由楼层通行段、楼梯/平台分段、大厅内缩扫描区域和返程段组成。大厅使用边界内缩避免贴墙；走廊与楼梯按通过记为已巡视。每段送入同一 A*→EGO 规划执行链，巡检管理器按任务身份和路径进度滚动提交控制参考。楼梯平台需要停车转向；地图中预存的楼梯剖面用于约束跨层路线。

已记录的一轮双层场景实验完成80/80任务段并返回落地；按实际下视图像投影计算的楼面几何覆盖率为98.567%，实际巡检飞行路径436.892 m。统计包括遮挡剔除和相机位姿时间配对，覆盖率不表示缺陷识别成功或绝对无漏扫。详细指标、图表和统计口径见 [`reports/two_floor_patrol_20261009_153401/REPORT.md`](../reports/two_floor_patrol_20261009_153401/REPORT.md)。

## 操作与限制

- 默认 Gazebo 无图形窗口运行。需要查看仿真时，启动世界前设置 `DRONE_SHOW_GAZEBO=1`。
- Qt综合页的目标导航与巡检页的区域任务/整栋任务分开显示；整栋任务由对应建筑任务入口触发。
- 授权、解锁、起飞由操作员发起；无任务时不会自动飞行。正常终止前先降落并确认上锁。
- `start/stop_all.sh` 仅清理进程，不执行降落。
- 已报告的结果来自仿真，不构成楼梯实机飞行或硬件验收。
