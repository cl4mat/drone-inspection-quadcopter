# 仿真验证范围

本文件替代早期按日期堆叠的验收终端记录。历史逐轮日志已清理；可引用的汇总数据和图表位于 [`reports/`](../../reports/README.md)。

## 已有仿真证据

- 定位输入、ENU坐标、外部位姿融合、机体中心换算及健康状态链路已在Gazebo/PX4仿真运行。
- 三维占据地图、空闲射线更新、A*参考路线、EGO局部曲线审核及Qt路径显示已有有限场景记录。
- 双层楼道建图/飞行、动态障碍和ICP几何比较数据分别见实验目录说明。

## 数据解释

- 动态障碍10组按“规划地图感知到注入障碍且整机包络最小间距大于0”统计无碰撞避障10/10；9/10组到达目标并回归原路线。第7组EGO拒绝危险候选路线，安全悬停但目标未完成。
- ICP数字针对占据存档体素中心与Gazebo几何模型的单向距离重合度，不代表原始PCD精度或场景表面覆盖率。
- 以上有限仿真数据不证明所有场景无缺陷，不构成实机验收。Qt端到端延迟没有正式实测值。

## 后续验收入口

- 当前运行状态：[`DRONE_SIM_PROGRESS.md`](../../DRONE_SIM_PROGRESS.md)
- 系统与规划架构：[`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)、[`docs/PLANNING_RUNTIME.md`](../../docs/PLANNING_RUNTIME.md)
- 实机检查：[`HARDWARE_ACCEPTANCE.md`](HARDWARE_ACCEPTANCE.md)
