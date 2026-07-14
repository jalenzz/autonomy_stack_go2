# 楼梯地形导航适配

## 目的

本适配针对机器狗已经具备上下楼梯能力的场景。规划层不再把连续的台阶或坡面仅因为高度变化判定为障碍，同时保留墙、箱体和立柱等垂直障碍的检测。

当前实现覆盖 `terrain_analysis`、`terrain_analysis_ext` 和 FAR planner。`local_planner` 继续使用既有 `/terrain_map` 接口：可通行地形点的 `PointXYZI.intensity` 为 0，普通障碍点保留高度残差。

## 工作方式

terrain analysis 为每个平面栅格估计地面高度，并检查邻域是否存在连续高度变化。满足以下条件的栅格会被标记为楼梯/坡面候选：

- 栅格有足够的地面点；
- 至少有指定数量的邻居栅格也有地面点；
- 邻域高度差超过 `stairMinRise`。

候选栅格的输出 intensity 被置为 0，因此不会进入局部障碍检测。FAR planner 通过 `map_handler/allow_terrain_height_jumps` 允许 free terrain 栅格跨越高度差连通。

这是基于局部栅格的启发式分类，不是专门的语义楼梯识别器。楼梯旁的墙体仍依赖点云垂直结构和原有障碍阈值进行过滤。

## 参数

两个 terrain analysis 节点都支持：

```yaml
stairEnable: true
stairMinRise: 0.08
stairMinNeighborCells: 2
```

FAR planner 支持：

```yaml
map_handler/allow_terrain_height_jumps: true
```

调参建议：

- 楼梯仍被判为障碍：降低 `stairMinRise`，或适当降低 `stairMinNeighborCells`；
- 平地障碍被误判为可通行：提高 `stairMinNeighborCells`，并检查 terrain analysis 的点云分辨率；
- 只想恢复旧行为：将 `stairEnable` 和 `allow_terrain_height_jumps` 设为 `false`。

不建议通过单纯增大 `local_planner` 的 `obstacleHeightThre` 来适配楼梯，因为这会同时放行矮墙和箱体。

## ROS2 验证

项目使用 ROS2 Foxy 时，先加载 Unitree 环境，再加载工作空间；不要直接 source `/opt/ros`：

```bash
source ~/unitree_ros2/setup.sh
source ~/autonomy_stack_go2/install/setup.bash
```

构建必须使用：

```bash
colcon build --symlink-install
```

建议依次回放：

1. `rosbags/flat_case_1`
2. `rosbags/flat_case_2`
3. `rosbags/stairs_case_1`
4. `rosbags/stairs_case_2`

检查以下结果：

- `/terrain_map` 中楼梯点的 intensity 接近 0；
- 墙和箱体仍具有较高 intensity；
- `/free_paths` 覆盖楼梯行进方向；
- `/path` 在上楼和下楼过程中不间断；
- FAR planner 的局部地形连通性不在台阶处断开。

当前四个 bag 只包含处理后的 `/terrain_map` 等话题，没有原始激光点云。因此它们可用于验证下游 planner，但不能单独用于重新标定楼梯几何分类器。后续标定应重新录制包含原始点云和 TF 的 bag。
