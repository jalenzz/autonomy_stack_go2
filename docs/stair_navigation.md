# 可支撑连续地形滤波（楼梯/高程过渡）

## 目的

机器狗运控已能处理较大范围的高度、宽度、坡度与方向变化。规划层不做「语义楼梯识别」，而是判断：**这里是否由一组可落脚支撑面组成的连续高程过渡**，从而放行可通行区域，同时保留墙、箱体、立柱等垂直障碍。

覆盖 `terrain_analysis`、`terrain_analysis_ext` 与 FAR planner。

- `local_planner` 使用 `/terrain_map`：intensity 低于 `obstacleHeightThre`（默认 0.3）视为可通行。
- transition 放行点 intensity 为 `-1`：低于障碍阈值，同时供 FAR 标记「可跨高度」格。
- RViz 中 `/terrain_map` 用 intensity 着色时，紫/蓝附近通常对应 `intensity=-1` 的 transition 点（具体取决于 Color Transformer）。

## 判据

实现见 [`traversable_terrain.hpp`](../src/base_autonomy/terrain_analysis/include/terrain_analysis/traversable_terrain.hpp)。栅格边长约 `0.2 m`。

1. **支撑面**  
   `planarVoxelElev` 附近（`supportBandEps`）点数 ≥ `minSupportPoints`，且车高范围内竖直占据比例 &lt; `wallFillThre`（拒绝实心墙柱）。

2. **方向连续过渡**  
   沿 8 方向取长度 `seqLength` 的短序列（默认 4 格 ≈ 0.8 m），要求：
   - 整体单调升/降；
   - `|Δh| ≤ flatEps` 视为平台段（**允许有平台，当前不强制要求有平台**）；
   - 显著跳变须满足 `minStepRise ≤ |Δh| ≤ maxStepRise`，且次数 ≥ `minSignificantSteps`；
   - 序列首尾总高差 `|elev末 - elev首| ≥ minSeqRise`（只针对该窗口，不是整段楼梯累计高度）。

3. **横向宽度**  
   沿主方向法向至少 `minWidthCells` 个高度接近的支撑格（滤箱边、墙脚、窄柱）。

4. **点级放行**  
   仅对 transition 格：放行贴支撑面的点，以及夹在相邻支撑高度之间的 riser 点；明显高于局部支撑上界的点仍保留 `disZ`。

FAR：`map_handler/allow_terrain_height_jumps` 为 true 时，**仅当当前格或邻格带有 transition 标记（free intensity &lt; 0）** 才允许跨高度连通。

### 「级数」如何计数

相邻两格高度差 `|Δh|`：

| 条件 | 行为 |
|------|------|
| `\|Δh\| ≤ flatEps` | 平台，不计级 |
| `flatEps < \|Δh\| < minStepRise` | 过小起伏，不计级（也不因此否决序列） |
| `minStepRise ≤ \|Δh\| ≤ maxStepRise` | 计为 1 级显著跳变 |
| `\|Δh\| > maxStepRise` | 整段序列否决 |

因此当前算法**可以接受没有踩踏平面的连续跳变**（例如偏陡的缓坡），主要靠 `flatEps` / `minStepRise` / `minSeqRise` / `minSignificantSteps` 压误判。

## 参数

以 `terrain_analysis` / `terrain_analysis_ext` launch 为准（与代码结构体默认值可能略有差别，以 launch 为准）：

| 参数 | Launch 当前值 | 含义 |
|------|---------------|------|
| `stairEnable` | true | 总开关；false 时不做 transition 放行 |
| `supportBandEps` | 0.06 | 支撑面带宽 (m) |
| `minSupportPoints` | 3 | 支撑面最少贴地点数 |
| `wallBinSize` | 0.10 | 竖直分箱高度 (m) |
| `wallFillThre` | 0.55 | 竖直占据比例上限 |
| `seqLength` | 4 | 方向序列长度（格） |
| `flatEps` | 0.08 | 平台容差；`\|Δh\|≤此值` 不计级 |
| `minSeqRise` | 0.10 | 窗口首尾总高差下限 (m) |
| `minStepRise` | 0.10 | 单级最小高度；小于此值不计为显著跳变 |
| `maxStepRise` | 0.25 | 单级最大高度；超过则整段否决 |
| `minSignificantSteps` | 2 | 窗口内最少显著跳变次数 |
| `minWidthCells` | 2 | 横向最小宽度（格，含自身） |
| `widthHeightEps` | 0.08 | 横向邻格高度容差 (m) |
| `surfaceEps` | 0.06 | 点级贴面 / riser 容差 (m) |

FAR：`far_planner` 配置中 `map_handler/allow_terrain_height_jumps: true`。

### 调参提示

- **楼梯仍被挡**：略降 `minStepRise` / `flatEps` / `minSeqRise` / `minSignificantSteps` / `minWidthCells`；台阶本身偏高则提高 `maxStepRise`。
- **平地起伏 / 缓坡误放行**：提高 `flatEps`、`minStepRise`、`minSeqRise`、`minSignificantSteps` 或 `minWidthCells`。
- **墙柱误当支撑**：降低 `wallFillThre` 或提高 `minSupportPoints`。
- **恢复旧行为**：`stairEnable:=false`，并将 FAR `allow_terrain_height_jumps` 设为 false。

## 构建与环境

不要 source `/opt/ros`，使用 unitree 环境：

```bash
source ~/unitree_ros2/setup.sh
cd ~/autonomy_stack_go2
colcon build --symlink-install --allow-overriding terrain_analysis terrain_analysis_ext \
  --packages-select terrain_analysis terrain_analysis_ext far_planner
source install/setup.bash
```

若改动未生效，确认二进制含新参数名（如 `minStepRise`），并**重启** launch（参数在节点启动时加载）。

## 单元测试

```bash
source ~/unitree_ros2/setup.sh
source ~/autonomy_stack_go2/install/setup.bash
colcon test --packages-select terrain_analysis far_planner --event-handlers console_direct+
```

覆盖：多级/斜向台阶、平地、箱体、窄柱、墙柱、短 curb、过高单级、过小跳变不计级、点级放行；以及 FAR 仅 transition 可跨高度。

## Bag 回归与可视化

四个 bag（`rosbags/flat_case_*`、`rosbags/stairs_case_*`）含 `/registered_scan`、`/state_estimation`、`/tf`。bag 内旧 `/terrain_map` **不是**新算法真值。

离线回归：

```bash
source ~/unitree_ros2/setup.sh
source ~/autonomy_stack_go2/install/setup.bash
python3 scripts/eval_traversable_terrain.py --bags-root rosbags
```

期望：

- `stairs_case_*`：机头前方扇区障碍比例较低
- `flat_case_*`：全局仍保持一定障碍比例（墙/箱体未被大面积清掉）

Bag 回放着色（橙色=transition 等）：

```bash
./scripts/run_stair_viz.sh rosbags/stairs_case_2
```

实机 / 完整栈运行时，直接在 RViz 看 `/terrain_map` 的 intensity 着色即可。

## 后续可选：双通道方案（暂未实现）

当前不强制踩踏平面，是因为正对楼梯时激光常主要打在踢面，踏面回波稀疏，硬性要求「每一级都有平台」容易漏检真台阶。

若后续误判仍偏多、又希望更贴近台阶语义，可考虑**双通道**：

1. **高置信**：窗口内能看到踏面（平台段）→ 按台阶放行；
2. **低置信**：看不到踏面，但存在多次足够大的跳变（更严的 `minSignificantSteps` / `minStepRise`）→ 收紧条件后仍可放行，或仅做更保守的点级放行。

这样既能挡住纯缓坡，又能在踏面漏检时保留一定召回。目前以调参为主，该方案留作后续增强。
