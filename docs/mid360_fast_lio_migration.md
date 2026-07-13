# Go2 MID360 + Fast-LIO 接入说明

## 数据链与接口

真实机器人启动文件已经由 `point_lio_unilidar + Unitree L1` 切换为：

```text
MID360
  ├─ /livox/lidar  (livox_ros_driver2/msg/CustomMsg)
  └─ /livox/imu    (sensor_msgs/msg/Imu)
          │
          ▼
       fast_lio
  ├─ /fast_lio/odometry_raw  (camera_init -> body)
  └─ /registered_scan        (camera_init 坐标系)
          │
          ▼
 fast_lio_vehicle_adapter
  └─ /state_estimation       (camera_init -> vehicle)
```

现有 `terrain_analysis`、`local_planner`、`sensor_scan_generation` 等节点仍然使用
`/state_estimation` 和 `/registered_scan`，不需要迁移到 Fast-LIO 的原生话题名。

MID360s 启动必须使用 `msg_MID360s_launch.py`，其中保持 `xfer_format=1`。
`CustomMsg.points[].offset_time` 是 Fast-LIO 对每帧点云进行运动去畸变所需的逐点时间。

真实机器人配置关闭 Fast-LIO 自带的 `/path`、持续累积的 `/Laser_map` 和无人消费的
`/cloud_registered_body`。其中 `/path` 已由 `localPlanner/pathFollower` 使用，不能让
SLAM 轨迹以同名话题进入执行链；在线导航也不需要无限增长的全局点云和轨迹缓存。

## 两组外参不能混用

`src/slam/FAST_LIO/config/mid360.yaml` 中的 `mapping.extrinsic_T/R` 是 MID360
内部的 LiDAR 到 IMU 外参。它由设备型号决定，并由 Fast-LIO 用于点云和 IMU 融合。
当前配置关闭在线外参估计，直接使用该固定值。

机器人安装外参位于：

```text
src/slam/fast_lio_vehicle_adapter/config/mid360_vehicle_extrinsic.yaml
```

这里配置的是 `T_vehicle_body`：Fast-LIO 的 `body`（MID360 内置 IMU参考系）
在 Go2 `vehicle` 坐标系中的位姿。`vehicle` 约定为机器人机身几何中心，轴方向为
`x` 向前、`y` 向左、`z` 向上。

```yaml
calibration.configured: true
vehicle_to_body.translation: [x, y, z]
vehicle_to_body.rotation_xyzw: [qx, qy, qz, qw]
```

当前默认配置已写入本机实测的 `T_body_vehicle`（适配器配置中保存其逆变换）：

```text
T_body_vehicle.xyz = [-0.2450, -0.0273, -0.1255]
T_body_vehicle.rpy = [0, -0.314159, 0]
```

`calibration.configured` 同时也是安全门。更换安装位置或使用另一台机器人时，应先将其
设为 `false`，重新测量并填写外参；未配置状态下适配器会拒绝运行，不会静默使用零外参。

适配器内部求逆得到 `T_body_vehicle`，并计算：

```text
T_camera_init_vehicle = T_camera_init_body * inverse(T_vehicle_body)
```

因此 `/state_estimation` 表示机身中心，而不是雷达安装点。真实机器人 launch 会把
`localPlanner` 和 `pathFollower` 的 `sensorOffsetX/Y` 固定为零，禁止第二次平移补偿。

Fast-LIO 当前不填充 Odometry twist，适配器也不发布未经坐标变换的 twist。若后续上游
开始提供速度，必须同时实现角速度旋转和安装杠杆臂引起的线速度修正后才能透传。

## 安装外参测量

1. 在 Go2 机身上确定 `vehicle` 原点和正方向。
2. 测量 MID360 内置 IMU参考点相对 `vehicle` 的 `x/y/z`，单位为米。
3. 测量 MID360 坐标轴相对 `vehicle` 的完整 roll、pitch、yaw，并转换为
   `x/y/z/w` 顺序的单位四元数。
4. 写入 `mid360_vehicle_extrinsic.yaml`，确认数值后设置
   `calibration.configured: true`。
5. 原地旋转 Go2：修正后的 `vehicle` 平移轨迹应接近以机身中心为圆心；如果形成
   明显圆周，优先检查平移外参。
6. 在平地直行：运动应主要沿 `vehicle +x`，roll/pitch 和地面方向应与机身一致；
   若方向或倾角异常，优先检查安装旋转外参。

优先使用机械 CAD 和安装尺寸。人工测量可作为初值，但在启用自主运动前必须完成
直行、原地旋转和坡面验证。

## TF 结构

正常运行时主 TF 链为：

```text
map -> camera_init -> body -> vehicle -> sensor -> camera
```

- `camera_init -> body`：Fast-LIO 动态发布。
- `body -> vehicle`：适配器根据安装外参静态发布。
- `vehicle -> sensor`：兼容旧 RViz/相机消费者的恒等 TF。
- `sensor -> camera`：现有 `local_planner.launch` 发布。

旧的 `aft_mapped -> sensor` 和 `sensor -> vehicle` 补偿已在 MID360 真实机器人链中禁用，
避免 `vehicle` 出现多个父坐标系或重复外参。

## 构建与启动

所有 ROS2 命令均使用 Unitree 环境，不要 source `/opt/ros`：

```bash
source ~/unitree_ros2/setup.sh
source ~/ws_livox/install/setup.bash
cd ~/autonomy_stack_go2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
./system_real_robot.sh
```

带全局路径规划时使用：

```bash
./system_real_robot_with_route_planner.sh
```

也可以通过 launch 参数加载另一份机器人安装外参：

```bash
ros2 launch vehicle_simulator system_real_robot.launch \
  vehicleExtrinsicConfig:=/absolute/path/to/extrinsic.yaml
```

## 验收检查

```bash
ros2 topic info /livox/lidar -v
ros2 topic hz /livox/imu
ros2 topic hz /fast_lio/odometry_raw
ros2 topic hz /state_estimation
ros2 topic hz /registered_scan
ros2 run tf2_ros tf2_echo camera_init vehicle
```

验收至少覆盖静止、直行、原地旋转、上下坡和快速转向。确认：

- Fast-LIO 没有 LiDAR/IMU 时间不同步告警。
- `/registered_scan` 无明显重影、扭曲或分层。
- `/state_estimation.child_frame_id` 为 `vehicle`。
- TF 中 `vehicle` 只有一个父坐标系，且 `map` 到 `vehicle` 连通。
- `terrain_analysis`、`local_planner` 以及启用时的
  `sensor_scan_generation/elevation_mapping_cupy` 能持续消费兼容话题。

回滚时恢复真实机器人 launch 中的 `mapping_utlidar.launch` 和旧 TF/平移补偿即可；
`point_lio_unilidar` 源码未从仓库删除。
