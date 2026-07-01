# alohamini_bringup

AlohaMini 的 SLAM 建图与 Nav2 导航 bringup。提供**两套底盘驱动**可选，其余（激光
扇区滤波、SLAM、Nav2）完全共用。

## 两套底盘驱动

| 驱动 | sensors 层 | 建图 | 导航 | 说明 |
|------|-----------|------|------|------|
| **ros2_control（推荐）** | `sensors_ros2_control.launch.py` | `mapping_ros2_control.launch.py` | `navigation_ros2_control.launch.py` | C++ 原生串口驱动 `alohamini_base_control`，直接按 Feetech 协议驱动底盘，无需 lerobot host |
| ZMQ 桥（旧） | `sensors_bridge.launch.py` | `mapping.launch.py` | `navigation.launch.py` | Python `alohamini_nav_bridge`，需在机器人上另跑 `lekiwi_host.py` |

两套对外接口完全一致（`/cmd_vel`、`/odom`、`odom→base_link` TF、`/scan_filtered`），
SLAM 和 Nav2 配置（`config/slam_toolbox.yaml`、`config/nav2_params.yaml`）无需任何改动。

## ros2_control 版用法（推荐）

先确保工作区已构建、每个终端都 source：

```bash
source /opt/ros/humble/setup.bash
source ~/alohamini_lidar_imu/ros2_ws/install/setup.bash
```

### 建图

```bash
ros2 launch alohamini_bringup mapping_ros2_control.launch.py serial_port:=/dev/ttyACM0
```

建完用 slam_toolbox 存图（或用现成的 `scripts/alohamini_mapping_session`）：
```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

### 导航

```bash
ros2 launch alohamini_bringup navigation_ros2_control.launch.py \
    serial_port:=/dev/ttyACM0 \
    map:=~/my_map.yaml
```

### 常用参数

- `serial_port`（默认 `/dev/ttyACM0`）：底盘三个轮子舵机（ID 8/9/10）所在的那条总线。
  编号可能随插拔变化，建议优先用固定 udev 别名，如
  `serial_port:=/dev/am_arm_follower_left`。
- `baud_rate`（默认 `1000000`）。
- `use_mock_hardware`（默认 `false`）：设 `true` 用 ros2_control mock 组件，不接硬件也能
  跑通建图/导航链路（验证 TF、话题、SLAM/Nav2 配置用）。
- `map`（导航必填）：`map_saver_cli` 存下的 `.yaml` 地图。
- `params_file` / `slam_params_file`：覆盖默认 Nav2 / slam_toolbox 参数。

## 关节状态与 TF 说明

ros2_control 的 `joint_state_broadcaster` 只发布**三个轮子**关节的状态。为了让
`robot_state_publisher` 能发出完整 TF（尤其 `base_link→laser_frame`，SLAM/Nav2 必需），
`sensors_ros2_control.launch.py` 里：

1. 把 broadcaster 的 `/joint_states` 重映射到 `/wheel_joint_states`（经
   `base_control.launch.py` 的 `joint_states_topic` 参数）；
2. 起一个 `joint_state_publisher`，用 `source_list=['/wheel_joint_states']` 纳入轮子的
   真实状态，其余手臂/升降关节补零，统一发布完整 `/joint_states`。

这样 `/joint_states` 含全部 16 个关节，TF 树完整。（已在 mock 模式验证：16 关节、
`base_link→laser_frame` 与 `odom→base_link` TF 均可解析。）

## 实物注意事项

- **串口独占**：底盘与左臂/升降轴共用同一串口。运行本 bringup 时**不要**同时跑 lerobot
  的 `lekiwi_host.py`。
- **首次架空低速测试**，确认三轮转向正确再落地。方向约定见
  `alohamini_base_control` 的 README（`w/z/a` ↔ `/cmd_vel` 的 x/y/θ）。
- **树莓派上若遇 `libfastrtps.so` 段错误**：Fast-DDS 在 ARM64 的偶发崩溃，与本包无关。
  换 CycloneDDS：`sudo apt install ros-humble-rmw-cyclonedds-cpp`，然后
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。

底盘驱动、协议、Gazebo 仿真等细节见 `alohamini_base_control/README.md`。
