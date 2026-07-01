# alohamini_nav_bridge

这个 ROS2 包用于把 Nav2 风格的速度命令桥接到现有 AlohaMini ZMQ host。

## ROS 接口

Standalone 启动时订阅：

- `/cmd_vel`：`geometry_msgs/msg/Twist`

Standalone 启动时发布：

- `/odom`：`nav_msgs/msg/Odometry`
- `odom -> base_link` TF

通过 `alohamini_bringup sensors_bridge.launch.py` 启动时，bridge 默认订阅 `/cmd_vel_safe`，发布 `/wheel/odom`，并关闭自身 TF；`robot_localization` EKF 会发布 `/odom` 和 `odom -> base_link` TF。

## ZMQ 接口

- 命令端口：`tcp://<host>:5555`
- 观测端口：`tcp://<host>:5556`

桥接节点发送给 AlohaMini host 的最小 JSON 动作格式：

```json
{"x.vel": 0.0, "y.vel": 0.0, "theta.vel": 0.0}
```

注意：`theta.vel` 单位是度 / 秒，因为 AlohaMini host 里的底盘控制使用 deg/s。

## 启动

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py host:=127.0.0.1
```

如果 AlohaMini host 在同一台树莓派、容器也使用 host 网络，`127.0.0.1` 通常可用。
如果连不上，改成树莓派局域网 IP。

## 方向与比例标定

默认行为和 `lerobot_alohamini` 的键盘控制保持一致：ROS `linear.x` 直接发送为 host `x.vel`，ROS `linear.y` 直接发送为 host `y.vel`，因此 `swap_xy=false`。

如果现场测试发现 `/cmd_vel` 的 `linear.x` 表现为横移，再开启 XY 交换：

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py \
  swap_xy:=true
```

`swap_xy` 同时作用于命令和 observation：发送命令时在 ROS `/cmd_vel` 与 host `x.vel`/`y.vel` 之间换位；读取 host observation 时按同一映射换回 ROS `x`/`y`，保持 `/odom` 坐标方向一致。

前置雷达默认只使用前向扇区，所以 bridge 默认 `allow_reverse=false`、`allow_lateral_motion=false`：负 `linear.x` 和横向 `linear.y` 会被拦截为 0。需要现场调试倒退或横移时，可显式传 `allow_reverse:=true` 或 `allow_lateral_motion:=true`。

比例参数分两组，默认都是 `1.0`：

- Command scale：`linear_x_scale`、`linear_y_scale`、`angular_z_scale`，只影响 `/cmd_vel` -> host 命令。用于修正遥控或 Nav2 输出到实际底盘命令的方向和幅度。
- Odom observation scale：`odom_linear_x_scale`、`odom_linear_y_scale`、`odom_angular_z_scale`，只影响 host observation -> `/odom` 的速度发布和积分。用于修正里程计方向和距离/角度估计，不改变发给底盘的命令。

如果命令方向反了，只改 command scale：

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py \
  linear_x_scale:=-1.0 \
  angular_z_scale:=-1.0
```

如果底盘运动方向正确，但 `/odom` 的距离或角度反向/偏大偏小，只改 odom observation scale：

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py \
  odom_linear_x_scale:=0.95 \
  odom_angular_z_scale:=-1.0
```
