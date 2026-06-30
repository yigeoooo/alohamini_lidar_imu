# alohamini_nav_bridge

这个 ROS2 包用于把 Nav2 风格的速度命令桥接到现有 AlohaMini ZMQ host。

## ROS 接口

订阅：

- `/cmd_vel`：`geometry_msgs/msg/Twist`

发布：

- `/odom`：`nav_msgs/msg/Odometry`
- `odom -> base_link` TF

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

## 方向标定

默认行为和 `lerobot_alohamini` 的键盘控制保持一致：ROS `linear.x` 直接发送为 host `x.vel`，ROS `linear.y` 直接发送为 host `y.vel`，因此 `swap_xy=false`。

如果现场测试发现 `/cmd_vel` 的 `linear.x` 表现为横移，再开启 XY 交换：

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py \
  swap_xy:=true
```

如果方向反了，再用这些参数调整符号或比例：

- `linear_x_scale`
- `linear_y_scale`
- `angular_z_scale`

示例：

```bash
ros2 launch alohamini_nav_bridge bridge.launch.py \
  linear_x_scale:=-1.0 \
  angular_z_scale:=-1.0
```
