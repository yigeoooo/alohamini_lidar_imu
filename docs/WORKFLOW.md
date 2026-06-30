# AlohaMini 雷达 / IMU 建图、导航、RViz 完整使用流程

所有 ROS2/Nav2 命令默认在：

```bash
cd ~/alohamini_lidar_imu
```

## 1. 启动 LeRobot AlohaMini host

在树莓派端进入 LeRobot 工程：

```bash
cd ~/lerobot_alohamini
```

根据你的真机型号选择一个命令：

```bash
python -m lerobot.robots.alohamini.lekiwi_host --robot_model alohamini1
python -m lerobot.robots.alohamini.lekiwi_host --robot_model alohamini2
python -m lerobot.robots.alohamini.lekiwi_host --robot_model alohamini2pro
```

## 2. 启动 micro-ROS Agent

micro-ROS Agent 负责把开发板发布的雷达和 IMU 数据接入 ROS2 网络。

```bash
cd ~/alohamini_lidar_imu

docker start microros_agent
```

查看日志：

```bash
docker logs -f microros_agent
```

应能看到 micro-ROS session、topic、datawriter 创建日志。

## 3. 启动 / 进入 Nav2 容器

```bash
docker exec -it alohamini_nav2 bash
```

## 4. 构建 ROS2 工作空间

进入alohamini_nav2容器后，在树莓派执行：

```bash
source /opt/ros/humble/setup.bash
cd /root/ws
colcon build --symlink-install
source /root/ws/install/setup.bash
```

构建成功后应包含：

```text
alohamini_description
alohamini_nav_bridge
alohamini_bringup
```

## 5. 当前阶段检查：传感器、TF、odom、bridge（首次联调或故障排查）

这一节只是独立检查，不是建图或导航时必须额外运行的步骤。`mapping.launch.py` 和 `navigation.launch.py` 都会自动包含 `sensors_bridge.launch.py`，因此建图或导航运行时不要再单独启动本节的 `sensors_bridge.launch.py`，否则会重复启动 bridge、robot_state_publisher 和 scan filter。

以后命令均在树莓派端执行。

先确认 LeRobot host 和 micro-ROS Agent 都已经运行。

在 `alohamini_nav2` 容器内启动基础 bringup：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 launch alohamini_bringup sensors_bridge.launch.py host:=127.0.0.1
```

如果 ZMQ loopback 不通，用树莓派 IP：

```bash
ros2 launch alohamini_bringup sensors_bridge.launch.py host:=192.168.10.29
```

另开一个容器 shell 检查：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic hz /scan
ros2 topic hz /imu
ros2 topic hz /odom

ros2 run tf2_ros tf2_echo base_link laser_frame
ros2 run tf2_ros tf2_echo base_link imu_frame
ros2 run tf2_ros tf2_echo odom base_link
```

## 6. 建图

### 方式 A：带键盘保存的建图会话

推荐现场使用这个方式。它会启动 `mapping.launch.py`，同时提供按键保存地图：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 run alohamini_bringup alohamini_mapping_session \
  --host 127.0.0.1 \
  --map /root/ws/maps/alohamini_map
```

按键含义：

```text
w / s       前进 / 后退
z / x       左移 / 右移
a / d       原地左转 / 原地右转
r / f       加速 / 减速
Space 或 0  发送一次零速 /cmd_vel
Shift+S     保存当前地图，建图继续运行
Shift+X     保存当前地图，保存成功后停止 mapping.launch.py 并退出
q           不保存，停止 mapping.launch.py 并退出
Ctrl+C      不保存，停止 mapping.launch.py 并退出
```

### 方式 B：直接启动 mapping.launch.py

这种方式更接近 ROS 原生命令。保存地图时需要另开一个容器 shell 执行 `map_saver_cli`。

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 launch alohamini_bringup mapping.launch.py host:=127.0.0.1
```

另开 shell 进入容器检查建图输出：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 run tf2_ros tf2_echo map odom
ros2 topic echo /map --once
```

## 7. 保存地图

保存地图有两种方式，取决于第 6 节怎么启动建图。

### 如果使用 `alohamini_mapping_session`

不需要另开保存终端，直接在建图会话终端按键：

```text
Shift+S  保存当前地图，建图继续运行
Shift+X  保存当前地图，保存成功后停止 mapping.launch.py 并退出
```

### 如果直接使用 `mapping.launch.py`

需要另开一个终端执行保存命令，并且保存时不要先关闭建图 launch。

正确顺序：

```text
1. mapping.launch.py 继续运行。
2. 新开一个 alohamini_nav2 容器 shell。
3. 执行 map_saver_cli 保存地图。
4. 看到保存成功后，再回到建图终端 Ctrl+C 停止 mapping.launch.py。
```

`Ctrl+C` 只会退出建图进程，不会自动保存地图；只有 `alohamini_mapping_session` 的 `Shift+X` 会执行“保存成功后退出”。

另开 shell 保存地图：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
mkdir -p /root/ws/maps

ros2 run nav2_map_server map_saver_cli -f /root/ws/maps/alohamini_map
```

生成文件：

```text
/root/ws/maps/alohamini_map.yaml
/root/ws/maps/alohamini_map.pgm
```

地图文件在宿主机对应路径：

```text
~/alohamini_lidar_imu/ros2_ws/maps/
```

## 8. 导航

导航前必须关闭建图 launch，避免 SLAM Toolbox 和 AMCL 同时处理 `map -> odom`。

启动导航：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 launch alohamini_bringup navigation.launch.py \
  host:=127.0.0.1 \
  map:=/root/ws/maps/alohamini_map.yaml
```

导航阶段由 AMCL 发布 `map -> odom`，Nav2 发布 `/cmd_vel`，bridge 转成 ZMQ 速度给 LeRobot host。

### 8.1 怎么确认起点和终点

点到点导航的“点”都在同一张保存好的地图 `map` 坐标系里。

```text
起点：机器人当前在地图里的位姿，也就是 AMCL 估计出的 map -> base_link。
终点：你在 RViz 里给 Nav2 的目标位姿，也就是 goal pose。
```

```text
1. RViz Fixed Frame 设为 map。
2. 观察 RobotModel / LaserScan 是否和地图轮廓大致重合。
3. 如果不重合，用 2D Pose Estimate 在地图上点机器人实际位置，并拖出机器人朝向。
4. 等 particle cloud 收敛到机器人附近。
5. 再看 RobotModel、LaserScan 和地图是否对齐。
```

容器内可以辅助检查当前起点估计：

```bash
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
```

终点在 RViz 里给：

```text
1. 使用 Nav2 Goal / 2D Goal Pose 工具。
2. 在地图可通行区域点击终点。
3. 拖动箭头设置到达后的朝向。
4. 先给 0.5m 到 1m 的短距离目标。
5. 看到 global plan / local plan 出现后，再观察机器人是否开始低速移动。
```

如果起点没对准，导航会从错误位置规划；如果终点点在障碍物、未知区域或代价地图外，Nav2 可能不会生成 plan，或者很快失败。

## 9. RViz 可视化

建图和导航时都可以开 RViz。推荐方式是在本机/开发机运行 RViz，树莓派只运行 LeRobot host、micro-ROS Agent、`alohamini_nav2` 容器和机器人进程。

### 9.1 本机已有 ROS2 Humble

如果本机是 Ubuntu 22.04，或者已经有可用的 ROS2 Humble 环境，直接使用本机 Humble。确认本机和树莓派在同一网络，并使用同一个 ROS Domain：

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
```

首次使用或代码更新后，在本机构建这个 ROS2 工作空间：

```bash
cd ~/project/alohamini_lidar_imu/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

启动本项目的 RViz 配置：

```bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
ros2 launch alohamini_bringup rviz.launch.py
```

### 9.2 本机是 Ubuntu 24.04

Ubuntu 24.04 不适合作为原生 ROS2 Humble 环境。此时在本机用 Docker 创建一个 Humble RViz 容器，容器通过 host 网络加入树莓派同一个 ROS2 网络。

本机先允许 Docker 容器访问 X11 显示：

```bash
xhost +local:docker
```

创建并进入本机 RViz 容器：


下面是首次创建命令。`docker rm -f` 会删除旧容器；如果已经安装过依赖，不要重复执行这一行，直接用后面的 `docker start -ai alohamini_rviz_humble` 复用容器。

```bash
cd ~/project/alohamini_lidar_imu

docker rm -f alohamini_rviz_humble 2>/dev/null || true

docker run -it \
  --name alohamini_rviz_humble \
  --net=host \
  --ipc=host \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID=5 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/ros2_ws:/root/ws" \
  osrf/ros:humble-desktop \
  bash
```

容器内安装本项目 RViz 需要的依赖并构建工作空间：

```bash
apt update
apt install -y --no-install-recommends \
  ros-humble-nav2-rviz-plugins \
  python3-colcon-common-extensions

cd /root/ws
source /opt/ros/humble/setup.bash
colcon --log-base /root/rviz_ws_log build --symlink-install \
  --build-base /root/rviz_ws_build \
  --install-base /root/rviz_ws_install
source /root/rviz_ws_install/setup.bash
```

容器内启动 RViz：

```bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch alohamini_bringup rviz.launch.py
```

以后再次使用这个本机 RViz 容器：

```bash
docker start -ai alohamini_rviz_humble
cd /root/ws
source /opt/ros/humble/setup.bash
source /root/rviz_ws_install/setup.bash
export ROS_DOMAIN_ID=5
export ROS_LOCALHOST_ONLY=0
ros2 launch alohamini_bringup rviz.launch.py
```

如果 GUI 打不开，先确认本机正在使用 X11/XWayland，并重新执行 `xhost +local:docker`。

### 9.3 RViz 连接检查

这个 launch 会打开 `ros2_ws/src/alohamini_bringup/rviz/alohamini_nav.rviz`。本机 RViz 只负责可视化和发送初始位姿/导航目标，不需要在本机启动 `sensors_bridge.launch.py`、`mapping.launch.py` 或 `navigation.launch.py`。

如果本机看不到树莓派上的话题，先检查：

```bash
ros2 topic list
ros2 topic echo /scan_filtered --once
ros2 topic echo /odom --once
```

建图或导航时，RViz 重点看：

```text
Fixed Frame: map
TF: map -> odom -> base_link -> laser_frame / imu_frame
LaserScan: /scan_filtered
Odometry: /odom
Map: /map
Global Plan: /plan
Global/Local Costmap: /global_costmap/costmap, /local_costmap/costmap
```

如果只是启动了第 5 节的基础 bringup，还没有建图或导航，`map` frame 可能不存在。此时可以临时把 RViz Fixed Frame 改成 `odom` 来检查 RobotModel、LaserScan 和 Odometry。

