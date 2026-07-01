# lidar_imu_publisher 固件工程

这个 ESP-IDF 工程用于 MicroROS 开发板，同时发布：

- `scan`：`sensor_msgs/msg/LaserScan`
- `imu`：`sensor_msgs/msg/Imu`

工程基于已验证的 `lidar_publisher` 示例，并合入了 `imu_publisher` 需要的 IMU 组件。

## 当前配置

关键配置在本机 `sdkconfig`（含 WiFi 密码，已被 .gitignore 忽略，不提交到 GitHub）：

- `CONFIG_MICRO_ROS_DOMAIN_ID=5`
- `CONFIG_MICRO_ROS_AGENT_IP="192.168.10.157"`
- `CONFIG_MICRO_ROS_AGENT_PORT="8090"`
- `CONFIG_ESP_WIFI_SSID="Dexforce"`

## 构建依赖

本整理包保留了原工程依赖的相对路径：

```text
../../extra_components
```

所以把整个 `alohamini_lidar_imu` 文件夹复制到树莓派后，固件工程仍然能找到依赖。

## Agent 启动

树莓派上使用已验证的 arm64 Agent 镜像：

```bash
docker pull --platform linux/arm64/v8 docker.io/microros/micro-ros-agent:humble

docker run -it --rm \
  --platform linux/arm64/v8 \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  --privileged \
  --net=host \
  -e ROS_DOMAIN_ID=5 \
  docker.io/microros/micro-ros-agent:humble \
  udp4 --port 8090 -v4
```

## 话题检查

在同一 ROS Domain ID 下检查：

```bash
export ROS_DOMAIN_ID=5
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
ros2 topic hz /scan
ros2 topic hz /imu
```

烧录命令和具体串口记录请写到：

```text
docs/FIRMWARE_FLASHING.md
```

