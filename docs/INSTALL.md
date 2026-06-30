# AlohaMini 树莓派安装依赖与容器创建

## 1. 开发机环境
ros2的humble版本即可，含有rviz


项目目录：

```bash
cd ~/alohamini_lidar_imu
```

默认参数：

```text
ROS_DOMAIN_ID=5
micro-ROS Agent UDP 端口=8090
AlohaMini host ZMQ command=5555
AlohaMini host ZMQ observation=5556
Nav2 容器名=alohamini_nav2
micro-ROS Agent 容器名=microros_agent
```

## 2. 树莓派宿主机依赖

基础工具：

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg git rsync openssh-server
```

安装 Docker。树莓派系统如果是 Ubuntu / Debian，可以直接用 Docker 官方安装脚本：

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
newgrp docker
```

验证 Docker 可用：

```bash
docker version
docker run --rm hello-world
```

## 3. 需要 pull 的镜像

Pi 侧需要两个上游镜像：

```bash
docker pull --platform linux/arm64/v8 docker.io/microros/micro-ros-agent:humble
docker pull --platform linux/arm64/v8 docker.io/library/ros:humble-ros-base-jammy
```

用途：

```text
docker.io/microros/micro-ros-agent:humble  接收 ESP32 / micro-ROS 发布的 /scan 和 /imu
docker.io/library/ros:humble-ros-base-jammy  手工安装 Nav2、SLAM Toolbox、bridge 和 RViz 依赖的基础容器
```

如果树莓派访问 Docker Hub 慢，可以在网络更好的电脑上拉 arm64 镜像并导出：

```bash
docker pull --platform linux/arm64/v8 docker.io/library/ros:humble-ros-base-jammy
docker save docker.io/library/ros:humble-ros-base-jammy | gzip > ros-humble-ros-base-jammy-arm64.tar.gz
scp ros-humble-ros-base-jammy-arm64.tar.gz pi5@pi5_ip:~/
```

树莓派上导入：

```bash
gunzip -c ~/ros-humble-ros-base-jammy-arm64.tar.gz | docker load
```

## 4. 创建 micro-ROS Agent 容器

micro-ROS Agent 负责把开发板发布的雷达和 IMU 接入 ROS2 网络。

```bash
docker rm -f microros_agent 2>/dev/null || true

docker run -d --restart=unless-stopped \
  --platform linux/arm64/v8 \
  --net=host \
  --privileged \
  -v /dev:/dev \
  -v /dev/shm:/dev/shm \
  -e ROS_DOMAIN_ID=5 \
  --name microros_agent \
  docker.io/microros/micro-ros-agent:humble \
  udp4 --port 8090 -v4
```

查看日志：

```bash
docker logs -f microros_agent
```

看到 session、topic、datawriter 相关日志，说明开发板已经连上 Agent。

## 5. 创建 Nav2 / SLAM 运行容器

```bash
docker rm -f alohamini_nav2 2>/dev/null || true

docker run -dit \
  --platform linux/arm64/v8 \
  --net=host \
  --ipc=host \
  --privileged \
  -e ROS_DOMAIN_ID=5 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v "$HOME/alohamini_lidar_imu/ros2_ws:/root/ws" \
  --name alohamini_nav2 \
  docker.io/library/ros:humble-ros-base-jammy \
  bash
```

进入容器：

```bash
docker exec -it alohamini_nav2 bash
```

## 6. 在 Nav2 容器内安装 ROS2 依赖

下面命令在 `alohamini_nav2` 容器内执行：

```bash
apt update
apt install -y --no-install-recommends \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-xacro \
  ros-humble-laser-filters \
  ros-humble-teleop-twist-keyboard \
  ros-humble-rmw-fastrtps-cpp \
  python3-colcon-common-extensions \
  python3-pip \
  python3-zmq \
  iproute2 \
  iputils-ping \
  net-tools \
  nano

pip3 install --no-cache-dir pyzmq
```

写入默认 ROS 环境：

```bash
echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
echo "export ROS_DOMAIN_ID=5" >> /root/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> /root/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> /root/.bashrc
```

## 7. 构建本项目 ROS2 工作空间

仍然在 `alohamini_nav2` 容器内执行：

```bash
cd /root/ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 8. 首次运行检查

先确认容器都在运行：

```bash
docker ps
```

进入 Nav2 容器：

```bash
docker exec -it alohamini_nav2 bash
```

容器内检查 micro-ROS 话题：

```bash
source /opt/ros/humble/setup.bash
source /root/ws/install/setup.bash
export ROS_DOMAIN_ID=5

ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /imu --once
```
