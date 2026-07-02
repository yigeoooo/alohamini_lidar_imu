# AlohaMini 雷达 / IMU 导航接入整理包

这个目录是 AlohaMini micro-ROS 雷达 / IMU 接入、ROS2 bridge、SLAM 建图和 Nav2 导航的独立整理包。

## 目录结构

```text
alohamini_lidar_imu/
├── firmware/
│   └── lidar_imu_publisher/        # ESP-IDF micro-ROS 雷达 / IMU 固件工程
├── extra_components/               # 固件构建依赖，保持相对路径可用
├── ros2_ws/
│   └── src/
│       ├── alohamini_description/  # AlohaMini 机器人模型和雷达 / IMU TF
│       ├── alohamini_nav_bridge/   # 旧 /cmd_vel <-> AlohaMini ZMQ host 桥接
│       ├── alohamini_base_control/ # 推荐 ros2_control 底盘串口驱动
│       └── alohamini_bringup/      # SLAM、Nav2、RViz、传感器过滤启动入口
└── docs/
    ├── INSTALL.md                  # 树莓派依赖、镜像 pull 和容器创建
    ├── WORKFLOW.md                 # 建图、保存地图、导航和 RViz 完整流程
    └── FIRMWARE_FLASHING.md        # 固件烧录和 ESP-IDF 环境说明
```

## 文档入口

- [依赖安装](docs/INSTALL.md)：树莓派宿主机依赖、Docker 镜像、容器创建和 ROS2 依赖安装。
- [Micro Ros代码烧录](docs/FIRMWARE_FLASHING.md)：micro-ROS 控制板固件烧录记录和 ESP-IDF 使用说明。
- [工作流程](docs/WORKFLOW.md)：micro-ROS Agent、建图、保存地图、导航和本机 RViz 可视化流程。


## 当前配置

- 树莓派 IP：`192.168.10.157`（实际IP根据连上网络后自行替换为真实IP地址）
- micro-ROS Agent UDP 端口：`8090`
- ROS Domain ID：`5`
- 原始雷达话题：`/scan`
- 建图 / 导航雷达话题：`/scan_filtered`，由 `scan_sector_filter` 发布；默认只保留物理前方 `[-180°, 0°]` 扇区
- IMU 话题：`/imu`

## 树莓派项目位置

项目文件放在树莓派上的位置为：

```text
~/alohamini_lidar_imu/
```

Nav2 容器里把宿主机的 `~/alohamini_lidar_imu/ros2_ws` 挂载为 `/root/ws`。容器内构建和运行 ROS2 命令时默认使用：

```bash
cd /root/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=5
```

## 雷达和 IMU 安装 URDF 位置

当前 URDF 中雷达和 IMU 位姿是未实测前的保守默认：

```text
ros2_ws/src/alohamini_description/urdf/alohamini_nav.urdf
```

- `laser_frame`：URDF 里写在 CAD `base_link` 坐标下：`xyz="0 -0.20 0.12"`、`rpy="0 0 0"`；经 `base_footprint→base_link` +90° 静态变换后，等效到机器人视觉前方中线，`base_footprint` 约 `xyz="0.20 0.00 0.12"`。当前 MS200 固件发布的 LaserScan 0° 对应 `base_footprint` +Y，物理正前方对应 LaserScan -90°。
- `imu_frame`：URDF 里写在 CAD `base_link` 坐标下：`xyz="0.00978916757496985 0.00084647910851246 0.344753325406094"`，等效为 `base_footprint` 下的 base inertial origin，作为当前质心估计。

后续量出真实安装位置后，仍建议复核并修改这两个 fixed joint：

```xml
<joint name="base_link_to_laser_frame" type="fixed">
  <origin xyz="0 -0.20 0.12" rpy="0 0 0" />
</joint>

<joint name="base_link_to_imu_frame" type="fixed">
  <origin xyz="X Y Z" rpy="ROLL PITCH YAW" />
</joint>
```

## 必要说明

extra_components文件夹下的为烧录代码时所需要的三方依赖，为独立的代码仓库。暂时不放置在本仓库
