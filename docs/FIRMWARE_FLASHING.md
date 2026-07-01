# 固件烧录

MicroROS 开发板固件烧录命令和操作注意事项。

## 依赖下载
### 1. 开发环境
1. 开发环境使用ubuntu22.04版本，ros版本使用humble版本。ros2下载参考此链接[此链接](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2. 安装依赖
1. 打开Ubuntu系统终端，并运行以下命令安装相关依赖。
```bash
sudo apt-get install \
  git wget flex bison gperf \
  python3 python3-pip python3-venv \
  cmake ninja-build ccache \
  libffi-dev libssl-dev \
  dfu-util libusb-1.0-0
```

### 3. 下载ESP-IDF
打开Ubuntu系统终端，运行以下命令下载esp-idf-v5.1.2版本
```bash
mkdir -p ~/esp

cd ~/esp

git clone -b v5.1.2 --recursive https://github.com/espressif/esp-idf.git
```
设置工具支持的芯片esp32s3。
```bash
cd esp-idf

./install.sh esp32s3
```
## 激活ESP-IDF开发环境
在esp-idf工具目录下运行以下命令
```bash
source ~/esp/esp-idf/export.sh
```
注意：每次打开新终端都需要先激活ESP-IDF开发环境才可以编译ESP-IDF的工程。看到如下信息则表示激活成功

![激活成功示例](./image/1.png)

## 编译和烧录固件

将microROS控制板连接到本机电脑的usb口，并进入本项目的/alohamini_lidar_imu/firmware/lidar_imu_publisher位置（若未激活开发环境，则运行命令激活环境。具体参考激活ESP-IDF开发环境）

### 1. 打开ESP-IDF的配置工具。

```bash
idf.py menuconfig
```

### 2. 打开micro-ROS Settings

在micro-ROS Agent IP填入代理主机的IP地址(这里填树莓派接入wifi后的真实IP地址)，在micro-ROS Agent Port填入代理主机的端口号（默认8090，可选其他端口）

![IP设置](./image/2.png)

### 3. wifi设置

依次打开micro-ROS Settings->WiFi Configuration，在WiFi SSID和WiFi Password这两栏填入WiFi名称和密码。

![wifi设置](./image/3.png)

打开micro-ROS example-app settings，Ros domain id of the micro-ROS为5，如果局域网内有多用户同时使用的情况，可修改参数以避免冲突。Ros namespace of the micro-ROS默认为空，正常情况下可以不修改，如果修改非空字符（10个字符以内），则会在节点和话题前加上namespace参数。

![domainID设置](./image/4.png)

## 编译烧录
```bash
idf.py build flash
```
完成后，Micro Ros板子所需要的代码就烧录完成了



