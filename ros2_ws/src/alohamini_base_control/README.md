# alohamini_base_control

**AlohaMini 三全向轮移动底盘**的 C++ `ros2_control` 驱动，从 `lerobot_alohamini`
的 Python 底盘逻辑（`src/lerobot/robots/alohamini/lekiwi.py`）移植而来。

它取代了原来的 ZMQ 桥（`alohamini_nav_bridge`，需要依赖 lerobot 的 `lekiwi_host.py`
进程），改为**直接通过串口按 Feetech 协议 0 通信**的原生接口——不再需要 host 进程。

本功能包**完全自包含**：硬件接口、控制器、ros2_control 描述、配置、launch、协议单元
测试都在包内。**不修改** `alohamini_description`、`alohamini_bringup`、
`alohamini_nav_bridge` 任何文件。

## 组成

- **`alohamini_base_control/AlohaMiniBaseHardware`** — `hardware_interface::SystemInterface`。
  打开串口（termios，1 Mbps），将轮子设为速度模式，对外暴露每个轮子的 `velocity`
  命令接口和 `velocity`/`position` 状态接口（单位 rad/s、rad）。
- **`alohamini_base_control/OmniBaseController`** — `controller_interface::ControllerInterface`。
  订阅 `/cmd_vel`（`geometry_msgs/Twist`），用全向正运动学算出三个轮速，并从轮速反馈
  用逆运动学发布 `/odom`（+ `odom`→`base_link` TF）。含 `/cmd_vel` 看门狗。
- **`feetech_protocol`** — 不依赖 ROS 的 Feetech 协议 0（SYNC 读写、符号-幅值编码、
  校验和），在 `test/test_feetech_protocol.cpp` 里有单元测试。

## 硬件映射

| URDF 关节  | 电机 ID | 位置  | 运动学角 |
| -------- | ----- | --- | ---- |
| `wheel1` | 8     | 左   | 240° |
| `wheel2` | 10    | 右   | 120° |
| `wheel3` | 9     | 后   | 0°   |

这与 `lekiwi.py` 的「ID↔角度」配对完全一致——原代码中运动学矩阵
`angles = [240, 0, 120] - 90` 里，`base_left_wheel`=ID 8 是第 0 行(240°)、
`base_back_wheel`=ID 9 是第 1 行(0°)、`base_right_wheel`=ID 10 是第 2 行(120°)。
本包中角度跟着关节走（`controllers.yaml` 里 `wheel_names` ↔ `wheel_angles_deg`
同索引），电机 ID 也跟着关节走（xacro 里的 `motor_id` 参数）。控制器把每个轮速命令
写到**按名字**声明的命令接口，硬件再发给该关节对应的 `motor_id`——所以「行→ID」的
对应关系是全程按名字匹配的（比原代码位置式的 `wheel_raw[0/1/2]` 列表更安全，索引不会
错位）。

角度偏移 `-90°`，`wheel_radius=0.05 m`，`base_radius=0.125 m`（均来自 `lekiwi.py`）。
电机 ID 和角度都是参数（`config/controllers.yaml`、
`urdf/alohamini_base.ros2_control.xacro`）——若实机上转向不对，改这里即可，无需重新编译。

## 依赖

在 `u_2204` distrobox 容器里的 **ROS 2 Humble** 下构建并测试通过（该环境已装
`ros2_control`）。若在全新环境搭建：

```bash
# Humble
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-controller-manager ros-humble-hardware-interface \
                 ros-humble-pluginlib ros-humble-realtime-tools \
                 ros-humble-joint-state-broadcaster ros-humble-xacro
```

> **架构提示**：树莓派是 ARM64、开发机常是 x86，编译出的 `.so` 不能跨架构拷贝。
> 部署到树莓派时要把**源码**同步过去、在树莓派上重新 `colcon build`。

## 构建与测试

```bash
distrobox enter u_2204
source /opt/ros/humble/setup.bash
cd ~/alohamini_lidar_imu/ros2_ws
colcon build --packages-up-to alohamini_base_control
colcon test --packages-select alohamini_base_control && colcon test-result --verbose
```

## 运行

Mock 模式（无硬件，验证运动学/odom/TF）：

```bash
ros2 launch alohamini_base_control base_control.launch.py use_mock_hardware:=true
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}' -r 10
ros2 topic echo /odom
```

实物硬件：

```bash
# serial_port 用底盘三个轮子(ID 8/9/10)所在的那条总线。
# 注意：不同机器上 ttyACM 编号可能变化，建议优先用固定的 udev 别名。
ros2 launch alohamini_base_control base_control.launch.py \
    serial_port:=/dev/ttyACM0
```

> **串口是共用的**——底盘与左臂、升降轴共用同一条总线。运行本驱动时**不要**同时运行
> lerobot 的 host（`lekiwi_host.py`），否则两者会争抢同一个串口设备。

### 实物启动步骤与注意事项

1. **每个新终端都要先 source**（否则会报 `find_package(hardware_interface)` 找不到）：
   
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ws/install/setup.bash        # 换成你的工作区路径
   ```

2. **选对串口**：底盘三个轮子舵机（ID 8/9/10）在哪条总线上就用哪个。多条 `ttyACM`
   分不清时：
   
   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB*
   readlink -f /dev/am_arm_follower_left   # 若有这个 udev 别名，优先用它，编号不会漂
   ```

3. **串口权限**：`sudo usermod -aG dialout $USER`（重新登录生效），或临时
   `sudo chmod 666 /dev/ttyACM0`。

4. **首次务必架空轮子**（离地）低速测试，确认三轮转向正确再落地。

5. **急停**：`Ctrl-C` 停 launch 时 `on_deactivate` 会自动写 0 速度停车；`/cmd_vel`
   断流超过 `cmd_timeout`（默认 0.5s）看门狗也会自动停车。

6. **速度限幅**：`controllers.yaml` 默认 `max_linear_speed=0.20`、
   `max_angular_speed=0.8`，首测建议再调小。

7. **暂无过流保护**：原 Python 驱动有过流跳闸（2000mA），本 C++ 版只移植了速度控制，
   未含过流监控——长时间堵转请留意舵机温度。

### 方向 / 符号约定（源自 lerobot WASD 遥操）

权威的 body 速度约定来自 lerobot 的键盘遥操
（`robots/alohamini/lekiwi_client.py::_from_keyboard_to_base_action`）。它是标准
REP-103，因此与 `/cmd_vel` 一一对应：

| 按键  | 动作      | 遥操下发             | `/cmd_vel` 字段   |
| --- | ------- | ---------------- | --------------- |
| `w` | 前进      | `x.vel += xy`    | `linear.x  > 0` |
| `s` | 后退      | `x.vel -= xy`    | `linear.x  < 0` |
| `z` | 左移      | `y.vel += xy`    | `linear.y  > 0` |
| `x` | 右移      | `y.vel -= xy`    | `linear.y  < 0` |
| `a` | 左转(逆时针) | `theta.vel += θ` | `angular.z > 0` |
| `d` | 右转(顺时针) | `theta.vel -= θ` | `angular.z < 0` |

`OmniBaseController` 逐行复刻了 lekiwi 的 `_body_to_wheel_raw` 运动学，所以实机上轮子
转向与 lerobot 遥操完全一致——预期无需调整 `wheel_angles_deg` 或电机 ID 映射。（与
Gazebo planar 仿真不同，实物驱动在轮子层面计算，不依赖 `base_link` 坐标系朝向，因此
这里不需要 `base_footprint` 旋转。）

**单位**：lerobot 遥操里 `theta.vel` 用 **deg/s**；`/cmd_vel` 的 `angular.z` 用
**rad/s**。控制器内部按 rad/s 计算，直接发 rad/s 即可。参考速度分挡：慢
`xy=0.15 m/s, θ=45°/s (0.79 rad/s)`、中 `0.20, 60°/s`、快 `0.25, 75°/s`——首测请用
「慢」挡或更低。

## Gazebo Classic 仿真

有**两个**仿真变体（按你想看什么来选）：

| Launch                    | 驱动路径                                            | 能否全向移动        | 何时用               |
| ------------------------- | ----------------------------------------------- | ------------- | ----------------- |
| `gazebo.launch.py`        | 真实 `OmniBaseController`，经 `gazebo_ros2_control` | 弱（无滚子物理）      | 想验证真实控制器 + 硬件接口链路 |
| `gazebo_planar.launch.py` | `libgazebo_ros_planar_move` 插件                  | **能**（真横移/自转） | 想*看到*底盘全向移动       |

两者对外都是相同的 `/cmd_vel`、`/odom`、`odom→base_link` TF，Nav2/SLAM 配置都无需改动。

### 变体 1 — ros2_control（`gazebo.launch.py`）

原样运行真实的 `OmniBaseController`，只把 `<ros2_control>` 硬件插件换成
`gazebo_ros2_control/GazeboSystem`。

```bash
source /opt/ros/humble/setup.bash
source ~/alohamini_lidar_imu/ros2_ws/install/setup.bash

# 带界面：
ros2 launch alohamini_base_control gazebo.launch.py
# 无界面：
ros2 launch alohamini_base_control gazebo.launch.py gui:=false

# 驱动：
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 20
ros2 topic echo /odom
```

> 注意：Gazebo Classic 把轮子当作普通旋转圆柱建模（没有全向滚子），所以底盘几乎不平移。
> 控制/odom/TF 链路是完整跑通的，但要看到明显的全向运动请用变体 2。

### 变体 2 — PlanarMove 全向（`gazebo_planar.launch.py`）

用 `libgazebo_ros_planar_move` 把底盘当作可在平面内自由滑动的刚体，直接由 `/cmd_vel`
驱动。前进、横移、自转都有效。

```bash
ros2 launch alohamini_base_control gazebo_planar.launch.py            # 带界面
ros2 launch alohamini_base_control gazebo_planar.launch.py gui:=false # 无界面

# 前进、横移、自转：
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}' -r 20
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {y: 0.2}}' -r 20
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}' -r 20
ros2 topic echo /odom
```

> 这是运动学层面的近似：轮子不通过接触产生运动（它们被设为低摩擦，仅用于视觉转动）。
> 这是在 Classic Gazebo 里不建模滚子的情况下仿真全向底盘的标准做法。

> **坐标系约定**：SolidWorks 导出的 `base_link` 不符合 REP-103（它的 +X 朝左、+Y 朝后）。
> planar 变体加了一个符合 REP-103 的 `base_footprint` 根坐标系，绕 Z 转 +90°（xacro 参数
> `base_yaw_deg`，默认 90），并驱动它，从而 `cmd_vel.linear.x`=前进、`linear.y`=左移
> （已在 GUI 里验证）。若在别的构建上前后/左右反了，用不同角度启动即可，如
> `base_yaw_deg:=-90`（改 xacro 默认值或包一层传参）——无需改代码。`/odom` 和 TF 在
> `base_footprint` 系下发布。

仿真专用文件（实物相关文件保持不动）：

- `urdf/alohamini_base.gazebo.xacro` — 描述 URDF + `GazeboSystem` ros2_control 块 +
  `gazebo_ros2_control` 插件 + 每轮摩擦（变体 1）。
- `urdf/alohamini_base.gazebo_planar.xacro` — 描述 URDF + `planar_move` 插件 +
  低摩擦轮子（变体 2）。
- `config/controllers_sim.yaml` — `controllers.yaml` 的副本，加了 `use_sim_time: true`
  和 `wheelN_joint` 命名（见下方说明）。变体 1 使用。
- `launch/gazebo.launch.py` — 变体 1 launch（起 Gazebo、spawn 机器人 + 两个控制器）。
  含下方的 Humble 专属修复。
- `launch/gazebo_planar.launch.py` — 变体 2 launch（起 Gazebo、spawn 机器人 +
  planar_move 插件）。

**全向轮近似**：Gazebo Classic 无法建模全向轮滚子接触。三个轮子被设为低横向摩擦
（`wheel_mu2:=0.0`），使底盘不会被相互对抗的轮子锁死。这验证了
`cmd_vel → 轮速 → odom → TF` 的链路和关节运动，但**不是**真实的轮胎/牵引模型。可用
`wheel_mu1:=... wheel_mu2:=...` launch/xacro 参数调节。

### 复现 / 干净环境提示

仿真是在 `u_2204`（Humble）下开发并修好两个拦路问题的，但测试期间该共享容器里**还有
其他用户的 Gazebo 实例在跑**，占用了默认 master 端口、留下孤儿 `gzserver` 进程。为了
可靠运行，请用干净环境：

```bash
# 1. 确保没有你自己的残留 gazebo：
pkill -9 -x gzserver

# 2. 若默认端口 11345 被别人占用，用你自己的 master 端口：
export GAZEBO_MASTER_URI=http://localhost:11346

# 3. 启动（服务器/CI 上无界面最稳）：
ros2 launch alohamini_base_control gazebo.launch.py gui:=false

# 4. 另开终端（同 ROS_DOMAIN_ID）驱动并观察：
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15}}' -r 20
ros2 topic echo /odom
ros2 control list_controllers      # 两个都应为 active
```

在 `gazebo.launch.py` 里修好了源码资产中的两个真实问题（都无需改
`alohamini_description`）：

1. **关节名与链接名冲突**：SolidWorks 导出的 URDF 把每个轮子关节命名成与其子链接相同
   （`wheel1/2/3`）。Gazebo Classic 会拒绝（`has a name collision` / 坐标系图成环）。
   launch 只把物理 `<joint ... type="continuous">` 声明重命名为 `wheelN_joint`；链接名
   仍是 `wheelN`。`controllers_sim.yaml` 和仿真 ros2_control 块都用 `wheelN_joint`。
2. **注释破坏 `gazebo_ros2_control`（Humble）**：该插件把 `robot_description` 作为
   `--param` 覆盖传给 controller_manager；含 `--` 的 XML 注释会触发
   `Couldn't parse parameter override rule`，导致 controller_manager 起不来（上游已在
   gz_ros2_control PR #505 修复，但已安装版本没有）。launch 会剥离生成 URDF 里的所有
   XML 注释。

launch 还会设 `GAZEBO_MODEL_DATABASE_URI=""`，避免 gzserver 启动时卡在联网拉取模型库；
并把描述包安装空间的 `share` 目录加进 `GAZEBO_MODEL_PATH`，使 URDF 里的
`package://alohamini_description/...` 网格路径（spawn 时会变成 `model://...`）能解析到
本地 `.STL` 文件，而不是报 `Failed to find mesh file`。

## 坐标系与方向约定（实机已验证）

底盘的 `base_link`（SolidWorks 导出）不符合 REP-103——它的坐标轴相对「x=前进、
y=左」转了约 90°，且 lekiwi 运动学里有 x/y 取负的历史约定。本驱动通过**三处解耦的
处理**把方向理顺，让实机、模型、里程计三者一致：

| 部件 | 处理 | 效果 |
|------|------|------|
| 实机运动 | 命令侧**不做任何旋转** | `/cmd_vel` `linear.x>0` → 底盘物理前进（与 lerobot WASD 遥操一致） |
| 模型朝向 | xacro 里 `base_footprint→base_link` 静态转 `base_yaw_deg`（默认 90°） | RViz 里车头朝 odom 的 +x |
| 里程计 | `omni_base_controller` 里把恢复出的 `vel.x/vel.y` 取负（抵消 lekiwi 正解取负、逆解不取负的 180° 翻转） | `/odom` 的前进方向、箭头朝向与车头、物理运动全部一致 |

里程计发布在 **`odom → base_footprint`**（REP-103，x=前进、y=左），再经静态关节到
`base_link`。整条 TF 链自洽，Nav2/SLAM 直接可用。

- **若 RViz 里方向仍不对**：只调 `base_yaw_deg`（launch 参数，免编译，试 `90/-90/180/0`）
  修车头朝向；`vel.x/y` 取负修的是「前进/后退」翻转。命令侧始终不动，怎么调都**不会**
  把实机方向带偏。
- **每轮转向符号**：电机 ID 和运动学角都是 `config/controllers.yaml` 与 xacro 里的参数，
  改动无需重新编译。
- 在 `u_2204`（Humble）验证：两个控制器都到 `active`，三个 `wheelN/velocity` 命令接口
  都被 claim，`/odom`、`/tf`、`/joint_states` 正常发布，轮速与 Python 参考实现一致
  （`x=0.1, wz=0.2` 时 `[左, 后, 右] = [2.232, 0.5, -1.232]` rad/s）。实机验证：`x+` 前进、
  车头/箭头/移动方向在 RViz 中一致。

## 常见问题排查

- **`find_package hardware_interface` 找不到 / 编译报错**：当前终端没 source ROS。先
  `source /opt/ros/humble/setup.bash` 再 `colcon build`。
- **`Failed to open serial port ... No such file or directory`**：串口名写错或设备不存在。
  用 `ls /dev/ttyACM* /dev/ttyUSB*` 核对，注意别把 `ttyACM0` 敲成 `ttytACM0`。
- **控制器都 active，但轮子不转、`/odom` twist 为 0**：多为舵机没切到速度模式或 ID 不符。
  本驱动在 `on_activate` 里已按「关扭矩→解锁→写 Operating_Mode=速度→重新开扭矩」的正确
  顺序切换模式（Feetech 的 Operating_Mode 在 EEPROM 区，扭矩开着时改不动）。若仍不转，
  确认这条总线上舵机 ID 确为 8/9/10、动力电源已接。
- **轮子会动，但 `/odom` 一直不变（twist 恒为 0）**：底盘在转、里程计却不更新，说明
  `read()` 读不回轮子的 Present_Velocity。原因通常是舵机固件**不支持 SYNC READ(0x82)**
  ——写命令是广播、不需要应答，所以轮子照转，但批量读会超时、速度恒读回 0。本驱动**默认
  用逐个 READ(0x02) 读速度**（`read()` 里 `use_sync_read=false`），对 Feetech 最兼容。
  若你的舵机确认支持 SYNC READ、想用更快的批量读，在 `<ros2_control>` 硬件参数里加
  `<param name="use_sync_read">true</param>`。排查时先看 `/wheel_joint_states`（或
  `/joint_states`）里轮子的 `velocity` 字段：持续发速度时若非零 → 读取正常；恒为 0 →
  读取失败。
- **`ros2_control_node` 段错误，堆栈在 `libfastrtps.so`**：这是 Fast-DDS 在树莓派
  (ARM64) 上的偶发崩溃，**与本包代码无关**。建议换 CycloneDDS：
  `sudo apt install ros-humble-rmw-cyclonedds-cpp`，然后
  `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`；本机跑可加 `export ROS_LOCALHOST_ONLY=1`，
  并固定 `ROS_DOMAIN_ID`。

## 替换 bringup 里的旧 ZMQ 桥

`alohamini_bringup` 保持不动。要把整个栈切到本驱动，在你自己的 launch（例如复制一份
`sensors_bridge.launch.py`）里，把对 `alohamini_nav_bridge/launch/bridge.launch.py` 的
include 换成 `alohamini_base_control/launch/base_control.launch.py`。`/cmd_vel`、
`/odom`、`odom`→`base_link` TF 接口完全一致，所以 Nav2/SLAM 配置无需改动。
