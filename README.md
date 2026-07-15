# TeleaquaH4Z 仿真工作区

本仓库用于维护 TeleaquaH4Z / TeleaquaH8 在 **PX4 + Gazebo Harmonic + ROS 2 Humble** 场景中的仿真代码与资源。当前仓库同时包含 ROS 2 workspace 和 Gazebo/PX4 外部资源：

- 根目录是 ROS 2 workspace，主要提供 TeleH4Z 模式管理、手柄映射、水下推进器控制和 PX4 消息接口。
- `teleaqua-resources/` 存放 Gazebo 模型、世界文件、自定义插件、PX4 airframe、实验脚本和一键启动脚本。

> [!TIP]
> 新电脑部署、Docker 启动、插件编译、PX4/ROS 2 启动流程请优先阅读 [teleaqua-resources/环境搭建指南.md](teleaqua-resources/环境搭建指南.md)。

## 快速启动

一键启动脚本在 **宿主机** 执行，进入的是本仓库中的 `teleaqua-resources/` 目录。脚本会自动检查并启动 `teleh4z-sim` 容器，再在容器内启动 Gazebo、PX4 和 ROS 2 相关进程，因此不需要先手动进入容器。

> 启动脚本会把宿主机的 `TeleaquaH4Z_ros_ws/teleaqua-resources` 作为容器内 `/home/user/external` 使用；根目录 workspace 通常挂载为容器内 `/home/user/ros2_ws`。

一键启动 TeleH4Z 仿真：

```bash
cd "$HOME/TeleaquaH4Z_ros_ws/teleaqua-resources"
./scripts/start_sim.sh
```

一键启动 TeleH4Z 仿真并自动打开 QGroundControl 手柄控制链路：

```bash
cd "$HOME/TeleaquaH4Z_ros_ws/teleaqua-resources"
./scripts/start_sim_gamepad.sh
```

如果只是调试 ROS 2 节点，不希望每次重新编译插件：

```bash
cd "$HOME/TeleaquaH4Z_ros_ws/teleaqua-resources"
SKIP_PLUGIN_BUILD=1 ./scripts/start_sim.sh
```

## 仓库结构

```text
TeleaquaH4Z_ros_ws/
├── src/
│   ├── teleh4z_manager/        # TeleH4Z 模式管理与控制节点
│   └── px4_msgs/               # PX4 ROS 2 消息定义包
├── teleaqua-resources/         # Gazebo / PX4 仿真资源与启动脚本
├── archive/                    # 早期辅助脚本归档
├── README.md
└── .gitignore
```

`teleaqua-resources/` 的主要内容：

```text
teleaqua-resources/
├── models/                     # Gazebo 模型与网格资源
├── worlds/                     # Gazebo 世界文件
├── plugins/                    # 自定义 Gazebo 插件源码与编译脚本
├── teleai_airframes/           # PX4 SITL airframe 配置
├── ros2_packages/              # 随资源仓库同步的 ROS 2 package 副本
├── experiments/                # 实验脚本、数据处理和实验说明
├── scripts/                    # 宿主机一键启动与实验入口脚本
└── 环境搭建指南.md
```

## ROS 2 工作区功能

`src/teleh4z_manager` 是当前 ROS 2 侧的核心控制包，主要功能包括：

1. 水下到空中（WATER -> AIR）切换流程。
2. 空中到水下（AIR -> WATER）切换流程。
3. 切换超时保护与回滚逻辑。
4. 机械臂轨迹控制与到位判定。
5. QGroundControl 手柄按钮到空/水模式请求的映射。
6. 水下推进器 PWM 旁路控制与 Gazebo topic bridge。

`src/px4_msgs` 提供 PX4 ROS 2 消息定义，包含 TeleH4Z 使用的 `VehicleAirWaterStatus`、UUV setpoint、manual control 等消息接口。

## 仿真资源

### models

`teleaqua-resources/models/` 存放仿真载具模型：

```text
teleaqua-resources/models/
├── teleh4z/                         # TeleH4Z 主模型
├── teleh4z_zaxis/                   # Z 轴跨域实验模型
├── teleh4z_zaxis_static_deployed/   # 静态水面实验模型
├── teleaquah8p/                     # TeleaquaH8P 本体
└── teleaquah8p_mono_cam/            # 带单目相机的 H8P 组合模型
```

TeleH4Z 的执行器、模式和 topic 约定见 [teleaqua-resources/models/teleh4z/TELEH4Z_CONTRACT.md](teleaqua-resources/models/teleh4z/TELEH4Z_CONTRACT.md)。

### worlds

`teleaqua-resources/worlds/` 存放 Gazebo 仿真世界：

```text
teleaqua-resources/worlds/
├── playground.sdf
└── pool_apriltagex.sdf
```

`playground.sdf` 是当前主要使用的水池环境，包含水池结构、AprilTag、物理系统和传感器系统配置。

### plugins

`teleaqua-resources/plugins/` 存放自定义 Gazebo 插件源码：

```text
teleaqua-resources/plugins/
├── bidir_motor_model/          # 双向水下推进器电机模型
├── hydrodynamics/              # 水动力 / 附加质量 / 空水过渡效果
├── buoyancy/                   # 浮力相关插件
├── hydrodynamics_offical/      # 备用水动力插件实现
└── build_plugin.sh             # 插件编译入口
```

容器内编译常用插件：

```bash
cd /home/user/external/plugins
./build_plugin.sh
```

### teleai_airframes

`teleaqua-resources/teleai_airframes/` 存放 PX4 SITL 自定义 airframe：

```text
teleaqua-resources/teleai_airframes/
├── 4024_gz_teleaquah8
└── 4026_gz_teleh4z
```

修改 airframe、模型或世界文件后，需要同步到 PX4 运行目录：

```bash
/home/user/sync_external_into_px4.sh
```

## 构建 ROS 2 Workspace

在容器内执行：

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

只构建 TeleH4Z 控制包：

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select teleh4z_manager --symlink-install
source install/setup.bash
```

启动模式管理与桥接：

```bash
ros2 launch teleh4z_manager mode_manager.launch.py
```

## 使用示例

向模式管理节点发送切换请求：

```bash
# 切换到 AIR
ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String "{data: 'air'}"

# 切换到 WATER
ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String "{data: 'water'}"
```

## 常用参数

`mode_manager.launch.py` 常用参数包括：

- `model_name`：Gazebo 模型名，默认 `teleh4z_0`。
- `world_name`：Gazebo 世界名，默认 `playground`。
- `arm_move_duration`：机械臂动作时长，默认 `2.0`。
- `arm_settle_duration`：机械臂稳定判定时间，默认 `0.25`。
- `arm_motion_timeout`：机械臂动作超时，默认 `8.0`。
- `px4_switch_timeout`：PX4 模式切换超时，默认 `5.0`。

水下手柄旁路、QGC 手柄按钮映射和完整启动流程见 [teleaqua-resources/环境搭建指南.md](teleaqua-resources/环境搭建指南.md)。

## 上传前检查

提交到 GitHub 前建议检查：

```bash
git status --short
find . -type d \( -name build -o -name install -o -name log \) -prune -print
```

注意：

- 根目录 `build/`、`install/`、`log/` 是 ROS 2 构建产物，不应提交。
- `teleaqua-resources/plugins/*/build/` 是 Gazebo 插件编译产物，不应提交。
- 修改模型、世界、airframe 后，需要重新同步到 PX4 目录。
- 修改 ROS 2 节点后，需要重新构建 workspace。
