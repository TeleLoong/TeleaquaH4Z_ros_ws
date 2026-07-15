# Teleaqua 仿真资源仓库

本仓库用于维护 Teleaqua 系列载具在 **PX4 + Gazebo Harmonic + ROS 2 Humble** 仿真中的外部资源，包括 Gazebo 模型、世界文件、自定义插件、PX4 airframe、ROS 2 控制包和常用启动脚本。

> [!TIP]
> 新电脑部署、Docker 启动、插件编译、PX4/ROS 2 启动流程请优先阅读 [环境搭建指南.md](环境搭建指南.md)。

## 快速启动

本节命令在 **宿主机** 执行，进入的是从 GitHub 拉取到本机的 `teleaqua-resources` 仓库目录。启动脚本会自动检查并启动 `teleh4z-sim` 容器，再在容器内启动 Gazebo、PX4 和 ROS 2 相关进程，因此不需要先手动进入容器。

> 启动脚本会把宿主机的 `$HOME/teleaqua-resources` 作为容器内 `/home/user/external` 使用。后续容器内命令中的 `/home/user/external` 都对应这个本机仓库目录。

一键启动 TeleH4Z 仿真：

```bash
cd "$HOME/teleaqua-resources"
./scripts/start_sim.sh
```

一键启动 TeleH4Z 仿真并自动打开 QGroundControl 手柄控制链路：

```bash
cd "$HOME/teleaqua-resources"
./scripts/start_sim_gamepad.sh
```

如果只是调试 ROS 2 节点，不希望每次重新编译插件：

```bash
cd "$HOME/teleaqua-resources"
SKIP_PLUGIN_BUILD=1 ./scripts/start_sim.sh
```

## 仓库结构

```text
teleaqua-resources/
├── models/                 # Gazebo 模型与网格资源
├── worlds/                 # Gazebo 世界文件
├── plugins/                # 自定义 Gazebo 插件源码与编译脚本
├── teleai_airframes/       # PX4 SITL airframe 配置
├── ros2_packages/          # 随资源仓库同步的 ROS 2 package
├── experiments/            # 实验脚本、数据处理和实验说明
├── scripts/                # 宿主机一键启动与实验入口脚本
├── user/                   # 用户记录、日报等非核心运行资料
├── README.md
└── 环境搭建指南.md
```

## 主要资源

### models

`models/` 存放仿真载具模型：

```text
models/
├── teleh4z/                         # TeleH4Z 主模型
├── teleh4z_zaxis/                   # Z 轴跨域实验模型
├── teleh4z_zaxis_static_deployed/   # 静态水面实验模型
├── teleaquah8p/                     # TeleaquaH8P 本体
└── teleaquah8p_mono_cam/            # 带单目相机的 H8P 组合模型
```

TeleH4Z 的执行器、模式和 topic 约定见 [models/teleh4z/TELEH4Z_CONTRACT.md](models/teleh4z/TELEH4Z_CONTRACT.md)。

### worlds

`worlds/` 存放 Gazebo 仿真世界：

```text
worlds/
├── playground.sdf
└── pool_apriltagex.sdf
```

`playground.sdf` 是当前主要使用的水池环境，包含水池结构、AprilTag、物理系统和传感器系统配置。

### plugins

`plugins/` 存放自定义 Gazebo 插件源码：

```text
plugins/
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

`teleai_airframes/` 存放 PX4 SITL 自定义 airframe：

```text
teleai_airframes/
├── 4024_gz_teleaquah8
└── 4026_gz_teleh4z
```

修改 airframe、模型或世界文件后，需要同步到 PX4 运行目录：

```bash
/home/user/sync_external_into_px4.sh
```

### ros2_packages

`ros2_packages/teleh4z_manager` 提供 TeleH4Z 模式管理、手柄映射、水下推进器旁路和 Z 轴跨域实验 runner。容器内可同步到 ROS 2 workspace 后构建：

```bash
mkdir -p /home/user/ros2_ws/src/teleh4z_manager
cp -a /home/user/external/ros2_packages/teleh4z_manager/. /home/user/ros2_ws/src/teleh4z_manager/
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select teleh4z_manager --symlink-install
```

### scripts

`scripts/` 存放宿主机侧入口脚本：

```text
scripts/
├── start_sim.sh                       # TeleH4Z 基础仿真一键启动
├── start_sim_gamepad.sh               # TeleH4Z + QGC 手柄控制一键启动
├── start_zaxis_cross_domain.sh        # Z 轴跨域 PX4/Gazebo 实验入口
└── start_static_free_surface_sweep.sh # 静态水面扫描实验入口
```

## 上传前检查

提交到公司 GitHub 前建议检查：

```bash
git status --short
git submodule status --recursive
find . -type d -name build -prune -print
```

注意：

- `plugins/*/build/` 是插件编译产物，通常不建议作为源码维护重点。
- `models/`、`worlds/`、`teleai_airframes/` 和 `ros2_packages/` 是核心源码入口。
- 修改插件源码后需要重新编译插件。
- 修改模型、世界、airframe 后需要重新同步到 PX4 目录。
- 上传前确认没有误提交本机临时日志、ROS 2 `build/install/log`、PX4 build 产物或 Docker 状态文件。
