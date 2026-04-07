# TeleaquaH4Z ROS 2 Workspace

TeleaquaH4Z 的 ROS 2 仿真工作区，面向 PX4 + Gazebo 场景，提供空/水模式切换管理与桥接能力。

## 项目简介

本仓库用于承载 TeleaquaH4Z 在 ROS 2 侧的核心功能，当前重点是：
- 基于状态机的空中/水下模式切换管理
- 机械臂展开/收拢与 PX4 模式切换联动
- ROS 2 与 Gazebo 话题桥接（轨迹、关节状态、过渡状态、相机、位姿）
- PX4 消息接口包集成（px4_msgs）

## 主要功能

1. 水下到空中（WATER -> AIR）切换流程
2. 空中到水下（AIR -> WATER）切换流程
3. 切换超时保护与回滚逻辑
4. 机械臂轨迹控制与到位判定

## 仓库结构

```text
ros2_ws/
|- src/
|  |- teleh4z_manager/     # 模式切换管理节点（Python）
|  `- px4_msgs/            # PX4 ROS 2 消息定义包
|- build/                  # colcon 构建产物（没有加入版本控制）
|- install/                # colcon 安装产物（没有加入版本控制）
`- log/                    # colcon 日志（没有加入版本控制）
```

## 环境依赖

- ROS 2（Humble）
- PX4 SITL （这里加入docker image镜像链接）
- Gazebo （Harmonic）

## 快速开始

### 1) 构建

```bash
colcon build --symlink-install
```
### 2) 加载环境

```bash
source install/setup.bash
```
### 3) 启动模式管理与桥接

```bash
ros2 launch teleh4z_manager mode_manager.launch.py
```
## 使用示例

向模式管理节点发送切换请求：

```bash
# 切换到 AIR
ros2 topic pub /teleh4z/mode_request std_msgs/msg/String "{data: 'air'}" -1

# 切换到 WATER
ros2 topic pub /teleh4z/mode_request std_msgs/msg/String "{data: 'water'}" -1
```
## 关键话题

- 输入：`/teleh4z/mode_request`（std_msgs/String）
- PX4 状态订阅：`/fmu/out/vehicle_air_water_status`
- PX4 设定发布：`/fmu/in/vehicle_air_water_status_setpoint`
- 机械臂轨迹发布：`/arm_trajectory`

## 可配置参数

- `model_name`：Gazebo 模型名，默认 `teleh4z_0`
- `world_name`：Gazebo 世界名，默认 `playground`
- `arm_move_duration`：机械臂动作时长，默认 `2.0`
- `arm_settle_duration`：机械臂稳定判定时间，默认 `0.25`
- `arm_motion_timeout`：机械臂动作超时，默认 `8.0`
- `px4_switch_timeout`：PX4 模式切换超时，默认 `5.0`

