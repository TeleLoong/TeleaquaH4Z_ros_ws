# TeleaquaH4Z Workspace Knowledge Base

## Overview

This repository is a combined TeleaquaH4Z simulation workspace:

- The repository root is a ROS 2 workspace wrapper.
- `src/teleh4z_manager/` contains TeleH4Z ROS 2 control nodes.
- `src/px4_msgs/` contains PX4 ROS 2 message definitions.
- `teleaqua-resources/` contains Gazebo/PX4 simulation assets, custom plugins, airframes, experiments, and host-side launch scripts.

Generated build outputs such as `build/`, `install/`, and `log/` should stay out of source control.

## Structure

```text
TeleaquaH4Z_ros_ws/
├── src/
│   ├── teleh4z_manager/        # TeleH4Z mode manager and simulation control nodes
│   └── px4_msgs/               # PX4 ROS 2 message definitions
├── teleaqua-resources/         # Gazebo/PX4 external simulation resources
├── archive/                    # archived helper scripts
├── build/                      # generated colcon build output
├── install/                    # generated install tree
└── log/                        # generated colcon logs
```

`teleaqua-resources/` has the resource-oriented layout:

```text
teleaqua-resources/
├── models/                     # Gazebo models and mesh assets
├── worlds/                     # Gazebo world SDF files
├── plugins/                    # custom Gazebo plugin source and build helper
├── teleai_airframes/           # PX4 SITL airframe definitions
├── ros2_packages/              # resource-side copy of ROS 2 packages
├── experiments/                # experiment scripts, data processing, docs
├── scripts/                    # host-side launch and experiment entrypoints
└── 环境搭建指南.md
```

## Where To Look

| Task | Location | Notes |
|---|---|---|
| ROS 2 mode/control code | `src/teleh4z_manager/` | mode manager, water thruster commander, launch files |
| PX4 ROS 2 message schema | `src/px4_msgs/` | PX4 interface messages |
| Gazebo model assets | `teleaqua-resources/models/` | TeleH4Z and TeleaquaH8 model SDF/meshes |
| Gazebo worlds | `teleaqua-resources/worlds/` | pool/playground environments |
| Gazebo plugin source | `teleaqua-resources/plugins/` | C++ plugin source and CMake roots |
| Plugin build helper | `teleaqua-resources/plugins/build_plugin.sh` | rebuilds custom Gazebo plugins |
| PX4 SITL airframes | `teleaqua-resources/teleai_airframes/` | 4024 H8 and 4026 H4Z airframes |
| Host launch scripts | `teleaqua-resources/scripts/` | one-command simulation and experiment launchers |
| Deployment docs | `teleaqua-resources/环境搭建指南.md` | Docker, PX4, Gazebo, ROS 2 workflow |
| Archived helpers | `archive/` | older Docker/PX4 setup helpers |

## Conventions

- Treat the repository root as the ROS 2 workspace. Build from `/home/user/ros2_ws` inside the container.
- Treat `teleaqua-resources/` as the external resource tree. It is usually mounted in the container as `/home/user/external`.
- Keep source code under `src/` and `teleaqua-resources/`; do not edit generated output under `build/`, `install/`, or `log/`.
- Models and worlds are SDF-first; plugin code lives under `teleaqua-resources/plugins/`.
- `teleaqua-resources/plugins/build_plugin.sh` is the normal plugin rebuild entrypoint.
- Host-side one-command launchers live under `teleaqua-resources/scripts/` and are run from the host, not from inside the container.

## Anti-Patterns

- Do not place source guidance or code in `build/`, `install/`, or `log/`.
- Do not edit generated headers/libs under `install/px4_msgs`; change `src/px4_msgs` instead.
- Do not edit files under `teleaqua-resources/plugins/*/build/`; those are generated plugin build outputs.
- Do not assume asset directories contain code ownership; most non-plugin content is meshes, SDF, textures, or config.
- Do not commit local experiment output directories such as `teleaqua-resources/experiments/*/output`.

## Commands

Build the ROS 2 workspace inside the container:

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Build only the TeleH4Z manager package:

```bash
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select teleh4z_manager --symlink-install
```

Rebuild Gazebo plugins inside the container:

```bash
cd /home/user/external/plugins
./build_plugin.sh
```

Run the host-side one-command launcher:

```bash
cd "$HOME/TeleaquaH4Z_ros_ws/teleaqua-resources"
./scripts/start_sim.sh
```

## Notes

- Child AGENTS files may provide narrower guidance and should be followed for their subtrees.
- `src/px4_msgs/` has its own AGENTS guidance.
- `teleaqua-resources/plugins/` has its own AGENTS guidance.
- The historical `teleaqua-resources/AGENTS.md` guidance is merged here so root-level agents understand both the ROS workspace and the external simulation resource tree.
