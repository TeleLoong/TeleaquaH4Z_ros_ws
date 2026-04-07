# ros2_ws KNOWLEDGE BASE

## OVERVIEW
ROS 2 workspace wrapper. The only source package present is `src/px4_msgs`; `build/`, `install/`, and `log/` are generated colcon outputs.

## STRUCTURE
```
ros2_ws/
├── src/px4_msgs/             # real source package
├── build/                    # generated colcon build output
├── install/                  # generated install tree
├── log/                      # generated colcon logs
├── docker_airframe_4024.sh   # custom airframe setup helper
└── docker_rcS_boot.sh        # PX4 rcS/SITL boot helper
```

## WHERE TO LOOK
| Task | Location | Notes |
|---|---|---|
| ROS interface schema | `src/px4_msgs/` | only contributor-owned package here |
| PX4 SITL boot customization | `docker_rcS_boot.sh` | shell startup logic |
| Custom airframe params | `docker_airframe_4024.sh` | TeleAI MCUUV defaults |

## CONVENTIONS
- Work in `src/px4_msgs`; treat the rest of the workspace as generated/runtime scaffolding.
- PX4 shell commands in helper scripts use the `px4-` alias environment.
- `docker_airframe_4024.sh` is parameter-heavy config, not general ROS package code.

## ANTI-PATTERNS
- Do not place source guidance in `build/`, `install/`, or `log/`.
- Do not edit generated headers/libs under `install/px4_msgs`; change `src/px4_msgs` instead.

## COMMANDS
```bash
colcon build --symlink-install
cmake --build /home/user/ros2_ws/build/px4_msgs
cmake --install /home/user/ros2_ws/build/px4_msgs
```

## NOTES
- Child AGENTS exists at `src/px4_msgs/`.
- `src/px4_msgs/` has its own `.git` boundary even though `/home/user` and `PX4-Autopilot/` do not.
