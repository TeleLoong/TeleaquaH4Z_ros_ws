# px4_msgs KNOWLEDGE BASE

## OVERVIEW
ROS 2 interface package generated from PX4 message definitions. This package is schema-first: mostly `.msg` files plus one `.srv`, with `ament_cmake`/`rosidl` build wiring.

## WHERE TO LOOK
| Task | Location | Notes |
|---|---|---|
| Message definitions | `msg/` | 236 source `.msg` files |
| Service definitions | `srv/` | `VehicleCommand.srv` only |
| Build wiring | `CMakeLists.txt` | `rosidl_generate_interfaces(...)` |
| Package metadata | `package.xml` | `ament_cmake`, `rosidl_default_generators` |
| Sync/build guidance | `README.md` | manual copy and colcon workflow |

## CONVENTIONS
- Keep all ROS messages directly under `msg/`; the README explicitly says the generation pipeline does not support subdirectories.
- This package tracks PX4 message branches/versions; branch compatibility matters.
- `package.xml` declares `ament_lint_common` test dependency and `rosidl_interface_packages` membership.

## ANTI-PATTERNS
- Do not edit generated `build/` or `install/` artifacts when changing interfaces.
- Do not forget the matching PX4 source-of-truth in `/home/user/PX4-Autopilot/msg`.
- Do not introduce nested message layouts.

## COMMANDS
```bash
cd /home/user/ros2_ws && colcon build --symlink-install --packages-select px4_msgs
rm -f msg/*.msg srv/*.srv
cp /home/user/PX4-Autopilot/msg/*.msg msg/
cp /home/user/PX4-Autopilot/msg/versioned/*.msg msg/
cp /home/user/PX4-Autopilot/srv/*.srv srv/
```

## NOTES
- In this container snapshot, the package appears as a standalone source checkout inside a ROS workspace, not as a generated mirror.
