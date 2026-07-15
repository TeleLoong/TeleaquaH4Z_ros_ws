# external/plugins KNOWLEDGE BASE

## OVERVIEW
Three local Gazebo plugin projects: `bidir_motor_model`, `buoyancy`, and `hydrodynamics`. Each plugin is an independent mini CMake project with its own generated `build/` directory.

## WHERE TO LOOK
| Task | Location | Notes |
|---|---|---|
| Motor plugin | `bidir_motor_model/` | `BiDirectionalMotorModel.cpp` |
| Buoyancy plugin | `buoyancy/` | `buoyancy.cpp`, private helpers |
| Hydrodynamics plugin | `hydrodynamics/` | `hydrodynamics.cpp`, header |
| Bulk build helper | `build_plugin.sh` | rebuilds all three plugins |

## CONVENTIONS
- Plugin source is C++ + CMake only; build products live in per-plugin `build/` directories.
- `build_plugin.sh` removes `build/`, recreates it, and runs `cmake .. && make -j$(nproc)`.

## ANTI-PATTERNS
- Do not hand-edit `build/Makefile`, `CMakeCache.txt`, or compiled `.so` outputs.
- Do not make plugin-local fixes in the bulk build script unless the workflow itself is broken.

## NOTES
- Keep changes plugin-local unless you truly need to alter the common rebuild flow.
