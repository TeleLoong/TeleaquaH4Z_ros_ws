# external KNOWLEDGE BASE

## OVERVIEW
Workspace-local simulation assets and Gazebo plugins. This tree is separate from upstream PX4 and mixes model assets, custom worlds, plugin source, and local plugin build outputs.

## STRUCTURE
```
external/
├── models/            # custom Gazebo models
├── worlds/            # custom world files
├── plugins/           # local Gazebo plugin source
└── custom_airframes/  # custom airframe assets/config staging
```

## WHERE TO LOOK
| Task | Location | Notes |
|---|---|---|
| Gazebo model assets | `models/` | teleaquah8p variants |
| World setup | `worlds/` | pool environment SDF |
| Plugin code | `plugins/` | C++ source + CMake roots |
| Build helper | `plugins/build_plugin.sh` | compiles all three plugins |

## CONVENTIONS
- Treat this as custom local simulation code/assets, not upstream PX4 source.
- Models/worlds are SDF-first; plugin code lives only under `plugins/`.
- `build_plugin.sh` rebuilds each plugin by deleting its `build/` directory first.

## ANTI-PATTERNS
- Do not edit files under `plugins/*/build/`; those are generated outputs.
- Do not assume asset directories contain code ownership; most non-plugin content is meshes/config/SDF.

## NOTES
- Child AGENTS exists at `plugins/`.
