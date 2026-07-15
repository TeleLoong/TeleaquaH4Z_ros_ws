# Teleh4z static free-surface sweep

Run the default Gazebo sweep (pitch 0, +10, -10 degrees; z -1 to +1 m):

```bash
cd /home/clj/teleaqua-resources
./scripts/start_static_free_surface_sweep.sh
```

The default is headless for repeatable batch execution. To watch the sweep in
the Gazebo GUI, allow the container to use the display and set `GUI=1`:

```bash
xhost +local:docker
GUI=1 ./scripts/start_static_free_surface_sweep.sh
```

Pass runner options directly to the launcher, for example:

```bash
./scripts/start_static_free_surface_sweep.sh \
  --z-step 0.02 --settle-steps 20 \
  --output experiments/static_free_surface/output/gazebo.csv
```

The launcher builds the hydrodynamics plugin, starts Gazebo server-only, and
loads `models/teleh4z_zaxis_static_deployed`, whose two arm joints are fixed at
the deployed angles. It runs without PX4 or motor commands. The four
per-propeller ratios are the deployed air propellers in right-front,
left-front, left-back, right-back order.

For cross-domain consistency, both the normal `teleh4z_zaxis` model and this
dedicated static model are mass/buoyancy matched to the Stonefish deployed-arm
mesh model: total robot mass approximately `1.489192474 kg` and full-submergence
buoyancy `15.377871758 N`. The normal model retains its Z-axis guide, movable
arm joints, and controllers; the dedicated model fixes the arms in the
deployed configuration and removes experiment-irrelevant constraints.

Generate Fig. 1 and Table 1 on the host:

```bash
python3 experiments/static_free_surface/analyze_static_sweep.py \
  --gazebo experiments/static_free_surface/output/gazebo.csv \
  --stonefish /path/to/stonefish_static_sweep.csv \
  --output-dir experiments/static_free_surface/output/compare
```

Omit `--stonefish` to generate Gazebo-only figures and metrics immediately
after a Gazebo sweep.

Figures use an IEEE/ICRA-oriented double-column layout, serif/STIX fonts,
colorblind-safe colors plus distinct line styles, and a shared legend outside
the plotting area. Each pitch is exported as both vector PDF and 400 dpi PNG.
Table 1 is also rendered automatically as a booktabs-style vector PDF and
400 dpi PNG next to its machine-readable CSV.

Use `--stonefish-z-sign -1` only when the Stonefish input uses positive-down Z.
The center of buoyancy is the world-Z coordinate of the submerged-weighted
sample points; it is a discrete approximation and is `NaN` when fully dry.
