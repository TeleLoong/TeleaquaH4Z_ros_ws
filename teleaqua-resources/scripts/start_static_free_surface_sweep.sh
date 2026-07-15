#!/usr/bin/env bash
set -euo pipefail

CONTAINER="${CONTAINER:-teleh4z-sim}"
MODEL_NAME="${MODEL_NAME:-teleh4z_zaxis_0}"
WORLD_NAME="${WORLD_NAME:-playground}"
GUI="${GUI:-0}"

if ! docker ps --filter "name=^/${CONTAINER}$" --format '{{.Names}}' |
  grep -qx "$CONTAINER"; then
  docker start "$CONTAINER" >/dev/null
fi

docker exec "$CONTAINER" bash -lc '
  set -e
  cd /home/user/external/plugins/hydrodynamics
  cmake -S . -B build
  cmake --build build -j"$(nproc)"
'

docker exec \
  -e MODEL_NAME="$MODEL_NAME" \
  -e WORLD_NAME="$WORLD_NAME" \
  -e GUI="$GUI" \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e SWEEP_ARGS="$*" \
  "$CONTAINER" bash -lc '
    set -euo pipefail
    export GZ_SIM_RESOURCE_PATH="/home/user/external/models:/home/user/external/worlds:/home/user/PX4-Autopilot/Tools/simulation/gz/models:${GZ_SIM_RESOURCE_PATH:-}"
    export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/user/external/plugins/hydrodynamics/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
    cleanup() {
      [[ -n "${GZ_PID:-}" ]] && kill "$GZ_PID" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    GZ_ARGS=(-r "/home/user/external/worlds/${WORLD_NAME}.sdf")
    [[ "$GUI" == "1" ]] || GZ_ARGS=(-s "${GZ_ARGS[@]}")
    gz sim "${GZ_ARGS[@]}" \
      >/tmp/teleh4z_static_sweep_gazebo.log 2>&1 &
    GZ_PID=$!
    for _ in $(seq 1 60); do
      gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create" && break
      kill -0 "$GZ_PID" 2>/dev/null || {
        tail -n 80 /tmp/teleh4z_static_sweep_gazebo.log
        exit 1
      }
      sleep 0.5
    done
    gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create" || {
      echo "Gazebo world did not expose the create service"
      exit 1
    }

    gz service -s "/world/${WORLD_NAME}/create" \
      --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
      --timeout 5000 \
      --req "sdf_filename: \"/home/user/external/models/teleh4z_zaxis_static_deployed/model.sdf\", name: \"${MODEL_NAME}\", pose {position {z: -1.0}}"

    cd /home/user/external
    # SWEEP_ARGS intentionally expands into CLI arguments supplied to this launcher.
    python3 experiments/static_free_surface/run_static_sweep.py \
      --model-name "$MODEL_NAME" --world-name "$WORLD_NAME" $SWEEP_ARGS
  '
