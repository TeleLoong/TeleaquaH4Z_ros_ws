#!/bin/bash
# =============================================================================
# Teleh4Z Gazebo + PX4 Z-axis cross-domain experiment launcher
#
# This starts the PX4-controlled Z-axis cross-domain path:
#   - Terminal A: Micro XRCE-DDS Agent + Gazebo + PX4 SITL
#   - Terminal B: ROS 2 PX4 offboard Z-axis task + CSV logger
#
# The runner does not publish Gazebo motor speeds. PX4 publishes motor commands
# to the hybrid air propeller plugins through the normal Gazebo motor topic.
#
# Usage:
#   ./start_zaxis_cross_domain.sh
#   SKIP_PLUGIN_BUILD=1 ./start_zaxis_cross_domain.sh
#   RESTART_CONTAINER=0 ./start_zaxis_cross_domain.sh
#
# Optional:
#   OUTPUT_CSV_PATH=/home/user/external/experiments/zaxis/src/my_trial.csv TRIAL_ID=3 CONDITION=nominal ./start_zaxis_cross_domain.sh
#   TIMESTAMP=20260706_153000 ./start_zaxis_cross_domain.sh
# =============================================================================

set -e

CONTAINER="${CONTAINER:-teleh4z-sim}"
RESTART_CONTAINER="${RESTART_CONTAINER:-1}"
MODEL_NAME="${MODEL_NAME:-teleh4z_zaxis_0}"
WORLD_NAME="${WORLD_NAME:-playground}"
TIMESTAMP="${TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_CSV_PATH="${OUTPUT_CSV_PATH:-/home/user/external/experiments/zaxis/src/teleh4z_z_axis_cross_domain_px4_${TIMESTAMP}.csv}"
TRIAL_ID="${TRIAL_ID:-0}"
CONDITION="${CONDITION:-px4_offboard}"
START_DELAY_S="${START_DELAY_S:-0.0}"
OFFBOARD_ARM_DELAY_S="${OFFBOARD_ARM_DELAY_S:-0.1}"
PREARM_SETTLE_S="${PREARM_SETTLE_S:-2.0}"
OFFBOARD_CONFIRM_SETTLE_S="${OFFBOARD_CONFIRM_SETTLE_S:-0.5}"
REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM="${REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM:-true}"
PREFLIGHT_READY_TIMEOUT_S="${PREFLIGHT_READY_TIMEOUT_S:-30.0}"
REQUIRE_PROP_DIAG_BEFORE_ARM="${REQUIRE_PROP_DIAG_BEFORE_ARM:-true}"
PROP_DIAG_READY_TIMEOUT_S="${PROP_DIAG_READY_TIMEOUT_S:-12.0}"
WATER_DEPTH_M="${WATER_DEPTH_M:-1.0}"
AIR_HEIGHT_M="${AIR_HEIGHT_M:-0.7}"
UNDERWATER_HOVER_S="${UNDERWATER_HOVER_S:-5.0}"
UNDERWATER_ASCENT_S="${UNDERWATER_ASCENT_S:-5.0}"
UNDERWATER_ASCENT_TARGET_Z_M="${UNDERWATER_ASCENT_TARGET_Z_M:--0.10}"
SURFACE_EXIT_S="${SURFACE_EXIT_S:-5.0}"
SURFACE_EXIT_GUARD_ENABLED="${SURFACE_EXIT_GUARD_ENABLED:-true}"
SURFACE_EXIT_GUARD_HEIGHT_M="${SURFACE_EXIT_GUARD_HEIGHT_M:-0.40}"
SURFACE_EXIT_GUARD_TIMEOUT_S="${SURFACE_EXIT_GUARD_TIMEOUT_S:-0.0}"
SURFACE_EXIT_GUARD_THRUST_FRACTION="${SURFACE_EXIT_GUARD_THRUST_FRACTION:-0.8}"
AIR_HOVER_THRUST_N="${AIR_HOVER_THRUST_N:-16.7}"
MODEL_START_Z="${MODEL_START_Z:--$WATER_DEPTH_M}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4026}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-teleh4z_zaxis}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

banner() {
    echo -e "${GREEN}"
    echo '  ╔════════════════════════════════════════════╗'
    echo '  ║   Teleh4Z Z 轴跨域 PX4/Gazebo 实验启动      ║'
    echo '  ╚════════════════════════════════════════════╝'
    echo -e "${NC}"
}

find_terminal() {
    if command -v gnome-terminal &>/dev/null; then
        echo "gnome-terminal"
    elif command -v terminator &>/dev/null; then
        echo "terminator"
    elif command -v xterm &>/dev/null; then
        echo "xterm"
    elif command -v konsole &>/dev/null; then
        echo "konsole"
    else
        echo ""
    fi
}

launch_terminal() {
    local title="$1"
    local command="$2"
    if [[ "$TERM_CMD" == "gnome-terminal" ]]; then
        gnome-terminal --title="$title" -- bash -c "$command" &
    elif [[ "$TERM_CMD" == "terminator" ]]; then
        terminator -T "$title" -e "$command" &
    elif [[ "$TERM_CMD" == "xterm" ]]; then
        xterm -T "$title" -e "$command" &
    elif [[ "$TERM_CMD" == "konsole" ]]; then
        konsole --new-tab --title "$title" -e bash -lc "$command" &
    fi
}

banner
xhost +local:docker > /dev/null 2>&1 || true

echo -e "${YELLOW}[1/6] 检查并准备容器...${NC}"
if ! docker ps -a --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo -e "  ${RED}✗ 容器 $CONTAINER 不存在。请先按环境搭建指南创建容器。${NC}"
    exit 1
fi

if [[ "$RESTART_CONTAINER" == "1" ]]; then
    echo -e "  ${YELLOW}⏳${NC} 重启容器 ${CYAN}$CONTAINER${NC}，清理上一轮残留..."
    docker restart "$CONTAINER" >/dev/null
    sleep 2
    echo -e "  ${GREEN}✓${NC} 容器已重启"
elif docker ps --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo -e "  ${GREEN}✓${NC} 容器已在运行"
else
    docker start "$CONTAINER" >/dev/null
    sleep 2
    echo -e "  ${GREEN}✓${NC} 容器已启动"
fi

echo -e "${YELLOW}[2/6] 更新资源、插件和 ROS 2 package...${NC}"
if [[ "${SKIP_PLUGIN_BUILD:-0}" == "1" ]]; then
    docker exec "$CONTAINER" bash -lc '
        set -e
        mkdir -p /home/user/external/experiments/zaxis/src
        /home/user/sync_external_into_px4.sh
        mkdir -p /home/user/ros2_ws/src/teleh4z_manager
        cp -a /home/user/external/ros2_packages/teleh4z_manager/. /home/user/ros2_ws/src/teleh4z_manager/
        cd /home/user/ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --packages-select teleh4z_manager --symlink-install
    '
else
    docker exec "$CONTAINER" bash -lc '
        set -e
        mkdir -p /home/user/external/experiments/zaxis/src
        cd /home/user/external/plugins
        ./build_plugin.sh
        /home/user/sync_external_into_px4.sh
        mkdir -p /home/user/ros2_ws/src/teleh4z_manager
        cp -a /home/user/external/ros2_packages/teleh4z_manager/. /home/user/ros2_ws/src/teleh4z_manager/
        cd /home/user/ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --packages-select teleh4z_manager --symlink-install
    '
fi
echo -e "  ${GREEN}✓${NC} 资源同步和 ROS 2 构建完成"

echo -e "${YELLOW}[3/6] 清理旧 Gazebo / PX4 / ROS 2 进程...${NC}"
docker exec "$CONTAINER" bash -lc '
    pkill -x parameter_bridge 2>/dev/null || true
    pkill -x z_axis_cross_domain_runner 2>/dev/null || true
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    pkill -f "[g]z sim" 2>/dev/null || true
    pkill -f "[s]imulation-gazebo" 2>/dev/null || true
    sleep 1
'
echo -e "  ${GREEN}✓${NC} 清理完成"

echo -e "${YELLOW}[4/6] 生成容器内启动脚本...${NC}"
HOST_TMP="/tmp/teleh4z-zaxis-scripts"
rm -rf "$HOST_TMP" && mkdir -p "$HOST_TMP"

cat > "$HOST_TMP/start-px4-zaxis.sh" << 'EOFSCRIPT'
#!/bin/bash
set -e

WORLD_NAME="${WORLD_NAME:-playground}"
MODEL_NAME="${MODEL_NAME:-teleh4z_zaxis_0}"
MODEL_START_Z="${MODEL_START_Z:--1.0}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-4026}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-teleh4z_zaxis}"

cleanup() {
    echo ""
    echo "Stopping PX4 Z-axis experiment..."
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    pkill -f "[g]z sim" 2>/dev/null || true
    pkill -f "[s]imulation-gazebo" 2>/dev/null || true
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "=== PX4-controlled Gazebo Z-axis experiment ==="
echo "World: $WORLD_NAME"
echo "PX4 model: $PX4_SIM_MODEL"
echo "Gazebo model instance: $MODEL_NAME"
echo "Spawn world z: $MODEL_START_Z"

MicroXRCEAgent udp4 -p 8888 > /tmp/microxrce_zaxis.log 2>&1 &
echo "Micro XRCE-DDS Agent started."

python3 "$PX4_ROOT/Tools/simulation/gz/simulation-gazebo" \
    --model_store "$PX4_ROOT/Tools/simulation/gz" \
    --world "$WORLD_NAME" \
    --render_engine ogre2 > /tmp/gazebo_zaxis.log 2>&1 &
GAZEBO_PID=$!

echo "Waiting for Gazebo world..."
for i in $(seq 1 45); do
    if gz topic -l 2>/dev/null | grep -q "/world/$WORLD_NAME/clock"; then
        echo "Gazebo world is ready (${i}s)."
        break
    fi
    sleep 1
    if [[ "$i" -eq 45 ]]; then
        echo "Gazebo did not become ready in time. See /tmp/gazebo_zaxis.log"
        wait "$GAZEBO_PID"
    fi
done

cd "$PX4_ROOT"
PX4_GZ_STANDALONE=1 \
PX4_GZ_WORLD="$WORLD_NAME" \
PX4_SYS_AUTOSTART="$PX4_SYS_AUTOSTART" \
PX4_SIM_MODEL="$PX4_SIM_MODEL" \
PX4_GZ_MODEL_POSE="0,0,$MODEL_START_Z,0,0,0" \
./build/px4_sitl_default/bin/px4 \
    -d "./build/px4_sitl_default/etc" \
    -s "etc/init.d-posix/rcS" \
    -i 0

cleanup
EOFSCRIPT

cat > "$HOST_TMP/start-runner-zaxis.sh" << 'EOFSCRIPT'
#!/bin/bash
set -e

MODEL_NAME="${MODEL_NAME:-teleh4z_zaxis_0}"
TIMESTAMP="${TIMESTAMP:-$(date +%Y%m%d_%H%M%S)}"
OUTPUT_CSV_PATH="${OUTPUT_CSV_PATH:-/home/user/external/experiments/zaxis/src/teleh4z_z_axis_cross_domain_px4_${TIMESTAMP}.csv}"
TRIAL_ID="${TRIAL_ID:-0}"
CONDITION="${CONDITION:-px4_offboard}"
START_DELAY_S="${START_DELAY_S:-0.0}"
OFFBOARD_ARM_DELAY_S="${OFFBOARD_ARM_DELAY_S:-0.1}"
PREARM_SETTLE_S="${PREARM_SETTLE_S:-2.0}"
OFFBOARD_CONFIRM_SETTLE_S="${OFFBOARD_CONFIRM_SETTLE_S:-0.5}"
REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM="${REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM:-true}"
PREFLIGHT_READY_TIMEOUT_S="${PREFLIGHT_READY_TIMEOUT_S:-30.0}"
REQUIRE_PROP_DIAG_BEFORE_ARM="${REQUIRE_PROP_DIAG_BEFORE_ARM:-true}"
PROP_DIAG_READY_TIMEOUT_S="${PROP_DIAG_READY_TIMEOUT_S:-12.0}"
WATER_DEPTH_M="${WATER_DEPTH_M:-1.0}"
AIR_HEIGHT_M="${AIR_HEIGHT_M:-0.7}"
UNDERWATER_HOVER_S="${UNDERWATER_HOVER_S:-5.0}"
UNDERWATER_ASCENT_S="${UNDERWATER_ASCENT_S:-5.0}"
UNDERWATER_ASCENT_TARGET_Z_M="${UNDERWATER_ASCENT_TARGET_Z_M:--0.10}"
SURFACE_EXIT_S="${SURFACE_EXIT_S:-5.0}"
SURFACE_EXIT_GUARD_ENABLED="${SURFACE_EXIT_GUARD_ENABLED:-true}"
SURFACE_EXIT_GUARD_HEIGHT_M="${SURFACE_EXIT_GUARD_HEIGHT_M:-0.40}"
SURFACE_EXIT_GUARD_TIMEOUT_S="${SURFACE_EXIT_GUARD_TIMEOUT_S:-0.0}"
SURFACE_EXIT_GUARD_THRUST_FRACTION="${SURFACE_EXIT_GUARD_THRUST_FRACTION:-0.8}"
AIR_HOVER_THRUST_N="${AIR_HOVER_THRUST_N:-16.7}"

echo "=== ROS 2 PX4 Z-axis offboard runner ==="
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash

echo "Waiting for PX4 and Gazebo model topics..."
for i in $(seq 1 90); do
    if gz topic -l 2>/dev/null | grep -q "/model/$MODEL_NAME/odometry"; then
        if ros2 topic list 2>/dev/null | grep -q "/fmu/out/vehicle_local_position"; then
            echo "PX4 and model topics are ready (${i}s)."
            break
        fi
    fi
    sleep 1
    if [[ "$i" -eq 90 ]]; then
        echo "PX4/model topics were not found. Check PX4 and Gazebo startup."
        exec bash
    fi
done

cd /home/user/ros2_ws

ros2 launch teleh4z_manager z_axis_cross_domain.launch.py \
    model_name:="$MODEL_NAME" \
    output_csv_path:="$OUTPUT_CSV_PATH" \
    trial_id:="$TRIAL_ID" \
    condition:="$CONDITION" \
    start_delay_s:="$START_DELAY_S" \
    offboard_arm_delay_s:="$OFFBOARD_ARM_DELAY_S" \
    prearm_settle_s:="$PREARM_SETTLE_S" \
    offboard_confirm_settle_s:="$OFFBOARD_CONFIRM_SETTLE_S" \
    require_preflight_checks_before_arm:="$REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM" \
    preflight_ready_timeout_s:="$PREFLIGHT_READY_TIMEOUT_S" \
    require_prop_diag_before_arm:="$REQUIRE_PROP_DIAG_BEFORE_ARM" \
    prop_diag_ready_timeout_s:="$PROP_DIAG_READY_TIMEOUT_S" \
    water_depth_m:="$WATER_DEPTH_M" \
    air_height_m:="$AIR_HEIGHT_M" \
    underwater_hover_s:="$UNDERWATER_HOVER_S" \
    underwater_ascent_s:="$UNDERWATER_ASCENT_S" \
    underwater_ascent_target_z_m:="$UNDERWATER_ASCENT_TARGET_Z_M" \
    surface_exit_s:="$SURFACE_EXIT_S" \
    surface_exit_guard_enabled:="$SURFACE_EXIT_GUARD_ENABLED" \
    surface_exit_guard_height_m:="$SURFACE_EXIT_GUARD_HEIGHT_M" \
    surface_exit_guard_timeout_s:="$SURFACE_EXIT_GUARD_TIMEOUT_S" \
    surface_exit_guard_thrust_fraction:="$SURFACE_EXIT_GUARD_THRUST_FRACTION" \
    air_hover_thrust_N:="$AIR_HOVER_THRUST_N"
exec bash
EOFSCRIPT

chmod +x "$HOST_TMP"/*.sh
docker cp "$HOST_TMP/." "$CONTAINER:/tmp/teleh4z-zaxis-scripts/"
echo -e "  ${GREEN}✓${NC} 启动脚本已写入容器"

echo -e "${YELLOW}[5/6] 启动终端...${NC}"
TERM_CMD="$(find_terminal)"
if [[ -z "$TERM_CMD" ]]; then
    echo -e "${RED}✗ 未找到 gnome-terminal/terminator/xterm/konsole。${NC}"
    echo -e "${YELLOW}请手动运行以下两个命令：${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER env WORLD_NAME=$WORLD_NAME MODEL_NAME=$MODEL_NAME MODEL_START_Z=$MODEL_START_Z PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL bash /tmp/teleh4z-zaxis-scripts/start-px4-zaxis.sh${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER env MODEL_NAME=$MODEL_NAME OUTPUT_CSV_PATH=$OUTPUT_CSV_PATH TRIAL_ID=$TRIAL_ID CONDITION=$CONDITION START_DELAY_S=$START_DELAY_S OFFBOARD_ARM_DELAY_S=$OFFBOARD_ARM_DELAY_S PREARM_SETTLE_S=$PREARM_SETTLE_S OFFBOARD_CONFIRM_SETTLE_S=$OFFBOARD_CONFIRM_SETTLE_S REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM=$REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM PREFLIGHT_READY_TIMEOUT_S=$PREFLIGHT_READY_TIMEOUT_S REQUIRE_PROP_DIAG_BEFORE_ARM=$REQUIRE_PROP_DIAG_BEFORE_ARM PROP_DIAG_READY_TIMEOUT_S=$PROP_DIAG_READY_TIMEOUT_S WATER_DEPTH_M=$WATER_DEPTH_M AIR_HEIGHT_M=$AIR_HEIGHT_M UNDERWATER_HOVER_S=$UNDERWATER_HOVER_S UNDERWATER_ASCENT_S=$UNDERWATER_ASCENT_S UNDERWATER_ASCENT_TARGET_Z_M=$UNDERWATER_ASCENT_TARGET_Z_M SURFACE_EXIT_S=$SURFACE_EXIT_S SURFACE_EXIT_GUARD_ENABLED=$SURFACE_EXIT_GUARD_ENABLED SURFACE_EXIT_GUARD_HEIGHT_M=$SURFACE_EXIT_GUARD_HEIGHT_M SURFACE_EXIT_GUARD_TIMEOUT_S=$SURFACE_EXIT_GUARD_TIMEOUT_S SURFACE_EXIT_GUARD_THRUST_FRACTION=$SURFACE_EXIT_GUARD_THRUST_FRACTION AIR_HOVER_THRUST_N=$AIR_HOVER_THRUST_N bash /tmp/teleh4z-zaxis-scripts/start-runner-zaxis.sh${NC}"
    exit 1
fi

echo -e "  使用终端: ${CYAN}$TERM_CMD${NC}"
launch_terminal "Teleh4Z Z-axis PX4" \
    "docker exec -it $CONTAINER env WORLD_NAME=$WORLD_NAME MODEL_NAME=$MODEL_NAME MODEL_START_Z=$MODEL_START_Z PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL bash /tmp/teleh4z-zaxis-scripts/start-px4-zaxis.sh"
sleep 1
launch_terminal "Teleh4Z Z-axis PX4 Runner" \
    "docker exec -it $CONTAINER env MODEL_NAME=$MODEL_NAME OUTPUT_CSV_PATH=$OUTPUT_CSV_PATH TRIAL_ID=$TRIAL_ID CONDITION=$CONDITION START_DELAY_S=$START_DELAY_S OFFBOARD_ARM_DELAY_S=$OFFBOARD_ARM_DELAY_S PREARM_SETTLE_S=$PREARM_SETTLE_S OFFBOARD_CONFIRM_SETTLE_S=$OFFBOARD_CONFIRM_SETTLE_S REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM=$REQUIRE_PREFLIGHT_CHECKS_BEFORE_ARM PREFLIGHT_READY_TIMEOUT_S=$PREFLIGHT_READY_TIMEOUT_S REQUIRE_PROP_DIAG_BEFORE_ARM=$REQUIRE_PROP_DIAG_BEFORE_ARM PROP_DIAG_READY_TIMEOUT_S=$PROP_DIAG_READY_TIMEOUT_S WATER_DEPTH_M=$WATER_DEPTH_M AIR_HEIGHT_M=$AIR_HEIGHT_M UNDERWATER_HOVER_S=$UNDERWATER_HOVER_S UNDERWATER_ASCENT_S=$UNDERWATER_ASCENT_S UNDERWATER_ASCENT_TARGET_Z_M=$UNDERWATER_ASCENT_TARGET_Z_M SURFACE_EXIT_S=$SURFACE_EXIT_S SURFACE_EXIT_GUARD_ENABLED=$SURFACE_EXIT_GUARD_ENABLED SURFACE_EXIT_GUARD_HEIGHT_M=$SURFACE_EXIT_GUARD_HEIGHT_M SURFACE_EXIT_GUARD_TIMEOUT_S=$SURFACE_EXIT_GUARD_TIMEOUT_S SURFACE_EXIT_GUARD_THRUST_FRACTION=$SURFACE_EXIT_GUARD_THRUST_FRACTION AIR_HOVER_THRUST_N=$AIR_HOVER_THRUST_N bash /tmp/teleh4z-zaxis-scripts/start-runner-zaxis.sh"

echo -e "${YELLOW}[6/6] 已启动。${NC}"
echo -e "  控制链路: ${CYAN}PX4 offboard setpoint -> PX4 controller -> Gazebo command/motor_speed -> HybridAirPropellerModel${NC}"
echo -e "  CSV 输出: ${CYAN}$OUTPUT_CSV_PATH${NC}"
echo -e "  Gazebo 日志: ${CYAN}/tmp/gazebo_zaxis.log${NC}（容器内）"
echo -e "  XRCE 日志: ${CYAN}/tmp/microxrce_zaxis.log${NC}（容器内）"
