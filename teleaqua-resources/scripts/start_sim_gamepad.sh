#!/bin/bash
# =============================================================================
# Teleh4Z Gazebo + QGroundControl + gamepad control one-key launcher
#
# This script keeps the same integrated simulation-terminal style as start_sim.sh:
#   - Terminal A: Micro XRCE-DDS Agent + Gazebo + PX4 SITL
#   - Terminal B: container shell running `ros2 launch teleh4z_manager mode_manager.launch.py`
#   - Host: QGroundControl-x86_64.AppImage
#
# Usage:
#   ./start_sim_gamepad.sh
#   SKIP_PLUGIN_BUILD=1 ./start_sim_gamepad.sh
#   RESTART_CONTAINER=0 ./start_sim_gamepad.sh
#
# Optional:
#   QGC_DIR=/path/to/QGroundControlQGroundControl ./start_sim_gamepad.sh
#   QGC_APPIMAGE=/path/to/QGroundControl-x86_64.AppImage ./start_sim_gamepad.sh
# =============================================================================

set -e

CONTAINER="${CONTAINER:-teleh4z-sim}"
RESTART_CONTAINER="${RESTART_CONTAINER:-1}"
QGC_DIR="${QGC_DIR:-$HOME/QGroundControlQGroundControl}"
QGC_APPIMAGE="${QGC_APPIMAGE:-$QGC_DIR/QGroundControl-x86_64.AppImage}"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

banner() {
    echo -e "${GREEN}"
    echo '  ╔════════════════════════════════════════════╗'
    echo '  ║   Teleh4Z 仿真 + QGC 手柄控制一键启动       ║'
    echo '  ╚════════════════════════════════════════════╝'
    echo -e "${NC}"
}

launch_qgc() {
    if [[ ! -f "$QGC_APPIMAGE" ]]; then
        echo -e "  ${YELLOW}!${NC} 未找到 QGroundControl AppImage，跳过自动启动 QGC"
        echo -e "    期望路径: ${CYAN}$QGC_APPIMAGE${NC}"
        echo -e "    可用 ${CYAN}QGC_APPIMAGE=/path/to/QGroundControl-x86_64.AppImage ./start_sim_gamepad.sh${NC} 指定"
        return
    fi

    if [[ ! -x "$QGC_APPIMAGE" ]]; then
        chmod +x "$QGC_APPIMAGE" 2>/dev/null || true
    fi

    echo -e "  ${GREEN}✓${NC} 在宿主机启动 QGroundControl"
    echo -e "    ${CYAN}cd \"$QGC_DIR\" && QT_QPA_PLATFORM=xcb ./$(basename "$QGC_APPIMAGE")${NC}"
    (cd "$QGC_DIR" && QT_QPA_PLATFORM=xcb "./$(basename "$QGC_APPIMAGE")" >/tmp/teleh4z-qgc.log 2>&1 &)
}

banner

xhost +local:docker > /dev/null 2>&1 || true

echo -e "${YELLOW}[1/7] 检查并准备容器...${NC}"
if ! docker ps -a --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo -e "  ${RED}✗ 容器 $CONTAINER 不存在！${NC}"
    echo -e "  ${RED}请先按环境搭建指南创建容器${NC}"
    exit 1
fi

if [[ "$RESTART_CONTAINER" == "1" ]]; then
    echo -e "  ${YELLOW}⏳${NC} 正在重启容器 ${CYAN}$CONTAINER${NC}，清理上一轮仿真残留..."
    docker restart "$CONTAINER" >/dev/null
    sleep 2
    echo -e "  ${GREEN}✓${NC} 容器已重启"
elif docker ps --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo -e "  ${GREEN}✓${NC} 容器 ${CYAN}$CONTAINER${NC} 已在运行 (RESTART_CONTAINER=0)"
else
    echo -e "  ${YELLOW}⏳${NC} 容器已停止，正在启动..."
    docker start "$CONTAINER" >/dev/null
    sleep 2
    echo -e "  ${GREEN}✓${NC} 容器已启动"
fi

if [[ "${SKIP_PLUGIN_BUILD:-0}" == "1" ]]; then
    echo -e "${YELLOW}[2/7] 跳过插件编译、资源同步与 ROS2 构建 (SKIP_PLUGIN_BUILD=1)${NC}"
else
    echo -e "${YELLOW}[2/7] 编译插件、同步资源并构建 ROS2 控制包...${NC}"
    docker exec "$CONTAINER" bash -lc '
        set -e
        cd /home/user/external/plugins
        ./build_plugin.sh
        /home/user/sync_external_into_px4.sh
        cd /home/user/ros2_ws
        source /opt/ros/humble/setup.bash
        colcon build --packages-select teleh4z_manager --symlink-install
    '
    echo -e "  ${GREEN}✓${NC} 插件、资源与 ROS2 控制包已更新"
fi

echo -e "${YELLOW}[3/7] 清理旧仿真进程...${NC}"
docker exec "$CONTAINER" bash -lc '
    pkill -x mode_manager 2>/dev/null || true
    pkill -x joystick_mapper 2>/dev/null || true
    pkill -x parameter_bridge 2>/dev/null || true
    pkill -x water_thruster_ 2>/dev/null || true
    pkill -x manual_water_thruster_mapper 2>/dev/null || true
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    sleep 1
'
echo -e "  ${GREEN}✓${NC} 旧 ROS2/Agent/Gazebo/PX4 进程与 FastDDS SHM 残留已清理"

echo -e "${YELLOW}[4/7] 生成容器内启动脚本...${NC}"

HOST_TMP="/tmp/teleh4z-gamepad-scripts"
rm -rf "$HOST_TMP" && mkdir -p "$HOST_TMP"

cat > "$HOST_TMP/start-all.sh" << 'EOFSCRIPT'
#!/bin/bash
# ============================================================
# Teleh4Z integrated simulation launcher:
# Agent -> Gazebo -> wait ready -> PX4
# Ctrl+C stops all simulation processes.
# ============================================================

cleanup() {
    echo ""
    echo -e "\033[33m正在停止所有仿真进程...\033[0m"
    pkill -x mode_manager 2>/dev/null || true
    pkill -x joystick_mapper 2>/dev/null || true
    pkill -x parameter_bridge 2>/dev/null || true
    pkill -x water_thruster_ 2>/dev/null || true
    pkill -x manual_water_thruster_mapper 2>/dev/null || true
    pkill -x MicroXRCEAgent 2>/dev/null || true
    pkill -x px4 2>/dev/null || true
    sleep 1
    echo -e "\033[32m已全部停止\033[0m"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo -e "\033[1;33m[0/4]\033[0m 清理旧仿真进程..."
pkill -x mode_manager 2>/dev/null || true
pkill -x joystick_mapper 2>/dev/null || true
pkill -x parameter_bridge 2>/dev/null || true
pkill -x water_thruster_ 2>/dev/null || true
pkill -x manual_water_thruster_mapper 2>/dev/null || true
pkill -x MicroXRCEAgent 2>/dev/null || true
pkill -x px4 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 1

echo -e "\033[0;32m╔══════════════════════════════════════╗\033[0m"
echo -e "\033[0;32m║   Teleh4Z 仿真环境启动中...          ║\033[0m"
echo -e "\033[0;32m╚══════════════════════════════════════╝\033[0m"
echo ""

echo -e "\033[1;33m[1/4]\033[0m 启动 Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
sleep 1
echo -e "  \033[32m✓\033[0m Agent 已启动 (UDP :8888)"

echo -e "\033[1;33m[2/4]\033[0m 启动 Gazebo Harmonic..."
python3 "$PX4_ROOT/Tools/simulation/gz/simulation-gazebo" \
    --model_store "$PX4_ROOT/Tools/simulation/gz" \
    --world playground \
    --render_engine ogre2 > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo -e "  \033[32m✓\033[0m Gazebo 已启动 (PID $GAZEBO_PID)"
echo "  等待 GUI 窗口出现..."
sleep 5

echo -e "\033[1;33m[3/4]\033[0m 等待 Gazebo 世界加载..."
WAITED=0
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/playground/clock"; then
        echo -e "  \033[32m✓\033[0m Gazebo 已就绪 (${i}s)"
        break
    fi
    sleep 1
    WAITED=$i
done
if [ "$WAITED" -eq 30 ]; then
    echo -e "  \033[31m✗\033[0m Gazebo 超时未就绪, 仍尝试启动 PX4"
fi

echo -e "\033[1;33m[4/4]\033[0m 启动 PX4 SITL (Teleh4Z, airframe 4026)..."
cd "$PX4_ROOT"
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4026 \
PX4_SIM_MODEL=teleh4z \
PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0" \
./build/px4_sitl_default/bin/px4 \
    -d "./build/px4_sitl_default/etc" \
    -s "etc/init.d-posix/rcS" \
    -i 0 &
PX4_PID=$!
sleep 3

echo ""
echo -e "\033[0;32m╔══════════════════════════════════════╗\033[0m"
echo -e "\033[0;32m║  仿真已启动                          ║\033[0m"
echo -e "\033[0;32m╠══════════════════════════════════════╣\033[0m"
echo -e "\033[0;32m║  按 Ctrl+C 停止仿真                  ║\033[0m"
echo -e "\033[0;32m║  手柄控制节点在另一个容器终端中运行   ║\033[0m"
echo -e "\033[0;32m╚══════════════════════════════════════╝\033[0m"
echo ""
echo "===== PX4 运行输出 ====="
echo ""

wait "$PX4_PID" 2>/dev/null
cleanup
EOFSCRIPT

cat > "$HOST_TMP/term5-gamepad.sh" << 'EOFSCRIPT'
#!/bin/bash
echo "=== 终端 5: ROS 2 手柄控制节点 ==="
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
echo "正在启动 mode_manager.launch.py..."
echo "该 launch 默认启动 mode_manager、joystick_mapper、水推旁路 mapper 和 Gazebo bridge。"
ros2 launch teleh4z_manager mode_manager.launch.py
exec bash
EOFSCRIPT

chmod +x "$HOST_TMP"/start-all.sh "$HOST_TMP"/term5-gamepad.sh
docker cp "$HOST_TMP/." "$CONTAINER:/tmp/teleh4z-gamepad-scripts/"
echo -e "  ${GREEN}✓${NC} 脚本已传入容器"

echo -e "${YELLOW}[5/7] 启动仿真终端和手柄节点终端...${NC}"

TERM_CMD=""
if command -v gnome-terminal &>/dev/null; then
    TERM_CMD="gnome-terminal"
elif command -v terminator &>/dev/null; then
    TERM_CMD="terminator"
elif command -v xterm &>/dev/null; then
    TERM_CMD="xterm"
elif command -v konsole &>/dev/null; then
    TERM_CMD="konsole"
fi

if [[ -z "$TERM_CMD" ]]; then
    echo -e "${RED}✗ 未找到支持的终端模拟器${NC}"
    echo -e "${YELLOW}请手动打开 2 个终端执行:${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/start-all.sh${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/term5-gamepad.sh${NC}"
    exit 1
fi

echo -e "  使用终端: ${CYAN}$TERM_CMD${NC}"

if [[ "$TERM_CMD" == "gnome-terminal" ]]; then
    gnome-terminal --title="Teleh4Z 仿真" -- bash -c "docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/start-all.sh" &
    sleep 1
    gnome-terminal --title="Teleh4Z 手柄控制节点" -- bash -c "docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/term5-gamepad.sh" &
elif [[ "$TERM_CMD" == "terminator" ]]; then
    terminator -T "Teleh4Z 仿真" -e "docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/start-all.sh" &
    sleep 1
    terminator -T "Teleh4Z 手柄控制节点" -e "docker exec -it $CONTAINER bash /tmp/teleh4z-gamepad-scripts/term5-gamepad.sh" &
elif [[ "$TERM_CMD" == "xterm" ]]; then
    xterm -T "Teleh4Z 仿真" -e docker exec -it "$CONTAINER" bash /tmp/teleh4z-gamepad-scripts/start-all.sh &
    sleep 1
    xterm -T "Teleh4Z 手柄控制节点" -e docker exec -it "$CONTAINER" bash /tmp/teleh4z-gamepad-scripts/term5-gamepad.sh &
elif [[ "$TERM_CMD" == "konsole" ]]; then
    konsole --new-tab -p tabtitle="Teleh4Z 仿真" -e docker exec -it "$CONTAINER" bash /tmp/teleh4z-gamepad-scripts/start-all.sh &
    sleep 1
    konsole --new-tab -p tabtitle="Teleh4Z 手柄控制节点" -e docker exec -it "$CONTAINER" bash /tmp/teleh4z-gamepad-scripts/term5-gamepad.sh &
fi

sleep 2

echo -e "${YELLOW}[6/7] 启动宿主机 QGroundControl...${NC}"
launch_qgc

echo -e "${YELLOW}[7/7] 启动完成${NC}"
echo ""
echo -e "${GREEN}  ╔════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}  ║  Teleh4Z + QGC 手柄控制已启动              ║${NC}"
echo -e "${GREEN}  ╠════════════════════════════════════════════╣${NC}"
echo -e "${GREEN}  ║  终端 A: Agent + Gazebo + PX4 集成启动     ║${NC}"
echo -e "${GREEN}  ║  终端 B: 容器内 mode_manager.launch.py    ║${NC}"
echo -e "${GREEN}  ║  宿主机: QGroundControl                   ║${NC}"
echo -e "${GREEN}  ╚════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}使用提示:${NC}"
echo -e "  • QGC 路径默认: ${CYAN}$QGC_APPIMAGE${NC}"
echo -e "  • QGC 连接 PX4 后，在 Joystick 页面选择并校准手柄"
echo -e "  • 手柄节点由容器中的 ${CYAN}ros2 launch teleh4z_manager mode_manager.launch.py${NC} 启动"
