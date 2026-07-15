#!/bin/bash
# =============================================================================
# Teleh4Z 仿真环境一键启动脚本
# 自动检查容器状态，在 4 个终端标签页中分别启动各服务
#
# 终端分配：
#   终端 1 — Micro XRCE-DDS Agent (UDP 8888)
#   终端 2 — Gazebo Harmonic (playground world, ogre2 渲染)
#   终端 3 — PX4 SITL (Teleh4Z, airframe 4026)
#   终端 4 — ROS 2 模式管理器 (mode_manager)
#
# 用法: ./start_sim.sh
# =============================================================================

set -e

CONTAINER="teleh4z-sim"

# ---- 颜色 ----
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

banner() {
    echo -e "${GREEN}"
    echo '  ╔══════════════════════════════════════╗'
    echo '  ║   Teleh4Z 仿真环境一键启动              ║'
    echo '  ╚══════════════════════════════════════╝'
    echo -e "${NC}"
}

banner

# ---- 确保 X11 授权 ----
xhost +local:docker > /dev/null 2>&1

# =========================================================================
# 1. 确保容器在运行
# =========================================================================
echo -e "${YELLOW}[1/6] 检查容器状态...${NC}"

if docker ps --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "$CONTAINER"; then
    echo -e "  ${GREEN}✓${NC} 容器 ${CYAN}$CONTAINER${NC} 已在运行"
elif docker ps -a --filter "name=$CONTAINER" --format "{{.Names}}" 2>/dev/null | grep -q "$CONTAINER"; then
    echo -e "  ${YELLOW}⏳${NC} 容器已停止，正在启动..."
    docker start "$CONTAINER"
    sleep 2
    echo -e "  ${GREEN}✓${NC} 容器已启动"
else
    echo -e "  ${RED}✗ 容器 $CONTAINER 不存在！${NC}"
    echo -e "  ${RED}请先执行 docker run 创建容器（参考环境搭建指南第4节）${NC}"
    exit 1
fi

# =========================================================================
# 2. 启动前编译并同步插件 / 模型资源
# =========================================================================
if [[ "${SKIP_PLUGIN_BUILD:-0}" == "1" ]]; then
    echo -e "${YELLOW}[2/6] 跳过插件编译与资源同步 (SKIP_PLUGIN_BUILD=1)${NC}"
else
    echo -e "${YELLOW}[2/6] 编译插件并同步资源到 PX4...${NC}"
    docker exec "$CONTAINER" bash -lc '
        set -e
        cd /home/user/external/plugins
        ./build_plugin.sh
        /home/user/sync_external_into_px4.sh
    '
    echo -e "  ${GREEN}✓${NC} 插件编译与资源同步完成"
fi

# =========================================================================
# 3. 在 4 个 gnome-terminal 标签页中分别启动服务
# =========================================================================
echo -e "${YELLOW}[3/6] 启动 4 个终端标签页...${NC}"

# 解释一下引号嵌套：
#   gnome-terminal -- bash -c "docker exec -it ... bash -c '...'"
#
#   $CONTAINER  — 在脚本 shell 中展开（double quotes 内）
#   \$PX4_ROOT  — \$ 在 double quotes 内被处理为 $，传入 bash -c，
#                 再被 single quotes 保护原样传给容器内的 bash，最终在里面展开
#   exec bash   — 命令结束后保持终端不关闭

# ---- 在宿主机生成 4 个启动脚本，再用 docker cp 传入容器 ----
echo -e "${YELLOW}[3/6] 生成启动脚本...${NC}"

HOST_TMP="/tmp/teleh4z-sim-scripts"
rm -rf "$HOST_TMP" && mkdir -p "$HOST_TMP"

cat > "$HOST_TMP/term1-agent.sh" << 'EOFSCRIPT'
#!/bin/bash
echo "=== 终端 1: Micro XRCE-DDS Agent ==="
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
echo "XRCE Agent 已启动 (UDP :8888)"
exec bash
EOFSCRIPT

cat > "$HOST_TMP/term2-gazebo.sh" << 'EOFSCRIPT'
#!/bin/bash
echo "=== 终端 2: Gazebo Harmonic ==="
echo "正在启动 Gazebo (playground world, ogre2 渲染)..."
python3 $PX4_ROOT/Tools/simulation/gz/simulation-gazebo \
    --model_store $PX4_ROOT/Tools/simulation/gz \
    --world playground \
    --render_engine ogre2 &
sleep 3
echo "Gazebo 已启动，请等待 GUI 窗口出现 (约 10-20 秒)"
exec bash
EOFSCRIPT

cat > "$HOST_TMP/term3-px4.sh" << 'EOFSCRIPT'
#!/bin/bash
echo "=== 终端 3: PX4 SITL (Teleh4Z, airframe 4026) ==="
echo "等待 Gazebo 就绪..."
# 轮询 Gazebo world 主题，等待仿真世界加载完毕
for i in $(seq 1 30); do
    if gz topic -l 2>/dev/null | grep -q "/world/playground/clock"; then
        echo "Gazebo 已就绪 (耗时 ${i}s)"
        break
    fi
    sleep 1
done
cd $PX4_ROOT
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4026 \
PX4_SIM_MODEL=teleh4z \
PX4_GZ_MODEL_POSE="0,0,0.2,0,0,0" \
./build/px4_sitl_default/bin/px4 \
    -d "./build/px4_sitl_default/etc" \
    -s "etc/init.d-posix/rcS" \
    -i 0
exec bash
EOFSCRIPT

cat > "$HOST_TMP/start-all.sh" << 'EOFSCRIPT'
#!/bin/bash
# ============================================================
# Teleh4Z 一站式启动: Agent → Gazebo → 等待就绪 → PX4 → ROS2
# Ctrl+C 停止全部仿真
# ============================================================

cleanup() {
    echo ""
    echo -e "\033[33m正在停止所有仿真进程...\033[0m"
    pkill -f MicroXRCEAgent 2>/dev/null || true
    pkill -f "gz sim"       2>/dev/null || true
    pkill -f px4            2>/dev/null || true
    sleep 1
    echo -e "\033[32m已全部停止\033[0m"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo -e "\033[0;32m╔══════════════════════════════════════╗\033[0m"
echo -e "\033[0;32m║   Teleh4Z 仿真环境启动中...          ║\033[0m"
echo -e "\033[0;32m╚══════════════════════════════════════╝\033[0m"
echo ""

# ---- 1. Agent ----
echo -e "\033[1;33m[1/4]\033[0m 启动 Micro XRCE-DDS Agent..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
sleep 1
echo -e "  \033[32m✓\033[0m Agent 已启动 (UDP :8888)"

# ---- 2. Gazebo ----
echo -e "\033[1;33m[2/4]\033[0m 启动 Gazebo Harmonic..."
python3 $PX4_ROOT/Tools/simulation/gz/simulation-gazebo \
    --model_store $PX4_ROOT/Tools/simulation/gz \
    --world playground \
    --render_engine ogre2 > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo -e "  \033[32m✓\033[0m Gazebo 已启动 (PID $GAZEBO_PID)"
echo "  等待 GUI 窗口出现..."
sleep 5

# ---- 3. 等待 Gazebo 就绪 ----
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
if [ $WAITED -eq 30 ]; then
    echo -e "  \033[31m✗\033[0m Gazebo 超时未就绪, 仍尝试启动 PX4"
fi

# ---- 4. PX4 ----
echo -e "\033[1;33m[4/4]\033[0m 启动 PX4 SITL (Teleh4Z, airframe 4026)..."
cd $PX4_ROOT
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
echo -e "\033[0;32m║  仿真已全部启动                      ║\033[0m"
echo -e "\033[0;32m╠══════════════════════════════════════╣\033[0m"
echo -e "\033[0;32m║  按 Ctrl+C 停止全部仿真              ║\033[0m"
echo -e "\033[0;32m║  启动 ROS2 模式管理器:               ║\033[0m"
echo -e "\033[0;32m║    ros2 launch teleh4z_manager ...   ║\033[0m"
echo -e "\033[0;32m╚══════════════════════════════════════╝\033[0m"
echo ""
echo "===== PX4 运行输出 ====="
echo ""

# 等待 PX4（或用户 Ctrl+C）
wait $PX4_PID 2>/dev/null
cleanup
EOFSCRIPT

cat > "$HOST_TMP/term4-mode.sh" << 'EOFSCRIPT'
#!/bin/bash
echo "=== 终端 4: ROS 2 模式管理器 ==="
cd /home/user/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
echo "正在启动 mode_manager..."
ros2 launch teleh4z_manager mode_manager.launch.py
exec bash
EOFSCRIPT

chmod +x "$HOST_TMP"/term*.sh
docker cp "$HOST_TMP/." "$CONTAINER:/tmp/teleh4z-scripts/"
echo -e "  ${GREEN}✓${NC} 脚本已传入容器"

# ---- 选择终端模拟器并启动 ----
echo -e "${YELLOW}[4/6] 启动 4 个终端...${NC}"

TERM_CMD=""
# 优先级: gnome-terminal > terminator > xterm > konsole
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
    echo -e "${YELLOW}请手动打开 4 个终端，每个执行:${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term1-agent.sh${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term2-gazebo.sh${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term3-px4.sh${NC}"
    echo -e "  ${CYAN}docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term4-mode.sh${NC}"
    exit 1
fi

echo -e "  使用终端: ${CYAN}$TERM_CMD${NC}"

if [[ "$TERM_CMD" == "gnome-terminal" ]]; then
    # 单窗口方案：在一个终端内依次启动所有进程，
    # Agent/Gazebo 后台运行，PX4 前台（用户可直接看到 PX4 输出）
    # Ctrl+C 即可停止全部仿真
    gnome-terminal --title="Teleh4Z 仿真" -- bash -c "docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/start-all.sh" &
elif [[ "$TERM_CMD" == "terminator" ]]; then
    terminator -T "Teleh4Z 仿真" -e "docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/start-all.sh" &
elif [[ "$TERM_CMD" == "xterm" ]]; then
    xterm -T "1-Agent" -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term1-agent.sh &
    xterm -T "2-Gazebo" -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term2-gazebo.sh &
    xterm -T "3-PX4" -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term3-px4.sh &
    xterm -T "4-Mode" -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term4-mode.sh &
elif [[ "$TERM_CMD" == "konsole" ]]; then
    konsole --new-tab -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term1-agent.sh &
    sleep 1
    konsole --new-tab -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term2-gazebo.sh &
    sleep 1
    konsole --new-tab -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term3-px4.sh &
    sleep 1
    konsole --new-tab -e docker exec -it $CONTAINER bash /tmp/teleh4z-scripts/term4-mode.sh &
fi

sleep 2   # 等窗口创建完成

# =========================================================================
# 5. 完成提示
# =========================================================================
echo ""
echo -e "${GREEN}  ╔══════════════════════════════════════════╗${NC}"
echo -e "${GREEN}  ║  4 个终端已启动                        ║${NC}"
echo -e "${GREEN}  ╠══════════════════════════════════════════╣${NC}"
echo -e "${GREEN}  ║  终端 1: Micro XRCE-DDS Agent          ║${NC}"
echo -e "${GREEN}  ║  终端 2: Gazebo Harmonic               ║${NC}"
echo -e "${GREEN}  ║  终端 3: PX4 SITL (按 Enter 启动)     ║${NC}"
echo -e "${GREEN}  ║  终端 4: ROS 2 模式管理器              ║${NC}"
echo -e "${GREEN}  ╚══════════════════════════════════════════╝${NC}"
echo ""
echo -e "${YELLOW}提示:${NC}"
echo -e "  • 终端 2 出现 Gazebo 窗口后，切到终端 3 按 Enter 启动 PX4"
echo -e "  • 启动 QGroundControl (宿主机):"
echo -e "    ${CYAN}cd \$HOME/teleh4z-sim/external && QT_QPA_PLATFORM=xcb ./QGroundControl.AppImage${NC}"
echo -e "  • 额外终端进入容器:"
echo -e "    ${CYAN}docker exec -it $CONTAINER bash${NC}"
echo ""
echo -e "  ${GREEN}模式切换测试:${NC}"
echo -e "    ${CYAN}ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String \"{data: 'air'}\"${NC}"
echo -e "    ${CYAN}ros2 topic pub --once /teleh4z/mode_request std_msgs/msg/String \"{data: 'water'}\"${NC}"
