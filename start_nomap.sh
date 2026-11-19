#!/usr/bin/env bash
# 不建图模式一键启动：micro-ROS Agent + 主系统（相机/图传/rosbridge）+ 位姿桥接
# 用法:
#   ./start_nomap.sh            # 启动全部进程，Ctrl+C 统一退出
#   ./start_nomap.sh --check    # 只做环境自检，不启动进程
#   ./start_nomap.sh --stop     # 停止 Agent、主系统与位姿桥接
# 可用环境变量: ROS_DOMAIN_ID(默认40)、AGENT_WS、AGENT_PORT、LOG_DIR
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$ROOT_DIR/mircoros_v2"
AGENT_WS="${AGENT_WS:-$HOME/microros_agent_ws}"
DOMAIN_ID="${ROS_DOMAIN_ID:-40}"
AGENT_PORT="${AGENT_PORT:-8090}"
LOG_DIR="${LOG_DIR:-$HOME/.ros/nomap_logs}"
CFG_DIR="$HOME/microros_ws/.system_cfg"
CFG_FILE="$CFG_DIR/robot_info.json"
CACHE_DIR="$HOME/microros_ws/cache"

AGENT_PID=""
LAUNCH_PID=""
BRIDGE_PID=""

log() { printf '[nomap] %s\n' "$*"; }
die() { printf '[nomap] 错误: %s\n' "$*" >&2; exit 1; }

usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# launch 通过 GetCameraUrl() 读取该文件，缺失会直接抛 FileNotFoundError
ensure_config() {
    if [ ! -f "$CFG_FILE" ]; then
        local src="$WS_DIR/.system_cfg/robot_info.json"
        [ -f "$src" ] || die "缺少 $CFG_FILE，且没有可复制的 $src"
        mkdir -p "$CFG_DIR"
        cp "$src" "$CFG_FILE"
        log "已把项目配置复制到 $CFG_FILE"
    fi
    mkdir -p "$CACHE_DIR" "$LOG_DIR"
}

get_camera_ip() {
    python3 - "$CFG_FILE" <<'PY'
import json, sys
try:
    print(json.load(open(sys.argv[1], encoding='utf-8')).get('camera_ip', ''))
except Exception:
    print('')
PY
}

check_env() {
    local ok=0
    [ -f /opt/ros/jazzy/setup.bash ] || { log "缺少 /opt/ros/jazzy/setup.bash"; ok=1; }
    [ -f "$WS_DIR/install/setup.bash" ] || { log "缺少 $WS_DIR/install/setup.bash，请先 colcon build"; ok=1; }
    [ -f "$AGENT_WS/install/local_setup.bash" ] || { log "缺少 $AGENT_WS/install/local_setup.bash"; ok=1; }

    if [ -f "$CFG_FILE" ]; then
        log "配置文件: $CFG_FILE"
    else
        log "配置文件缺失，启动时会从项目 .system_cfg 复制"
    fi
    [ -d "$CACHE_DIR" ] || log "缓存目录缺失，启动时会创建 $CACHE_DIR"

    local camera_ip
    camera_ip="$(get_camera_ip 2>/dev/null)"
    if [ -z "$camera_ip" ] || [ "$camera_ip" = "None" ]; then
        log "警告: robot_info.json 里没有有效的 camera_ip，相机节点会连不上"
    else
        log "相机 IP: $camera_ip"
        if ping -c 1 -W 2 "$camera_ip" >/dev/null 2>&1; then
            log "相机连通性: 正常"
        else
            log "警告: ping 不通 $camera_ip，请确认热点和相机电源"
        fi
    fi

    if ss -lun 2>/dev/null | grep -q ":$AGENT_PORT "; then
        log "警告: UDP $AGENT_PORT 已被占用，Agent 可能无法启动"
    fi

    [ "$ok" -eq 0 ] && log "自检通过" || die "自检未通过"
}

# 子进程用 setsid 独立成组，Ctrl+C 只到脚本，由这里统一按进程组关闭
stop_tree() {
    local pid="${1:-}" i=0
    [ -n "$pid" ] || return 0
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    while [ "$i" -lt 20 ] && kill -0 "$pid" 2>/dev/null; do
        sleep 0.2
        i=$((i + 1))
    done
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
}

# 按可执行路径兜底清理，覆盖脱离进程组的 launch 子进程
stop_all() {
    pkill -f "$AGENT_WS/install/micro_ros_agent/lib/micro_ros_agent/" 2>/dev/null
    pkill -f "ros2 run micro_ros_agent" 2>/dev/null
    pkill -f "$WS_DIR/install/microros_v2_bringup/lib/" 2>/dev/null
    pkill -f "$WS_DIR/install/ros_robot_app/lib/ros_robot_app/" 2>/dev/null
    pkill -f "ros_robot_app/odom_pose_bridge.py" 2>/dev/null
    pkill -f "ros2 launch microros_v2_bringup" 2>/dev/null
    pkill -f "rosbridge_websocket_launch.xml" 2>/dev/null
    pkill -f "/opt/ros/jazzy/lib/rosbridge_server/rosbridge_websocket" 2>/dev/null
    pkill -f "/opt/ros/jazzy/lib/rosapi/rosapi_node" 2>/dev/null
}

cleanup() {
    log "正在停止所有进程..."
    stop_tree "$LAUNCH_PID"
    stop_tree "$BRIDGE_PID"
    stop_tree "$AGENT_PID"
    stop_all
    wait 2>/dev/null
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
    usage
    exit 0
fi

if [ "${1:-}" = "--check" ]; then
    check_env
    exit 0
fi

if [ "${1:-}" = "--stop" ]; then
    stop_all
    log "已停止 Agent、主系统与位姿桥接"
    exit 0
fi

ensure_config
check_env
mkdir -p "$LOG_DIR"

# ROS 的 setup.bash 引用了未定义变量，source 期间必须临时关闭 nounset
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$AGENT_WS/install/local_setup.bash"
# shellcheck disable=SC1091
source "$WS_DIR/install/setup.bash"
set -u
export ROS_DOMAIN_ID="$DOMAIN_ID"

# 防止重复启动
exec 9>"$LOG_DIR/.start_nomap.lock"
if command -v flock >/dev/null 2>&1; then
    flock -n 9 || die "已有 start_nomap 实例在运行"
fi

log "ROS_DOMAIN_ID=$DOMAIN_ID，日志目录 $LOG_DIR"

# 必须在启动子进程前挂上信号处理，否则启动过程中按 Ctrl+C 会留下孤儿进程并占住锁
trap 'cleanup; exit 0' INT TERM

setsid ros2 run micro_ros_agent micro_ros_agent udp4 --port "$AGENT_PORT" -v6 \
    >"$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
sleep 2
kill -0 "$AGENT_PID" 2>/dev/null || die "Agent 启动失败，见 $LOG_DIR/agent.log"

setsid ros2 launch microros_v2_bringup bringup_app.launch.py \
    slam_mode:=none navigation_mode:=False \
    >"$LOG_DIR/bringup.log" 2>&1 &
LAUNCH_PID=$!
sleep 3
if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "主系统启动失败，日志末尾："
    tail -n 20 "$LOG_DIR/bringup.log" >&2
    cleanup
    exit 1
fi

setsid python3 "$WS_DIR/src/ros_robot_app/ros_robot_app/odom_pose_bridge.py" \
    >"$LOG_DIR/odom_bridge.log" 2>&1 &
BRIDGE_PID=$!

log "Agent PID=$AGENT_PID，主系统 PID=$LAUNCH_PID，位姿桥接 PID=$BRIDGE_PID"
log "相机与压缩图传日志: $LOG_DIR/bringup.log"
log "按 Ctrl+C 停止全部进程"

while kill -0 "$AGENT_PID" 2>/dev/null \
    && kill -0 "$LAUNCH_PID" 2>/dev/null \
    && kill -0 "$BRIDGE_PID" 2>/dev/null; do
    sleep 2
done

log "有进程退出，统一关闭"
cleanup
