#!/usr/bin/env bash
# 语音识别节点启动脚本（电脑端，配合 start_nomap.sh 一起使用）
# 用法: ./start_audio_input.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$SCRIPT_DIR/mircoros_v2"

WHISPER_BIN="${WHISPER_BIN:-$HOME/tools/whisper.cpp/build/bin/whisper-cli}"
WHISPER_MODEL="${WHISPER_MODEL:-$HOME/tools/whisper.cpp/models/ggml-small.bin}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-40}"

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export ROS_DOMAIN_ID

exec python3 "$SCRIPT_DIR/audio_input_node.py" \
    --ros-args \
    -p whisper_bin:="$WHISPER_BIN" \
    -p model_path:="$WHISPER_MODEL" \
    -p language:=zh \
    -p record_seconds:=5.0
