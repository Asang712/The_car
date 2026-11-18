#!/usr/bin/env bash
# 自动提交并推送当前项目到 GitHub
# 用法:
#   ./auto_push.sh                  # 检测一次，有改动就提交并推送
#   ./auto_push.sh --watch          # 常驻运行，按间隔轮询（默认 300 秒）
#   ./auto_push.sh --watch --interval 600
# 可通过环境变量覆盖: REMOTE / BRANCH / INTERVAL / LOG_FILE
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO_DIR" || exit 1

REMOTE="${REMOTE:-origin}"
BRANCH="${BRANCH:-main}"
INTERVAL="${INTERVAL:-300}"
LOG_FILE="${LOG_FILE:-$REPO_DIR/.git/auto_push.log}"
MAX_LOG_SIZE=1048576

log() { printf '%s %s\n' "$(date '+%F %T')" "$*" >>"$LOG_FILE"; }

usage() {
    sed -n '2,8p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# 避免同时运行多个实例，重复提交或抢占索引
take_lock() {
    command -v flock >/dev/null 2>&1 || return 0
    exec 9>"$REPO_DIR/.git/auto_push.lock"
    flock -n 9 || { echo "已有 auto_push 实例在运行，退出"; exit 0; }
}

# 日志超过 1MB 时保留最近 300 行，避免长期运行占用磁盘
rotate_log() {
    [ -f "$LOG_FILE" ] || return 0
    local size
    size=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
    if [ "$size" -gt "$MAX_LOG_SIZE" ]; then
        tail -n 300 "$LOG_FILE" >"$LOG_FILE.tmp" && mv "$LOG_FILE.tmp" "$LOG_FILE"
    fi
}

push_once() {
    rotate_log

    # 变基或合并未完成时先不动，等待人工处理
    if [ -d .git/rebase-merge ] || [ -d .git/rebase-apply ] || [ -f .git/MERGE_HEAD ]; then
        log "存在未完成的 rebase/merge，跳过本轮"
        return 0
    fi

    local changes count msg
    changes=$(git status --porcelain)
    if [ -n "$changes" ]; then
        if ! git add -A; then
            log "git add 失败，跳过本轮"
            return 0
        fi
        count=$(printf '%s\n' "$changes" | wc -l)
        msg="auto: sync $(date '+%F %T') (${count} files)"
        if git commit -q -m "$msg"; then
            log "已提交: $msg"
        else
            log "git commit 失败，跳过本轮"
            return 0
        fi
    elif ! git rev-parse --verify --quiet "$REMOTE/$BRANCH" >/dev/null; then
        log "没有新改动，且远端分支尚不存在，跳过"
        return 0
    fi

    # 本地没有领先提交时不必推送
    if git rev-parse --verify --quiet "$REMOTE/$BRANCH" >/dev/null; then
        if [ -z "$(git log --oneline "$REMOTE/$BRANCH..$BRANCH" 2>/dev/null)" ]; then
            return 0
        fi
    fi

    if git push -q "$REMOTE" "$BRANCH" >>"$LOG_FILE" 2>&1; then
        log "推送成功 -> $REMOTE/$BRANCH"
        return 0
    fi

    # 推送被拒绝（远端有新提交）时，先变基再重试一次
    log "推送失败，尝试 pull --rebase 后重试"
    if git pull -q --rebase --autostash "$REMOTE" "$BRANCH" >>"$LOG_FILE" 2>&1; then
        if git push -q "$REMOTE" "$BRANCH" >>"$LOG_FILE" 2>&1; then
            log "重试推送成功"
        else
            log "重试推送仍失败，等待下一轮"
        fi
    else
        git rebase --abort >/dev/null 2>&1
        log "rebase 失败并已中止，等待下一轮"
    fi
}

MODE="once"
while [ $# -gt 0 ]; do
    case "$1" in
        --watch) MODE="watch" ;;
        --interval)
            INTERVAL="${2:-}"
            if ! [[ "$INTERVAL" =~ ^[0-9]+$ ]] || [ "$INTERVAL" -lt 10 ]; then
                echo "间隔必须是 >=10 的整数秒" >&2
                exit 1
            fi
            shift
            ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage; exit 1 ;;
    esac
    shift
done

if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "当前目录不是 Git 仓库: $REPO_DIR" >&2
    exit 1
fi
if ! git remote get-url "$REMOTE" >/dev/null 2>&1; then
    echo "未配置远端 $REMOTE，请先执行 git remote add $REMOTE <地址>" >&2
    exit 1
fi

take_lock
mkdir -p "$(dirname "$LOG_FILE")"

if [ "$MODE" = "once" ]; then
    push_once
    exit 0
fi

trap 'log "收到退出信号，停止自动推送"; exit 0' INT TERM
log "自动推送已启动: $REMOTE/$BRANCH，间隔 ${INTERVAL}s，仓库 $REPO_DIR"
while :; do
    push_once
    sleep "$INTERVAL" &
    wait $!
done
