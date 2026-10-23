#!/usr/bin/env bash
# 清理 SITL 仿真日志：只保留最新一个 .BIN，删除其余（释放磁盘空间）。
#
# 用法:
#   Tools/autotest/cleanup_sitl_logs.sh          # 清理当前仓库 logs/
#   Tools/autotest/cleanup_sitl_logs.sh <路径>   # 清理指定 logs 目录
#
# 约定: 每次 SITL 仿真结束后运行一次, 仅保留最新日志。

set -euo pipefail

LOGS_DIR="${1:-$(cd "$(dirname "$0")/../.." && pwd)/logs}"

if [ ! -d "$LOGS_DIR" ]; then
    echo "logs 目录不存在: $LOGS_DIR" >&2
    exit 1
fi

# 按时间排序找最新 .BIN
LATEST=$(ls -1t "$LOGS_DIR"/*.BIN 2>/dev/null | head -1) || true

if [ -z "$LATEST" ]; then
    echo "没有找到 .BIN 日志, 无需清理"
    exit 0
fi

echo "保留最新日志: $(basename "$LATEST") ($(du -h "$LATEST" | cut -f1))"

REMOVED=0
for f in "$LOGS_DIR"/*.BIN; do
    [ -e "$f" ] || continue
    if [ "$f" != "$LATEST" ]; then
        echo "删除旧日志: $(basename "$f") ($(du -h "$f" | cut -f1))"
        rm -f "$f"
        REMOVED=$((REMOVED+1))
    fi
done

# 更新 LASTLOG.TXT 指向保留的日志
if [ -f "$LATEST" ]; then
    basename "$LATEST" > "$LOGS_DIR/LASTLOG.TXT"
fi

echo "清理完成: 删除 $REMOVED 个旧日志, 保留 $(basename "$LATEST")"