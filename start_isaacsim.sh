#!/usr/bin/env bash
# 启动 Isaac Sim（GUI），并开启代码执行服务器，供 VS Code 的
# "NVIDIA Isaac Sim (vscode edition)" 插件连接、远程执行 Python 代码。
#
# 工作原理：
#   - isaacsim.code_editor.python_server  在 127.0.0.1:8226 上监听，接收并执行 Python
#   - isaacsim.code_editor.vscode         提供 Window > VS Code 菜单 + 连接信息
#   VS Code 插件把当前文件（或选中片段）的代码发到 8226，在运行中的 Isaac Sim 里执行。
#
# 用法：
#   ./start_isaacsim.sh                # 启动完整 GUI（默认）
#   ./start_isaacsim.sh --headless     # 无窗口，仅作为远程执行后端
#   其余参数原样转发给 isaacsim / Kit。
set -euo pipefail

VENV="${ISAACSIM_VENV:-$HOME/.venvs/isaacsim}"
EXPERIENCE="${ISAACSIM_EXP:-isaacsim.exp.full}"
HOST="${ISAACSIM_SERVER_HOST:-127.0.0.1}"
PORT="${ISAACSIM_SERVER_PORT:-8226}"

# shellcheck disable=SC1091
source "$VENV/bin/activate"

echo "启动 Isaac Sim（$EXPERIENCE），代码执行服务器: $HOST:$PORT"
echo "就绪后在 VS Code 里用 Isaac Sim 插件执行代码（默认快捷键 Ctrl+Enter / 命令面板搜 'Isaac Sim'）。"

exec isaacsim "$EXPERIENCE" \
  --enable isaacsim.code_editor.python_server \
  --enable isaacsim.code_editor.vscode \
  "--/exts/isaacsim.code_editor.python_server/host=$HOST" \
  "--/exts/isaacsim.code_editor.python_server/port=$PORT" \
  "$@"
