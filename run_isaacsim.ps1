# 启动 Isaac Sim 全功能 GUI，并开启 VS Code 代码执行服务器 (127.0.0.1:8226)
# 用法: 右键 "用 PowerShell 运行" 或在终端执行  .\run_isaacsim.ps1
# 启动后保持这个窗口/Isaac Sim 开着，然后去 VS Code 用 "Isaac Sim VS Code Edition" 插件发送代码。

$ErrorActionPreference = "Stop"

$IsaacExe = "E:\isaac_proj\.venv\Scripts\isaacsim.exe"
$Experience = "isaacsim.exp.full.kit"

if (-not (Test-Path $IsaacExe)) {
    Write-Error "找不到 $IsaacExe，确认 venv 路径是否正确。"
}

Write-Host "启动 Isaac Sim ($Experience) ，代码服务器: 127.0.0.1:8226 ..." -ForegroundColor Cyan

& $IsaacExe $Experience `
    --enable isaacsim.code_editor.python_server `
    --enable isaacsim.code_editor.vscode
