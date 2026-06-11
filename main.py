"""把 l1_h0602 机器人 USD 显示到运行中的 Isaac Sim。

用法：
  1. 先用 ./start_isaacsim.sh 启动 Isaac Sim（确认 8226 服务器就绪）。
  2. 在 VS Code 打开本文件，Ctrl+Enter 发送给 Isaac Sim 执行。

说明：通过 VS Code 插件远程执行时 __file__ 不可用，这里用绝对路径。
"""
import omni.usd
from pxr import UsdLux

USD_PATH = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/usd/l1_h0602.usda"

# 打开机器人 stage（会替换当前场景）。
ctx = omni.usd.get_context()
ctx.open_stage(USD_PATH)
stage = ctx.get_stage()

# 机器人 USD 里通常没有灯光，补一盏 dome light 以免模型全黑。
if not stage.GetPrimAtPath("/World/DomeLight").IsValid():
    UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)

print("已打开机器人 stage:", stage.GetRootLayer().identifier)
print("根节点 prims:", [p.GetPath().pathString for p in stage.GetPseudoRoot().GetChildren()])
