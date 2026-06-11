"""把 l1_h0602 机器人挂到运行中的 Isaac Sim 的 /World/l1_h0602 下显示。

前置：./start_isaacsim.sh 启动 Isaac Sim（确认 8226 服务器就绪）。
用法：VS Code 打开本文件，Ctrl+Enter 发送执行。

约定：机器人固定挂在 /World/l1_h0602，viz_*/plan_* 脚本都用这个路径。
"""
import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from pxr import UsdGeom, UsdLux

USD_PATH = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/usd/l1_h0602.usda"
ROBOT_ROOT = "/World/l1_h0602"

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()

UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

add_reference_to_stage(USD_PATH, ROBOT_ROOT)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)

print(f"已把机器人挂到 {ROBOT_ROOT}")
