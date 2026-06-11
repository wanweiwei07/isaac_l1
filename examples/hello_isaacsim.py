"""通过 VS Code 的 Isaac Sim 插件发送到运行中的 Isaac Sim 执行。

用法：
  1. 先用 ./start_isaacsim.sh 启动 Isaac Sim（确保看到 8226 服务器就绪）。
  2. 在 VS Code 打开本文件，命令面板执行 “Isaac Sim VS Code: Run”
     （或默认快捷键 Ctrl+Enter），代码会在 Isaac Sim 进程里执行。
  3. 视口里应出现一个新建空场景 + 一个立方体；print 输出回显在 VS Code 输出面板。

注意：这些 omni.* 导入要在 Isaac Sim 进程内执行，本文件不要用本地 python 直接运行。
"""
import omni.usd
from pxr import Gf, UsdGeom, UsdLux

# 新建一个空 stage 并加一个立方体，确认远程执行链路通畅。
ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()

# 空场景没有灯光，物体会全黑——加一盏 dome light 提供环境光。
dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
dome.CreateIntensityAttr(1000.0)

# 再加一盏平行光，让立方体有明暗层次。
distant = UsdLux.DistantLight.Define(stage, "/World/DistantLight")
distant.CreateIntensityAttr(3000.0)

cube = UsdGeom.Cube.Define(stage, "/World/HelloCube")
# 给个顶点色，物体不再是默认灰/黑。
cube.CreateDisplayColorAttr([Gf.Vec3f(0.1, 0.5, 0.9)])

print("已创建立方体 + 灯光，stage:", stage.GetRootLayer().identifier)
