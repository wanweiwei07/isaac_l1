"""把 l1_h0612 机器人挂到运行中的 Isaac Sim 的 /World/l1_h0602 下显示。

前置：run_isaacsim.ps1 启动 Isaac Sim（确认 127.0.0.1:8226 代码服务器就绪）。
用法：VS Code 装 "Isaac Sim VS Code Edition" 插件，打开本文件 Ctrl+Enter 发送执行。

约定：机器人固定挂在 /World/l1_h0602，viz_*/plan_* 脚本都用这个路径。
"""
import os

import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, Vt

USD_PATH = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/usd/l1_h0612.usda"
ROBOT_ROOT = "/World/l1_h0602"

# 分拣工作台（四角桌；单位 m，输入 cm 已换算）：宽(x)0.6 长(y)1.2 高(z)0.9，底面中心 (0.3,0,0)
WORKBENCH_PATH = "/World/Workbench"
WB_SIZE = (0.6, 1.2, 0.9)            # x宽, y长, z总高
WB_BOTTOM_CENTER = (0.3, 0.0, 0.0)   # 底面中心 (m)
WB_TOP_THICK = 0.05                  # 台面板厚
WB_LEG = 0.06                        # 桌腿方截面边长

ctx = omni.usd.get_context()
ctx.new_stage()
stage = ctx.get_stage()

UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

add_reference_to_stage(USD_PATH, ROBOT_ROOT)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)

# 物理场景：加大 Newton 求解器迭代，收紧固定基座约束（减小自重反力下基座的微小后移）
_scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
_sp = _scene.GetPrim()
_sp.AddAppliedSchema("NewtonSceneAPI")
_sp.CreateAttribute("newton:maxSolverIterations", Sdf.ValueTypeNames.Int).Set(64)

# ---- 分拣工作台：四角桌（台面 + 四腿），各部件静态碰撞体（CollisionAPI、无 RigidBody）----
wb = UsdGeom.Xform.Define(stage, WORKBENCH_PATH)
UsdGeom.Xformable(wb).AddTranslateOp().Set(Gf.Vec3d(*WB_BOTTOM_CENTER))  # 整桌挂在底面中心
_WB_COLOR = Gf.Vec3f(0.55, 0.45, 0.35)


def _wb_box(name, size, center):
    """在工作台下建一个单位立方、缩放成 size、平移到 center（相对底面中心）的静态碰撞盒。"""
    c = UsdGeom.Cube.Define(stage, f"{WORKBENCH_PATH}/{name}")
    c.CreateSizeAttr(1.0)
    c.CreateDisplayColorAttr(Vt.Vec3fArray([_WB_COLOR]))
    x = UsdGeom.Xformable(c)
    x.AddTranslateOp().Set(Gf.Vec3d(*center))
    x.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim())


_w, _l, _h = WB_SIZE
_leg_h = _h - WB_TOP_THICK
_wb_box("Top", (_w, _l, WB_TOP_THICK), (0, 0, _h - WB_TOP_THICK / 2.0))   # 台面
_dx, _dy = _w / 2 - WB_LEG / 2, _l / 2 - WB_LEG / 2                       # 腿心内缩到台面下
for _i, (_sx, _sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
    _wb_box(f"Leg_{_i}", (WB_LEG, WB_LEG, _leg_h), (_sx * _dx, _sy * _dy, _leg_h / 2.0))

print(f"已把机器人挂到 {ROBOT_ROOT}（{os.path.basename(USD_PATH)}）")
print(f"已建四角工作台 {WORKBENCH_PATH}：{WB_SIZE} m，底面中心 {WB_BOTTOM_CENTER}")
