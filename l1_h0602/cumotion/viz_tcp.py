"""把 4 个 TCP 坐标系画在视口里（X红/Y绿/Z蓝 三轴 + 原点小球），随机器人实时跟随。

原理：TCP 是各自 hand_base_link 的固定子坐标系（旋转 identity、偏移常量），
所以读 hand_base_link 的世界变换(USD XformCache，随机器人更新)再套常量偏移即可，
不依赖 cuMotion FK / 物理。与 robot.urdf 里加的 TCP 定义一致。

用法：先发 main.py，再发本文件。再发一次先清旧的再重画。改 AXIS_LEN 调轴长。
"""
import builtins

import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom, Vt

ROBOT_ROOT = "/World/l1_h0602"
VIZ_ROOT = "/World/viz_tcp"
_REG = "_viz_tcp_sub"
AXIS_LEN = 0.05      # 每根轴长 (m)
AXIS_WIDTH = 0.004   # 轴线粗细
DOT_R = 0.006        # 原点小球半径

# 与 robot.urdf 一致：name, 父 link, 偏移(在父系下)
TCPS = [
    ("lh_power_tcp", "lh_hand_base_link", (0.041, -0.011, 0.093)),
    ("lh_pinch_tcp", "lh_hand_base_link", (0.039, -0.027, 0.095)),
    ("lh_tripod_tcp", "lh_hand_base_link", (0.038, -0.017, 0.096)),
    ("rh_power_tcp", "rh_hand_base_link", (0.041, 0.011, 0.093)),
    ("rh_pinch_tcp", "rh_hand_base_link", (0.039, 0.027, 0.095)),
    ("rh_tripod_tcp", "rh_hand_base_link", (0.038, 0.017, 0.096)),
]
_AXCOL = [Gf.Vec3f(1, 0.1, 0.1), Gf.Vec3f(0.1, 1, 0.1), Gf.Vec3f(0.2, 0.4, 1)]  # X,Y,Z

stage = omni.usd.get_context().get_stage()

# 清上一次
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass
    delattr(builtins, _REG)
if stage.GetPrimAtPath(VIZ_ROOT).IsValid():
    stage.RemovePrim(VIZ_ROOT)

# 父 link prim
link_prim = {}
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
    n = prim.GetName()
    if n in {p for _, p, _ in TCPS} and n not in link_prim:
        link_prim[n] = prim

UsdGeom.Xform.Define(stage, VIZ_ROOT)
viz = []   # (curves, dot, parent_prim, offset)
for name, parent, off in TCPS:
    if parent not in link_prim:
        print(f"  [警告] 找不到父 link {parent}，跳过 {name}")
        continue
    curves = UsdGeom.BasisCurves.Define(stage, f"{VIZ_ROOT}/{name}")
    curves.CreateTypeAttr("linear")
    curves.CreateCurveVertexCountsAttr([2, 2, 2])           # 三段，各 2 点
    curves.CreateWidthsAttr(Vt.FloatArray([AXIS_WIDTH]))
    curves.SetWidthsInterpolation("constant")
    cda = curves.CreateDisplayColorAttr(Vt.Vec3fArray(_AXCOL))
    UsdGeom.Primvar(cda).SetInterpolation("uniform")        # 每段一色
    dot = UsdGeom.Sphere.Define(stage, f"{VIZ_ROOT}/{name}/dot")
    dot.CreateRadiusAttr(DOT_R)
    dot.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(1, 1, 0.2)]))
    dot.AddTranslateOp()
    viz.append((curves, dot, link_prim[parent], off))

print(f"画了 {len(viz)} 个 TCP 坐标系。X红 Y绿 Z蓝，黄球=原点。")


def _on_update(e):
    cache = UsdGeom.XformCache()
    for curves, dot, parent, off in viz:
        m = cache.GetLocalToWorldTransform(parent)
        o = m.Transform(Gf.Vec3d(off[0], off[1], off[2]))
        xe = m.Transform(Gf.Vec3d(off[0] + AXIS_LEN, off[1], off[2]))
        ye = m.Transform(Gf.Vec3d(off[0], off[1] + AXIS_LEN, off[2]))
        ze = m.Transform(Gf.Vec3d(off[0], off[1], off[2] + AXIS_LEN))
        pts = [o, xe, o, ye, o, ze]
        curves.GetPointsAttr().Set(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
        dot.GetPrim().GetAttribute("xformOp:translate").Set(o)


sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _on_update, name="viz_tcp")
setattr(builtins, _REG, sub)
print("✅ TCP 坐标系将实时跟随机器人。再发一次可刷新。")
