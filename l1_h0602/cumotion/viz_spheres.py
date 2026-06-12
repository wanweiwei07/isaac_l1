"""在 run_isaacsim.ps1 启动的 Isaac Sim 里显示 cuMotion 碰撞球（半透明红球，实时跟随机器人）。

前置：
  1. 用 run_isaacsim.ps1 启动 Isaac Sim（已开 127.0.0.1:8226 代码服务器）。
  2. VS Code 装 "Isaac Sim VS Code Edition" 插件，打开本文件 Ctrl+Enter 发送执行。

机器人若尚未加载会自动挂到 /World/l1_h0602（不清场景）。
再发一次会先清掉旧球再重建——改完 XRDF 重跑 gen_xrdf.py 后，发一次即可刷新。
"""
import builtins
import os

import yaml
import omni.kit.app
import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, Vt

# ---- Windows 绝对路径（代码在本机运行中的 Isaac Sim 内执行）----
PROJ = r"E:\isaac_proj\linx_isc6_l1\l1_h0602"
USD_PATH = os.path.join(PROJ, "usd", "l1_h0612.usda")
XRDF = os.path.join(PROJ, "cumotion", "robot.xrdf")
ROBOT_ROOT = "/World/l1_h0602"
VIZ_ROOT = "/World/viz_spheres"
_REG = "_viz_spheres_sub"

stage = omni.usd.get_context().get_stage()

# ---- 机器人没加载就先挂上（不新建 stage，避免清掉现有场景）----
if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")
    add_reference_to_stage(USD_PATH, ROBOT_ROOT)
    print(f"机器人未加载，已挂到 {ROBOT_ROOT}")

# ---- 清理上一次的 viz（重复执行时）----
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass
if stage.GetPrimAtPath(VIZ_ROOT).IsValid():
    stage.RemovePrim(VIZ_ROOT)

# ---- 读 XRDF 碰撞球（连杆局部坐标）----
spheres_by_link = yaml.safe_load(open(XRDF, encoding="utf-8"))[
    "geometry"]["robot_collision_spheres"]["spheres"]

# ---- 连杆名 -> stage 里的 Xform prim ----
link_prim = {}
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
    name = prim.GetName()
    if name in spheres_by_link and name not in link_prim and prim.IsA(UsdGeom.Xformable):
        link_prim[name] = prim

# ---- 建球 prim（半径固定，每帧只更新平移）----
UsdGeom.Xform.Define(stage, VIZ_ROOT)
viz = []  # (sphere_prim, link_name, local_center)
for link, sphs in spheres_by_link.items():
    if link not in link_prim:
        print(f"  [警告] 找不到连杆 {link}，跳过其球")
        continue
    for s in sphs:
        sphere = UsdGeom.Sphere.Define(stage, f"{VIZ_ROOT}/{link}_{len(viz)}")
        sphere.CreateRadiusAttr(float(s["radius"]))
        sphere.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.1, 0.1)]))
        sphere.CreateDisplayOpacityAttr(Vt.FloatArray([0.35]))
        sphere.AddTranslateOp()
        viz.append((sphere.GetPrim(), link, Gf.Vec3d(*[float(x) for x in s["center"]])))

print(f"建了 {len(viz)} 个碰撞球，覆盖 {len(link_prim)} 个连杆。")

# ---- update 回调：每帧把球放到对应连杆的世界位置 ----
def _on_update(e):
    cache = UsdGeom.XformCache()
    for prim, link, center in viz:
        world = cache.GetLocalToWorldTransform(link_prim[link]).Transform(center)
        prim.GetAttribute("xformOp:translate").Set(world)


sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _on_update, name="viz_spheres")
setattr(builtins, _REG, sub)  # 防 GC，保持回调存活
print("✅ 碰撞球将实时跟随机器人。再发一次本文件可刷新。")
