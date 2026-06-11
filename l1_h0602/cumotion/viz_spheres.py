"""用 update 回调显示 cuMotion 碰撞球（半透明红球，随机器人实时跟随）。

前置：Isaac Sim 运行中 + main.py 已把机器人挂到 /World/l1_h0602。
用法：VS Code 打开本文件 Ctrl+Enter 发送。再发一次会先清掉旧的再重建（改 XRDF 后刷新用）。
"""
import builtins

import yaml
import omni.kit.app
import omni.usd
from pxr import Gf, Usd, UsdGeom, Vt

ROBOT_ROOT = "/World/l1_h0602"
VIZ_ROOT = "/World/viz_spheres"
XRDF = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/cumotion/robot.xrdf"
_REG = "_viz_spheres_sub"

stage = omni.usd.get_context().get_stage()

# ---- 清理上一次的 viz（重复执行时）----
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass
if stage.GetPrimAtPath(VIZ_ROOT):
    stage.RemovePrim(VIZ_ROOT)

# ---- 读 XRDF 碰撞球（连杆局部坐标）----
spheres_by_link = yaml.safe_load(open(XRDF))["geometry"]["robot_collision_spheres"]["spheres"]

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
