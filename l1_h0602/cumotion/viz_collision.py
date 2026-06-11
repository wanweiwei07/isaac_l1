"""显示/隐藏机器人的 collision 几何（convexHull 网格）。

l1_h0602 的 collision 网格在 USD 里是 purpose="guide" 的 <link>_1 网格，默认隐藏。
本脚本把它们切到可见、染成半透明绿；再发一次切回隐藏（toggle）。

前置：Isaac Sim 运行中 + main.py 已把机器人挂到 /World/l1_h0602。
用法：VS Code 打开本文件 Ctrl+Enter 发送。
"""
import omni.usd
from pxr import Gf, Usd, UsdGeom, UsdPhysics, Vt

ROBOT_ROOT = "/World/l1_h0602"

stage = omni.usd.get_context().get_stage()

# ---- 找 collision 网格（purpose=guide 或带 CollisionAPI）----
collision_prims = []
for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
    if not prim.IsA(UsdGeom.Mesh):
        continue
    img = UsdGeom.Imageable(prim)
    if img.GetPurposeAttr().Get() == UsdGeom.Tokens.guide or prim.HasAPI(UsdPhysics.CollisionAPI):
        collision_prims.append((prim, img))

if not collision_prims:
    print("没找到 collision 网格，检查机器人是否已加载。")
else:
    # toggle：当前是 guide(隐藏) 这次就显示，否则隐藏
    show = collision_prims[0][1].GetPurposeAttr().Get() == UsdGeom.Tokens.guide
    for prim, img in collision_prims:
        if show:
            img.GetPurposeAttr().Set(UsdGeom.Tokens.default_)
            mesh = UsdGeom.Mesh(prim)
            mesh.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.1, 0.8, 0.2)]))
            mesh.CreateDisplayOpacityAttr(Vt.FloatArray([0.4]))
        else:
            img.GetPurposeAttr().Set(UsdGeom.Tokens.guide)
    print(f"{'显示' if show else '隐藏'} {len(collision_prims)} 个 collision 网格。再发一次切换。")
