"""交互式调 cuMotion 碰撞球：在视口里直接拖位置 + 缩放改大小，调好写回 robot.xrdf。

原理：把每个球作为**子节点**挂到对应连杆 prim 下，于是
  - 球自动跟随连杆（无需回调）；
  - 球的 xformOp:translate 就是连杆局部坐标 = XRDF 的 center；
  - 球的 xformOp:scale × radius 就是有效半径。
所以你用移动工具拖 = 改 center，用缩放工具拉 = 改 radius，所见即所得。

前置：run_isaacsim.ps1 启动 Isaac Sim；VS Code "Isaac Sim VS Code Edition" 插件 Ctrl+Enter 发送。

用法：
  1. MODE = "build"  发送 → 生成可拖动的球（机器人没加载会自动挂上，不清场景）。
     在视口选中球，用移动(W)/缩放(R)工具调。可反复发送 build 重置成 XRDF 现值。
  2. 调好后把 MODE 改成 "dump" 再发送一次 → 读回所有球的局部坐标+半径，
     只重写 robot.xrdf 的 geometry: 段（上面的 cspace/ignore 等原样保留）。
"""
import builtins
import os

import yaml
import omni.usd
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from pxr import Gf, Usd, UsdGeom, Vt

MODE = "build"   # "build" 建可拖动的球；调好后改成 "dump" 写回 XRDF

PROJ = "/home/wrs/Workspace/isaac_linxl1/l1_h0602"
USD_PATH = os.path.join(PROJ, "usd", "l1_h0612.usda")
XRDF = os.path.join(PROJ, "cumotion", "robot.xrdf")
ROBOT_ROOT = "/World/l1_h0602"
_REG = "_viz_edit_spheres"   # builtins: 已建编辑球列表 [(link, sphere_path)]

stage = omni.usd.get_context().get_stage()


def _round(x, n=4):
    return round(float(x), n)


def _clear_existing():
    """删掉上一次 build 出来的编辑球。"""
    for _link, path in getattr(builtins, _REG, []):
        if stage.GetPrimAtPath(path).IsValid():
            stage.RemovePrim(path)
    setattr(builtins, _REG, [])


if MODE == "build":
    # 机器人没加载就先挂上（不新建 stage）
    if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        add_reference_to_stage(USD_PATH, ROBOT_ROOT)
        print(f"机器人未加载，已挂到 {ROBOT_ROOT}")

    _clear_existing()

    spheres_by_link = yaml.safe_load(open(XRDF, encoding="utf-8"))[
        "geometry"]["robot_collision_spheres"]["spheres"]

    # 连杆名 -> stage prim
    link_prim = {}
    for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT)):
        name = prim.GetName()
        if name in spheres_by_link and name not in link_prim and prim.IsA(UsdGeom.Xformable):
            link_prim[name] = prim

    reg = []
    n = 0
    for link, sphs in spheres_by_link.items():
        parent = link_prim.get(link)
        if parent is None:
            print(f"  [警告] 找不到连杆 {link}，跳过其球")
            continue
        for j, s in enumerate(sphs):
            path = f"{parent.GetPath().pathString}/_edit_{link}_{j}"
            if stage.GetPrimAtPath(path).IsValid():
                stage.RemovePrim(path)   # 防 builtins 注册表与 stage 失同步导致 op 重复
            sphere = UsdGeom.Sphere.Define(stage, path)
            sphere.CreateRadiusAttr(float(s["radius"]))   # 基准半径；缩放在它上面叠
            sphere.CreateDisplayColorAttr(Vt.Vec3fArray([Gf.Vec3f(0.9, 0.1, 0.1)]))
            sphere.CreateDisplayOpacityAttr(Vt.FloatArray([0.35]))
            xf = UsdGeom.Xformable(sphere)
            xf.AddTranslateOp().Set(Gf.Vec3d(*[float(x) for x in s["center"]]))
            xf.AddScaleOp().Set(Gf.Vec3f(1.0, 1.0, 1.0))
            reg.append((link, path))
            n += 1
    setattr(builtins, _REG, reg)
    print(f"建了 {n} 个可编辑球，覆盖 {len(link_prim)} 个连杆。"
          f"\n  拖动(W)=改 center，缩放(R)=改 radius。调好后把 MODE 改成 'dump' 再发送写回。")

elif MODE == "dump":
    reg = getattr(builtins, _REG, [])
    if not reg:
        print("没有可导出的编辑球——先用 MODE='build' 发送一次。")
    else:
        grouped = {}   # link -> [(center3, radius), ...]，按 build 顺序
        for link, path in reg:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            t = prim.GetAttribute("xformOp:translate").Get()
            r = float(prim.GetAttribute("radius").Get())
            sc = prim.GetAttribute("xformOp:scale").Get()
            if sc is not None:
                r *= (float(sc[0]) + float(sc[1]) + float(sc[2])) / 3.0   # 均匀缩放折算
            center = [_round(t[0]), _round(t[1]), _round(t[2])]
            grouped.setdefault(link, []).append((center, _round(r)))

        # 只重写 geometry: 段，上面部分原样保留
        text = open(XRDF, encoding="utf-8").read()
        head = text.split("\ngeometry:")[0].rstrip("\n")
        lines = [head, "", "geometry:", "  robot_collision_spheres:", "    spheres:"]
        nsph = 0
        for link, items in grouped.items():
            lines.append(f"      {link}:")
            for c, r in items:
                lines.append(f"        - center: [ {c[0]}, {c[1]}, {c[2]} ]")
                lines.append(f"          radius: {r}")
                nsph += 1
        open(XRDF, "w", encoding="utf-8").write("\n".join(lines) + "\n")
        print(f"✅ 已把 {nsph} 个球（{len(grouped)} 连杆）的局部坐标+半径写回 {XRDF}。"
              f"\n  其余段（cspace/ignore 等）保持不变。")

else:
    print(f"未知 MODE={MODE!r}，应为 'build' 或 'dump'。")
