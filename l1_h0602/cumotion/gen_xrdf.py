"""根据 l1_h0602 的 URDF 关节偏移 + STL 包围盒，生成 cuMotion 用的 robot.xrdf。

碰撞球策略：把每条连杆近似成「沿它到下一关节的线段排布的一串球」；躯干/头/手
等末端连杆用显式球簇覆盖。球坐标都在各自连杆的局部坐标系下。
生成后建议在 Isaac Sim 里用 cuMotion 的球编辑器微调。重新调参就改本文件再跑一次。
"""
import xml.etree.ElementTree as ET

URDF = "robot.urdf"
OUT = "robot.xrdf"

# ---- 15 个受控关节（腰1 + 左臂6 + 右臂6 + 颈2）----
CONTROLLED = (
    ["waist_joint2"]
    + [f"left_arm_joint_{i}" for i in range(1, 7)]
    + [f"right_arm_joint_{i}" for i in range(1, 7)]
    + ["neck_joint1", "neck_joint2"]
)

# 锁定的手指关节（不在 cspace → cuMotion 固定在 default 值）。
# 注意：*_ip / *_dip 是 mimic 关节（跟随 *_cmc_pitch / *_mcp_pitch 联动），
# 不能给它们设 default_joint_positions，否则 load 报错。只列 12 个非 mimic 的。
HAND_JOINTS = []
for side in ("lh", "rh"):
    HAND_JOINTS += [
        f"{side}_thumb_cmc_yaw", f"{side}_thumb_cmc_pitch",
        f"{side}_index_mcp_pitch", f"{side}_middle_mcp_pitch",
        f"{side}_ring_mcp_pitch", f"{side}_pinky_mcp_pitch",
    ]

# 每个受控关节的加速度/加加速度限位（rad/s^2, rad/s^3），速度限位由 URDF 提供
ACCEL = {j: 10.0 for j in CONTROLLED}
JERK = {j: 1000.0 for j in CONTROLLED}

# ---- 读取 URDF，拿到每个 joint 的 child 偏移向量 d ----
root = ET.parse(URDF).getroot()
JD = {}
for j in root.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0").split() if o is not None else [0, 0, 0])]
    JD[j.get("name")] = {"child": j.find("child").get("link"), "xyz": xyz}


def seg(link_joint, n, r, start=0.0):
    """沿『生成该连杆的关节』到『它下一个关节』的线段放 n 个球。"""
    d = JD[link_joint]["xyz"]  # 该连杆原点 -> 下一关节 的偏移（局部系）
    out = []
    for k in range(n):
        t = start + (1 - start) * (k / max(n - 1, 1))
        out.append(([round(t * d[i], 4) for i in range(3)], r))
    return out


def line_to(next_joint, n, r):
    """连杆原点 = next_joint 的父关节原点；球沿到 next_joint 的偏移排布。"""
    d = JD[next_joint]["xyz"]
    return [([round((k / max(n - 1, 1)) * d[i], 4) for i in range(3)], r) for k in range(n)]


# ---- 各连杆碰撞球（局部坐标系）----
SPHERES = {}

# 躯干 / 头
SPHERES["base_link"] = [([0, 0, 0.30], 0.18), ([0, 0, 0.45], 0.18)]
SPHERES["waist_link1"] = line_to("waist_joint2", 5, 0.085)            # 腰柱，向上到上躯干
SPHERES["waist_link2"] = [                                            # 胸腔/双肩根部
    ([0, 0.0, 0.05], 0.13), ([0, 0.0, 0.16], 0.12),
    ([0, 0.11, 0.13], 0.08), ([0, -0.11, 0.13], 0.08),
]
SPHERES["neck_link1"] = [([0, 0, 0.02], 0.05)]
SPHERES["neck_link2"] = [([0, 0.05, 0], 0.10), ([0, 0.09, 0], 0.09)]  # 头

# 双臂（每条连杆用它自己的关节偏移，左右方向自动正确）
for side in ("left", "right"):
    SPHERES[f"{side}_arm_link_1"] = seg(f"{side}_arm_joint_2", 2, 0.06)
    SPHERES[f"{side}_arm_link_2"] = seg(f"{side}_arm_joint_3", 5, 0.055)   # 大臂（长）
    SPHERES[f"{side}_arm_link_3"] = seg(f"{side}_arm_joint_4", 3, 0.06)    # 肘
    SPHERES[f"{side}_arm_link_4"] = seg(f"{side}_arm_joint_5", 4, 0.05)    # 小臂
    SPHERES[f"{side}_arm_link_5"] = seg(f"{side}_arm_joint_6", 2, 0.045)   # 腕
    SPHERES[f"{side}_arm_link_6"] = [                                      # 腕末端
        ([0, 0, 0.02], 0.05), ([0, -0.04, 0.03], 0.05), ([0, -0.08, 0.04], 0.045),
    ]
    # 锁定的手 → 用一团球近似（mount 在 link_6 的 +z0.034 处，手向前伸）
    sh = "lh" if side == "left" else "rh"
    SPHERES[f"{sh}_hand_base_link"] = [
        ([0, 0, 0.03], 0.05), ([0, 0, 0.08], 0.045), ([0, 0, 0.12], 0.04),
    ]

# ---- 自碰撞忽略：相邻/永远贴近的连杆对 ----
IGNORE = {
    "base_link": ["waist_link1", "waist_link2"],
    "waist_link1": ["waist_link2"],
    "waist_link2": ["neck_link1", "left_arm_link_1", "right_arm_link_1"],
    "neck_link1": ["neck_link2"],
}
for side, sh in (("left", "lh"), ("right", "rh")):
    IGNORE[f"{side}_arm_link_1"] = [f"{side}_arm_link_2"]
    IGNORE[f"{side}_arm_link_2"] = [f"{side}_arm_link_3"]
    IGNORE[f"{side}_arm_link_3"] = [f"{side}_arm_link_4"]
    IGNORE[f"{side}_arm_link_4"] = [f"{side}_arm_link_5"]
    IGNORE[f"{side}_arm_link_5"] = [f"{side}_arm_link_6"]
    IGNORE[f"{side}_arm_link_6"] = [f"{sh}_hand_base_link"]

# ---- 输出 XRDF ----
L = []
w = L.append
w("# 由 gen_xrdf.py 从 l1_h0602 URDF 生成；碰撞球为近似值，建议在 GUI 里微调。")
w("format: xrdf")
w("format_version: 2.0")
w("")
w("default_joint_positions:")
for j in HAND_JOINTS:
    w(f"  {j}: 0.0")
w("")
w("cspace:")
w("  joint_names:")
for j in CONTROLLED:
    w(f'    - "{j}"')
w("  acceleration_limits: [" + ", ".join(f"{ACCEL[j]}" for j in CONTROLLED) + "]")
w("  jerk_limits: [" + ", ".join(f"{JERK[j]}" for j in CONTROLLED) + "]")
w("")
w('tool_frames: ["lh_hand_base_link", "rh_hand_base_link"]')
w("")
w('world_collision:')
w('  geometry: "robot_collision_spheres"')
w("")
w("self_collision:")
w('  geometry: "robot_collision_spheres"')
w("  ignore:")
for k, v in IGNORE.items():
    w(f'    {k}: [ ' + ", ".join(f'"{x}"' for x in v) + " ]")
w("")
w("geometry:")
w("  robot_collision_spheres:")
w("    spheres:")
for link, spheres in SPHERES.items():
    w(f"      {link}:")
    for c, r in spheres:
        w(f"        - center: [ {c[0]}, {c[1]}, {c[2]} ]")
        w(f"          radius: {r}")

open(OUT, "w").write("\n".join(L) + "\n")
nsph = sum(len(s) for s in SPHERES.values())
print(f"已生成 {OUT}: {len(CONTROLLED)} 受控关节, {len(HAND_JOINTS)} 锁定手指关节, "
      f"{len(SPHERES)} 连杆 / {nsph} 个碰撞球")
