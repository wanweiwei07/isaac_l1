"""从 l1_h0612 的 collision 网格自动生成 cuMotion 碰撞球，写入 robot.xrdf。

Isaac Sim 6 / cuMotion（XRDF 格式，非 lula）。
用 cuMotion 的 `create_collision_sphere_generator(V,T).generate_spheres(N, offset)`：
对每个连杆读它的 collision STL，按配置球数 N 均匀生成内切球，再用 radius_offset 适度膨胀。
所有 collision origin 都是 0、scale=1，所以球心即连杆系坐标，无需变换。

在 isaacsim venv 里运行（脚本会自动把 cuMotion 的 bundled 包加进 sys.path）：
    python l1_h0602/cumotion/gen_xrdf.py
"""
import glob
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "..", "urdf", "h0612.urdf")
MESH_ROOT = os.path.join(HERE, "..", "urdf")  # collision filename 形如 ../meshes/xxx
OUT = os.path.join(HERE, "robot.xrdf")

# 让 bundled `cumotion` 包可导入（独立运行也能用）。Win 下是 Lib\site-packages，
# Linux 下是 lib/pythonX/site-packages，所以用递归 glob 兼容两者。
for cand in glob.glob(os.path.join(
        sys.prefix, "**", "isaacsim.robot_motion.cumotion", "pip_prebundle"),
        recursive=True):
    sys.path.insert(0, cand)
import cumotion  # noqa: E402

# ---- 15 受控关节（waist_joint1 是 fixed，不计）----
CONTROLLED = (
    ["waist_joint2"]
    + [f"left_arm_joint_{i}" for i in range(1, 7)]
    + [f"right_arm_joint_{i}" for i in range(1, 7)]
    + ["neck_joint1", "neck_joint2"]
)
# 锁定的非 mimic 手指关节（*_dip / *_ip 是 mimic，由父关节驱动，不能设默认位）
HAND_JOINTS = []
for s in ("lh", "rh"):
    HAND_JOINTS += [f"{s}_thumb_cmc_yaw", f"{s}_thumb_cmc_pitch",
                    f"{s}_index_mcp_pitch", f"{s}_middle_mcp_pitch",
                    f"{s}_ring_mcp_pitch", f"{s}_pinky_mcp_pitch"]
ACCEL = {j: 10.0 for j in CONTROLLED}
JERK = {j: 1000.0 for j in CONTROLLED}

# ---- 每个连杆生成多少球 / 半径膨胀（按连杆大小手调）----
# (num_spheres, radius_offset)
SPHERE_CFG = {
    "base_link": (6, 0.05), "waist_link1": (4, 0.04), "waist_link2": (8, 0.04),
    "neck_link1": (1, 0.03), "neck_link2": (3, 0.04),
}
for side in ("left", "right"):
    SPHERE_CFG[f"{side}_arm_link_1"] = (3, 0.03)
    SPHERE_CFG[f"{side}_arm_link_2"] = (6, 0.03)
    SPHERE_CFG[f"{side}_arm_link_3"] = (3, 0.03)
    SPHERE_CFG[f"{side}_arm_link_4"] = (4, 0.03)
    SPHERE_CFG[f"{side}_arm_link_5"] = (2, 0.025)
    SPHERE_CFG[f"{side}_arm_link_6"] = (3, 0.03)
# 手：base2 是拇指根部小转接件，给 1 个小球
for sh in ("lh", "rh"):
    SPHERE_CFG[f"{sh}_hand_base_link"] = (4, 0.02)
    SPHERE_CFG[f"{sh}_thumb_metacarpals_base2"] = (1, 0.012)
    for f in ("thumb_metacarpals", "thumb_distal", "index_proximal", "index_distal",
              "middle_proximal", "middle_distal", "ring_proximal", "ring_distal",
              "pinky_proximal", "pinky_distal"):
        SPHERE_CFG[f"{sh}_{f}"] = (1, 0.012)


def load_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        V, T = [], []
        for _ in range(n):
            d = f.read(50)
            if len(d) < 50:
                break
            b = len(V)
            for v in range(3):
                V.append(struct.unpack("<fff", d[12 + v * 12:24 + v * 12]))
            T.append((b, b + 1, b + 2))
    return np.array(V, np.float64), np.array(T, np.int32)


# ---- 读 URDF：link -> collision mesh 路径 ----
root = ET.parse(URDF).getroot()
LINK_MESH = {}
for link in root.findall("link"):
    col = link.find("collision")
    if col is None:
        continue
    m = col.find(".//mesh")
    if m is None:
        continue
    LINK_MESH[link.get("name")] = os.path.normpath(os.path.join(MESH_ROOT, m.get("filename")))

# ---- 生成球 ----
SPHERES = {}
for link, (n, off) in SPHERE_CFG.items():
    mesh = LINK_MESH.get(link)
    if not mesh or not os.path.exists(mesh):
        print(f"  [跳过] {link}: 找不到 collision mesh")
        continue
    V, T = load_stl(mesh)
    gen = cumotion.create_collision_sphere_generator(vertices=V, triangles=T)
    sph = gen.generate_spheres(num_spheres=n, radius_offset=off)
    SPHERES[link] = [([round(c, 4) for c in s.center], round(s.radius, 4)) for s in sph]

# ---- 自碰撞忽略 ----
IGNORE = {
    "base_link": ["waist_link1", "waist_link2"],
    "waist_link1": ["waist_link2"],
    "waist_link2": ["neck_link1", "left_arm_link_1", "right_arm_link_1"],
    "neck_link1": ["neck_link2"],
}
for side, sh in (("left", "lh"), ("right", "rh")):
    for i in range(1, 6):
        IGNORE[f"{side}_arm_link_{i}"] = [f"{side}_arm_link_{i + 1}"]
    IGNORE[f"{side}_arm_link_6"] = [f"{sh}_hand_base_link"]
    # 同一只手内的连杆永远刚性相邻 -> 互相忽略
    hand_links = [f"{sh}_hand_base_link", f"{sh}_thumb_metacarpals_base2"] + [
        f"{sh}_{f}" for f in ("thumb_metacarpals", "thumb_distal", "index_proximal",
                              "index_distal", "middle_proximal", "middle_distal",
                              "ring_proximal", "ring_distal", "pinky_proximal", "pinky_distal")
    ]
    hand_links = [h for h in hand_links if h in SPHERES]
    for i, a in enumerate(hand_links):
        rest = hand_links[i + 1:]
        if rest:
            IGNORE.setdefault(a, []).extend(rest)

# ---- 输出 XRDF ----
L = ["# 由 gen_xrdf.py 从 l1_h0612 collision 网格自动生成（cuMotion 球生成器，Isaac Sim 6）。",
     "format: xrdf", "format_version: 2.0", "", "default_joint_positions:"]
L += [f"  {j}: 0.0" for j in HAND_JOINTS]
L += ["", "cspace:", "  joint_names:"]
L += [f'    - "{j}"' for j in CONTROLLED]
L += ["  acceleration_limits: [" + ", ".join(str(ACCEL[j]) for j in CONTROLLED) + "]",
      "  jerk_limits: [" + ", ".join(str(JERK[j]) for j in CONTROLLED) + "]", "",
      'tool_frames: ["lh_hand_base_link", "rh_hand_base_link"]', "",
      "world_collision:", '  geometry: "robot_collision_spheres"', "",
      "self_collision:", '  geometry: "robot_collision_spheres"', "  ignore:"]
for k, v in IGNORE.items():
    L.append(f"    {k}: [ " + ", ".join(f'"{x}"' for x in v) + " ]")
L += ["", "geometry:", "  robot_collision_spheres:", "    spheres:"]
for link, spheres in SPHERES.items():
    L.append(f"      {link}:")
    for c, r in spheres:
        L.append(f"        - center: [ {c[0]}, {c[1]}, {c[2]} ]")
        L.append(f"          radius: {r}")

open(OUT, "w").write("\n".join(L) + "\n")
nsph = sum(len(s) for s in SPHERES.values())
print(f"已生成 {OUT}: {len(CONTROLLED)} 受控关节, {len(HAND_JOINTS)} 锁定手指关节, "
      f"{len(SPHERES)} 连杆 / {nsph} 个碰撞球（源自 collision 网格）")
