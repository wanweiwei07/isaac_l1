"""l1_h0602 双臂 cuMotion 无碰撞规划 + 物理执行（VS Code 插件版，发到运行中的 Isaac Sim）。

前置：
  1. start_isaacsim.sh 起 GUI（127.0.0.1:8226 代码服务器就绪）。
  2. 关节已加 position drive（见 Physics.usda），先发 main.py 加载机器人。
  3. 本文件 Ctrl+Enter 发送：规划 q_init->q_target 的无碰撞轨迹，Play 时间线，
     每帧用 set_dof_position_targets 把 PD drive 驱到轨迹目标（真·动力学执行，
     带对外碰撞——可接着做抓取/擦拭）。再发一次重规划。

设计：
  - 规划纯 cuMotion，不依赖物理 view（基座位姿从 USD 读），robot_joint_space=受控关节。
  - 执行回调惰性获取 articulation（等 Play 后物理 view 就绪再拿，带重试），
    避开 stopped->play 句柄失效。
"""
import builtins
import os

import numpy as np
import warp as wp
import omni.kit.app
import omni.timeline
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.cumotion")
from isaacsim.robot_motion.cumotion import (  # noqa: E402
    CumotionWorldInterface,
    GraphBasedMotionPlanner,
    load_cumotion_robot,
)
from pxr import Usd, UsdGeom, UsdPhysics  # noqa: E402

PROJ = "/home/wrs/Workspace/isaac_linxl1/l1_h0602"
USD_PATH = os.path.join(PROJ, "usd", "l1_h0612.usda")
CFG_DIR = os.path.join(PROJ, "cumotion")
ROBOT_ROOT = "/World/l1_h0602"
_REG = "_plan_dual_arm_sub"
DT = 1.0 / 60.0
# drive 增益已烤进 Physics.usda（stiffness=160000/damping=1000/maxForce=1e6），加载即带，无需运行时设。

stage = omni.usd.get_context().get_stage()

# ---- 机器人没加载就挂上 ----
if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")
    add_reference_to_stage(USD_PATH, ROBOT_ROOT)
    print(f"机器人未加载，已挂到 {ROBOT_ROOT}")
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

# ---- 注销上一次的执行回调 ----
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass
    delattr(builtins, _REG)

# ---- 加载 cuMotion 配置（纯 cuMotion，不碰物理 view）----
robot = load_cumotion_robot(CFG_DIR, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf")
controlled = list(robot.controlled_joint_names)
print("受控关节:", controlled)

# ---- 起止位形（cspace 顺序: [0]waist [1-6]左臂 [7-12]右臂 [13-14]颈）----
q_init = robot.robot_description.default_cspace_configuration().copy()
q_target = q_init.copy()
q_target[2] = 0.8    # left_arm_joint_2
q_target[3] = -1.0   # left_arm_joint_3
q_target[8] = -0.8   # right_arm_joint_2
q_target[9] = 1.0    # right_arm_joint_3

# ---- 基座(base_link)世界位姿（从 USD 读，喂给 world interface）----
base_prim = next((p for p in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_ROOT))
                  if p.GetName() == "base_link"), stage.GetPrimAtPath(ROBOT_ROOT))
m = UsdGeom.XformCache().GetLocalToWorldTransform(base_prim)
t, q = m.ExtractTranslation(), m.ExtractRotationQuat()
pos_wp = wp.array([[float(t[0]), float(t[1]), float(t[2])]], dtype=wp.float32)
quat_wp = wp.array([[float(q.GetReal()), *(float(x) for x in q.GetImaginary())]], dtype=wp.float32)

world_interface = CumotionWorldInterface()
world_interface.update_world_to_robot_root_transforms(poses=(pos_wp, quat_wp))
planner = GraphBasedMotionPlanner(
    cumotion_robot=robot, cumotion_world_interface=world_interface)

# ---- 规划 ----
print("规划中 (15-DOF 无碰撞双臂)...")
path = planner.plan_to_cspace_target(q_init, q_target)
if path is None:
    raise SystemExit("❌ 规划失败：检查起止位形是否越限/自碰撞。")

trajectory = path.to_minimal_time_joint_trajectory(
    max_velocities=np.full(15, 2.0),
    max_accelerations=np.full(15, 2.0),
    robot_joint_space=controlled,   # 轨迹直接在 15 个受控关节空间
    active_joints=controlled,
)
print(f"✅ 规划成功，轨迹时长 {trajectory.duration:.2f}s。")

# ---- Play 时间线（物理步进，drive 才会动作）----
omni.timeline.get_timeline_interface().play()

# ---- 逐帧把 PD drive 目标设到轨迹上；播完停 1s 再循环（存 builtins 防 GC）----
_exec = {"t": 0.0, "hold": 0.0, "art": None, "idx": None}


def _on_update(e):
    # 惰性获取 articulation：等 Play 后物理 view 就绪再拿（带重试）
    if _exec["art"] is None:
        try:
            art = Articulation(ROBOT_ROOT)
            _exec["idx"] = art.get_dof_indices(controlled).numpy().flatten()
            _exec["art"] = art
            print("articulation 就绪，开始物理执行。")
        except Exception:
            return
    s = trajectory.get_target_state(min(_exec["t"], trajectory.duration))
    if s is not None and s.joints.positions is not None:
        # positions 顺序 = controlled，与 _exec["idx"] 对齐
        _exec["art"].set_dof_position_targets(s.joints.positions, dof_indices=_exec["idx"])
    _exec["t"] += DT
    if _exec["t"] > trajectory.duration:
        _exec["hold"] += DT
        if _exec["hold"] > 1.0:
            _exec["t"], _exec["hold"] = 0.0, 0.0


sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _on_update, name="plan_dual_arm")
setattr(builtins, _REG, sub)
print("已 Play 并注册执行回调。drive 会把关节驱到轨迹目标（带对外碰撞）。再发一次重规划。")
