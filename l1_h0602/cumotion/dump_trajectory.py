"""规划同一条轨迹，按时间采样把 15-DOF 关节角序列打印成表格（纯文本，不画图、不动机器人停留）。

单独 Ctrl+Enter 即可。输出每行 = 一个时刻的 15 个受控关节角（rad）。
列顺序 = cspace: waist2 | 左臂1-6 | 右臂1-6 | neck1 neck2。改 K 调采样行数。
"""
import os

import numpy as np
import omni.usd
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.cumotion")
from isaacsim.robot_motion.cumotion import (  # noqa: E402
    CumotionWorldInterface,
    GraphBasedMotionPlanner,
    load_cumotion_robot,
)
from pxr import UsdGeom, UsdPhysics  # noqa: E402

K = 12   # 采样行数（含起止）

PROJ = "/home/wrs/Workspace/isaac_linxl1/l1_h0602"
USD_PATH = os.path.join(PROJ, "usd", "l1_h0612.usda")
CFG_DIR = os.path.join(PROJ, "cumotion")
ROBOT_ROOT = "/World/l1_h0602"

stage = omni.usd.get_context().get_stage()
if not stage.GetPrimAtPath(ROBOT_ROOT).IsValid():
    if not stage.GetPrimAtPath("/World").IsValid():
        UsdGeom.Xform.Define(stage, "/World")
    add_reference_to_stage(USD_PATH, ROBOT_ROOT)
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
if SimulationManager.get_physics_sim_view() is None:
    SimulationManager.initialize_physics()

articulation = Articulation(ROBOT_ROOT)
robot = load_cumotion_robot(CFG_DIR, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf")
controlled = list(robot.controlled_joint_names)
controlled_idx = articulation.get_dof_indices(controlled).numpy().flatten()


def _np(x):
    try:
        return np.asarray(x.numpy())
    except Exception:
        return np.asarray(x)


# ---- 起止位形 + 规划 ----
q_init = robot.robot_description.default_cspace_configuration().copy()
q_target = q_init.copy()
q_target[2] = 0.8
q_target[3] = -1.0
q_target[8] = -0.8
q_target[9] = 1.0
articulation.set_dof_positions(q_init.astype(np.float32), dof_indices=controlled_idx)

world_interface = CumotionWorldInterface()
pos, quat = articulation.get_world_poses()
world_interface.update_world_to_robot_root_transforms(poses=(pos, quat))
planner = GraphBasedMotionPlanner(
    cumotion_robot=robot, cumotion_world_interface=world_interface)

print("规划中...")
path = planner.plan_to_cspace_target(q_init, q_target)
if path is None:
    raise SystemExit("❌ 规划失败。")
trajectory = path.to_minimal_time_joint_trajectory(
    max_velocities=np.full(15, 2.0),
    max_accelerations=np.full(15, 2.0),
    robot_joint_space=articulation.dof_names,
    active_joints=controlled,
)

# ---- 短列名 ----
short = ["waist2"] + [f"L{i}" for i in range(1, 7)] + [f"R{i}" for i in range(1, 7)] + ["nk1", "nk2"]

print(f"\n轨迹时长 {trajectory.duration:.3f}s，采样 {K} 行（角度 rad）")
print("受控关节顺序:", controlled)
print("\n  t(s) | " + " ".join(f"{h:>7}" for h in short))
print("-" * (9 + 8 * len(short)))

ci = [int(j) for j in controlled_idx]
for k in range(K):
    t = (k / (K - 1) if K > 1 else 0.0) * trajectory.duration
    st = trajectory.get_target_state(min(t, trajectory.duration))
    p = _np(st.joints.positions)
    idx = [int(i) for i in _np(st.joints.position_indices)]
    val = {i: float(v) for i, v in zip(idx, p)}
    row = [val.get(j, float("nan")) for j in ci]
    print(f"{t:6.3f} | " + " ".join(f"{v:7.3f}" for v in row))

print("\n（首行≈q_init 起点，末行≈q_target 终点）")
