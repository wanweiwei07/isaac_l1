"""l1_h0602 双臂 cuMotion 无碰撞规划 + 执行（独立脚本，自带仿真循环）。

运行（机器人无关节 drive，所以用逐帧写关节位置的方式可视化轨迹）：
    source ~/.venvs/isaacsim/bin/activate
    python l1_h0602/cumotion/plan_dual_arm.py

流程：开 GUI -> 载入机器人 -> 初始化物理 -> load cuMotion 配置 ->
在 15 自由度 cspace 里规划从起始位形到目标位形的无碰撞路径（两臂自动互避，
锁定的手指当碰撞体）-> 转最小时间轨迹 -> 逐帧执行。
"""
import os

import numpy as np
from isaacsim import SimulationApp

# 设 PLAN_TEST=1：headless 跑、规划出轨迹就退出（用于快速验证整条管线）。
SELFTEST = os.environ.get("PLAN_TEST") == "1"
simulation_app = SimulationApp({"headless": SELFTEST})

# ---- Isaac Sim / cuMotion 模块要在 SimulationApp 之后导入 ----
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.experimental.utils.stage import add_reference_to_stage
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.cumotion")
from isaacsim.robot_motion.cumotion import (  # noqa: E402
    CumotionWorldInterface,
    GraphBasedMotionPlanner,
    load_cumotion_robot,
)
from pxr import UsdLux, UsdPhysics  # noqa: E402

ROOT = "/home/wrs/Workspace/isaac_linxl1/l1_h0602"
ROBOT_USD = os.path.join(ROOT, "usd", "l1_h0602.usda")
CFG_DIR = os.path.join(ROOT, "cumotion")
PRIM = "/World/l1_h0602"

# ---- 搭场景：载入机器人 + 补光 + 物理场景 ----
stage = stage_utils.get_current_stage()
add_reference_to_stage(ROBOT_USD, PRIM)
UsdLux.DomeLight.Define(stage, "/World/DomeLight").CreateIntensityAttr(1000.0)
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")

articulation = Articulation(PRIM)

# 让扩展/物理初始化（不 play，纯运动学摆位，避免无 drive 的关节在重力下下垂）
for _ in range(10):
    simulation_app.update()
if SimulationManager.get_physics_sim_view() is None:
    SimulationManager.initialize_physics()

# ---- 加载 cuMotion 配置 ----
robot = load_cumotion_robot(CFG_DIR, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf")
controlled_idx = articulation.get_dof_indices(robot.controlled_joint_names).numpy().flatten()
print("受控关节:", robot.controlled_joint_names)

# cspace 顺序: [0]waist [1-6]左臂 [7-12]右臂 [13-14]颈
q_init = robot.robot_description.default_cspace_configuration().copy()
q_target = q_init.copy()
q_target[2] = 0.8    # left_arm_joint_2
q_target[3] = -1.0   # left_arm_joint_3
q_target[8] = -0.8   # right_arm_joint_2
q_target[9] = 1.0    # right_arm_joint_3

# 把机器人摆到起始位形（运动学），并作为规划起点
articulation.set_dof_positions(q_init.astype(np.float32), dof_indices=controlled_idx)
simulation_app.update()

# ---- 规划 ----
world_interface = CumotionWorldInterface()
pos, quat = articulation.get_world_poses()
world_interface.update_world_to_robot_root_transforms(poses=(pos, quat))

planner = GraphBasedMotionPlanner(
    cumotion_robot=robot,
    cumotion_world_interface=world_interface,
)

print("规划中 (15-DOF 无碰撞双臂)...")
path = planner.plan_to_cspace_target(q_init, q_target)
if path is None:
    print("❌ 规划失败：检查起止位形是否越限/自碰撞")
    while simulation_app.is_running():
        simulation_app.update()
    simulation_app.close()
    raise SystemExit

trajectory = path.to_minimal_time_joint_trajectory(
    max_velocities=np.full(15, 2.0),
    max_accelerations=np.full(15, 2.0),
    robot_joint_space=articulation.dof_names,
    active_joints=robot.controlled_joint_names,
)
print(f"✅ 规划成功，轨迹时长 {trajectory.duration:.2f}s，"
      f"{trajectory.get_active_joints() and len(robot.controlled_joint_names)} 受控关节。")

if SELFTEST:
    print("PLAN_TEST 自检通过：加载 / 规划 / 轨迹生成全部 OK。退出。")
    simulation_app.close()
    raise SystemExit(0)

print("开始循环播放（关闭窗口退出）。")

# ---- 执行：逐帧写关节位置；播完停 1s 再从头循环 ----
t = 0.0
dt = 1.0 / 60.0
hold = 0.0
while simulation_app.is_running():
    state = trajectory.get_target_state(min(t, trajectory.duration))
    if state is not None and state.joints.positions is not None:
        articulation.set_dof_positions(state.joints.positions, dof_indices=state.joints.position_indices)
    t += dt
    if t > trajectory.duration:
        hold += dt
        if hold > 1.0:  # 停顿后回到起点重播
            t, hold = 0.0, 0.0
    simulation_app.update()

simulation_app.close()
