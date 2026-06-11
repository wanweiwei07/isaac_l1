"""双臂 cuMotion 无碰撞规划 + 执行（插件版，在运行中的 Isaac Sim 里跑）。

前置：./start_isaacsim.sh 起 GUI + main.py 已把机器人载入 /World/l1_h0602。
VS Code 打开本文件 Ctrl+Enter 发送：一次调用里 加载cuMotion -> 规划 -> 注册 update
回调逐帧执行轨迹（不自己开循环、不抢 Kit 主循环）。再发一次会重置并重新规划。

独立批处理版见 plan_dual_arm.py（自带 SimulationApp 循环，适合 headless/CI）。
"""
import builtins

import numpy as np
import omni.kit.app
import omni.usd
from pxr import UsdPhysics

from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.cumotion")
from isaacsim.robot_motion.cumotion import (  # noqa: E402
    CumotionWorldInterface,
    GraphBasedMotionPlanner,
    load_cumotion_robot,
)

ROBOT_ROOT = "/World/l1_h0602"
CFG_DIR = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/cumotion"

stage = omni.usd.get_context().get_stage()

# ---- 清掉上一次的执行回调（重复执行时）----
_REG = "_plan_dual_arm_sub"
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass

# ---- 物理：确保有 PhysicsScene 且 sim view 已初始化（不 play，纯运动学摆位）----
if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
    UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
if SimulationManager.get_physics_sim_view() is None:
    SimulationManager.initialize_physics()

# ---- cuMotion 配置 + articulation ----
robot = load_cumotion_robot(CFG_DIR, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf")
articulation = Articulation(ROBOT_ROOT)
controlled_idx = articulation.get_dof_indices(robot.controlled_joint_names).numpy().flatten()

# cspace 顺序: [0]waist [1-6]左臂 [7-12]右臂 [13-14]颈
q_init = robot.robot_description.default_cspace_configuration().copy()
q_target = q_init.copy()
q_target[2] = 0.8    # left_arm_joint_2
q_target[3] = -1.0   # left_arm_joint_3
q_target[8] = -0.8   # right_arm_joint_2
q_target[9] = 1.0    # right_arm_joint_3

articulation.set_dof_positions(q_init.astype(np.float32), dof_indices=controlled_idx)

# ---- 规划 ----
world_interface = CumotionWorldInterface()
pos, quat = articulation.get_world_poses()
world_interface.update_world_to_robot_root_transforms(poses=(pos, quat))
planner = GraphBasedMotionPlanner(cumotion_robot=robot, cumotion_world_interface=world_interface)

print("规划中 (15-DOF 无碰撞双臂)...")
path = planner.plan_to_cspace_target(q_init, q_target)
if path is None:
    raise RuntimeError("❌ 规划失败：检查起止位形是否越限/自碰撞")

trajectory = path.to_minimal_time_joint_trajectory(
    max_velocities=np.full(15, 2.0),
    max_accelerations=np.full(15, 2.0),
    robot_joint_space=articulation.dof_names,
    active_joints=robot.controlled_joint_names,
)
print(f"✅ 规划成功，轨迹时长 {trajectory.duration:.2f}s。回调循环播放中（再发一次重置）。")

# ---- 执行：注册 update 回调，逐帧推进轨迹（播完停 1s 回到起点循环）----
_state = {"t": 0.0, "hold": 0.0}
_DT = 1.0 / 60.0


def _on_update(e):
    if not articulation.is_physics_tensor_entity_valid():
        return
    s = trajectory.get_target_state(min(_state["t"], trajectory.duration))
    if s is not None and s.joints.positions is not None:
        articulation.set_dof_positions(s.joints.positions, dof_indices=s.joints.position_indices)
    _state["t"] += _DT
    if _state["t"] > trajectory.duration:
        _state["hold"] += _DT
        if _state["hold"] > 1.0:
            _state["t"], _state["hold"] = 0.0, 0.0


sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
    _on_update, name="plan_dual_arm_exec")
setattr(builtins, _REG, sub)
