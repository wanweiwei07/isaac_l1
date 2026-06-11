"""验证 l1_h0602 的 cuMotion 配置（robot.urdf + robot.xrdf）能否正确加载。

在运行中的 Isaac Sim 里用 VS Code 插件执行（Ctrl+Enter）。
通过则说明：关节名与 URDF 对得上、15 受控关节、tool frame、碰撞球都解析成功。
"""
import os

# cuMotion 扩展默认未必加载，先确保启用（它会把 bundled `cumotion` 包加进 sys.path）。
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.robot_motion.cumotion")

from isaacsim.robot_motion.cumotion import load_cumotion_robot

DIR = "/home/wrs/Workspace/isaac_linxl1/l1_h0602/cumotion"

robot = load_cumotion_robot(DIR, urdf_filename="robot.urdf", xrdf_filename="robot.xrdf")

cj = robot.controlled_joint_names
print("=" * 60)
print(f"✅ XRDF 加载成功")
print(f"受控关节数: {len(cj)}  (期望 15)")
for i, j in enumerate(cj):
    print(f"  [{i:2}] {j}")
print(f"末端 tool frames: {robot.robot_description.tool_frame_names()}")
print("=" * 60)
