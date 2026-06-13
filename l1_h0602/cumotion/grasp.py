"""灵巧手抓取控制（手指 drive 直接控；手指不在 cuMotion cspace 里，与手臂规划分开）。

手指 12 个非 mimic 驱动关节已加 drive；远端 *_ip/*_dip 是 mimic，自动跟随，表里不用管。
GRASP_TABLE 的短关节名 + 手前缀 = 全名（lh_/rh_）。

用法：机器人已加载 + Play（drive 在 USD 里，加载即带）。设好下面 HAND/GRASP/ACTION，
Ctrl+Enter 发送。
  ACTION="grasp"    预整形→(停顿)→闭合，整套（用 update 回调分两步）
  ACTION="preshape" 只摆预整形
  ACTION="close"    只闭合
  ACTION="open"     张开（6 个驱动关节回 0）
对应 IK 目标 frame：pinch/tripod → <hand>_pinch_tcp，power → <hand>_power_tcp。
"""
import builtins

import omni.kit.app
import omni.usd
from isaacsim.core.experimental.prims import Articulation

ROBOT_ROOT = "/World/l1_h0602"
_REG = "_grasp_sub"

# ---- 配置 ----
HAND = "rh"          # lh / rh
GRASP = "power"      # pinch / tripod / power
ACTION = "grasp"     # grasp / preshape / close / open
PRESHAPE_SETTLE = 40  # 预整形到闭合之间等待的帧数

GRASP_TABLE = {
    'pinch': {
        'preshape': {'thumb_cmc_yaw': 0.9},
        'closing': {'thumb_cmc_pitch': 0.45, 'index_mcp_pitch': 0.9},
        'pads': ('thumb', ['index']),
    },
    'tripod': {
        # 这只手重调(FK算)：yaw 0.9->1.2 把拇指扫到食指-中指中间让中指也够上
        # (拇指尖 y 0.028->0.018，d_middle 61->58mm)；pitch 0.45->0.55 多够一点。
        'preshape': {'thumb_cmc_yaw': 1.2},
        'closing': {'thumb_cmc_pitch': 0.55, 'index_mcp_pitch': 0.9,
                    'middle_mcp_pitch': 1.15},
        'pads': ('thumb', ['index', 'middle']),
    },
    'power': {   # 五指包络，非平行夹爪
        'preshape': {},
        'closing': {'thumb_cmc_yaw': 1.0, 'thumb_cmc_pitch': 0.4,
                    'index_mcp_pitch': 1.0, 'middle_mcp_pitch': 1.0,
                    'ring_mcp_pitch': 1.0, 'pinky_mcp_pitch': 1.0},
        'pads': None,
    },
}
# 抓取类型 -> 用哪个 TCP 当 IK 目标（tripod 复用 pinch_tcp）
TCP_FOR = {'pinch': 'pinch_tcp', 'tripod': 'tripod_tcp', 'power': 'power_tcp'}
FINGER_DRIVERS = ['thumb_cmc_yaw', 'thumb_cmc_pitch', 'index_mcp_pitch',
                  'middle_mcp_pitch', 'ring_mcp_pitch', 'pinky_mcp_pitch']


def tcp_frame(hand, gtype):
    """该抓取类型对应的 IK 目标 frame 全名，比如 rh_pinch_tcp。"""
    return f"{hand}_{TCP_FOR[gtype]}"


def apply_joints(art, hand, joint_dict):
    """把 {短名: 角度} 设成位置目标（drive 驱过去）。空 dict 跳过。"""
    if not joint_dict:
        return
    names = [f"{hand}_{k}" for k in joint_dict]
    idx = art.get_dof_indices(names).numpy().flatten()
    art.set_dof_position_targets(list(joint_dict.values()), dof_indices=idx)


def open_hand(art, hand):
    """张开：6 个驱动关节回 0（mimic 远端跟随）。"""
    names = [f"{hand}_{k}" for k in FINGER_DRIVERS]
    idx = art.get_dof_indices(names).numpy().flatten()
    art.set_dof_position_targets([0.0] * 6, dof_indices=idx)


# ---- 取 articulation（需 Play 后物理 view 就绪）----
try:
    art = Articulation(ROBOT_ROOT)
    art.get_dof_indices([f"{HAND}_index_mcp_pitch"])   # 探一下，确认可用
except Exception as e:
    raise SystemExit(f"articulation 未就绪（先 Play）：{e}")

# 注销上一次的抓取回调
if hasattr(builtins, _REG):
    try:
        getattr(builtins, _REG).unsubscribe()
    except Exception:
        pass
    delattr(builtins, _REG)

cfg = GRASP_TABLE[GRASP]

if ACTION == "open":
    open_hand(art, HAND)
    print(f"{HAND} 张开。")

elif ACTION == "preshape":
    apply_joints(art, HAND, cfg['preshape'])
    print(f"{HAND} {GRASP} 预整形。")

elif ACTION == "close":
    apply_joints(art, HAND, cfg['closing'])
    print(f"{HAND} {GRASP} 闭合。")

elif ACTION == "grasp":
    # 预整形 -> 等 PRESHAPE_SETTLE 帧 -> 闭合（update 回调分两步）
    apply_joints(art, HAND, cfg['preshape'])
    _st = {"n": 0}

    def _seq(e):
        _st["n"] += 1
        if _st["n"] == PRESHAPE_SETTLE:
            apply_joints(art, HAND, cfg['closing'])
        elif _st["n"] > PRESHAPE_SETTLE + 60:   # 闭合后再跑 1s 收尾
            try:
                getattr(builtins, _REG).unsubscribe()
            except Exception:
                pass
            if hasattr(builtins, _REG):
                delattr(builtins, _REG)

    sub = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
        _seq, name="grasp")
    setattr(builtins, _REG, sub)
    print(f"{HAND} {GRASP} 抓取中：预整形→闭合（IK 目标 frame = {tcp_frame(HAND, GRASP)}）。")

else:
    print(f"未知 ACTION={ACTION!r}")
