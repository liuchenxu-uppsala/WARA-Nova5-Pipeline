#!/usr/bin/env python3
"""【容器内运行,只需 MoveIt 启动,机械臂不会动】
同一个目标位姿 → (A) 自己的 IK(nova5_kinematics.py) 和 (B) MoveIt /compute_ik → 比较。

目标位姿怎么来:给 6 个关节角 q_true,用自己的 FK 算出 base_link→Link6 的位姿(保证可达)。
两边的 IK 都从 home 出发(初始猜测相同),各自求一组关节角。

比较两件事:
  1. 每一边的解代回 FK,是否回到目标位姿(这才是 IK 对不对的标准)
  2. 两边解出的关节角是否相同(IK 有多解,不同也不一定是错)

用法:
  python3 compare_ik_simple.py 0.3 0.2 -0.8 0.1 1.0 -0.2
  python3 compare_ik_simple.py            # 默认用上面这组
需要同目录有 nova5_model.py 和 nova5_kinematics.py
"""
import os
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nova5_model import fk, HOME
from nova5_kinematics import ik_numeric, ik_all, pose_error_vec, wrap_to_pi

JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GROUP = "nova5_group"          # SRDF 里的规划组名
TIP = "Link6"

q_true = np.array([float(x) for x in sys.argv[1:7]]) if len(sys.argv) >= 7 \
    else np.array([0.3, 0.2, -0.8, 0.1, 1.0, -0.2])
T_target = fk(q_true)
print("生成目标用的关节角 q_true:", np.round(q_true, 4).tolist())
print("目标位置 (mm):", np.round(T_target[:3, 3] * 1000, 3).tolist())


def err_mm_deg(q):
    e = pose_error_vec(T_target, fk(q))
    return np.linalg.norm(e[:3]) * 1000, np.degrees(np.linalg.norm(e[3:]))


# ---------------- (A) 自己的 IK,从 home 出发 ----------------
q_own, ok, it, _ = ik_numeric(T_target, HOME)
print(f"\n(A) 自己的 IK: 收敛={ok}, 迭代 {it} 次")

# ---------------- (B) MoveIt /compute_ik,也从 home 出发 ----------------
rclpy.init()
node = Node("compare_ik_simple")
cli = node.create_client(GetPositionIK, "/compute_ik")
if not cli.wait_for_service(timeout_sec=10.0):
    sys.exit("找不到 /compute_ik,MoveIt 启动了吗?")

req = GetPositionIK.Request()
r = req.ik_request
r.group_name = GROUP
r.ik_link_name = TIP
r.avoid_collisions = False                       # 只比运动学,不考虑碰撞
seed = JointState(name=JOINT_NAMES, position=[float(x) for x in HOME])
r.robot_state = RobotState(joint_state=seed)     # 初始猜测 = home
ps = PoseStamped()
ps.header.frame_id = "base_link"
ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = [float(v) for v in T_target[:3, 3]]
qx, qy, qz, qw = Rotation.from_matrix(T_target[:3, :3]).as_quat()   # scipy 顺序 x,y,z,w
ps.pose.orientation.x, ps.pose.orientation.y = float(qx), float(qy)
ps.pose.orientation.z, ps.pose.orientation.w = float(qz), float(qw)
r.pose_stamped = ps
r.timeout.sec = 1                                # 最多算 1 秒

fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
res = fut.result()
node.destroy_node()
rclpy.shutdown()

if res is None:
    sys.exit("/compute_ik 没有返回")
print(f"(B) MoveIt /compute_ik: error_code = {res.error_code.val}  (1 = 成功, -31 = 找不到解)")
q_mv = None
if res.error_code.val == 1:
    js = res.solution.joint_state
    pos = dict(zip(js.name, js.position))
    q_mv = np.array([pos[n] for n in JOINT_NAMES])

# ---------------- 比较 ----------------
print("\n关节角 (rad):")
print("  q_true(生成目标用)  ", np.round(q_true, 4).tolist())
print("  自己的 IK           ", np.round(q_own, 4).tolist())
if q_mv is not None:
    print("  MoveIt /compute_ik  ", np.round(q_mv, 4).tolist())

print("\n把解代回 FK,与目标位姿的差:")
dp, da = err_mm_deg(q_own)
print(f"  自己的 IK           位置 {dp:.4f} mm, 姿态 {da:.4f}°")
if q_mv is not None:
    dp, da = err_mm_deg(q_mv)
    print(f"  MoveIt /compute_ik  位置 {dp:.4f} mm, 姿态 {da:.4f}°")
    dq = np.abs(wrap_to_pi(q_own - q_mv))
    print(f"\n两边关节角之差(折到 ±π 后)最大 {dq.max():.4f} rad",
          "→ 同一组解" if dq.max() < 1e-3 else "→ 不同的解(IK 有多解,只要都回到目标位姿就都对)")

# ---------------- 附加:自己的多起点一共能找到几组解 ----------------
sols = ik_all(T_target, n_starts=20, seed=0)
print(f"\n自己的多起点 IK 一共找到 {len(sols)} 组解:")
for s in sols:
    tag = ""
    if q_mv is not None and np.abs(wrap_to_pi(s - q_mv)).max() < 1e-3:
        tag = "  ← 与 MoveIt 的解相同"
    print("  ", np.round(s, 4).tolist(), tag)
