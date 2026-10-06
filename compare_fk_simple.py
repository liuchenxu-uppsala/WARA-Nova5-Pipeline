#!/usr/bin/env python3
"""【容器内运行,只需 MoveIt 启动,机械臂不会动】
同一组 6 个关节角 → (A) 自己的 FK(nova5_model.py) 和 (B) MoveIt /compute_fk → 比较 base_link→Link6。

用法: python3 compare_fk_simple.py 0.3 0.2 -0.8 0.1 1.0 -0.2
      python3 compare_fk_simple.py            # 默认用 home 姿态
"""
import os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nova5_model import fk, HOME

q = np.array([float(x) for x in sys.argv[1:7]]) if len(sys.argv) >= 7 else HOME
print("关节角 (rad):", np.round(q, 4).tolist())

# (A) 自己的 FK
T_own = fk(q, "moveit")

# (B) MoveIt /compute_fk
rclpy.init()
node = Node("compare_fk_simple")
cli = node.create_client(GetPositionFK, "/compute_fk")
if not cli.wait_for_service(timeout_sec=10.0):
    sys.exit("找不到 /compute_fk,MoveIt 启动了吗?")
req = GetPositionFK.Request()
req.header.frame_id = "base_link"
req.fk_link_names = ["Link6"]
js = JointState()
js.name = [f"joint{i}" for i in range(1, 7)]
js.position = [float(x) for x in q]
req.robot_state = RobotState(joint_state=js)
fut = cli.call_async(req)
rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
res = fut.result()
if res is None or res.error_code.val != 1:
    sys.exit(f"/compute_fk 失败: {res}")
p = res.pose_stamped[0].pose
T_mv = np.eye(4)
T_mv[:3, :3] = Rotation.from_quat([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
T_mv[:3, 3] = [p.position.x, p.position.y, p.position.z]

def show(name, T):
    print(f"{name:18s} x={T[0,3]*1000:9.3f}  y={T[1,3]*1000:9.3f}  z={T[2,3]*1000:9.3f} mm")

show("自己的 FK", T_own)
show("MoveIt /compute_fk", T_mv)
dpos = np.linalg.norm(T_own[:3, 3] - T_mv[:3, 3]) * 1000
dang = np.degrees(np.linalg.norm(Rotation.from_matrix(T_own[:3, :3].T @ T_mv[:3, :3]).as_rotvec()))
print(f"差: 位置 {dpos:.4f} mm, 姿态 {dang:.4f}°  ->", "一致" if dpos < 1 and dang < 0.1 else "不一致")
node.destroy_node(); rclpy.shutdown()
