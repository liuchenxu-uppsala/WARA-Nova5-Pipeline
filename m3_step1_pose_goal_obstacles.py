#!/usr/bin/env python3
"""M3 第 1 步:在 MoveIt 的规划场景里放桌子和障碍物,用【位姿目标】让机械臂绕开障碍物到达目标。

【容器内运行,需要 MoveIt 已启动】(Gazebo 可开可不开:
   - 只开 MoveIt demo:ros2 launch dobot_moveit moveit_demo.launch.py   → 只在 RViz 里看
   - 开 Gazebo + MoveIt:先 gazebo_moveit.launch.py,再 moveit_gazebo.launch.py → Gazebo 里真的动)

和之前 my_move_group_client.py 的区别:
   之前:目标是 6 个关节角(JointConstraint),IK 是我们自己在外面算的
   现在:目标是 Link6 的【位置 + 姿态】(PositionConstraint + OrientationConstraint),
        IK 由 MoveIt 在规划过程中自己做,并且会结合碰撞检查选一个不撞障碍物的解

用法:
  python3 m3_step1_pose_goal_obstacles.py                 # 加桌子+障碍物,规划并执行到默认目标
  python3 m3_step1_pose_goal_obstacles.py --no-obstacle   # 只加桌子,不加障碍物(用来对比路径)
  python3 m3_step1_pose_goal_obstacles.py --plan-only     # 只规划,不执行(RViz 里可看规划结果)
  python3 m3_step1_pose_goal_obstacles.py --home          # 回到 home(关节目标),方便重复实验
  python3 m3_step1_pose_goal_obstacles.py --clear         # 从场景里删掉桌子和障碍物
  python3 m3_step1_pose_goal_obstacles.py --target 0.45 0.15 0.30   # 自定义目标位置(m),姿态仍为竖直朝下

默认目标:Link6 原点到 (0.45, 0.15, 0.30) m,Link6 的 z 轴竖直朝下(为以后“夹爪从上往下抓”做准备)。
默认障碍物:中心 (0.50, 0.00, 0.70) m、尺寸 0.10 × 0.20 × 0.20 m 的方块。
   这个位置是用 nova5_model.py 的 FK 选出来的:离 home 姿态和目标姿态的连杆中心线都有 > 0.2 m 的距离,
   但如果在关节空间里从 home 直接“直线插值”到目标,手臂会穿过它 —— 所以规划器必须绕路。
"""
import argparse
import math
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, PlanningOptions, Constraints, JointConstraint,
    PositionConstraint, OrientationConstraint, WorkspaceParameters,
    CollisionObject, PlanningScene,
)
from moveit_msgs.srv import ApplyPlanningScene

GROUP = "nova5_group"
FRAME = "base_link"
EE_LINK = "Link6"                       # 还没装夹爪,先以 Link6(法兰)为末端
JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
HOME = [0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0]

DEFAULT_TARGET = (0.45, 0.15, 0.30)
# 竖直朝下:绕 base 的 x 轴转 180°。四元数 (x, y, z, w) = (1, 0, 0, 0)
# 对应旋转矩阵 [[1,0,0],[0,-1,0],[0,0,-1]],即 Link6 的 z 轴指向 base 的 −z
DOWN_QUAT = (1.0, 0.0, 0.0, 0.0)

# 场景物体:(id, 中心 xyz, 尺寸 xyz)
TABLE = ("table", (0.0, 0.0, -0.04), (1.2, 1.2, 0.02))          # 桌面顶在 z = -0.03,比底座低 3 cm,避免一开始就和底座“碰撞”
OBSTACLE = ("obstacle", (0.50, 0.00, 0.70), (0.10, 0.20, 0.20))


def make_pose(xyz, quat=(0.0, 0.0, 0.0, 1.0)):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, xyz)
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, quat)
    return p


class M3Step1(Node):
    def __init__(self):
        super().__init__("m3_step1_pose_goal_obstacles")
        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.scene_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    # ------------------------------------------------------------ ① 规划场景
    def apply_objects(self, objects, remove_ids=()):
        """把碰撞物体加进(或移出)move_group 的规划场景。"""
        if not self.scene_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("找不到 /apply_planning_scene,MoveIt 启动了吗?")
            return False
        scene = PlanningScene()
        scene.is_diff = True                      # 只是“增量修改”,不替换整个场景
        for oid, center, size in objects:
            co = CollisionObject()
            co.header.frame_id = FRAME            # 物体位置以 base_link 为参照
            co.id = oid
            co.pose = make_pose((0, 0, 0))        # 物体自身坐标系 = base_link
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [float(s) for s in size]      # 长 宽 高(m)
            co.primitives = [box]
            co.primitive_poses = [make_pose(center)]       # 方块中心
            co.operation = CollisionObject.ADD
            scene.world.collision_objects.append(co)
        for oid in remove_ids:
            co = CollisionObject()
            co.header.frame_id = FRAME
            co.id = oid
            co.operation = CollisionObject.REMOVE
            scene.world.collision_objects.append(co)
        fut = self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        ok = fut.result() is not None and fut.result().success
        self.get_logger().info(f"规划场景更新{'成功' if ok else '失败'}:"
                               f"加入 {[o[0] for o in objects]},移除 {list(remove_ids)}")
        return ok

    # ------------------------------------------------------------ ② 目标
    @staticmethod
    def pose_goal(xyz, quat, pos_tol=0.005, rot_tol=0.01):
        """位姿目标 = 位置约束(EE_LINK 的原点要落在一个半径 pos_tol 的小球里)
                    + 姿态约束(EE_LINK 的姿态与 quat 的偏差,绕三轴都不超过 rot_tol rad)"""
        pc = PositionConstraint()
        pc.header.frame_id = FRAME
        pc.link_name = EE_LINK
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        pc.constraint_region.primitives = [sphere]
        pc.constraint_region.primitive_poses = [make_pose(xyz)]
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = FRAME
        oc.link_name = EE_LINK
        oc.orientation = make_pose((0, 0, 0), quat).orientation
        oc.absolute_x_axis_tolerance = rot_tol
        oc.absolute_y_axis_tolerance = rot_tol
        oc.absolute_z_axis_tolerance = rot_tol
        oc.weight = 1.0

        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        return c

    @staticmethod
    def joint_goal(values, tol=0.01):
        c = Constraints()
        for n, v in zip(JOINT_NAMES, values):
            jc = JointConstraint()
            jc.joint_name, jc.position = n, float(v)
            jc.tolerance_above = jc.tolerance_below = tol
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return c

    # ------------------------------------------------------------ ③ 规划 / 执行
    def plan_and_execute(self, goal_constraints, plan_only=False, timeout=90.0):
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("30 秒内没等到 /move_action")
            return None
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10            # 多试几次,取最好的
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3
        ws = WorkspaceParameters()
        ws.header.frame_id = FRAME
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.5, -1.5, -0.5
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.5, 1.5, 1.5
        req.workspace_parameters = ws
        req.start_state.is_diff = True            # 从当前状态出发
        req.goal_constraints = [goal_constraints]

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = plan_only
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opts

        f = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f, timeout_sec=10.0)
        gh = f.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("目标被拒绝或没有回应")
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if not rf.done():
            self.get_logger().error(f"{timeout:.0f} 秒内没有结果,取消")
            gh.cancel_goal_async()
            return None
        return rf.result().result

    # ------------------------------------------------------------ ④ 检查
    def ee_pose_from_tf(self, settle=2.0):
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            t = self.tf_buffer.lookup_transform(FRAME, EE_LINK, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"读不到 TF {FRAME}→{EE_LINK}: {e}")
            return None
        tr, ro = t.transform.translation, t.transform.rotation
        return np.array([tr.x, tr.y, tr.z]), np.array([ro.x, ro.y, ro.z, ro.w])


def summarize(result, label):
    code = result.error_code.val
    print(f"\n===== {label} =====")
    print(f"error_code = {code}  (1 = SUCCESS, -1 = PLANNING_FAILED, -31 = NO_IK_SOLUTION)")
    print(f"规划耗时 = {result.planning_time:.3f} s")
    jt = result.planned_trajectory.joint_trajectory
    if not jt.points:
        return None
    names = list(jt.joint_names)
    Q = np.array([[dict(zip(names, p.positions))[n] for n in JOINT_NAMES] for p in jt.points])
    dur = jt.points[-1].time_from_start.sec + jt.points[-1].time_from_start.nanosec * 1e-9
    path_len = np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1))
    direct = np.linalg.norm(Q[-1] - Q[0])
    print(f"轨迹点数 = {len(Q)},轨迹时长 = {dur:.2f} s")
    print(f"关节空间路径长度 = {path_len:.3f} rad(起点到终点直线距离 {direct:.3f} rad,"
          f"比值 {path_len / max(direct, 1e-9):.2f};比值越大说明绕得越远)")
    print("起点关节角 =", np.round(Q[0], 4).tolist())
    print("终点关节角 =", np.round(Q[-1], 4).tolist(), "  ← MoveIt 选中的 IK 解")
    return Q[-1]


def compare_with_own_ik(q_final, target_xyz):
    """可选:如果同目录有 M2 的 nova5_model / nova5_kinematics,就把 MoveIt 的 IK 解和自己的多起点 IK 解对照。"""
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, here)
    try:
        from nova5_kinematics import ik_all, wrap_to_pi
    except ImportError:
        return
    T = np.eye(4)
    T[:3, :3] = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1.0]])
    T[:3, 3] = target_xyz
    sols = ik_all(T, n_starts=30, seed=0)
    print(f"\n自己的多起点 IK 对同一目标找到 {len(sols)} 组解:")
    for s in sols:
        same = np.max(np.abs(wrap_to_pi(s - q_final))) < 0.02
        print("  ", np.round(s, 3).tolist(), "  ← 与 MoveIt 选中的解相同" if same else "")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-obstacle", action="store_true")
    ap.add_argument("--plan-only", action="store_true")
    ap.add_argument("--home", action="store_true")
    ap.add_argument("--clear", action="store_true")
    ap.add_argument("--target", type=float, nargs=3, default=DEFAULT_TARGET)
    a = ap.parse_args()

    rclpy.init()
    node = M3Step1()
    try:
        if a.clear:
            node.apply_objects([], remove_ids=[TABLE[0], OBSTACLE[0]])
            return
        if a.home:
            r = node.plan_and_execute(node.joint_goal(HOME), plan_only=a.plan_only)
            if r is not None:
                summarize(r, "回到 home")
            return

        # ① 场景:桌子总是加;障碍物看参数
        if a.no_obstacle:
            node.apply_objects([TABLE], remove_ids=[OBSTACLE[0]])
        else:
            node.apply_objects([TABLE, OBSTACLE])

        # ② 目标:Link6 到 target,竖直朝下
        target = np.array(a.target, dtype=float)
        print(f"目标:{EE_LINK} 原点到 {target.tolist()} m,姿态竖直朝下(四元数 {DOWN_QUAT})")
        goal = node.pose_goal(target, DOWN_QUAT)

        # ③ 规划(并执行)
        r = node.plan_and_execute(goal, plan_only=a.plan_only)
        if r is None:
            return
        label = ("有障碍物" if not a.no_obstacle else "无障碍物") + ("(只规划)" if a.plan_only else "(规划并执行)")
        q_final = summarize(r, label)

        # ④ 检查:执行后从 TF 读实际末端位姿,与目标比较
        if not a.plan_only and r.error_code.val == 1:
            got = node.ee_pose_from_tf()
            if got is not None:
                p, q = got
                pos_err = np.linalg.norm(p - target) * 1000
                # 姿态差:两个四元数之间的夹角
                dot = abs(float(np.dot(q, np.array([DOWN_QUAT[0], DOWN_QUAT[1], DOWN_QUAT[2], DOWN_QUAT[3]]))))
                ang = math.degrees(2 * math.acos(min(1.0, dot)))
                print(f"\n执行后实际 {EE_LINK} 位置 = {np.round(p, 4).tolist()} m")
                print(f"与目标的差:位置 {pos_err:.2f} mm,姿态 {ang:.2f}°")
        if q_final is not None and r.error_code.val == 1:
            compare_with_own_ik(q_final, target)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
