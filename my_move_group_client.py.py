#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    PlanningOptions,
    Constraints,
    JointConstraint,
    WorkspaceParameters,
)


class MyMoveGroupClient(Node):
    def __init__(self):
        super().__init__('my_move_group_client')
        self._client = ActionClient(self, MoveGroup, 'move_action')

    def send_joint_goal(self, joint_names, joint_values, plan_only=True):
        self.get_logger().info('等待 move_group 的 /move_action 上线...')
        self._client.wait_for_server()

        goal_msg = MoveGroup.Goal()

        # ---- 1. MotionPlanRequest:我想要什么 ----
        req = MotionPlanRequest()
        req.group_name = 'nova5_group'          # 对应 SRDF 里 <group name="nova5_group">
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3    # 先保守点,30% 速度
        req.max_acceleration_scaling_factor = 0.3

        ws = WorkspaceParameters()
        ws.header.frame_id = 'base_link'
        ws.min_corner.x, ws.min_corner.y, ws.min_corner.z = -1.0, -1.0, -1.0
        ws.max_corner.x, ws.max_corner.y, ws.max_corner.z = 1.0, 1.0, 1.0
        req.workspace_parameters = ws

        req.start_state.is_diff = True   # 起点 = 当前实际状态

        goal_constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        req.goal_constraints.append(goal_constraints)

        goal_msg.request = req

        # ---- 2. PlanningOptions:算完之后怎么办 ----
        opts = PlanningOptions()
        opts.plan_only = plan_only
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        goal_msg.planning_options = opts

        self.get_logger().info(f'发送目标: {dict(zip(joint_names, joint_values))}')
        send_goal_future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('目标被 move_group 拒绝了')
            return

        self.get_logger().info('目标已接受,等待结果...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        self.get_logger().info(f'error_code.val = {result.error_code.val}  (1 表示 SUCCESS)')


def main():
    rclpy.init()
    node = MyMoveGroupClient()

    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    # 直接用 SRDF 里现成的 "home" 姿态当目标,方便你核对结果对不对
    home_values = [0.0, 0.5378, -1.1869, 0.0, 1.7785, 0.0]
    test_values = [0.3, 0.2, -0.8, 0.1, 1.0, -0.2]  # 随便给的,幅度别太夸张就行
    node.send_joint_goal(joint_names, test_values, plan_only=False)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()