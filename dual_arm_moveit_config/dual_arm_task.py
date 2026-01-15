#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes

class DualArmTask(Node):
    def __init__(self):
        super().__init__('dual_arm_task_client')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')

    def send_goal(self):
        print(">>> 正在连接 MoveIt 动作服务器...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print("❌ 错误：MoveIt 服务器未响应！请检查 demo.launch.py 是否运行。")
            return

        # 1. 构建目标
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "dual_arms"
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5

        # 2. 设置目标姿态 (抓取姿态)
        # 对应 SRDF 顺序：左臂7个 + 右臂7个
        target_joints = [
            # 左臂 (伸手)
            0.5, -0.5, 0.0, -2.0, 0.0, 2.0, 0.8,
            # 右臂 (对称伸手)
            -0.5, -0.5, 0.0, -2.0, 0.0, 2.0, 0.8
        ]

        # 3. 添加约束
        constraints = Constraints()
        # 自动生成关节名
        joint_names =  [f"panda_left_joint{i}" for i in range(1, 8)] + \
                       [f"panda_right_joint{i}" for i in range(1, 8)]

        for i, name in enumerate(joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(target_joints[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        # 4. 发送
        print(">>> 正在发送双臂运动指令...")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print(">>> ❌ MoveIt 拒绝了规划请求！")
            return

        print(">>> ✅ 规划请求已接受，正在执行...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            print(">>> 🎉🎉🎉 双臂运动执行成功！")
        else:
            print(f">>> ❌ 执行失败，错误码: {result.error_code.val}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    task = DualArmTask()
    task.send_goal()
    rclpy.spin(task)

if __name__ == '__main__':
    main()

