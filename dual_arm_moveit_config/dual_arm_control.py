#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint
from sensor_msgs.msg import JointState

class DualArmMover(Node):
    def __init__(self):
        super().__init__('dual_arm_mover')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        self.group_name = "dual_arms"

    def send_goal(self):
        goal_msg = MoveGroup.Goal()
        
        # 1. 设置规划组
        goal_msg.request.group_name = self.group_name
        
        # 2. 设置目标类型 (关节空间目标)
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.5
        goal_msg.request.max_acceleration_scaling_factor = 0.5
        
        # 3. 设定目标关节角度
        # 注意：这里需要手动指定关节名，防止顺序乱了
        # 下面是一个让双臂稍微弯曲的姿态
        constraints = Constraints()
        
        # 左臂关节
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_left_joint1", position=0.0, tolerance_above=0.01, tolerance_below=0.01))
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_left_joint2", position=-0.785, tolerance_above=0.01, tolerance_below=0.01))
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_left_joint4", position=-2.0, tolerance_above=0.01, tolerance_below=0.01)) # 弯曲一点
        
        # 右臂关节
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_right_joint1", position=0.0, tolerance_above=0.01, tolerance_below=0.01))
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_right_joint2", position=-0.785, tolerance_above=0.01, tolerance_below=0.01))
        constraints.joint_constraints.append(JointConstraint(joint_name="panda_right_joint4", position=-2.0, tolerance_above=0.01, tolerance_below=0.01))

        goal_msg.request.goal_constraints.append(constraints)
        
        # 4. 发送请求
        self.get_logger().info('正在等待 MoveGroup 动作服务器...')
        self._action_client.wait_for_server()
        
        self.get_logger().info('正在发送规划请求...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('规划请求被拒绝')
            return

        self.get_logger().info('规划请求被接受，正在执行...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        # error_code 1 表示成功 (SUCCESS)
        if result.error_code.val == 1:
            self.get_logger().info('执行成功！')
        else:
            self.get_logger().info(f'执行失败，错误码: {result.error_code.val}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    mover = DualArmMover()
    mover.send_goal()
    rclpy.spin(mover)

if __name__ == '__main__':
    main()