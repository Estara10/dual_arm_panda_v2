#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

class DualArmMockHardware(Node):
    def __init__(self):
        super().__init__('dual_arm_mock_hardware')
        
        # 1. 关节状态发布者 (向 RViz 汇报当前状态)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # 2. 定义初始姿态 (自然下垂，无碰撞)
        self.current_joints = {
            'panda_left_joint1': 0.0, 'panda_left_joint2': -0.785, 'panda_left_joint3': 0.0,
            'panda_left_joint4': -2.356, 'panda_left_joint5': 0.0, 'panda_left_joint6': 1.571, 
            'panda_left_joint7': 0.785, 'panda_left_finger_joint1': 0.0, 'panda_left_finger_joint2': 0.0,
            'panda_right_joint1': 0.0, 'panda_right_joint2': -0.785, 'panda_right_joint3': 0.0,
            'panda_right_joint4': -2.356, 'panda_right_joint5': 0.0, 'panda_right_joint6': 1.571,
            'panda_right_joint7': 0.785, 'panda_right_finger_joint1': 0.0, 'panda_right_finger_joint2': 0.0,
        }
        
        # 3. 创建 Action Server (接收 MoveIt 的指令)
        # 左臂控制器监听
        self._left_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_left_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback_left,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        # 右臂控制器监听
        self._right_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/panda_right_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback_right,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # 4. 定时发布当前状态 (30Hz)
        self.timer = self.create_timer(0.033, self.timer_callback)
        self.get_logger().info(">>> 模拟硬件驱动已启动！等待 MoveIt 指令...")

    def goal_callback(self, goal_request):
        """接受所有目标请求"""
        self.get_logger().info("接收新的轨迹目标请求")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """处理取消请求"""
        self.get_logger().info("轨迹执行被取消")
        return CancelResponse.ACCEPT

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.current_joints.keys())
        msg.position = list(self.current_joints.values())
        self.joint_pub.publish(msg)

    async def execute_callback_left(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('收到左臂运动指令！')
        result = await self.execute_trajectory(goal_handle)
        goal_handle.succeed()  # 标记目标成功完成
        return result  # 返回规范的Result对象

    async def execute_callback_right(self, goal_handle: ServerGoalHandle):
        self.get_logger().info('收到右臂运动指令！')
        result = await self.execute_trajectory(goal_handle)
        goal_handle.succeed()  # 标记目标成功完成
        return result  # 返回规范的Result对象

    async def execute_trajectory(self, goal_handle: ServerGoalHandle):
        # 初始化结果对象
        result = FollowJointTrajectory.Result()
        result.error_code = result.SUCCESS  # 标记执行成功
        
        # 获取轨迹点和关节名
        trajectory_points = goal_handle.request.trajectory.points
        joint_names = goal_handle.request.trajectory.joint_names
        
        if not trajectory_points:
            self.get_logger().warn("空轨迹，直接返回成功")
            return result
        
        self.get_logger().info(f"开始执行轨迹，共 {len(trajectory_points)} 个点")
        
        # 初始化反馈对象
        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names
        
        # 遍历轨迹点并更新当前状态
        start_time = self.get_clock().now()
        for idx, point in enumerate(trajectory_points):
            # 模拟轨迹执行耗时（更贴近真实时间）
            time_from_start = point.time_from_start
            elapsed = self.get_clock().now() - start_time
            sleep_time = max(0.0, (time_from_start.sec + time_from_start.nanosec/1e9) - elapsed.nanoseconds/1e9)
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            # 更新当前关节状态
            for i, name in enumerate(joint_names):
                if name in self.current_joints:
                    self.current_joints[name] = point.positions[i]
            
            # 发送执行反馈（告诉MoveIt当前进度）
            feedback.current_position = [self.current_joints[name] for name in joint_names]
            goal_handle.publish_feedback(feedback)
            self.get_logger().debug(f"执行轨迹点 {idx+1}/{len(trajectory_points)}")

        self.get_logger().info('轨迹执行完毕')
        return result  # 返回规范的Result

def main(args=None):
    rclpy.init(args=args)
    node = DualArmMockHardware()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    