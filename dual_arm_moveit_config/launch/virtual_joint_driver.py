#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class VirtualJointDriver(Node):
    def __init__(self):
        super().__init__('virtual_joint_driver')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # 自然下垂姿态
        self.default_positions = {
            'panda_left_joint1': 0.0, 'panda_left_joint2': -0.785, 'panda_left_joint3': 0.0,
            'panda_left_joint4': -2.356, 'panda_left_joint5': 0.0, 'panda_left_joint6': 1.571,
            'panda_left_joint7': 0.785, 'panda_left_finger_joint1': 0.0, 'panda_left_finger_joint2': 0.0,
            'panda_right_joint1': 0.0, 'panda_right_joint2': -0.785, 'panda_right_joint3': 0.0,
            'panda_right_joint4': -2.356, 'panda_right_joint5': 0.0, 'panda_right_joint6': 1.571,
            'panda_right_joint7': 0.785, 'panda_right_finger_joint1': 0.0, 'panda_right_finger_joint2': 0.0,
        }
        print(">>> 虚拟驱动正在运行！正在向 /joint_states 发送数据...")

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.default_positions.keys())
        msg.position = list(self.default_positions.values())
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = VirtualJointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()