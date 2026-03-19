#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointStateRelay(Node):
    def __init__(self):
        super().__init__('joint_state_relay')
        self.pub = self.create_publisher(JointState, '/joint_states_clean', 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.get_logger().info('Joint state relay running...')

    def _cb(self, msg):
        clean = JointState()
        clean.header = msg.header
        clean.header.frame_id = 'base'
        clean.name = msg.name
        clean.position = [0.0 if math.isnan(p) else p for p in msg.position]
        clean.velocity = [0.0] * len(msg.name)
        clean.effort   = [0.0] * len(msg.name)
        self.pub.publish(clean)

def main():
    rclpy.init()
    rclpy.spin(JointStateRelay())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
