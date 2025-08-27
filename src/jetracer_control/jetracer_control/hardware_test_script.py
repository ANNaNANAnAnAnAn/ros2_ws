#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
import time

class HardwareTestNode(Node):
    def __init__(self):
        super().__init__('hardware_test_script')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        time.sleep(1)  # Give publisher time to set up

    def run_test(self):
        cmds = [
            (0.0, 0.0),  # center
            (0.0, 1.0),  # right
            (0.0, -1.0), # left
            (0.0, 0.0),
            (0.5, 0.0),  # forward
            (-0.5, 0.0), # backward
            (0.0, 0.0)
        ]
        for throttle, steer in cmds:
            msg = Twist()
            msg.linear.x = throttle
            msg.angular.z = steer
            self.get_logger().info(f'Setting throttle: {throttle}, steering: {steer}')
            self.pub.publish(msg)
            time.sleep(2)
        self.get_logger().info('Hardware test complete.')

def main():
    rclpy.init()
    node = HardwareTestNode()
    node.run_test()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
