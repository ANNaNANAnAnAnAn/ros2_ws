#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import time

class GamepadCmdVelNode(Node):
    def __init__(self):
        super().__init__('gamepad_cmd_vel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.update)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No gamepad detected!")
            exit(1)

        self.joy = pygame.joystick.Joystick(0)
        self.joy.init()
        self.get_logger().info(f"Gamepad connected: {self.joy.get_name()}")

    def update(self):
        pygame.event.pump()

        # Adjust axes as needed: axis 1 = left stick vertical, axis 2 = right stick horizontal
        throttle = -self.joy.get_axis(1)  # Invert: up = +1
        steering = self.joy.get_axis(2)

        # Optional deadzone
        if abs(throttle) < 0.1: throttle = 0.0
        if abs(steering) < 0.1: steering = 0.0

        msg = Twist()
        msg.linear.x = float(throttle)
        msg.angular.z = float(steering)

        self.pub.publish(msg)
        self.get_logger().info(f"Throttle: {throttle:.2f}, Steering: {steering:.2f}")

def main():
    rclpy.init()
    node = GamepadCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
