#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32


class AOANode(Node):
    def __init__(self):
        super().__init__('aoa_node')
        # Keep only the latest angle
        self.publisher_ = self.create_publisher(Float32, 'aoa_angle', 1)

        # Timer at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Open serial (short timeout!)
        self.ser = serial.Serial(
            '/dev/ttyUSB1', 1000000,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            rtscts=True,
            timeout=0.01  # non-blocking
        )

    def read_latest_aoa(self):
        """Flush serial buffer and return the latest valid angle"""
        angle = None
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if len(line) > 60:
                    parts = line.split(',')
                    azimuth = int(parts[2])
                    angle = azimuth
            except (IndexError, ValueError):
                continue
        return angle

    def timer_callback(self):
        angle = self.read_latest_aoa()
        if angle is None:
            self.get_logger().warn("No fresh AOA data")
            return

        msg = Float32()
        msg.data = float(angle)
        self.publisher_.publish(msg)

        # Debug log
        self.get_logger().info(f"AOA angle: {msg.data:.1f} deg")


def main(args=None):
    rclpy.init(args=args)
    node = AOANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
