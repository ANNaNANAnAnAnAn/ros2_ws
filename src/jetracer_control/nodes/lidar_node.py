#!/usr/bin/env python3
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        # Use queue_size=1 to drop stale messages
        self.publisher_ = self.create_publisher(Float32, 'lidar_distance', 1)

        # 10 Hz timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Open serial with small timeout (non-blocking)
        self.ser = serial.Serial(
            '/dev/ttyUSB0', 115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            rtscts=True,
            timeout=0.01   # short timeout, so it won’t block long
        )

    def read_latest_lidar(self):
        """Flush serial buffer and return the latest valid distance"""
        distance = None
        while self.ser.in_waiting >= 9:  # keep reading until buffer is empty
            if self.ser.read() == b'\x59' and self.ser.read() == b'\x59':
                rest = self.ser.read(7)
                if len(rest) == 7:
                    dist_l = rest[0]
                    dist_h = rest[1]
                    distance = dist_h * 256 + dist_l
        return distance

    def timer_callback(self):
        distance = self.read_latest_lidar()
        if distance is None:
            self.get_logger().warn("No fresh LiDAR data")
            return

        msg = Float32()
        msg.data = float(distance)
        self.publisher_.publish(msg)

        # Debug log
        self.get_logger().info(f"LiDAR distance: {msg.data:.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
