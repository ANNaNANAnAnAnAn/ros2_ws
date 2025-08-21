import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

class AOANode(Node):
    def __init__(self):
        super().__init__('aoa_node')
        self.publisher_ = self.create_publisher(Float32, 'aoa_angle', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = serial.Serial( '/dev/ttyUSB0', 1000000, 
                                 parity = serial.PARITY_NONE, 
                                 stopbits = serial.STOPBITS_ONE,
                                 bytesize = serial.EIGHTBITS,
                                 rtscts = True,
                                 timeout = 10 )
    
    def read_aoa(self):
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if len(line) > 60:
                parts = line.split(',')
                try:
                    azimuth = int(parts[2])
                    return azimuth
                except(IndexError, ValueError):
                    return 0
        return 0
    
    def timer_callback(self):
        angle = self.read_aoa()
        #later can create function for filterng the angles
        msg = Float32()
        msg.data = angle 
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AOANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




