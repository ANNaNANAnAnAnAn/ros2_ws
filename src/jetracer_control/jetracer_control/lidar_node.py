import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Float32

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.publisher_ = self.create_publisher(Float32, 'lidar_distance', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.ser = serial.Serial('/dev/ttyTHS1', 115200, 
                                 parity = serial.PARITY_NONE, 
                                 stopbits = serial.STOPBITS_ONE,
                                 bytesize = serial.EIGHTBITS,
                                 rtscts = True,
                                 timeout = 10 )
        
    def read_lidar(self):
        if self.ser.in_waiting >9:
            if self.ser.read() == b'\x59':
                if self.ser.read() == b'\x59':
                    rest = self.ser.read(7)
                    if len(rest) == 7:
                
                        dist_l = rest[0]
                        dist_h = rest[1]
                        distance = dist_h*256 + dist_l
        
                        return distance
        return None
    
            
    def timer_callback(self):
        distance = self.read_lidar()
        #implement a small filter for the data since that 
        #is noisy data 
        if distance is not None:    
            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()











