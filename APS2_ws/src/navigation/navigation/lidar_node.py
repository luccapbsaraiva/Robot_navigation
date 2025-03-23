import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np



 
class Main_node(Node):
    def __init__(self):
        super().__init__('Main_node')
        self.get_logger().info('Main node inicializado')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.subscription_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    

    def subscription_callback(self, msg):
        msg_twist = Twist()
        dist_min = 0.45
        start_idx = int(((-0.785 - msg.angle_min) / msg.angle_increment))
        end_idx = int(((0.785 - msg.angle_min) / msg.angle_increment))
        scan_45deg = msg.ranges[start_idx:end_idx]
        dist = np.min(scan_45deg)
        if dist > dist_min:
            flag = True
            msg_twist.linear.x = 0.1
            self.publisher.publish(msg_twist)
            self.get_logger().info(f'Movendo... | parede a:{dist:.2f} m' )
        else:
            msg_twist.linear.x = 0.0
            self.publisher.publish(msg_twist)
            self.get_logger().info(f'Obstáculo a: {dist:.2f} m')
        self.get_logger().info(f'Frente: {scan_45deg}')


def main(args=None):
    rclpy.init(args=args)
    main_node = Main_node()
    rclpy.spin(main_node)
    rclpy.shutdown()