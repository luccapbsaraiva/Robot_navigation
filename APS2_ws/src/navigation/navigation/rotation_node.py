import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import math


 
class Rotation(Node):
    def __init__(self):
        super().__init__('Rotation')
        self.get_logger().info('Rotation node inicializado')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.corr_yaw = 0.0
        

    

    def odometry_callback(self, msg):
        angle_twist = Twist()
        atitude = msg.pose.pose.orientation
        position_x = msg.pose.pose.position.x
        position_y = msg.pose.pose.position.y
        z = atitude.z
        w = atitude.w
        yaw = 2*math.atan2(z, w)

        if yaw < 0:
            self.corr_yaw = 2*math.pi - (-1*yaw)
        else:
            self.corr_yaw = yaw

        if self.corr_yaw < math.pi:
            angle_twist.angular.z = 0.3
            self.publisher.publish(angle_twist)
        else:
            angle_twist.angular.z = 0.0
            self.publisher.publish(angle_twist)

        self.get_logger().info(f"X: {position_x:.2f} | Y: {position_y:.2f}")
        


def main(args=None):
    rclpy.init(args=args)
    main_node = Rotation()
    rclpy.spin(main_node)
    rclpy.shutdown()

