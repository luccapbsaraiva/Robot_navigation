import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.action import RotateAbsolute
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import math


 
class Rotation(Node):
    def __init__(self):
        super().__init__('Rotation')
        self.get_logger().info('Rotation node inicializado')

        self.odom_cb_group = MutuallyExclusiveCallbackGroup()
        self.execute_cb_group = MutuallyExclusiveCallbackGroup()

        self.subscription = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10, callback_group=self.odom_cb_group)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.action_server = ActionServer(self, RotateAbsolute, "rotate_action", execute_callback=self.rotate_callback, callback_group=self.execute_cb_group)
        self.current_yaw = 0.0

        
        

    

    def odometry_callback(self, msg):
        atitude = msg.pose.pose.orientation
        z = atitude.z
        w = atitude.w
        yaw = 2*math.atan2(z, w)
        if yaw < 0:
            self.current_yaw = 2*math.pi - (-1*yaw)
        else:
            self.current_yaw = yaw

        self.get_logger().info(f"Yaw: {self.current_yaw:.2f}")
        

    def rotate_callback(self, goal_handle):
        target_yaw = goal_handle.request.theta
        self.get_logger().info(f"Rotating to {target_yaw:.2f} radians")

        twist = Twist()
        if target_yaw - self.current_yaw < 0 and abs(self.current_yaw)<0.1:
            twist.angular.z =-0.5
        elif target_yaw - self.current_yaw < 0:
            twist.angular.z =-0.5
        else:
            twist.angular.z =0.5

        while abs(target_yaw - self.current_yaw) > 0.09:
            self.publisher.publish(twist)
            self.get_logger().info(f"Dif: {target_yaw - self.current_yaw}")
            goal_handle.publish_feedback(RotateAbsolute.Feedback())
            #rclpy.spin_once(self, timeout_sec=0.1)

    
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        
        goal_handle.succeed()
        self.get_logger().info("Rotation complete!")
        return RotateAbsolute.Result()



def main(args=None):
    rclpy.init(args=args)
    rotation_node = Rotation()
    executor = MultiThreadedExecutor()
    executor.add_node(rotation_node)
    executor.spin()
    #rotation_node.destroy_node()
    rclpy.shutdown()

