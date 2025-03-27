import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.service import Service
from std_srvs.srv import Empty
from turtlesim.action import RotateAbsolute
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np



class Main_node(Node):
    def __init__(self):
        super().__init__('Main_node')
        self.get_logger().info('Main node inicializado')
        self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_service = self.create_service(Empty, 'start_navigation', self.start_navigation_callback)
        
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.closest_distance = float('inf')
        self.waypoints = [
            {'x': -2.0, 'y': -1.0, 'yaw': 4.71}
        ]
        self.current_waypoint = 0
        self.navigation_active = False

    

    def scan_callback(self, msg):
        #msg_twist = Twist()

        start_idx = int(((-0.785 - msg.angle_min) / msg.angle_increment))
        end_idx = int(((0.785 - msg.angle_min) / msg.angle_increment))
        scan_45deg = msg.ranges[start_idx:end_idx]
        dist = np.min(scan_45deg)
        #msg_twist.linear.x = 0.1
        #self.vel_publisher.publish(msg_twist)
        self.get_logger().info(f'Obstáculo a {dist:.2f}m')

    def odom_callback(self, msg):
        atitude = msg.pose.pose.orientation
        z = atitude.z
        w = atitude.w
        yaw = 2*math.atan2(z, w)
        if yaw < 0:
            self.current_yaw = 2*math.pi - (-1*yaw)
        else:
            self.current_yaw = yaw
        


    def start_navigation_callback(self, request, response):
        self.get_logger().info('Iniciando navegação...')
        self.navigation_active = True
        target_yaw = self.waypoints[self.current_waypoint]['yaw']
        yaw_dif = target_yaw-self.current_yaw
        while yaw_dif>0:
            self.get_logger().info(f'Yaw: {self.current_yaw:.2f}')
            yaw_twist = Twist()
            yaw_twist.angular.z = 0.3
            self.vel_publisher.publish(yaw_twist)

        yaw_twist = Twist()
        yaw_twist.angular.z = 0.0
        self.vel_publisher.publish(yaw_twist)
        self.get_logger().info('Rotação concluida.')
        #self.send_rotate_goal(self.waypoints[self.current_waypoint]['yaw'])
        #return response



def main(args=None):
    rclpy.init(args=args)
    main_node = Main_node()
    rclpy.spin(main_node)
    rclpy.shutdown()