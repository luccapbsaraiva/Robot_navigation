import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.service import Service
from std_srvs.srv import Empty
from turtlesim.action import RotateAbsolute
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import numpy as np



class Main_node(Node):
    def __init__(self):
        super().__init__('Main_node')
        self.get_logger().info('Main node inicializado')

        self.odom_cb_group = MutuallyExclusiveCallbackGroup()
        self.move_cb_group = MutuallyExclusiveCallbackGroup()

        self.lidar_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10, callback_group=self.odom_cb_group)
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.start_service = self.create_service(Empty, 'start_navigation', self.start_navigation_callback)
        self.action_client = ActionClient(self, RotateAbsolute, 'rotate_action')

        
        
        self.lim_obstacle = float('inf')
        self.waypoints = [
            {'yaw': 4.70, 'dist': 1.5},
            {'yaw': 3.14, 'dist': 8.15},
            {'yaw': 1.57, 'dist': 1.65},
            {'yaw': 3.14, 'dist': 2.8},
            {'yaw': 1.57, 'dist': 1.9},
            {'yaw': 0.0, 'dist': 2.75},
            {'yaw': 4.70, 'dist': 7.0}



        ]
        self.current_waypoint = 0
        self.navigation_active = False

    
    def scan_callback(self, msg):

        start_idx = int(((-0.1 - msg.angle_min) / msg.angle_increment))
        end_idx = int(((0.1 - msg.angle_min) / msg.angle_increment))
        scan_front = msg.ranges[start_idx:end_idx]
        self.lim_obstacle = np.max(scan_front)
        
    
    def odom_callback(self, msg):
        position = msg.pose.pose.position
        #self.current_x = position.x
        #self.current_y = position.y
        
            
        
    def start_navigation_callback(self, request, response):
        self.get_logger().info('Iniciando navegação...')
        self.navigation_active = True
        self.get_logger().info('Navegação iniciada.')
        self.send_rotate_goal(self.waypoints[self.current_waypoint]['yaw'])
        return response
    


    
    def send_rotate_goal(self, target_yaw):
        if not self.navigation_active:
            return
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = target_yaw
        self.action_client.wait_for_server()
        self.get_logger().info(f"Requisitando rotação para: {target_yaw:.2f} rad...")

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.rotation_done_callback)



    
    def rotation_done_callback(self, future):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info("Target aceito")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.start_moving)

        else:
            self.get_logger().info("Target rejeitado.")




    def start_moving(self, _):
        if not self.navigation_active:
            return
        
        self.get_logger().info("Rotação completa.")
        twist = Twist()
        while True:
            self.get_logger().info(f'Obstáculo a {self.lim_obstacle:.2f}m')

            if self.lim_obstacle <= self.waypoints[self.current_waypoint]['dist']:
                break

            twist.linear.x = 0.4
            self.vel_publisher.publish(twist)

            
        twist = Twist()
        twist.linear.x = 0.0
        self.vel_publisher.publish(twist)

        self.get_logger().info("Movimento concluído.")
        self.current_waypoint += 1

        if self.current_waypoint < len(self.waypoints):
            self.send_rotate_goal(self.waypoints[self.current_waypoint]['yaw'])

        else:
            self.get_logger().info("Navegação completa.")
            self.navigation_active = False





def main(args=None):
    rclpy.init(args=args)
    main_node = Main_node()
    executor = MultiThreadedExecutor()
    executor.add_node(main_node)
    executor.spin()
    rclpy.shutdown()



