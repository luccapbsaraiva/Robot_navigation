import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
  
    Rotation_node = Node(
        package = 'navigation',
        executable = 'rotation'
    )


    
    return launch.LaunchDescription([
        #main_node,
        Rotation_node
    ])


'''
    main_node = Node(
        package = 'navigation',
        executable ='main',
    )
    '''