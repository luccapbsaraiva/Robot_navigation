import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():

    world_file_name = 'world.world'
    xacro_file = "robot.urdf.xacro"
    rviz_config_file = "robot_vis.rviz"
    description_package_name = "robot_description"
    description_package_path = os.path.join(get_package_share_directory(description_package_name))
    xacro_file_path = os.path.join(description_package_path, 'urdf', xacro_file)

    # convert XACRO file into URDF
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'world': os.path.join(description_package_path, 'world', world_file_name)}.items()
             )
    
    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('navigation'),
                        'launch',
                        'lidar_node.launch.py')])
            )

    # Runs the robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(description_package_name), 'rviz', rviz_config_file)
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir]
        )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'rm', '-x', '-1.0', '-y', '0.0', '-z', '0.2',
                                   '-topic', 'robot_description'],
                        output='screen')
    

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        rviz_node,
        spawn_entity,
        lidar_launch

    ])




'''
    lidar_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('navigation'),
                        'launch',
                        'lidar_node.launch.py')])
            )
'''
    