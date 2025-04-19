import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')  # Add namespace

    pkg_path = os.path.join(get_package_share_directory('marl_bot'))
    xacro_file = os.path.join(pkg_path, 'multi_robot_description', 'robot.urdf.xacro')

    robot_description = Command([
        'xacro ', xacro_file,
        ' namespace:=', namespace
    ])

    # Create a robot_state_publisher node with namespace
    params = {
        'robot_description': robot_description,
        'use_sim_time': use_sim_time
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,  # Apply namespace
        output='screen',
        parameters=[params]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the robot'),

        node_robot_state_publisher
    ])
