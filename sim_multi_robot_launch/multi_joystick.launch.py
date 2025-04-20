from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = os.path.join(get_package_share_directory('marl_bot'),'params','joystick.yaml')

    # Joy node for Robot 1 with device_id set to 0
    joy_node_1 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node1',
        # parameters=[
        #     {'ros__parameters': {'device_id': 0, 'dev': '/dev/input/js0', 'use_sim_time': use_sim_time}},
        #     joy_params
        # ],
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('joy', 'joy1')]
    )

    # Joy node for Robot 2 with device_id set to 1
    joy_node_2 = Node(
        package='joy',
        executable='joy_node',
        name='joy_node2',
        # parameters=[
        #     {'ros__parameters': {'device_id': 1, 'dev': '/dev/input/js1', 'use_sim_time': use_sim_time}},
        #     joy_params
        # ],
        parameters=[joy_params, {'use_sim_time': use_sim_time}],
        remappings=[('joy', 'joy2')]
    )


    # Teleop node for Robot 1 (subscribes to joy1)
    teleop_node_1 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node1',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('joy', 'joy1'),
                        ('/cmd_vel', '/marl_bot1/cmd_vel')]
         )

    # Teleop node for Robot 2 (subscribes to joy2)
    teleop_node_2 = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node2',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('joy', 'joy2'),
                        ('/cmd_vel', '/marl_bot2/cmd_vel')]
         )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        joy_node_1,
        joy_node_2,
        teleop_node_1,
        teleop_node_2,
    ])


