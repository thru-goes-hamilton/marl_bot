import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'marl_bot'
    robot_name = 'marl_bot'
    no_of_robots = 1

    world_file = os.path.join(get_package_share_directory('marl_bot'), 'worlds', 'env1.world')

    # Include robot_state_publisher for both robots
    rsp1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'sim_multi_robot_launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'namespace': f'{robot_name}{no_of_robots}'}.items()
    )

    rsp2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'sim_multi_robot_launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'namespace': 'marl_bot2'}.items()
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': world_file}.items()
    )

    # Spawn robots with unique names
    spawn_marl_bot1 = Node(package='gazebo_ros', executable='spawn_entity.py',
                           arguments=['-topic', '/marl_bot1/robot_description',
                                      '-entity', 'marl_bot1','-x','0','-y','0','-z','0','-robot_namespace', 'marl_bot1'],
                           output='screen')

    spawn_marl_bot2 = Node(package='gazebo_ros', executable='spawn_entity.py',
                           arguments=['-topic', '/marl_bot2/robot_description',
                                      '-entity', 'marl_bot2','-x','1.0','-y','1.0','-z','0', '-robot_namespace', 'marl_bot2'],
                           output='screen')

    return LaunchDescription([
        rsp1,
        rsp2,
        gazebo,
        spawn_marl_bot1,
        spawn_marl_bot2
    ])
