from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    # Define where the package and files are
    pkg_marl_bot = get_package_share_directory('marl_bot')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_marl_bot, 'worlds', 'env1_1.world'),
        description='Path to the Gazebo world file'
    )

    # Include multi robot simulation launch (spawns robots in Gazebo)
    multi_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_marl_bot, 'sim_multi_robot_launch', 'multi_launch_sim.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Include SLAM launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_marl_bot, 'sim_multi_robot_launch', 'slam.launch.py')
        )
    )

    # Include Joystick launch
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_marl_bot, 'sim_multi_robot_launch', 'multi_joystick.launch.py')
        )
    )

    return LaunchDescription([
        world_arg,
        multi_sim_launch,
        slam_launch,
        joystick_launch,
    ])
