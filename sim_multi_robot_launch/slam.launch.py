import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # Load the parameter file for SLAM
    params_dir = os.path.join(get_package_share_directory('marl_bot'), 'params')

    slam_params_file = os.path.join(params_dir, 'remap_mapper_params_online_async.yaml')
    
    slam_toolbox_robot1 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox1',
        namespace='marl_bot1',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'scan_topic': '/marl_bot1/scan'},  
            {'base_frame': 'marl_bot1/base_link'},
            {'odom_frame': 'marl_bot1/odom'},
            {'map_frame': 'marl_bot1/map'}       # Consistent naming without leading slash
        ],
        remappings=[
            ('/map', '/marl_bot1/map'),
            ('/map_metadata', '/marl_bot1/map_metadata'),
            ('/map_updates', '/marl_bot1/map_updates')            
        ]
    )

    slam_toolbox_robot2 = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox2',  # Optional: Consider unique name if needed
        namespace='marl_bot2',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            {'scan_topic': '/marl_bot2/scan'},          # Relative: resolves to /marl_bot2/scan
            {'base_frame': 'marl_bot2/base_link'},
            {'odom_frame': 'marl_bot2/odom'},
            {'map_frame': 'marl_bot2/map'}             # Consistent naming
        ],
        remappings=[
            ('/map', '/marl_bot2/map'),
            ('/map_metadata', '/marl_bot2/map_metadata'),
            ('/map_updates', '/marl_bot2/map_updates')
        ]
    )
    
    # Launch everything
    return LaunchDescription([
        declare_use_sim_time_cmd,
        slam_toolbox_robot1,
        slam_toolbox_robot2
    ])