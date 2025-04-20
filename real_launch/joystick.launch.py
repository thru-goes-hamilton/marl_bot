#!/usr/bin/env python3

import os
import tempfile

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    device_id = LaunchConfiguration('device_id')

    return LaunchDescription([
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('device_id', default_value='0'),
        OpaqueFunction(function=launch_setup, args=[namespace,device_id])
    ])


def launch_setup(context, namespace_subst, device_id_subst):
    namespace = namespace_subst.perform(context)
    device_id = device_id_subst.perform(context)

    # ensure shared /tmp/params folder
    tmp_params_dir = os.path.join(tempfile.gettempdir(), 'params')
    os.makedirs(tmp_params_dir, exist_ok=True)

    # 1) load template
    pkg = get_package_share_directory('marl_bot')
    template_path = os.path.join(pkg, 'params', 'joystick_template.yaml')
    with open(template_path, 'r') as f:
        template = f.read()

    # 2) place fully-qualified node names & device_id
    joy_key    = f'{namespace}/joy_node'
    teleop_key = f'{namespace}/teleop_node'
    config_yaml = (template
        .replace('{joy_node_name}',    joy_key)
        .replace('{teleop_node_name}', teleop_key)
        .replace('{device_id}',        device_id)
    )

    # 3) write if missing
    out_path = os.path.join(tmp_params_dir, f'{namespace}_joystick_params.yaml')
    if not os.path.exists(out_path):
        with open(out_path, 'w') as f:
            f.write(config_yaml)

    # 4) launch joy + teleop under same YAML
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        namespace=namespace,
        parameters=[out_path, {'use_sim_time': False}],
        output='screen'
    )
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        namespace=namespace,
        parameters=[out_path, {'use_sim_time': False}],
        output='screen'
    )

    return [joy_node, teleop_node]
