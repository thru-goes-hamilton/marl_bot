#!/usr/bin/env python3

import os
import tempfile

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    return LaunchDescription([
        OpaqueFunction(function=launch_setup, args=[namespace])
    ])


def launch_setup(context, namespace_subst):
    namespace = namespace_subst.perform(context)

    # ensure our shared /tmp/params folder exists
    tmp_params_dir = os.path.join(tempfile.gettempdir(), 'params')
    os.makedirs(tmp_params_dir, exist_ok=True)

    # 1) read template
    pkg = get_package_share_directory('marl_bot')
    template_path = os.path.join(pkg, 'params', 'rpm_template.yaml')
    with open(template_path, 'r') as f:
        template = f.read()

    # 2) substitute namespace
    config_yaml = template.replace('{namespace}', namespace)

    # 3) write only if missing
    out_path = os.path.join(tmp_params_dir, f'{namespace}_rpm_params.yaml')
    if not os.path.exists(out_path):
        with open(out_path, 'w') as f:
            f.write(config_yaml)

    # 4) launch converter node
    rpm_node = Node(
        package='marl_bot',
        executable='rpm_node',     # same executable
        name='rpm_node',
        namespace=namespace,
        parameters=[out_path],
        output='screen'
    )

    return [rpm_node]
