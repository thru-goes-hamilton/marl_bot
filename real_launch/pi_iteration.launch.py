#!/usr/bin/env python3

import os
import tempfile

from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    device_id = LaunchConfiguration('device_id')

    return LaunchDescription([
        # 1) allow passing namespace / sim time / device id
        DeclareLaunchArgument('namespace'),
        DeclareLaunchArgument('device_id', default_value='0'),

        # 2) ensure /tmp/params exists before any templating
        OpaqueFunction(function=ensure_tmp_params_dir),

        # 3) now include everything under the same namespace
        OpaqueFunction(
            function=launch_setup,
            args=[namespace, device_id]
        )
    ])


def ensure_tmp_params_dir(context):
    """Create /tmp/params if it doesn't already exist."""
    tmp_params_dir = os.path.join(tempfile.gettempdir(), 'params')
    os.makedirs(tmp_params_dir, exist_ok=True)
    return []


def launch_setup(context, namespace_subst, device_id_subst):
    namespace = namespace_subst.perform(context)
    device_id = device_id_subst.perform(context)

    # ---- 1) YDLIDAR ----
    ydlidar_launch_path = os.path.join(
        get_package_share_directory('ydlidar_ros2_driver'),
        'launch',
        'ydlidar_launch.py'
    )
    ydlidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_path),
        launch_arguments={'namespace': namespace}.items()
    )

    # ---- 2) CMDVEL→RPM converter ----
    rpm_launch_path = os.path.join(
        get_package_share_directory('marl_bot'),
        'real_launch',
        'rpm.launch.py'           # using your rpm.launch.py here
    )
    rpm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rpm_launch_path),
        launch_arguments={'namespace': namespace}.items()
    )

    # ---- 3) JOYSTICK / TELEOP ----
    joy_launch_path = os.path.join(
        get_package_share_directory('marl_bot'),
        'real_launch',
        'joystick.launch.py'
    )
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joy_launch_path),
        launch_arguments={
            'namespace': namespace,
            'device_id': device_id
        }.items()
    )

    return [ydlidar, rpm, joystick]
