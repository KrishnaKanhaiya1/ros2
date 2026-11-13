# Copyright 2026 Motor Control Sim Authors
# Licensed under Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('motor_control_sim')
    params_file = os.path.join(pkg_share, 'config', 'motor_params.yaml')

    use_test_publisher_arg = DeclareLaunchArgument(
        'use_test_publisher',
        default_value='true',
        description='Whether to start the cmd_vel test publisher node'
    )

    motor_simulator_node = Node(
        package='motor_control_sim',
        executable='motor_simulator',
        name='motor_simulator',
        output='screen',
        parameters=[params_file]
    )

    cmd_vel_publisher_node = Node(
        package='motor_control_sim',
        executable='cmd_vel_publisher',
        name='cmd_vel_publisher',
        output='screen',
        parameters=[params_file],
        condition=IfCondition(LaunchConfiguration('use_test_publisher'))
    )

    return LaunchDescription([
        use_test_publisher_arg,
        motor_simulator_node,
        cmd_vel_publisher_node
    ])
