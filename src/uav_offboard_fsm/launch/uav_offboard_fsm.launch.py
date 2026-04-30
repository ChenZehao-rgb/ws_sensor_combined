#!/usr/bin/env python3

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params_file = (
        Path(get_package_share_directory('uav_offboard_fsm')) /
        'config' /
        'uav_offboard_fsm.yaml'
    )
    params_file = LaunchConfiguration('params_file')
    log_level = LaunchConfiguration('log_level')

    fsm = Node(
        package='uav_offboard_fsm',
        executable='uav_offboard_fsm_node',
        name='uav_offboard_fsm',
        output='screen',
        emulate_tty=True,
        parameters=[params_file],
        arguments=['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=str(default_params_file),
            description='YAML parameter file for uav_offboard_fsm_node'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for uav_offboard_fsm_node'),
        LogInfo(msg=['uav_offboard_fsm: params_file=', params_file]),
        fsm,
    ])
