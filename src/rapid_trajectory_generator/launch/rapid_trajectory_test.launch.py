#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Use a timestamped bag output under the bag/ directory so repeated launches
    # don't clash and all recordings stay grouped in one ignored folder.
    default_bag_output = Path('bag') / f'rapid_traj_bag_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    default_bag_output.parent.mkdir(parents=True, exist_ok=True)
    bag_output = LaunchConfiguration('bag_output')
    use_simulator = LaunchConfiguration('use_simulator')

    simulator_topics = [
        '/rapid_traj/current_state',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_imu',
        '/fmu/out/home_position',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/vehicle_command',
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'bag_output',
            default_value=str(default_bag_output),
            description='Output prefix for rosbag2 recording'),
        DeclareLaunchArgument(
            'use_simulator',
            default_value='true',
            description='true to launch rapid_trajectory_simulator_node, false to launch rapid_trajectory_test_node'),
        Node(
            package='rapid_trajectory_generator',
            executable='rapid_trajectory_simulator_node',
            name='rapid_trajectory_simulator_node',
            output='screen',
            condition=IfCondition(use_simulator)),
        Node(
            package='rapid_trajectory_generator',
            executable='rapid_trajectory_test_node',
            name='rapid_trajectory_test_node',
            output='screen',
            condition=UnlessCondition(use_simulator)),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--storage', 'sqlite3',
                '-o', bag_output,
                '/rapid_traj/current_state'
            ],
            output='screen',
            condition=UnlessCondition(use_simulator)),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--storage', 'sqlite3',
                '-o', bag_output,
                *simulator_topics
            ],
            output='screen',
            condition=IfCondition(use_simulator)),
    ])
