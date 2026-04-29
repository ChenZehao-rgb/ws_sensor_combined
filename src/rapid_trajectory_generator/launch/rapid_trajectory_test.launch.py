#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Compatibility launch kept under rapid_trajectory_generator, but it no
    # longer starts any rapid_trajectory_generator executables.
    default_bag_output = Path('bag') / f'traj_offboard_bag_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    default_bag_output.parent.mkdir(parents=True, exist_ok=True)
    bag_output = LaunchConfiguration('bag_output')
    use_mock_px4 = LaunchConfiguration('use_mock_px4')
    use_keyboard = LaunchConfiguration('use_keyboard')
    record_bag = LaunchConfiguration('record_bag')

    topics = [
        '/online_traj_generator/ruckig_state',
        '/online_traj_generator/ruckig_command',
        '/online_traj_generator/ruckig_targ',
        '/uav_offboard_fsm/status',
        '/uav_offboard_fsm/offboard_state',
        '/uav_offboard_fsm/control_command',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_imu',
        '/fmu/out/home_position',
        '/fmu/out/distance_sensor',
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
            'use_mock_px4',
            default_value='true',
            description='true to launch traj_offboard mock PX4 simulator'),
        DeclareLaunchArgument(
            'use_keyboard',
            default_value='true',
            description='true to launch keyboard control publisher'),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='true to record trajectory and FSM topics'),
        Node(
            package='traj_offboard',
            executable='mock_px4_sim_node',
            name='mock_px4_sim',
            output='screen',
            condition=IfCondition(use_mock_px4)),
        Node(
            package='traj_offboard',
            executable='offboard_control_bridge_node',
            name='offboard_control_bridge',
            output='screen'),
        Node(
            package='traj_offboard',
            executable='online_traj_node',
            name='online_traj_generator',
            output='screen'),
        Node(
            package='uav_offboard_fsm',
            executable='uav_offboard_fsm_node',
            name='uav_offboard_fsm',
            output='screen'),
        Node(
            package='uav_offboard_fsm',
            executable='uav_keyboard_control_node',
            name='uav_keyboard_control',
            output='screen',
            condition=IfCondition(use_keyboard)),
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '--storage', 'sqlite3',
                '-o', bag_output,
                *topics,
            ],
            output='screen',
            condition=IfCondition(record_bag)),
    ])
