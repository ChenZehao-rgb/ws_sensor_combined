#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def resolve_workspace_root(launch_file: Path) -> Path:
    for parent in launch_file.parents:
        if (parent / 'src').exists():
            return parent
    return launch_file.parent


def generate_launch_description() -> LaunchDescription:
    launch_file = Path(__file__).resolve()
    workspace_dir = resolve_workspace_root(launch_file)
    bag_dir = workspace_dir / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = f"online_traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}"

    topics_to_record = [
        '/online_traj_generator/ruckig_state',
        '/online_traj_generator/ruckig_command',
        '/online_traj_generator/ruckig_targ',
        '/fmu/out/vehicle_local_position',
        '/fmu/out/vehicle_attitude',
        '/fmu/out/vehicle_imu',
        '/fmu/out/home_position',
        '/fmu/in/trajectory_setpoint',
        '/fmu/in/offboard_control_mode',
        '/fmu/in/vehicle_command',
    ]

    offboard_bridge = Node(
        package='traj_offboard',
        executable='offboard_control_bridge_node',
        name='offboard_control_bridge',
        output='screen',
    )

    traj_generator = Node(
        package='traj_offboard',
        executable='online_traj_node',
        name='online_traj_generator',
        output='screen',
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--output', str(bag_dir / bag_name),
            *topics_to_record,
        ],
        output='screen',
    )

    return LaunchDescription([
        offboard_bridge,
        traj_generator,
        rosbag_record,
    ])
