#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
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
    use_mock_px4 = LaunchConfiguration('use_mock_px4')
    use_keyboard = LaunchConfiguration('use_keyboard')
    record_bag = LaunchConfiguration('record_bag')
    fsm_params = Path(get_package_share_directory('uav_offboard_fsm')) / 'config' / 'uav_offboard_fsm.yaml'

    topics_to_record = [
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

    fsm = Node(
        package='uav_offboard_fsm',
        executable='uav_offboard_fsm_node',
        name='uav_offboard_fsm',
        output='screen',
        parameters=[str(fsm_params)],
    )

    keyboard = Node(
        package='uav_offboard_fsm',
        executable='uav_keyboard_control_node',
        name='uav_keyboard_control',
        output='screen',
        condition=IfCondition(use_keyboard),
    )

    mock_px4 = Node(
        package='traj_offboard',
        executable='mock_px4_sim_node',
        name='mock_px4_sim',
        output='screen',
        condition=IfCondition(use_mock_px4),
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'sqlite3',
            '--output', str(bag_dir / bag_name),
            *topics_to_record,
        ],
        output='screen',
        condition=IfCondition(record_bag),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_mock_px4',
            default_value='false',
            description='true to launch the lightweight PX4 topic simulator for local validation'),
        DeclareLaunchArgument(
            'use_keyboard',
            default_value='true',
            description='true to launch keyboard control publisher'),
        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='true to record trajectory and FSM topics'),
        mock_px4,
        offboard_bridge,
        traj_generator,
        fsm,
        keyboard,
        rosbag_record,
    ])
