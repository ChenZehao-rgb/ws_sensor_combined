#!/usr/bin/env python3

from datetime import datetime
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_workspace_root(launch_file: Path) -> Path:
    for parent in launch_file.parents:
        if (parent / 'src').exists():
            return parent
    return Path.cwd()


def generate_launch_description() -> LaunchDescription:
    launch_file = Path(__file__).resolve()
    workspace_dir = resolve_workspace_root(launch_file)
    bag_dir = workspace_dir / 'bag'
    bag_dir.mkdir(parents=True, exist_ok=True)
    bag_name = f"online_traj_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    bag_output = str(bag_dir / bag_name)
    use_mock_px4 = LaunchConfiguration('use_mock_px4')
    record_bag = LaunchConfiguration('record_bag')
    log_level = LaunchConfiguration('log_level')
    ros_args = ['--ros-args', '--log-level', log_level]

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
        emulate_tty=True,
        arguments=ros_args,
    )

    traj_generator = Node(
        package='traj_offboard',
        executable='online_traj_node',
        name='online_traj_generator',
        output='screen',
        emulate_tty=True,
        arguments=ros_args,
    )

    mock_px4 = Node(
        package='traj_offboard',
        executable='mock_px4_sim_node',
        name='mock_px4_sim',
        output='screen',
        emulate_tty=True,
        arguments=ros_args,
        condition=IfCondition(use_mock_px4),
    )

    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '--storage', 'sqlite3',
            '--output', bag_output,
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
            'record_bag',
            default_value='true',
            description='true to record trajectory and FSM topics'),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='ROS log level for traj_offboard nodes'),
        LogInfo(msg='traj_offboard: starting offboard bridge and online trajectory generator'),
        LogInfo(msg='traj_offboard: start FSM with uav_offboard_fsm.launch.py and keyboard node in separate terminals'),
        LogInfo(msg=f'rosbag: output={bag_output}', condition=IfCondition(record_bag)),
        mock_px4,
        offboard_bridge,
        traj_generator,
        rosbag_record,
    ])
