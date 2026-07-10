sudo chmod 777 /dev/ttyACM0
MicroXRCEAgent serial --dev /dev/ttyACM0 -b 921600

ros2 launch traj_offboard offboard_traj.launch.py
ros2 launch uav_offboard_fsm uav_offboard_fsm.launch.py

ros2 topic pub /main_task_fsm/task_states status_interfaces_pkg/msg/TaskFSM main_task_state:\ 1\

ros2 run uav_offboard_fsm whole_body_setpoint_rock_test_node   --ros-args   -p trajectory_type:=fixed_velocity   -p axis:=north_east   -p fixed_velocity_north_m_s:=-1.0   -p fixed_velocity_east_m_s:=0.0