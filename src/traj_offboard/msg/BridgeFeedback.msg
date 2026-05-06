# Feedback owned by offboard_control_bridge and consumed by the mission FSM.

uint64 stamp_us

bool position_valid
float64 x
float64 y
float64 z
float64 yaw

bool home_valid

bool distance_valid
float64 distance_m

bool imu_valid

uint64 last_position_time_us
uint64 last_home_time_us
uint64 last_distance_sensor_time_us
uint64 last_imu_time_us
