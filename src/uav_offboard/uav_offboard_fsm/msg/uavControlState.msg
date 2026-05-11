# Documentation reference only. The C++ FSM uses an internal enum class ControlState.
# This file records the canonical state IDs for cross-package documentation.
uint8 uav_control_state
uint8 SELF_CHECK = 0
uint8 UAV_START= 1
uint8 TRANSIT_TO_AREA= 2
uint8 UAV_ARRIVED_AERA = 3
uint8 SEARCH_ADJUST_AUTO= 4
uint8 SEARCH_ADJUST_MANUAL= 5
uint8 APPROACH_PLANT= 6
uint8 UAV_PRE_HOLD = 7
uint8 SAMP_ADJUST_AUTO= 8
uint8 SAMP_ADJUST_MANUAL= 9
uint8 UAV_HOLD = 10
uint8 RETREAT = 11
uint8 UAV_BACK_HOME=12
uint8 UAV_TASK_TERM= 13
