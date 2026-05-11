uint8 mani_task_state

# main_task_state:: PRE_CHECK
uint8 SELF_CHECK = 1

# config for takeoff->main_task_state:: WAIT_TASK_ENABLE_ AUTH
uint8 MANI_ENABLE = 2

# config for flying-> main_task_state:: NAV_TO_TASK_DOM+ ARRIVE_ TASK _DOM+
#UAV_SEARCH_TARGS+ TARG_ GOT + TARG_ READY + UAV_POSE_ADAP。
uint8 HOLD = 3

# The preparatory configuration for sampling > main_task_state:: ARM_ CONFIG_ PREP
uint8 PRE_CONTACT = 4

# Sampling Auto -> main_task_state:: SAMPL_OPERA
uint8 SAMPL_OPERA_AUTO = 5

# Sampling Manual -> main_task_state:: SAMPL_OPERA
uint8 SAMPL_OPERA_MANUAL = 6

# config for flying--> main_task_state:: UAV_PRE_BACK_HOME+ BACK_HOME
uint8 PRE_BACK_HOME=7

# config for landing -> main_task_state::ARRIVE_HOME
uint8 PRE_LAND = 8

# Emergency situation - Request for return！-> main_task_state:: TASK_TERM
uint8 MANI_TASK_TERM = 30

bool mani_check_succeed
bool ready_for_takeoff
bool ready_for_transit
bool ready_for_sample
bool sample_succeed
bool ready_for_back
bool ready_for_land