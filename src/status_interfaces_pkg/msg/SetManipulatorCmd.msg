# End effector translation command（ velocity：-1~1 ）
float32 x_vel
float32 y_vel
float32 z_vel

# End joint rotation command.  （joint position：-1~1）
float32 joint_position

# Manual control last joint enabled
bool last_joint_enable

# Manual velocity control enabled
bool vel_control_enable