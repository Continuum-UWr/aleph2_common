# linear velocities of the end-effector tip (in m/s) along given axes in Cartesian space
float32 ee_x_cmd
float32 ee_y_cmd
float32 ee_z_cmd

# linear velocities of the end-effector tip (in m/s) along given axes in tip's reference frame
float32 ee_tip_x_cmd
float32 ee_tip_y_cmd
float32 ee_tip_z_cmd

# angular velocities of the end-effector tip in rad/s along given axes in tip's reference frame
float32 ee_pitch_cmd

# angular velocity of the 'wrist_roll` joint
float32 wrist_roll_cmd

# goes to a predefined pose
int8 SAFE_POSE = 1
int8 ZERO_POSE = 2
int8 pose_cmd