std_msgs/Header header
string mode
uint64 action_id
string reason
bool command_enabled
bool ft_bias_ready
bool fault_latched
geometry_msgs/Pose actual_pose
geometry_msgs/Pose nominal_pose
geometry_msgs/Pose reference_pose
# All Cartesian components below use header.frame_id (model world/base frame).
float64[3] translation_error
# log(R_reference * R_actual^T), world-axis rotation vector [rad], not Euler error.
float64[3] rotation_error
float64[6] joint_actual
float64[6] joint_command
float64[6] joint_error
# Wrenches are bias-corrected and expressed at the controlled wrist, in wrist axes.
float64[6] wrench_bias_corrected
float64[6] wrench_filtered
float64[6] admittance_displacement
float64[6] admittance_velocity
