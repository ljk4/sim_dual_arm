string side
string mode
bool position_ok
bool contact_ok
string contact_object
bool ambiguous
float64 target_position_m
float64 initial_reference_m
float64 final_reference_m
float64 measured_position_m
float64 measured_velocity_m_s
float64 target_error_m
# Named finger evidence for this gripper; identifiers are unique, indices are
# positive and bounded by the shared contract (1..8), and the count is not fixed.
robot_api_msgs/FingerFeedback[<=8] fingers
