# Immutable diagnostic fact for the independent stack task.  Object poses are
# truth for teacher/evaluation/diagnostics only and are never a policy input.
# All arrays are fixed length; there are no images on this topic.
robot_api_msgs/RunContext context
uint64 state_seq
builtin_interfaces/Time sample_time
uint64 task_epoch
string state
string reason
string task_config_digest
string plan_digest
builtin_interfaces/Time task_start_time
builtin_interfaces/Time first_satisfied_time
builtin_interfaces/Time completed_time
float64 stable_elapsed_s
float64[3] source_center
float64[4] source_quaternion_wxyz
float64[3] source_linear_velocity
float64[3] source_angular_velocity
float64[3] target_center
float64[4] target_quaternion_wxyz
float64[3] target_linear_velocity
float64[3] target_angular_velocity
float64 source_target_normal_force
float64 target_table_normal_force
float64 source_table_normal_force
uint32 finger_contact_count
float64 right_gripper_opening
float64 right_tcp_distance
bool conditions_passed
# Bit i is condition i in the fixed evaluator order.  See StackTaskState docs.
uint32 conditions_bitmask
