# Robot-clock timestamps and execution-clock duration; SI position arrays.
robot_api_msgs/RunContext context
string command_id
uint64 command_seq
uint64 observation_id
string phase
string reason
builtin_interfaces/Time sample_time  # robot-clock sample time (ns)
builtin_interfaces/Time accepted_at  # robot-clock ACCEPTED time (ns)
builtin_interfaces/Time started_at  # robot-clock STARTED time (ns)
builtin_interfaces/Duration elapsed  # actual execution duration on execution_clock (ns)
string execution_clock  # steady | robot
string interpolation_version  # linear_v1
# Position arrays are ordered exactly like joint_names.
string[] joint_names
float64[] initial_positions  # command start reference
float64[] reference_positions  # sampled reference positions
float64[] measured_positions  # sampled measured positions
float64[] tracking_error  # reference_positions - measured_positions
float64 progress  # finite, in [0, 1]
