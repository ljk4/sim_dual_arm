# Frozen overall execution feedback; terminal values are not rewritten by later state.
robot_api_msgs/RunContext context
string command_id
uint64 command_seq
uint64 observation_id
string phase
string reason
string stop_reason
uint64 motion_seq
string stream_id
uint64 chunk_seq
int64 accepted_backend_time_ns
int64 accepted_wall_time_ns
bool has_started
int64 started_backend_time_ns
int64 started_wall_time_ns
bool has_finished
int64 finished_backend_time_ns
int64 finished_wall_time_ns
string motion_phase
int64 reference_application_backend_time_ns
int64 measurement_backend_time_ns
int64 reference_elapsed_ns
int64 stable_for_ns
string[] joint_names
float64[<=64] reference_positions
float64[<=64] reference_velocities
float64[<=64] reference_accelerations
float64[<=64] measured_positions
float64[<=64] measured_velocities
float64[<=64] position_errors
float64[<=64] velocity_errors
string reference_kind
string tracking_quality
int64 tracking_violation_ns
# Optional named gripper set; empty is valid.  Side identifiers are unique and
# support multi-gripper or dual-arm configurations generically.
robot_api_msgs/GripperFeedback[<=8] grippers
