# Independent accepted bounded-follower record.  This is deliberately not an
# AcceptedReference: the follower curve is not required to pass through every
# raw prediction.  The complete raw NativeCommand (including any unselected
# tail) is retained in `request`; the explicit curve, limits, terminal speeds,
# versions and acceptance times are copied verbatim.
robot_api_msgs/NativeCommand request
uint64 motion_seq
int64 accepted_sim_time_ns
int64 accepted_wall_time_ns
string[] joint_names
# Original observation/hand-off window and selected source indices.
robot_api_msgs/ReferenceAnchor anchor
uint64 observation_id
int64 observation_time_ns
int64 handoff_time_ns
uint32 target_count
uint32[] selected_indices
robot_api_msgs/ReferenceSegment[<=128] segments
float64[<=64] raw_terminal_velocities
float64[<=64] restricted_terminal_velocities
float64[<=64] actual_terminal_positions
float64[<=64] actual_terminal_velocities
float64[<=64] actual_terminal_accelerations
float64[<=64] max_accelerations
float64[<=64] max_target_deviation_limits
float64[<=64] max_target_deviations
bool is_final
uint64 requested_window_ns
uint64 ruckig_duration_ns
bool has_duration_grid
uint64 duration_grid_ns
uint64 extension_ns
bool[<=64] velocity_limited
bool[<=64] acceleration_limited
bool[<=64] boundary_preserved
string algorithm_version
string sampler_version
string normalization_version
string reference_digest
