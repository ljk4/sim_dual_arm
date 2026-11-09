# Native request mode.  Only the bounded-follower stream entries
# (STREAM_FIRST/STREAM_NEXT) are served; MOTION is retained as the historical
# wire value so a stale request is rejected explicitly instead of mis-decoded.
uint8 MOTION=1
uint8 STREAM_FIRST=2
uint8 STREAM_NEXT=3
uint8 kind
# Command identity and lease are carried inline; there is no separate
# trajectory-command wrapper in the public contract.
robot_api_msgs/RunContext context
string lease_id
string command_id
uint64 command_seq
uint64 observation_id
builtin_interfaces/Time accept_before
trajectory_msgs/JointTrajectory trajectory
# Stream hand-off anchor.  All arrays are empty only for a malformed request.
robot_api_msgs/ReferenceAnchor anchor
int64 observation_time_ns
uint32 target_count
bool settle_at_end
bool has_final_velocities
float64[<=64] final_velocities
