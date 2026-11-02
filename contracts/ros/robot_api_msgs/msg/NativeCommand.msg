# Native request mode.  The old TrajectoryCommand remains the source identity.
uint8 MOTION=1
uint8 STREAM_FIRST=2
uint8 STREAM_NEXT=3
uint8 kind
robot_api_msgs/TrajectoryCommand command
# Absent for MOTION: stream_id is the empty sentinel and all arrays are empty.
robot_api_msgs/ReferenceAnchor anchor
int64 observation_time_ns
uint32 target_count
bool settle_at_end
bool has_final_velocities
float64[<=64] final_velocities
