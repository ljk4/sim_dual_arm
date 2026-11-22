# RETIRED stays RETIRED; it is not relabelled as a motion COMPLETED phase.
robot_api_msgs/RunContext context
string command_id
uint64 command_seq
uint64 chunk_seq
uint64 observation_id
string phase
string reason
int64 accepted_backend_time_ns
int64 accepted_wall_time_ns
bool has_started
int64 started_backend_time_ns
int64 started_wall_time_ns
bool has_finished
int64 finished_backend_time_ns
int64 finished_wall_time_ns
