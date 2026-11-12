# Fixed absolute stream hand-off reference shared by first/successor chunks.
string stream_id
uint64 predecessor_seq
int64 start_sim_time_ns
string robot_profile_hash
float64[<=64] positions
float64[<=64] velocities
float64[<=64] accelerations
