# Immutable diagnostic fact for one simulation task plugin.  Task-specific
# measurements travel in metrics_json; there are no images on this topic.
robot_api_msgs/RunContext context
uint64 state_seq
builtin_interfaces/Time sample_time
uint64 task_epoch
# Plugin identity, e.g. "stack".
string task_type
# Task plugin phase: IDLE, PREPARED, RUNNING, SUCCEEDED or FAILED.
string phase
bool succeeded
bool terminal
# Bounded failure reason; empty while the task is healthy.
string failure_reason
string plan_digest
string task_config_digest
# Canonical JSON object with task-specific metrics (no NaN/Infinity).
string metrics_json
