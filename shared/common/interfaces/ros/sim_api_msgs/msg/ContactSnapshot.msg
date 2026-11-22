# Immutable simulation-truth snapshot captured on the physics owner thread.
# Arrays use a documented stride; entries are in the transmitted order.
# The task interprets holds and supports; this message only reports facts.

robot_api_msgs/RunContext context
builtin_interfaces/Time sample_time
uint64 capture_seq

# Free task objects, three/four values per name.
string[] object_names
float64[] object_center
float64[] object_quaternion
float64[] object_linear_velocity
float64[] object_angular_velocity

# Per-finger contacts with a configured free object.
string[] contact_side
uint8[] contact_finger
string[] contact_object
float64[] contact_normal_force
uint32[] contact_count

# Contacts between a configured object and any non-finger body.
string[] support_object
string[] support_body
float64[] support_normal_force
uint32[] support_contact_count

# Measured gripper opening and TCP world pose, ordered identically to
# gripper_sides and the configured RobotProfile gripper order.
string[] gripper_sides
float64[] gripper_opening
float64[] tcp_position
float64[] tcp_quaternion
