# Fork-Pallet Alignment State for Ackermann AGV
uint8 UNKNOWN = 0
uint8 APPROACH = 1
uint8 FINE_ALIGN = 2
uint8 ALIGNED = 3
uint8 STEER_CENTER = 4
uint8 INSERTION = 5
uint8 STOPPED_SAFETY = 6
uint8 ERROR = 7

std_msgs/Header header
uint8 state
string state_text

# Detection quality
bool pallet_detected
bool detection_stable
float32 confidence
float32 depth_quality

# Error values
float32 error_lateral_m        # meters, + = fork right of center
float32 error_yaw_deg           # degrees, + = AGV rotated clockwise (yawing right)

# PID outputs
float32 pid_lateral_output
float32 pid_yaw_output
float32 desired_yaw_rate       # rad/s = lateral + yaw combined

# Ackermann quantities
float32 estimated_steering_deg  # computed from kinematics
float32 linear_velocity_cmd     # m/s
float32 angular_velocity_cmd     # rad/s

# Safety flags
bool steering_limit_active
bool safety_stop_active
bool data_valid                  # true if source data is fresh

# Alignment criteria
bool lateral_within_tolerance
bool yaw_within_tolerance
bool steering_centered
bool ready_for_insertion

# Timestamps for data freshness check
builtin_interfaces/Time pallet_header_stamp
