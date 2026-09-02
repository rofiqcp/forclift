# AGV Forklift Pallet Detection with 3D Pose Estimation
# Provides real-time pallet position and fork entry guidance

std_msgs/Header header

# Pallet identification (stable across frames via spatial tracking)
int32 pallet_id
string class_name        # "pallet" or "hole pallet"
float32 confidence

# Pallet center pose in base_link frame
Pose2DYaw pallet_pose

# Recommended fork entry pose (offset from pallet center)
Pose2DYaw entry_pose

# Fork hole positions (3D points in base_link frame)
geometry_msgs/Point left_hole
geometry_msgs/Point right_hole

# Hole dimensions (meters)
float32 hole_width
float32 hole_height
float32 hole_depth

# Tracking quality
bool tracking_stable
float32 depth_quality     # 0.0-1.0, based on depth measurement validity
int32 frames_tracked      # consecutive frames this pallet has been tracked
