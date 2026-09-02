# AGV Forklift Floor Marking Detection with Pose
# Provides position and orientation of floor tape markers

std_msgs/Header header

# Marking identification  
int32 marking_id
string marking_type      # "floor marking", "line", "arrow", etc.
float32 confidence

# Marking center pose in base_link frame (meters, radians)
geometry_msgs/Pose2D marking_pose

# Bounding polygon (2D points in image plane)
geometry_msgs/Point[] polygon_2d

# Estimated line endpoints (if linear marking)
geometry_msgs/Point line_start
geometry_msgs/Point line_end

# Tracking quality
bool tracking_stable
int32 frames_tracked
