# Docking guidance state
uint8 UNKNOWN = 0
uint8 APPROACHING = 1    # block detected, approaching
uint8 ALIGNING = 2        # block on path but not centered
uint8 DOCKING_READY = 3  # block centered, within docking distance
uint8 BLOCKED = 4        # path obstructed

std_msgs/Header header
uint8 state
string state_text
float32 alignment_error   # pixels, 0 = perfectly centered
float32 block_distance_m   # estimated distance to block
bool docking_safe          # true when state == DOCKING_READY
