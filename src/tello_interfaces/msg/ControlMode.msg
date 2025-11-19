# Control mode status message

std_msgs/Header header

# Current active mode
string mode  # "manual", "gesture", "tracking"

# Camera source for gesture mode
string camera_source  # "drone", "webcam"

# Control authority (which node can publish cmd_vel)
string control_authority  # "manual", "gesture", "tracking"

# Mode transition state
bool transitioning  # True if switching modes (hover command active)
string target_mode  # Mode being transitioned to (empty if not transitioning)
