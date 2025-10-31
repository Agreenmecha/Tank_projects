# Recovery action triggered by navigation failure
std_msgs/Header header

# Recovery action type
uint8 action_type
uint8 ACTION_NONE = 0
uint8 ACTION_REVERSE = 1
uint8 ACTION_PIVOT = 2
uint8 ACTION_WIGGLE = 3
uint8 ACTION_STOP = 4
uint8 ACTION_MANUAL_ASSIST = 5

# Trigger reason
uint8 trigger_reason
uint8 REASON_FORWARD_BLOCKED = 1
uint8 REASON_STUCK_DETECTED = 2
uint8 REASON_BELLY_STRIKE = 3
uint8 REASON_HIGH_SLIP = 4
uint8 REASON_PITCH_LIMIT = 5

# Action parameters
float32 reverse_distance  # meters
float32 pivot_angle       # radians
uint8 attempts_remaining

# Status
bool action_in_progress
float32 action_progress  # 0.0 to 1.0

