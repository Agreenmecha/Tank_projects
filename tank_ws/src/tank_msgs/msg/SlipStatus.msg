# Track slip detection status
std_msgs/Header header

# Slip percentage (encoder vs Point-LIO velocity)
float32 slip_percentage_left
float32 slip_percentage_right
float32 slip_percentage_avg

# Velocities
float32 encoder_velocity_left
float32 encoder_velocity_right
float32 pointlio_velocity_linear

# Status
uint8 slip_status  # 0=OK (<15%), 1=MODERATE (15-30%), 2=SEVERE (>30%)
bool action_required

