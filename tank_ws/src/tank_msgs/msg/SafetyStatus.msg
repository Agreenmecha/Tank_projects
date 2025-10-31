# Safety status message for tank navigation
std_msgs/Header header

# E-stop state
bool hardware_estop_active
bool software_estop_active

# Pitch angle monitoring (degrees)
float32 pitch_angle
uint8 pitch_status  # 0=OK, 1=WARNING, 2=CRITICAL, 3=EMERGENCY
float32 pitch_warning_threshold   # 28 degrees
float32 pitch_critical_threshold  # 32 degrees
float32 pitch_emergency_threshold # 38 degrees

# Motor thermal
float32 motor_left_temp
float32 motor_right_temp
uint8 thermal_status  # 0=OK, 1=WARNING (>80C), 2=CRITICAL (>90C)

# Watchdog
bool watchdog_ok
float32 cmd_vel_age  # seconds since last cmd_vel

# Geo-fence
bool within_geofence
float32 distance_to_boundary  # meters

# Overall safety state
uint8 overall_status  # 0=SAFE, 1=WARNING, 2=UNSAFE, 3=EMERGENCY_STOP

