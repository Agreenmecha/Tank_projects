# GNSS fix quality for corridor width adaptation
std_msgs/Header header

# Satellite information
uint8 satellites_visible
uint8 satellites_used

# Dilution of Precision
float32 hdop  # Horizontal DOP (primary metric for corridor width)
float32 vdop  # Vertical DOP
float32 pdop  # Position DOP

# Fix status
uint8 fix_type  # 0=NO_FIX, 1=2D, 2=3D, 3=DGPS, 4=RTK_FIXED, 5=RTK_FLOAT
bool differential_correction_available

# Accuracy estimates (meters)
float32 horizontal_accuracy
float32 vertical_accuracy

# Recommended corridor width
float32 recommended_corridor_width  # meters (4-7m based on quality)
uint8 quality_status  # 0=GOOD, 1=MODERATE, 2=POOR

