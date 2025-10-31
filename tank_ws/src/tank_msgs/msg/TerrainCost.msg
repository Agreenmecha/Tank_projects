# Fused terrain cost for a single costmap cell
std_msgs/Header header

# Cell location
int32 cell_x
int32 cell_y

# Individual cost components (0-255)
uint8 lidar_cost         # From geometry (slope, roughness, obstacles)
uint8 camera_cost        # From semantic segmentation
uint8 fused_cost         # max(lidar_cost, alpha * camera_cost)

# Terrain class (from camera)
uint8 terrain_class
uint8 TERRAIN_UNKNOWN = 0
uint8 TERRAIN_DRIVABLE = 1
uint8 TERRAIN_VEGETATION_LIGHT = 2
uint8 TERRAIN_VEGETATION_DENSE = 3
uint8 TERRAIN_ROCK = 4
uint8 TERRAIN_WATER_MUD = 5

# Geometry features (from LiDAR)
float32 slope_deg
float32 roughness
float32 height

# Confidence
float32 camera_confidence
bool camera_valid

