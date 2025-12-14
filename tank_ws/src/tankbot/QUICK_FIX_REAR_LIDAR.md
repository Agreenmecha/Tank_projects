# Quick Fix: View Rear LiDAR with Front Fixed Frame

## Problem
Fixed Frame is set to `lidar_front`, but rear point cloud (`frame_id: lidar_rear`) doesn't display because there's no transform between them.

## Solution: Create Static Transform

I've started a static transform publisher that connects `lidar_front` to `lidar_rear`. 

**In RViz:**
1. Wait 2-3 seconds for the transform to propagate
2. The rear point cloud should now appear!

**Or, simpler approach:**
- Set Fixed Frame to `lidar_rear` (type it manually)
- Add PointCloud2 for `/lidar_rear/cloud`
- You'll see the rear point cloud immediately

## View Both Simultaneously

To see both at once:
1. Keep Fixed Frame as `lidar_front`
2. Add PointCloud2 for `/lidar_front/cloud` (should work)
3. Add PointCloud2 for `/lidar_rear/cloud` (should work now with the transform)
4. Both should display!

## Alternative: Switch Fixed Frame

If you want to view them separately:
- To view front: Fixed Frame = `lidar_front`, topic = `/lidar_front/cloud`
- To view rear: Fixed Frame = `lidar_rear`, topic = `/lidar_rear/cloud`

Just change the Fixed Frame and the corresponding point cloud will display.

