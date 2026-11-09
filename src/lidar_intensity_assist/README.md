# lidar_intensity_assist (ROS 2)

Intensity-assisted LiDAR odometry front end for semi-solid LiDARs with a limited
horizontal field of view, such as the Hesai AT128. The node projects a LiDAR
cloud into a 2D intensity image, matches ORB features between consecutive
frames, estimates the inter-frame transform `T(cur -> prev)` from image features
and 3D correspondences, and optionally deskews the current scan.

## Requirements

ROS 2 Humble, Ceres, PCL, OpenCV, and OpenMP. The node expects a
`sensor_msgs/msg/PointCloud2` input on `/lidar_points` by default. Ring projection
needs `x`, `y`, `z`, `intensity`, and `ring` fields; timestamp deskewing also
needs a `timestamp` field of type `float64`.

## Build

Place this directory under a ROS 2 workspace's `src/`, then run from the
workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select lidar_intensity_assist
source install/setup.bash
```

## Run

```bash
ros2 launch lidar_intensity_assist start.launch.py
```

To start without RViz or select a different input topic:

```bash
ros2 launch lidar_intensity_assist start.launch.py start_rviz:=false
ros2 launch lidar_intensity_assist start.launch.py start_rviz:=false config_file:=/path/to/start.yaml
```
