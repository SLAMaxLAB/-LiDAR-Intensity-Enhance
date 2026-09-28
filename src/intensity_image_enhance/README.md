# intensity_image_enhance

Intensity-assisted LiDAR odometry front end for semi-solid LiDARs with a limited
horizontal field of view (for example the Hesai AT128). The node projects a
LiDAR cloud into a 2D intensity image, matches ORB features between consecutive
frames, estimates the inter-frame transform `T(cur -> prev)` from image features
and 3D correspondences, and optionally deskews the current scan.

See the repository root [README.md](../../README.md) for the processing pipeline,
topic list, configuration reference, and run instructions.

## Layout

```text
intensity_image_enhance/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── pcd_correction.yaml            # default node parameters
├── launch/
│   └── pcd_correction.launch          # node + optional RViz
├── rviz/
│   └── pcd_correction.rviz            # example display configuration
├── include/intensity_image_enhance/
│   ├── lidar_intensity_orb_matcher.hpp        # main node class declaration
│   ├── joint_deskew_iteration_callback.hpp    # Ceres iteration callback
│   ├── math_utils.hpp                         # small numeric helpers
│   └── pcd_io_utils.hpp                       # PCD writing helpers
└── src/
    ├── main.cpp                               # node entry point
    ├── lidar_intensity_orb_matcher.cpp        # node implementation
    └── utils/
        └── pcd_io_utils.cpp                   # PCD writing helpers
```

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg intensity_image_enhance
source devel/setup.bash
```

## Run

```bash
roslaunch intensity_image_enhance pcd_correction.launch start_rviz:=true
```

All tunable parameters live in [config/pcd_correction.yaml](config/pcd_correction.yaml).
