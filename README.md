# Intensity-Assisted Semi-Solid LiDAR Odometry Front End

This repository provides a ROS 1 front end for semi-solid LiDARs with a limited horizontal field of view, such as the Hesai AT128. It converts organized or unorganized LiDAR returns into a 2D intensity image, estimates an inter-frame pose from image features and 3D correspondences, and optionally deskews the current scan.

The implementation is designed as an intensity-assisted odometry module rather than a complete SLAM system. Its pose estimate and corrected cloud can be used as inputs to downstream LiDAR odometry or mapping systems.

## Demo

<p align="center">
  <img src="src/lidar_intensity_assist/fig/demo.gif" alt="lidar_intensity_assist demo" width="640">
</p>

## Highlights

- Projects 3D LiDAR points to an intensity image using either ring-based or angle-based cylindrical projection.
- Preserves the physical AT128 horizontal scan direction in the image columns.
- Uses the maximum-intensity return to render each image pixel while optionally retaining all 3D points in the pixel for joint optimization.
- Supports histogram equalization or CLAHE, optional Gaussian smoothing, and bilateral filtering before ORB extraction.
- Performs ORB descriptor matching, 2D geometric RANSAC, and 3D RANSAC before SVD pose initialization.
- Estimates `T(cur -> prev)`, i.e., the rigid transform that maps the current-frame coordinates into the previous-frame coordinates.
- Optionally refines pose estimation and constant-velocity deskewing jointly with Ceres and intensity-based correspondence weights.
- Uses the optimized pose to deskew the current scan and rebuilds the corrected feature cache for the next frame.
- Publishes raw/processed intensity images, match visualizations, feature clouds, pose, raw cloud, deskewed cloud, and sampled cloud for RViz.
- Uses latched publishers by default, so RViz receives the latest result after a Display is disabled and enabled again while a rosbag is paused.

## Layout

```text
lidar_intensity_assist/
├── README.md
└── src/
    └── lidar_intensity_assist/
        ├── CMakeLists.txt
        ├── package.xml
        ├── config/start.yaml                  # default node parameters
        ├── launch/start.launch                # node + optional RViz
        ├── rviz/start.rviz                    # example display configuration
        ├── fig/demo.gif                       # demo animation used by the README
        ├── include/lidar_intensity_assist/
        │   ├── lidar_intensity_match.hpp      # node class declaration
        │   └── math_utils.hpp                 # small numeric helpers
        └── src/
            ├── main.cpp                       # node entry point
            └── lidar_intensity_match.cpp      # node implementation
```

## Processing Pipeline

```text
PointCloud2
  -> origin-point filtering
  -> 3D-to-2D intensity projection (maximum intensity per pixel)
  -> contrast enhancement and bilateral filtering
  -> ORB feature extraction and descriptor matching
  -> 2D RANSAC -> 3D RANSAC
  -> SVD initialization of T(cur -> prev)
  -> optional joint pose/deskew optimization (Ceres)
  -> constant-velocity deskewing of the current scan
  -> corrected feature cache for the next frame
```

For `joint_pixel_match_mode: max_intensity`, the 3D-RANSAC inlier points are used directly in the joint optimization. For `joint_pixel_match_mode: all`, all retained 3D points in each matched-pixel pair are paired locally by nearest distance, followed by optional outlier rejection.

## Requirements

The project is developed and tested with:

- Ubuntu 20.04
- ROS Noetic (ROS 1)
- C++17 and catkin
- PCL 1.10 or newer
- OpenCV
- Ceres Solver
- OpenMP

The ROS package lives in `src/lidar_intensity_assist`. The LiDAR driver is not part of this repository; a compatible driver (for example the Hesai ROS driver) has to be provided separately.

## Build

This repository is a catkin workspace. From its root directory:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg lidar_intensity_assist
source devel/setup.bash
```

If required dependencies are missing, install the ROS, PCL, OpenCV, Ceres, and OpenMP development packages appropriate for your Ubuntu/ROS distribution before building.

## Input Cloud Format

The node subscribes to `sensor_msgs/PointCloud2`.

Required fields:

| Field | Type | Purpose |
| --- | --- | --- |
| `x`, `y`, `z` | floating point | 3D Cartesian coordinates |
| `intensity` | floating point | intensity-image rendering and optional correspondence weights |

Additional fields:

| Field | Required when | Purpose |
| --- | --- | --- |
| `ring` | `projection_mode: ring` or line sampling is enabled | vertical beam index |
| `timestamp` | deskewing or joint deskew optimization is enabled | per-point acquisition time within the scan |

The default input topic is `/lidar_points`. Configure it through `cloud_topic` in the YAML file. Invalid or near-origin placeholder points can be removed with `filter_origin_points` and `origin_filter_eps`.

## Experiment Data

The rosbag used for the experiments is available for download:

- **Baidu Netdisk** (百度网盘): https://pan.baidu.com/s/1PmtT0EybBszuGbufo2-25A?pwd=ga6b
- **Extraction code**: `ga6b`

Download the bag and play it after launching the node, as shown in [Run](#run).

## Run

The default configuration is [start.yaml](src/lidar_intensity_assist/config/start.yaml).

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch lidar_intensity_assist start.launch start_rviz:=true
```

To use a separate experiment configuration:

```bash
roslaunch lidar_intensity_assist start.launch \
  config_file:=$(pwd)/src/lidar_intensity_assist/config/start.yaml \
  start_rviz:=false
```

Play a bag in another terminal after sourcing the same workspace:

```bash
rosbag play your_recording.bag
```

When the source cloud uses another topic, change `cloud_topic` in the selected YAML configuration. The launch-file arguments `raw_cloud_topic` and `deskewed_cloud_topic` only remap the corresponding output topics.

## ROS Topics

### Subscribed Topic

| Topic | Type | Description |
| --- | --- | --- |
| `/lidar_points` | `sensor_msgs/PointCloud2` | input LiDAR cloud; configurable with `cloud_topic` |

### Published Topics

| Default topic | Type | Description |
| --- | --- | --- |
| `/raw_cloud` | `sensor_msgs/PointCloud2` | filtered current cloud before deskewing |
| `/deskewed_cloud` | `sensor_msgs/PointCloud2` | current cloud after accepted deskewing; otherwise the current processing cloud |
| `/sampled_points` | `sensor_msgs/PointCloud2` | ring-sampled cloud for visualization or downstream use |
| `/Tguess` | `nav_msgs/Odometry` | estimated `T(cur -> prev)` |
| `/raw_intensity_image` | `sensor_msgs/Image` | normalized raw intensity image with optional frame label |
| `/intensity_enh_image` | `sensor_msgs/Image` | final enhanced intensity image used by ORB |
| `/matched_orb_image` | `sensor_msgs/Image` | ORB descriptor matches before RANSAC |
| `/matched_2d_image` | `sensor_msgs/Image` | matches retained by 2D RANSAC |
| `/matched_3d_image` | `sensor_msgs/Image` | matches retained by 3D RANSAC |
| `/matched_prev_orb_points` | `sensor_msgs/PointCloud2` | previous-frame 3D matched feature points |
| `/matched_cur_orb_points` | `sensor_msgs/PointCloud2` | current-frame 3D matched feature points |

All output topics are latched when `latch_published_topics: true`. A newly enabled RViz Display immediately receives the newest available message. Latching retains one message per topic; it does not replay an entire bag or publish continuously after playback stops.


## Output

The node does not write any files to disk. Every result is published on ROS
topics: intensity images, match visualizations, 3D matched feature clouds, the
raw cloud, the deskewed cloud, the ring-sampled cloud, and the estimated
`T(cur -> prev)` odometry. Configured topics and latching behavior are listed
above. Generated `build/` and `devel/` directories should normally not be
committed to Gitee.

## Coordinate and Deskew Convention

The estimated transform is:

```text
p_prev = T(cur -> prev) * p_cur
```

The current scan is deskewed using the inverse motion, `T(prev -> cur)`, under a constant-velocity assumption. For a point acquired at fraction `s` of the scan span, the corresponding fraction of the scan motion is applied. When enabled, `deskew_scale_by_time` scales the motion using the ratio between the scan duration and the start-to-start interval of the two frames.

The joint optimizer keeps previous-frame 3D features as the reference and refines the current-to-previous pose. It does not re-run projection, ORB, or RANSAC for every Ceres iteration; the stable feature associations found by the initial matching pipeline are reused.

## Notes and Limitations

- The current implementation targets forward-looking, narrow-FOV semi-solid LiDAR. Parameters should be retuned for another sensor, field of view, or point format.
- Reliable deskewing requires valid per-point timestamps and sufficiently accurate inter-frame feature correspondences.
- The first received frame initializes the feature cache and therefore does not have inter-frame matches or a pose estimate.
- This module is a front end. It does not include loop closure, global pose-graph optimization, or map management.


