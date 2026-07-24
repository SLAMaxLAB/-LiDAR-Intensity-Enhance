# Intensity-Assisted Semi-Solid LiDAR Odometry Front End

This repository provides a ROS 1 front end for semi-solid LiDARs with a limited horizontal field of view, such as the Hesai AT128. It converts organized or unorganized LiDAR returns into a 2D intensity image, estimates an inter-frame pose from image features and 3D correspondences, and optionally deskews the current scan.

The implementation is designed as an intensity-assisted odometry module rather than a complete SLAM system. Its pose estimate and corrected cloud can be used as inputs to downstream LiDAR odometry or mapping systems.

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
- Saves intermediate images, PCD files, match files, pose logs, and joint-optimization statistics for offline analysis.

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

The point-cloud processing node is in `src/intensity_image_enhance`. The repository also contains `HesaiLidar_ROS_2.0`, which can be used as a Hesai driver reference or integration source.

## Build

This repository is a catkin workspace. From its root directory:

```bash
source /opt/ros/noetic/setup.bash
catkin_make --pkg intensity_image_enhance
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

## Run

The default configuration is [pcd_correction.yaml](src/intensity_image_enhance/config/pcd_correction.yaml).

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch intensity_image_enhance pcd_correction.launch start_rviz:=true
```

To use a separate experiment configuration:

```bash
roslaunch intensity_image_enhance pcd_correction.launch \
  config_file:=$(pwd)/src/intensity_image_enhance/config/pcd_correction.yaml \
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

## Important Configuration Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `projection_mode` | `ring` | `ring`: row is LiDAR ring; `angle`: row is elevation angle |
| `v_res`, `h_res` | `128`, `500` | intensity image height and width |
| `h_fov_deg` | `120.0` | horizontal field of view in degrees |
| `store_all_pixel_points` | `true` | retain all 3D points belonging to each intensity pixel |
| `contrast_mode` | `clahe` | `clahe`, `equalize`, or a mode that leaves the raw image unchanged |
| `enable_bilateral_filter` | `true` | apply edge-preserving bilateral filtering before ORB |
| `orb_nfeatures` | `200` | maximum number of ORB keypoints |
| `ratio_thresh`, `hamming_thresh` | `0.90`, `90` | descriptor matching filters |
| `ransac_2d_reproj` | `3.0` | 2D RANSAC reprojection threshold in pixels |
| `ransac_3d_thresh` | `0.3` | 3D RANSAC inlier threshold in meters |
| `enable_joint_tguess_deskew` | `true` | enable Ceres joint pose/deskew refinement |
| `joint_pixel_match_mode` | `max_intensity` | `max_intensity` or `all` 3D points inside matched pixels |
| `enable_deskew_current` | `false` | apply the accepted optimized pose to deskew the current cloud |
| `deskew_scale_by_time` | `true` | scale inter-frame motion by scan span / frame interval |
| `latch_published_topics` | `true` | preserve the latest output for late RViz subscribers |
| `output_dir` | `lidar_output` path | directory for saved images, PCDs, and logs |

Restart the node after changing a YAML parameter. Be careful not to edit a YAML copy outside the config file passed through `config_file`.

## Saved Results

When saving is enabled, the node creates subdirectories under `output_dir` automatically:

| Directory | Contents |
| --- | --- |
| `intensity_raw/` | normalized raw intensity images |
| `intensity_equalize/`, `intensity_clahe/` | intermediate contrast-enhancement images |
| `intensity_bilateral/`, `intensity_enh/` | final image-processing results |
| `orb_vis/` | ORB keypoint visualizations |
| `match_orb/`, `match_2d/`, `match_3d/` | match visualizations before/after RANSAC |
| `matches_3d/` | 3D match text files |
| `pcd_raw/`, `pcd_deskewed/` | raw and final deskewed point clouds |
| `pcd_features_raw/` | stable 3D feature pairs before joint optimization |
| `pcd_joint_iterations/` | Ceres iteration feature clouds in the previous-frame reference |
| `tguess/` | inter-frame pose records |
| `logs/` | timing and joint-optimization CSV logs |

Large bags can generate substantial image and PCD output. For online evaluation, consider disabling `save_image_results` and `save_pcd_results`, or direct `output_dir` to a dedicated experiment folder. Generated `build/`, `devel/`, and output directories should normally not be committed to Gitee.

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

## License

The ROS package currently declares a BSD license in its `package.xml`. Add a repository-level `LICENSE` file before publishing if you intend to distribute the project publicly.
