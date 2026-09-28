// Entry point of the pcd_correction_node executable.

#include "intensity_image_enhance/lidar_intensity_orb_matcher.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_intensity_orb_match_dual_sampling_pubonly");
    ros::NodeHandle nh("~");
    intensity_image_enhance::LidarIntensityORBMatchDual node(nh);
    ros::spin();
    return 0;
}
