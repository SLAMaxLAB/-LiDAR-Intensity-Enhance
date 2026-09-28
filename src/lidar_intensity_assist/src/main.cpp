// Entry point of the pcd_correction_node executable.

#include "lidar_intensity_assist/lidar_intensity_match.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_intensity_orb_match_dual_sampling_pubonly");
    ros::NodeHandle nh("~");
    lidar_intensity_assist::LidarIntensityORBMatchDual node(nh);
    ros::spin();
    return 0;
}
