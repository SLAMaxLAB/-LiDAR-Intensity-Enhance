#include "lidar_intensity_assist/lidar_intensity_match.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pcd_correction_node");
    lidar_intensity_assist::LidarIntensityORBMatchDual matcher(node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
