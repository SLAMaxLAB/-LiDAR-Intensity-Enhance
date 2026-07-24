#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <array>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <chrono>
#include <cmath>

class LidarRingSaver {
public:
    LidarRingSaver(ros::NodeHandle& nh) : frame_count_(0) {
        nh.param<std::string>("output_dir", output_dir_, "./lidar_output");
        if (mkdir(output_dir_.c_str(), 0777) && errno != EEXIST) {
            ROS_ERROR("Failed to create directory: %s", output_dir_.c_str());
        }
        sub_ = nh.subscribe("/points_raw", 1, &LidarRingSaver::callback, this);
    }

    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // if (frame_count_ >= 2) return;  // 只保存两帧

        const int NUM_RINGS = 128;
        const int POINTS_PER_RING = 1024;

        auto t_start = std::chrono::high_resolution_clock::now();

        cv::Mat intensity_img(NUM_RINGS, POINTS_PER_RING, CV_32F, cv::Scalar(0.0f));
        cv::Mat depth_img    (NUM_RINGS, POINTS_PER_RING, CV_32F, cv::Scalar(0.0f));

        std::vector<float*> intensity_rows(NUM_RINGS);
        std::vector<float*> depth_rows(NUM_RINGS);
        for (int r = 0; r < NUM_RINGS; ++r) {
            intensity_rows[r] = intensity_img.ptr<float>(r);
            depth_rows[r] = depth_img.ptr<float>(r);
        }

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
        sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_ring(*msg, "ring");

        // std::map<int, std::vector<std::array<float,4>>> ring_points;
        // std::string total_file = output_dir_ + "/lidar_points_frame" + std::to_string(frame_count_) + ".txt";
        // std::ofstream total_ofs(total_file, std::ios::trunc);

        std::vector<int> idx(NUM_RINGS, 0);

        for (; iter_x != iter_x.end();
               ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_ring) {
            int ring_id = static_cast<int>(*iter_ring);
            if (ring_id < 0 || ring_id >= NUM_RINGS) continue;

            int col = idx[ring_id];
            if (col >= POINTS_PER_RING) continue;

            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            float intensity = *iter_intensity;
            // float depth = std::sqrt(x*x + y*y + z*z);

            intensity_rows[ring_id][col] = intensity;
            // depth_rows[ring_id][col] = depth;

            // ring_points[ring_id].push_back({x, y, z, intensity});
            // total_ofs << x << " " << y << " " << z << " " << intensity << " " << ring_id << "\n";

            idx[ring_id]++;
        }
        // total_ofs.close();

        // 保存每条线
        // for (auto& kv : ring_points) {
        //     int ring_id = kv.first;
        //     std::string filename = output_dir_ + "/ring_" + std::to_string(ring_id) + "_frame" + std::to_string(frame_count_) + ".txt";
        //     std::ofstream ofs(filename, std::ios::trunc);
        //     for (auto& pt : kv.second) {
        //         ofs << pt[0] << " " << pt[1] << " " << pt[2] << " " << pt[3] << "\n";
        //     }
        // }

        // 归一化
        cv::Mat intensity_norm, depth_norm;
        cv::normalize(intensity_img, intensity_norm, 0, 255, cv::NORM_MINMAX, CV_8U);
        // cv::normalize(depth_img, depth_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

        // 伪彩色
        cv::Mat intensity_color, depth_color;
        cv::applyColorMap(intensity_norm, intensity_color, cv::COLORMAP_JET);
        // cv::applyColorMap(depth_norm, depth_color, cv::COLORMAP_JET);

        // 保存图像
        cv::imwrite(output_dir_ + "/intensity_gray_frame" + std::to_string(frame_count_) + ".png", intensity_norm);
        // cv::imwrite(output_dir_ + "/depth_gray_frame" + std::to_string(frame_count_) + ".png", depth_norm);
        // cv::imwrite(output_dir_ + "/intensity_colormap_frame" + std::to_string(frame_count_) + ".png", intensity_color);
        // cv::imwrite(output_dir_ + "/depth_colormap_frame" + std::to_string(frame_count_) + ".png", depth_color);

        // ORB 特征
        cv::Ptr<cv::ORB> orb = cv::ORB::create(500);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(intensity_norm, cv::noArray(), keypoints, descriptors);

cv::Mat orb_vis;
if (intensity_norm.channels() == 1) {
    // 灰度图转BGR，便于彩色绘制
    cv::cvtColor(intensity_norm, orb_vis, cv::COLOR_GRAY2BGR);
} else {
    orb_vis = intensity_norm.clone();
}

// 绘制小圆圈
for (const auto& kp : keypoints) {
    int radius = std::max(1, (int)std::round(kp.size * 0.1));  // 🔹 半径更小
    cv::circle(orb_vis, kp.pt, radius, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
}

                          
        cv::imwrite(output_dir_ + "/intensity_orb_frame" + std::to_string(frame_count_) + ".png", orb_vis);

        ROS_INFO("帧 %d 保存完成，特征数: %zu", frame_count_, keypoints.size());

        frame_count_++;
    }

private:
    ros::Subscriber sub_;
    int frame_count_;
    std::string output_dir_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_ring_saver_twoframes");
    ros::NodeHandle nh("~");

    LidarRingSaver saver(nh);

    ros::spin();
    return 0;
}
