#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <sys/types.h>
#include <string>
#include <cmath>

class LidarIntensityCLAHEORB {
public:
    LidarIntensityCLAHEORB(ros::NodeHandle& nh) : saved_(false) ,frame(0) {
        nh.param<std::string>("output_dir", output_dir_, "./lidar_intensity_orb_out");
        nh.param<int>("num_rings", num_rings_, 128);
        nh.param<float>("v_fov_min", v_fov_min_, -12.5f);
        nh.param<float>("v_fov_max", v_fov_max_, 12.9f);
        nh.param<float>("h_fov", h_fov_, 120.0f);
        nh.param<float>("sample_h_res", sample_res_, 0.1f);
        nh.param<int>("orb_nfeatures", orb_nfeatures_, 1000);
        nh.param<int>("orb_fastThreshold", orb_fastThreshold_, 10);
        nh.param<int>("orb_vis_radius", orb_vis_radius_, 2);

        // �������Ŀ�?
        if (mkdir(output_dir_.c_str(), 0777) && errno != EEXIST)
            ROS_ERROR("Failed to create directory: %s", output_dir_.c_str());

        sub_ = nh.subscribe("/lidar_points", 1, &LidarIntensityCLAHEORB::callback, this);
        ROS_INFO("LidarIntensityCLAHEORB node initialized.");
    }

private:
    void callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // if (saved_) return;  // ֻ������һ֡
        ROS_INFO("Processing lidar frame...");

        // int h_res_count = static_cast<int>(h_fov_ / sample_res_);
        int h_res_count =500;
        cv::Mat intensity_img = cv::Mat::zeros(num_rings_, h_res_count, CV_32F);

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
        sensor_msgs::PointCloud2ConstIterator<float> it_intensity(*msg, "intensity");

        float v_fov_range = v_fov_max_ - v_fov_min_;

        // ��������
        for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_intensity) {
            float x = *it_x;
            float y = *it_y;
            float z = *it_z;
            float intensity = *it_intensity;

            float vertical_angle = std::atan2(z, std::sqrt(x * x + y * y)) * 180.0 / M_PI;
            float horizontal_angle = std::atan2(y, x) * 180.0 / M_PI;

            int v = static_cast<int>((v_fov_max_ - vertical_angle) / v_fov_range * num_rings_);
            int u = static_cast<int>((horizontal_angle + h_fov_ / 2.0f) / h_fov_ * h_res_count);

            if (u >= 0 && u < h_res_count && v >= 0 && v < num_rings_) {
                float& pix = intensity_img.at<float>(v, u);
                if (intensity > pix) pix = intensity; // ȡ���ǿ��?
            }
        }

        // ��һ��Ϊ8λͼ��
        cv::Mat intensity_norm;
        cv::normalize(intensity_img, intensity_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

        // CLAHE ��ǿ
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
        cv::Mat clahe_img;
        clahe->apply(intensity_norm, clahe_img);

        // CLAHE + GaussianBlur
        cv::Mat clahe_blur_img;
        cv::GaussianBlur(clahe_img, clahe_blur_img, cv::Size(3, 3), 0);

        // ��������?
        cv::imwrite(output_dir_ + "/intensity_raw"+std::to_string(frame)+".png", intensity_norm);
        cv::imwrite(output_dir_ + "/intensity_CLAHE"+std::to_string(frame)+".png", clahe_img);
        cv::imwrite(output_dir_ + "/intensity_CLAHE_blur"+std::to_string(frame)+".png", clahe_blur_img);

        // ORB ������ȡ
        cv::Ptr<cv::ORB> orb = cv::ORB::create(
            orb_nfeatures_, 1.2f, 8, 31, 0, 2,
            cv::ORB::HARRIS_SCORE, 31, orb_fastThreshold_);

        std::vector<cv::KeyPoint> kp_raw, kp_clahe, kp_blur;
        cv::Mat desc_raw, desc_clahe, desc_blur;

        orb->detectAndCompute(intensity_norm, cv::noArray(), kp_raw, desc_raw);
        orb->detectAndCompute(clahe_img, cv::noArray(), kp_clahe, desc_clahe);
        orb->detectAndCompute(clahe_blur_img, cv::noArray(), kp_blur, desc_blur);

        ROS_INFO("ORB features: raw=%zu, clahe=%zu, clahe_blur=%zu",
                 kp_raw.size(), kp_clahe.size(), kp_blur.size());

        // ���ӻ���ֻ��СԲ��
        cv::Mat vis_raw, vis_clahe, vis_blur;
        cv::cvtColor(intensity_norm, vis_raw, cv::COLOR_GRAY2BGR);
        cv::cvtColor(clahe_img, vis_clahe, cv::COLOR_GRAY2BGR);
        cv::cvtColor(clahe_blur_img, vis_blur, cv::COLOR_GRAY2BGR);

        int r = orb_vis_radius_;
        for (auto& kp : kp_raw)
            cv::circle(vis_raw, kp.pt, r, cv::Scalar(0, 255, 0), -1);
        for (auto& kp : kp_clahe)
            cv::circle(vis_clahe, kp.pt, r, cv::Scalar(255, 0, 0), -1);
        for (auto& kp : kp_blur)
            cv::circle(vis_blur, kp.pt, r, cv::Scalar(0, 0, 255), -1);

        // ���� ORB ���ӻ�ͼ
        // cv::imwrite(output_dir_ + "/ORB_raw.png", vis_raw);
        // cv::imwrite(output_dir_ + "/ORB_CLAHE.png", vis_clahe);
        // cv::imwrite(output_dir_ + "/ORB_CLAHE_blur.png", vis_blur);

        // // ƴ�ӶԱ�
        // cv::Mat compare_gray, compare_orb;
        // cv::hconcat(std::vector<cv::Mat>{intensity_norm, clahe_img, clahe_blur_img}, compare_gray);
        // cv::hconcat(std::vector<cv::Mat>{vis_raw, vis_clahe, vis_blur}, compare_orb);
        // cv::imwrite(output_dir_ + "/compare_gray.png", compare_gray);
        // cv::imwrite(output_dir_ + "/compare_orb.png", compare_orb);

        ROS_INFO("Saved all intensity and ORB feature images to %s", output_dir_.c_str());
        // saved_ = true;
        frame++;
    }

    ros::Subscriber sub_;
    std::string output_dir_;
    bool saved_;
    int num_rings_;
    float v_fov_min_, v_fov_max_, h_fov_;
    float sample_res_;
    int orb_nfeatures_, orb_fastThreshold_;
    int orb_vis_radius_;
    int frame;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_intensity_clahe_orb_vis");
    ros::NodeHandle nh("~");

    LidarIntensityCLAHEORB node(nh);
    ros::spin();
    return 0;
}
