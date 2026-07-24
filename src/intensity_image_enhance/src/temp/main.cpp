// lidar_intensity_orb_match_node.cpp
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <chrono>
#include <vector>
#include <string>
#include <cmath>

template <typename T>
inline T clampValue(T v, T lo, T hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

class LidarIntensityORBMatch
{
public:
    explicit LidarIntensityORBMatch(ros::NodeHandle &nh) : nh_(nh), frame_idx_(0), has_prev_frame_(false)
    {
        nh_.param<std::string>("output_dir", output_dir_, "./lidar_output");
        nh_.param<int>("sample_step", sample_step_, 1);
        nh_.param<std::string>("fusion_mode", fusion_mode_, std::string("max"));
        nh_.param<int>("v_res", v_res_, 128);
        nh_.param<int>("h_res", h_res_, 500);
        nh_.param<double>("h_fov_deg", h_fov_deg_, 120.0);
        nh_.param<double>("v_min_deg", v_min_deg_, -12.5);
        nh_.param<double>("v_max_deg", v_max_deg_, 12.9);
        nh_.param<std::string>("contrast_mode", contrast_mode_, "clahe");
        nh_.param<bool>("enable_blur", enable_blur_, true);
        nh_.param<bool>("enable_inpaint", enable_inpaint_, false);

        nh_.param<int>("orb_nfeatures", orb_nfeatures_, 1000);
        nh_.param<double>("orb_scaleFactor", orb_scaleFactor_, 1.1);
        nh_.param<int>("orb_nlevels", orb_nlevels_, 4);
        nh_.param<int>("orb_edgeThreshold", orb_edgeThreshold_, 5);
        nh_.param<int>("orb_patchSize", orb_patchSize_, 15);
        nh_.param<int>("orb_fastThreshold", orb_fastThreshold_, 5);
        nh_.param<int>("orb_wta_k", orb_wta_k_, 2);
        orb_score_type_ = cv::ORB::HARRIS_SCORE;

        if (mkdir(output_dir_.c_str(), 0777) && errno != EEXIST)
            ROS_WARN("mkdir failed or exists: %s", output_dir_.c_str());

        min_az_ = -h_fov_deg_ / 2.0 * M_PI / 180.0;
        max_az_ = h_fov_deg_ / 2.0 * M_PI / 180.0;
        min_el_ = v_min_deg_ * M_PI / 180.0;
        max_el_ = v_max_deg_ * M_PI / 180.0;

        ROS_INFO("Node started. Saving to: %s", output_dir_.c_str());
        sub_ = nh_.subscribe("/lidar_points", 1, &LidarIntensityORBMatch::callback, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string output_dir_;
    int sample_step_, v_res_, h_res_;
    double h_fov_deg_, v_min_deg_, v_max_deg_;
    double min_az_, max_az_, min_el_, max_el_;
    std::string fusion_mode_, contrast_mode_;
    bool enable_blur_, enable_inpaint_;
    int orb_nfeatures_, orb_nlevels_, orb_edgeThreshold_, orb_patchSize_, orb_fastThreshold_, orb_wta_k_;
    double orb_scaleFactor_;
    cv::ORB::ScoreType orb_score_type_;
    int frame_idx_;
    bool has_prev_frame_;
    cv::Mat prev_img_, prev_desc_;
    std::vector<cv::KeyPoint> prev_kp_;

    struct PixAcc
    {
        float sum = 0;
        int cnt = 0;
        float maxv = 0;
    };
    void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!msg || msg->data.empty())
            return;
        auto t0 = std::chrono::high_resolution_clock::now();

        cv::Mat intensity(v_res_, h_res_, CV_32F, cv::Scalar(0));
        std::vector<PixAcc> acc(v_res_ * h_res_);

        sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z"), ii(*msg, "intensity");
        int idx = 0;
        for (; ix != ix.end(); ++ix, ++iy, ++iz, ++ii, ++idx)
        {
            if (sample_step_ > 1 && idx % sample_step_)
                continue;
            float x = *ix, y = *iy, z = *iz, i = *ii;
            double az = atan2(y, x), el = atan2(z, std::sqrt(x * x + y * y));
            if (az < min_az_ || az > max_az_ || el < min_el_ || el > max_el_)
                continue;
            int u = int((az - min_az_) / (max_az_ - min_az_) * (h_res_ - 1));
            int v = int((max_el_ - el) / (max_el_ - min_el_) * (v_res_ - 1));
            u = clampValue(u, 0, h_res_ - 1);
            v = clampValue(v, 0, v_res_ - 1);
            auto &p = acc[v * h_res_ + u];
            p.sum += i;
            p.cnt++;
            p.maxv = std::max(p.maxv, i);
        }
        for (int r = 0; r < v_res_; ++r)
            for (int c = 0; c < h_res_; ++c)
            {
                auto &p = acc[r * h_res_ + c];
                intensity.at<float>(r, c) = (fusion_mode_ == "max") ? p.maxv : (p.cnt ? p.sum / p.cnt : 0);
            }

        cv::Mat img8u;
        cv::normalize(intensity, img8u, 0, 255, cv::NORM_MINMAX, CV_8U);
        if (contrast_mode_ == "clahe")
        {
            cv::Ptr<cv::CLAHE> c = cv::createCLAHE(3.0, cv::Size(8, 8));
            c->apply(img8u, img8u);
        }
        else
            cv::equalizeHist(img8u, img8u);
        if (enable_blur_)
            cv::GaussianBlur(img8u, img8u, cv::Size(3, 3), 0);

        cv::Mat color;
        cv::applyColorMap(img8u, color, cv::COLORMAP_JET);
        cv::imwrite(output_dir_ + "/intensity_" + std::to_string(frame_idx_) + ".png", img8u);
        cv::imwrite(output_dir_ + "/color_" + std::to_string(frame_idx_) + ".png", color);

        auto orb = cv::ORB::create(orb_nfeatures_, float(orb_scaleFactor_), orb_nlevels_,
                                   orb_edgeThreshold_, 0, orb_wta_k_, orb_score_type_,
                                   orb_patchSize_, orb_fastThreshold_);
        std::vector<cv::KeyPoint> kp;
        cv::Mat desc;
        orb->detectAndCompute(img8u, cv::noArray(), kp, desc);

        if (has_prev_frame_ && !prev_desc_.empty() && !desc.empty())
        {
            // --- Matching with Ratio + RANSAC ---
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<std::vector<cv::DMatch>> knn;
            matcher.knnMatch(prev_desc_, desc, knn, 2);
            std::vector<cv::DMatch> good;
            for (auto &k : knn)
                if (k.size() >= 2 && k[0].distance < 0.75f * k[1].distance)
                    good.push_back(k[0]);
            std::vector<cv::DMatch> filt;
            for (auto &m : good)
                if (m.distance < 50)
                    filt.push_back(m);
            std::vector<cv::Point2f> p1, p2;
            for (auto &m : filt)
            {
                p1.push_back(prev_kp_[m.queryIdx].pt);
                p2.push_back(kp[m.trainIdx].pt);
            }
            std::vector<uchar> inliers;
            if (p1.size() >= 8)
                cv::findHomography(p1, p2, cv::RANSAC, 3.0, inliers);
            else
                inliers.assign(p1.size(), 1);
            std::vector<cv::DMatch> final;
            for (size_t i = 0; i < filt.size(); ++i)
                if (inliers[i])
                    final.push_back(filt[i]);
            ROS_INFO("Frame %d: %zu→%zu→%zu matches", frame_idx_, good.size(), filt.size(), final.size());

            // --- Vertical visualization ---
            int w = std::max(prev_img_.cols, img8u.cols);
            int h = prev_img_.rows + img8u.rows;
            cv::Mat vis(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
            cv::Mat top, bottom;
            cv::cvtColor(prev_img_, top, cv::COLOR_GRAY2BGR);
            cv::cvtColor(img8u, bottom, cv::COLOR_GRAY2BGR);
            top.copyTo(vis(cv::Rect(0, 0, top.cols, top.rows)));
            bottom.copyTo(vis(cv::Rect(0, top.rows, bottom.cols, bottom.rows)));
            for (auto &m : final)
            {
                cv::Point2f a = prev_kp_[m.queryIdx].pt;
                cv::Point2f b = kp[m.trainIdx].pt + cv::Point2f(0, top.rows);
                cv::line(vis, a, b, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
                cv::circle(vis, a, 2, cv::Scalar(0, 0, 255), -1);
                cv::circle(vis, b, 2, cv::Scalar(0, 255, 255), -1);
            }
            cv::imwrite(output_dir_ + "/match_" + std::to_string(frame_idx_) + ".png", vis);
        }

        prev_img_ = img8u.clone();
        prev_desc_ = desc.clone();
        prev_kp_ = kp;
        has_prev_frame_ = true;
        ROS_INFO("Frame %d: %zu keypoints", frame_idx_++, kp.size());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_intensity_orb_match_node");
    ros::NodeHandle nh("~");
    LidarIntensityORBMatch node(nh);
    ros::spin();
    return 0;
}
