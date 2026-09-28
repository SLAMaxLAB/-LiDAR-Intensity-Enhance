// Implementation of the intensity-assisted LiDAR odometry front end.
//
// The node projects a LiDAR cloud into a 2D intensity image, matches ORB
// features between consecutive frames, estimates the inter-frame transform
// T(cur -> prev), optionally refines it together with a constant-velocity
// deskew, and publishes the resulting clouds, images, and pose.

#include "LiDAR_Intensity_Assist/lidar_intensity_match.hpp"
#include "LiDAR_Intensity_Assist/math_utils.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <random>
#include <cstring>
#include <limits>
#include <cstdint>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace LiDAR_Intensity_Assist
{

LidarIntensityORBMatchDual::LidarIntensityORBMatchDual(ros::NodeHandle &nh)
        : nh_(nh), frame_idx_(0), has_prev_frame_(false)
{
    loadParameters();
    initAngleBounds();
    initRosInterfaces();
    logConfiguration();
}

void LidarIntensityORBMatchDual::loadParameters()
{
    // -------- parameter loading --------
    nh_.param<std::string>("projection_mode", projection_mode_, std::string("ring")); // "angle" or "ring"
    nh_.param<bool>("show_match_labels", show_match_labels_, true);
    nh_.param<bool>("publish_match_images", publish_match_images_, true);
    nh_.param<bool>("publish_raw_intensity_image", publish_raw_intensity_image_, true);
    nh_.param<bool>("publish_enhanced_intensity_image", publish_enhanced_intensity_image_, true);
    nh_.param<bool>("publish_matched_points", publish_matched_points_, true);
    nh_.param<bool>("latch_published_topics", latch_published_topics_, true);
    nh_.param<std::string>("raw_intensity_image_topic", raw_intensity_image_topic_, std::string("/raw_intensity_image"));
    nh_.param<std::string>("enhanced_intensity_image_topic", enhanced_intensity_image_topic_, std::string("/intensity_enh_image"));
    nh_.param<std::string>("matched_orb_image_topic", matched_orb_image_topic_, std::string("/matched_orb_image"));
    nh_.param<std::string>("matched_2d_image_topic", matched_2d_image_topic_, std::string("/matched_2d_image"));
    nh_.param<std::string>("matched_3d_image_topic", matched_3d_image_topic_, std::string("/matched_3d_image"));
    nh_.param<std::string>("matched_prev_points_topic", matched_prev_points_topic_, std::string("/matched_prev_orb_points"));
    nh_.param<std::string>("matched_cur_points_topic", matched_cur_points_topic_, std::string("/matched_cur_orb_points"));
    nh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/lidar_points"));
    nh_.param<std::string>("sampled_cloud_topic", sampled_cloud_topic_, std::string("/sampled_points"));
    nh_.param<std::string>("Tguess_topic", Tguess_topic_, std::string("/Tguess"));

    nh_.param<int>("v_res", v_res_, 128);
    nh_.param<int>("h_res", h_res_, 500);
    nh_.param<double>("h_fov_deg", h_fov_deg_, 120.0);
    nh_.param<double>("v_min_deg", v_min_deg_, -12.5);
    nh_.param<double>("v_max_deg", v_max_deg_, 12.9);
    nh_.param<int>("sample_step", sample_step_, 1);
    nh_.param<bool>("filter_origin_points", filter_origin_points_, true);
    nh_.param<float>("origin_filter_eps", origin_filter_eps_, 1e-6f);
    nh_.param<bool>("store_all_pixel_points", store_all_pixel_points_, true);

    nh_.param<bool>("enable_line_sampling", enable_line_sampling_, true);
    nh_.param<int>("source_lines", source_lines_, 128);
    nh_.param<int>("target_lines", target_lines_, 32);
    nh_.param<bool>("remap_ring_to_compact", remap_ring_to_compact_, true);

    // deskew & filtering (for Hesai AT128 with per-point timestamp)
    nh_.param<bool>("enable_deskew_current", enable_deskew_current_, true);
    nh_.param<bool>("deskew_use_inverse_tguess", deskew_use_inverse_tguess_, false);
    nh_.param<float>("deskew_min_range", deskew_min_range_, 0.1f);
    nh_.param<float>("deskew_placeholder_eps", deskew_placeholder_eps_, 1e-6f);
    nh_.param<bool>("sampling_filter_invalid", sampling_filter_invalid_, true);
    // deskew time scaling (scan span vs Tguess span)
    nh_.param<bool>("deskew_scale_by_time", deskew_scale_by_time_, true);
    nh_.param<double>("deskew_scale_min", deskew_scale_min_, 0.2);
    nh_.param<double>("deskew_scale_max", deskew_scale_max_, 2.0);

    nh_.param<float>("ratio_thresh", ratio_thresh_, 0.75f);
    nh_.param<int>("hamming_thresh", hamming_thresh_, 50);
    nh_.param<double>("ransac_2d_reproj", ransac_2d_reproj_, 3.0);
    nh_.param<double>("ransac_3d_thresh", ransac_3d_thresh_, 0.1);
    nh_.param<int>("ransac_3d_iters", ransac_3d_iters_, 100);

    nh_.param<bool>("enable_blur", enable_blur_, false);
    nh_.param<bool>("enable_bilateral_filter", enable_bilateral_filter_, true);
    nh_.param<int>("bilateral_d", bilateral_d_, 5);
    nh_.param<double>("bilateral_sigma_color", bilateral_sigma_color_, 30.0);
    nh_.param<double>("bilateral_sigma_space", bilateral_sigma_space_, 5.0);
    nh_.param<std::string>("contrast_mode", contrast_mode_, std::string("clahe"));
    nh_.param<int>("match_vis_radius", match_vis_radius_, 4);
    nh_.param<int>("match_vis_thickness", match_vis_thickness_, 1);

    nh_.param<bool>("enable_joint_tguess_deskew", enable_joint_tguess_deskew_, true);
    nh_.param<std::string>("joint_pixel_match_mode", joint_pixel_match_mode_, std::string("all"));
    if (joint_pixel_match_mode_ != "all" && joint_pixel_match_mode_ != "max_intensity")
    {
        ROS_WARN("Invalid joint_pixel_match_mode='%s', fallback to 'all'. Valid values: all, max_intensity",
                 joint_pixel_match_mode_.c_str());
        joint_pixel_match_mode_ = "all";
    }
    if (joint_pixel_match_mode_ == "all" && !store_all_pixel_points_)
    {
        ROS_WARN("joint_pixel_match_mode='all' requires store_all_pixel_points=true; fallback to 'max_intensity'.");
        joint_pixel_match_mode_ = "max_intensity";
    }
    nh_.param<int>("joint_tguess_max_iters", joint_tguess_max_iters_, 8);
    nh_.param<int>("joint_tguess_min_matches", joint_tguess_min_matches_, 6);
    nh_.param<double>("joint_tguess_huber_delta", joint_tguess_huber_delta_, 0.2);
    nh_.param<double>("joint_tguess_intensity_weight_scale", joint_tguess_intensity_weight_scale_, 0.5);
    nh_.param<int>("joint_tguess_num_threads", joint_tguess_num_threads_, 1);
    nh_.param<bool>("enable_joint_pair_outlier_rejection", enable_joint_pair_outlier_rejection_, true);
    nh_.param<double>("joint_pair_outlier_abs_thresh", joint_pair_outlier_abs_thresh_, 0.3);
    nh_.param<double>("joint_pair_outlier_mad_k", joint_pair_outlier_mad_k_, 4.0);
    nh_.param<double>("joint_pair_outlier_min_thresh", joint_pair_outlier_min_thresh_, 0.05);

    nh_.param<int>("orb_nfeatures", orb_nfeatures_, 300);
    nh_.param<double>("orb_scaleFactor", orb_scaleFactor_, 1.1);
    nh_.param<int>("orb_nlevels", orb_nlevels_, 4);
    nh_.param<int>("orb_edgeThreshold", orb_edgeThreshold_, 5);
    nh_.param<int>("orb_patchSize", orb_patchSize_, 31);
    nh_.param<int>("orb_fastThreshold", orb_fastThreshold_, 5);
    nh_.param<int>("orb_wta_k", orb_wta_k_, 2);
    int orb_score_type_int = 0;
    nh_.param<int>("orb_score_type", orb_score_type_int, 0);
    orb_score_type_ = (orb_score_type_int == 1) ? cv::ORB::FAST_SCORE : cv::ORB::HARRIS_SCORE;

    nh_.param<bool>("enable_ransac_2d", enable_ransac_2d_, true);
    nh_.param<bool>("enable_multithread", enable_multithread_, true);
    nh_.param<int>("num_threads", num_threads_, 6);
    nh_.param<bool>("enable_parallel_projection", enable_parallel_projection_, true);
    nh_.param<bool>("enable_parallel_ransac3d", enable_parallel_ransac3d_, true);
    nh_.param<int>("ransac3d_threads", ransac3d_threads_, num_threads_);
}

void LidarIntensityORBMatchDual::initAngleBounds()
{
    // angular bounds (radians)
    min_az_ = -h_fov_deg_ / 2.0 * M_PI / 180.0;
    max_az_ = h_fov_deg_ / 2.0 * M_PI / 180.0;
    min_el_ = v_min_deg_ * M_PI / 180.0;
    max_el_ = v_max_deg_ * M_PI / 180.0;
}

void LidarIntensityORBMatchDual::initRosInterfaces()
{
    // subscribers and publishers
    sub_ = nh_.subscribe(cloud_topic_, 1, &LidarIntensityORBMatchDual::callback, this);
    // Latching retains the newest result for RViz displays that subscribe after bag playback pauses.
    tguess_pub_ = nh_.advertise<nav_msgs::Odometry>(Tguess_topic_, 1, latch_published_topics_);
    sampled_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(sampled_cloud_topic_, 1, latch_published_topics_);
    cloudraw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("raw_cloud", 1, latch_published_topics_);
    deskewedcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("deskewed_cloud", 1, latch_published_topics_);
    raw_intensity_image_pub_ = nh_.advertise<sensor_msgs::Image>(raw_intensity_image_topic_, 1, latch_published_topics_);
    enhanced_intensity_image_pub_ = nh_.advertise<sensor_msgs::Image>(enhanced_intensity_image_topic_, 1, latch_published_topics_);
    matchedorb_image_pub_ = nh_.advertise<sensor_msgs::Image>(matched_orb_image_topic_, 1, latch_published_topics_);
    matched2d_image_pub_ = nh_.advertise<sensor_msgs::Image>(matched_2d_image_topic_, 1, latch_published_topics_);
    matched3d_image_pub_ = nh_.advertise<sensor_msgs::Image>(matched_3d_image_topic_, 1, latch_published_topics_);
    matchedprev_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(matched_prev_points_topic_, 1, latch_published_topics_);
    matchedcur_points_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(matched_cur_points_topic_, 1, latch_published_topics_);
    // RNG
    rng_.seed(std::random_device{}());

    // OpenCV threading setup
    if (enable_multithread_)
    {
        cv::setUseOptimized(true);
        cv::setNumThreads(num_threads_);
        ROS_INFO("OpenCV multithreading enabled. num_threads=%d", num_threads_);
    }
    else
    {
        cv::setNumThreads(1);
        ROS_INFO("OpenCV multithreading disabled, force num_threads=1");
    }
}

void LidarIntensityORBMatchDual::logConfiguration()
{
    ROS_INFO("Node initialized.");
    ROS_INFO("Image size: %d x %d ; h_fov: %.2f deg ; v_range: %.2f ~ %.2f deg",
             v_res_, h_res_, h_fov_deg_, v_min_deg_, v_max_deg_);
    ROS_INFO("Projection mode = %s (\"angle\" or \"ring\")",
             projection_mode_.c_str());
    ROS_INFO("Show match/image labels: %s",
             show_match_labels_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Publish match visualization images: %s",
             publish_match_images_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Publish raw intensity image: %s",
             publish_raw_intensity_image_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Raw intensity image topic: %s",
             raw_intensity_image_topic_.c_str());
    ROS_INFO("Publish enhanced intensity image: %s",
             publish_enhanced_intensity_image_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Enhanced intensity image topic: %s",
             enhanced_intensity_image_topic_.c_str());
    ROS_INFO("Publish matched point clouds: %s",
             publish_matched_points_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Latch published topics: %s",
             latch_published_topics_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Matched ORB image topic: %s",
             matched_orb_image_topic_.c_str());
    ROS_INFO("Matched 2D image topic: %s",
             matched_2d_image_topic_.c_str());
    ROS_INFO("Matched 3D image topic: %s",
             matched_3d_image_topic_.c_str());
    ROS_INFO("Matched prev 3D points topic: %s",
             matched_prev_points_topic_.c_str());
    ROS_INFO("Matched current 3D points topic: %s",
             matched_cur_points_topic_.c_str());
    ROS_INFO("Joint Tguess-deskew optimization: %s, iters=%d min_matches=%d huber=%.3f intensity_weight_scale=%.3f",
             enable_joint_tguess_deskew_ ? "ENABLED" : "DISABLED",
             joint_tguess_max_iters_, joint_tguess_min_matches_,
             joint_tguess_huber_delta_, joint_tguess_intensity_weight_scale_);
    ROS_INFO("Joint pixel match mode: %s",
             joint_pixel_match_mode_.c_str());
    ROS_INFO("Joint Tguess-deskew Ceres threads: %d",
             std::max(1, joint_tguess_num_threads_));
    ROS_INFO("Joint 3D pair outlier rejection: %s, abs_thresh=%.3f m mad_k=%.2f min_thresh=%.3f m",
             enable_joint_pair_outlier_rejection_ ? "ENABLED" : "DISABLED",
             joint_pair_outlier_abs_thresh_, joint_pair_outlier_mad_k_,
             joint_pair_outlier_min_thresh_);
    ROS_INFO("Image smoothing: gaussian=%s bilateral=%s d=%d sigma_color=%.2f sigma_space=%.2f",
             enable_blur_ ? "ENABLED" : "DISABLED",
             enable_bilateral_filter_ ? "ENABLED" : "DISABLED",
             bilateral_d_, bilateral_sigma_color_, bilateral_sigma_space_);
    ROS_INFO("Filter origin points: %s, eps=%.3e",
             filter_origin_points_ ? "ENABLED" : "DISABLED", origin_filter_eps_);
    ROS_INFO("Store all 3D points per intensity pixel: %s",
             store_all_pixel_points_ ? "ENABLED" : "DISABLED");
    ROS_INFO("Line sampling (publish only): %s, source_lines=%d target_lines=%d remap=%s",
             enable_line_sampling_ ? "ENABLED" : "DISABLED",
             source_lines_, target_lines_, remap_ring_to_compact_ ? "true" : "false");
    
}

cv::Point3d LidarIntensityORBMatchDual::deskewFeaturePointByPose(const cv::Point3f &q,
                                         double s,
                                         double alpha,
                                         const cv::Mat &R,
                                         const cv::Mat &t) const
{
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        rvec *= (s * alpha);

        cv::Mat R_s;
        cv::Rodrigues(rvec, R_s);

        cv::Mat q_mat = (cv::Mat_<double>(3, 1) << q.x, q.y, q.z);
        cv::Mat t_s = t * (s * alpha);
        cv::Mat q_deskew = R_s.t() * (q_mat - t_s);
        return cv::Point3d(q_deskew.at<double>(0),
                           q_deskew.at<double>(1),
                           q_deskew.at<double>(2));
    }

void LidarIntensityORBMatchDual::invertRigidTransform(const cv::Mat &R,
                                     const cv::Mat &t,
                                     cv::Mat &R_inv,
                                     cv::Mat &t_inv)
{
        R_inv = R.t();
        t_inv = -R_inv * t;
    }

cv::Point3d LidarIntensityORBMatchDual::mapCurrentFeatureToPreviousReference(const cv::Point3f &q,
                                                      double s,
                                                      double alpha,
                                                      const cv::Mat &R_cur_to_prev,
                                                      const cv::Mat &t_cur_to_prev,
                                                      const cv::Mat &R_prev_to_cur,
                                                      const cv::Mat &t_prev_to_cur) const
{
        const cv::Point3d q_deskew =
            deskewFeaturePointByPose(q, s, alpha, R_prev_to_cur, t_prev_to_cur);

        cv::Mat q_mat = (cv::Mat_<double>(3, 1) << q_deskew.x, q_deskew.y, q_deskew.z);
        cv::Mat q_in_prev = R_cur_to_prev * q_mat + t_cur_to_prev;
        return cv::Point3d(q_in_prev.at<double>(0),
                           q_in_prev.at<double>(1),
                           q_in_prev.at<double>(2));
    }

cv::Point3f LidarIntensityORBMatchDual::toPoint3f(const PixelPoint &p)
{
        return cv::Point3f(p.x, p.y, p.z);
    }

void LidarIntensityORBMatchDual::materializePixelPointBuckets(
        size_t total_pix,
        const std::vector<std::vector<PixelPointRecord>> &thread_point_records,
        std::vector<std::vector<PixelPoint>> &pixel_points) const
{
        pixel_points.assign(total_pix, std::vector<PixelPoint>());
        if (!store_all_pixel_points_)
            return;

        std::vector<size_t> counts(total_pix, 0);
        for (const auto &records : thread_point_records)
        {
            for (const auto &record : records)
            {
                if (record.pid < total_pix)
                    ++counts[record.pid];
            }
        }

        for (size_t pid = 0; pid < total_pix; ++pid)
        {
            if (counts[pid] > 0)
                pixel_points[pid].reserve(counts[pid]);
        }

        for (const auto &records : thread_point_records)
        {
            for (const auto &record : records)
            {
                if (record.pid < total_pix)
                    pixel_points[record.pid].push_back(record.point);
            }
        }
    }

double LidarIntensityORBMatchDual::medianValue(std::vector<double> values)
{
        if (values.empty())
            return std::numeric_limits<double>::quiet_NaN();

        std::sort(values.begin(), values.end());
        const size_t mid = values.size() / 2;
        if ((values.size() % 2) == 1)
            return values[mid];

        return 0.5 * (values[mid - 1] + values[mid]);
    }

size_t LidarIntensityORBMatchDual::buildNearestPixelSetCorrespondences(
        const std::vector<std::pair<int, int>> &pixel_pairs,
        const std::vector<std::vector<PixelPoint>> &prev_pixel_points,
        const std::vector<std::vector<PixelPoint>> &cur_pixel_points,
        double cur_ts_min,
        double cur_ts_span,
        double alpha,
        const cv::Mat &R,
        const cv::Mat &t,
        std::vector<cv::Point3f> &P,
        std::vector<cv::Point3f> &Q,
        std::vector<double> &P_intensity,
        std::vector<double> &Q_intensity,
        std::vector<double> &P_ts,
        std::vector<double> &Q_ts,
        size_t &raw_pairs_out,
        size_t &rejected_pairs_out,
        double &reject_threshold_out) const
{
        P.clear();
        Q.clear();
        P_intensity.clear();
        Q_intensity.clear();
        P_ts.clear();
        Q_ts.clear();
        raw_pairs_out = 0;
        rejected_pairs_out = 0;
        reject_threshold_out = std::numeric_limits<double>::infinity();

        if (!(cur_ts_span > 0.0))
            return 0;

        struct Candidate
        {
            cv::Point3f p;
            cv::Point3f q;
            double p_intensity = 0.0;
            double q_intensity = 0.0;
            double p_ts = std::numeric_limits<double>::quiet_NaN();
            double q_ts = std::numeric_limits<double>::quiet_NaN();
            double residual_dist = std::numeric_limits<double>::infinity();
        };

        std::vector<Candidate> candidates;
        cv::Mat R_prev_to_cur, t_prev_to_cur;
        invertRigidTransform(R, t, R_prev_to_cur, t_prev_to_cur);

        for (const auto &pair : pixel_pairs)
        {
            const int prev_id = pair.first;
            const int cur_id = pair.second;
            if (prev_id < 0 || cur_id < 0 ||
                prev_id >= static_cast<int>(prev_pixel_points.size()) ||
                cur_id >= static_cast<int>(cur_pixel_points.size()))
            {
                continue;
            }

            const auto &prev_set = prev_pixel_points[(size_t)prev_id];
            const auto &cur_set = cur_pixel_points[(size_t)cur_id];
            if (prev_set.empty() || cur_set.empty())
                continue;

            for (const PixelPoint &q_point : cur_set)
            {
                if (!std::isfinite(q_point.timestamp))
                    continue;

                double s = (q_point.timestamp - cur_ts_min) / cur_ts_span;
                s = std::max(0.0, std::min(1.0, s));
                const cv::Point3d q_in_prev = mapCurrentFeatureToPreviousReference(
                    toPoint3f(q_point), s, alpha, R, t, R_prev_to_cur, t_prev_to_cur);

                size_t best_prev_idx = std::numeric_limits<size_t>::max();
                double best_dist2 = std::numeric_limits<double>::infinity();
                for (size_t p_idx = 0; p_idx < prev_set.size(); ++p_idx)
                {
                    const PixelPoint &p_ref = prev_set[p_idx];
                    const double dx = q_in_prev.x - p_ref.x;
                    const double dy = q_in_prev.y - p_ref.y;
                    const double dz = q_in_prev.z - p_ref.z;
                    const double dist2 = dx * dx + dy * dy + dz * dz;
                    if (dist2 < best_dist2)
                    {
                        best_dist2 = dist2;
                        best_prev_idx = p_idx;
                    }
                }

                if (best_prev_idx >= prev_set.size())
                    continue;

                const PixelPoint &best_prev = prev_set[best_prev_idx];
                Candidate c;
                c.p = toPoint3f(best_prev);
                c.q = toPoint3f(q_point);
                c.p_intensity = best_prev.intensity;
                c.q_intensity = q_point.intensity;
                c.p_ts = best_prev.timestamp;
                c.q_ts = q_point.timestamp;
                c.residual_dist = std::sqrt(best_dist2);
                candidates.push_back(c);
            }
        }

        raw_pairs_out = candidates.size();
        if (candidates.empty())
            return 0;

        double reject_threshold = std::numeric_limits<double>::infinity();
        if (enable_joint_pair_outlier_rejection_)
        {
            if (joint_pair_outlier_abs_thresh_ > 0.0 &&
                std::isfinite(joint_pair_outlier_abs_thresh_))
            {
                reject_threshold = std::min(reject_threshold, joint_pair_outlier_abs_thresh_);
            }

            if (joint_pair_outlier_mad_k_ > 0.0 && candidates.size() >= 5)
            {
                std::vector<double> dists;
                dists.reserve(candidates.size());
                for (const auto &c : candidates)
                {
                    if (std::isfinite(c.residual_dist))
                        dists.push_back(c.residual_dist);
                }

                const double median_dist = medianValue(dists);
                if (std::isfinite(median_dist))
                {
                    std::vector<double> abs_dev;
                    abs_dev.reserve(dists.size());
                    for (double d : dists)
                        abs_dev.push_back(std::fabs(d - median_dist));

                    const double mad = medianValue(abs_dev);
                    if (std::isfinite(mad))
                    {
                        const double robust_sigma = 1.4826 * mad;
                        double robust_thresh = median_dist + joint_pair_outlier_mad_k_ * robust_sigma;
                        if (joint_pair_outlier_min_thresh_ > 0.0 &&
                            std::isfinite(joint_pair_outlier_min_thresh_))
                        {
                            robust_thresh = std::max(robust_thresh, joint_pair_outlier_min_thresh_);
                        }
                        reject_threshold = std::min(reject_threshold, robust_thresh);
                    }
                }
            }
        }
        reject_threshold_out = reject_threshold;

        P.reserve(candidates.size());
        Q.reserve(candidates.size());
        P_intensity.reserve(candidates.size());
        Q_intensity.reserve(candidates.size());
        P_ts.reserve(candidates.size());
        Q_ts.reserve(candidates.size());

        for (const auto &c : candidates)
        {
            if (!std::isfinite(c.residual_dist))
            {
                ++rejected_pairs_out;
                continue;
            }

            if (enable_joint_pair_outlier_rejection_ &&
                std::isfinite(reject_threshold) &&
                c.residual_dist > reject_threshold)
            {
                ++rejected_pairs_out;
                continue;
            }

            P.push_back(c.p);
            Q.push_back(c.q);
            P_intensity.push_back(c.p_intensity);
            Q_intensity.push_back(c.q_intensity);
            P_ts.push_back(c.p_ts);
            Q_ts.push_back(c.q_ts);
        }

        return P.size();
    }

double LidarIntensityORBMatchDual::computeJointTguessDeskewRmse(const std::vector<cv::Point3f> &P,
                                        const std::vector<cv::Point3f> &Q,
                                        const std::vector<double> &Q_ts,
                                        double cur_ts_min,
                                        double cur_ts_span,
                                        double alpha,
                                        const cv::Mat &R,
                                        const cv::Mat &t) const
{
        if (P.empty() || P.size() != Q.size() || Q_ts.size() != Q.size() ||
            !(cur_ts_span > 0.0))
        {
            return std::numeric_limits<double>::quiet_NaN();
        }

        double sum_sq = 0.0;
        size_t used = 0;
        cv::Mat R_prev_to_cur, t_prev_to_cur;
        invertRigidTransform(R, t, R_prev_to_cur, t_prev_to_cur);
        for (size_t i = 0; i < P.size(); ++i)
        {
            if (!std::isfinite(Q_ts[i]))
                continue;
            double s = (Q_ts[i] - cur_ts_min) / cur_ts_span;
            s = std::max(0.0, std::min(1.0, s));

            const cv::Point3d q_in_prev = mapCurrentFeatureToPreviousReference(
                Q[i], s, alpha, R, t, R_prev_to_cur, t_prev_to_cur);
            const double dx = q_in_prev.x - P[i].x;
            const double dy = q_in_prev.y - P[i].y;
            const double dz = q_in_prev.z - P[i].z;
            sum_sq += dx * dx + dy * dy + dz * dz;
            ++used;
        }
        if (used == 0)
            return std::numeric_limits<double>::quiet_NaN();
        return std::sqrt(sum_sq / static_cast<double>(used));
    }

bool LidarIntensityORBMatchDual::refineTguessWithJointDeskew(const std::vector<cv::Point3f> &P,
                                     const std::vector<cv::Point3f> &Q,
                                     const std::vector<double> &P_intensity,
                                     const std::vector<double> &Q_intensity,
                                     const std::vector<double> &Q_ts,
                                     double cur_ts_min,
                                     double cur_ts_span,
                                     double alpha,
                                     cv::Mat &R,
                                     cv::Mat &t,
                                     double &rmse_before,
                                     double &rmse_after,
                                     size_t &used_out,
                                     int &iterations_out,
                                     bool &solution_usable_out) const
{
        rmse_before = rmse_after = std::numeric_limits<double>::quiet_NaN();
        used_out = 0;
        iterations_out = 0;
        solution_usable_out = false;
        if (!enable_joint_tguess_deskew_ ||
            P.size() != Q.size() ||
            P.size() != Q_ts.size() ||
            P.size() < static_cast<size_t>(std::max(3, joint_tguess_min_matches_)) ||
            !(cur_ts_span > 0.0) ||
            joint_tguess_max_iters_ <= 0)
        {
            return false;
        }

        double max_avg_intensity = 0.0;
        for (size_t i = 0; i < P.size(); ++i)
        {
            if (i < P_intensity.size() && i < Q_intensity.size() &&
                std::isfinite(P_intensity[i]) && std::isfinite(Q_intensity[i]))
            {
                max_avg_intensity = std::max(max_avg_intensity, 0.5 * (P_intensity[i] + Q_intensity[i]));
            }
        }

        cv::Mat rvec;
        cv::Rodrigues(R, rvec);
        double pose[6] = {
            rvec.at<double>(0),
            rvec.at<double>(1),
            rvec.at<double>(2),
            t.at<double>(0),
            t.at<double>(1),
            t.at<double>(2),
        };

        rmse_before = computeJointTguessDeskewRmse(P, Q, Q_ts, cur_ts_min, cur_ts_span, alpha, R, t);

        ceres::Problem problem;
        size_t used = 0;
        for (size_t i = 0; i < P.size(); ++i)
        {
            if (!std::isfinite(Q_ts[i]))
                continue;
            double s = (Q_ts[i] - cur_ts_min) / cur_ts_span;
            s = std::max(0.0, std::min(1.0, s));

            double weight = 1.0;
            if (joint_tguess_intensity_weight_scale_ > 0.0 &&
                max_avg_intensity > 1e-12 &&
                i < P_intensity.size() && i < Q_intensity.size() &&
                std::isfinite(P_intensity[i]) && std::isfinite(Q_intensity[i]))
            {
                const double avg_intensity = 0.5 * (P_intensity[i] + Q_intensity[i]);
                const double norm_intensity = std::max(0.0, std::min(1.0, avg_intensity / max_avg_intensity));
                weight += joint_tguess_intensity_weight_scale_ * norm_intensity;
            }

            ceres::CostFunction *cost =
                new ceres::AutoDiffCostFunction<JointTguessDeskewResidual, 3, 6>(
                    new JointTguessDeskewResidual(P[i], Q[i], s, alpha, std::sqrt(weight)));
            ceres::LossFunction *loss = nullptr;
            if (joint_tguess_huber_delta_ > 0.0)
                loss = new ceres::HuberLoss(joint_tguess_huber_delta_);
            problem.AddResidualBlock(cost, loss, pose);
            ++used;
        }

        if (used < static_cast<size_t>(std::max(3, joint_tguess_min_matches_)))
            return false;
        used_out = used;

        ceres::Solver::Options options;
        options.max_num_iterations = joint_tguess_max_iters_;
        options.linear_solver_type = ceres::DENSE_QR;
        options.num_threads = std::max(1, joint_tguess_num_threads_);
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        cv::Mat rvec_opt = (cv::Mat_<double>(3, 1) << pose[0], pose[1], pose[2]);
        cv::Rodrigues(rvec_opt, R);
        t = (cv::Mat_<double>(3, 1) << pose[3], pose[4], pose[5]);
        rmse_after = computeJointTguessDeskewRmse(P, Q, Q_ts, cur_ts_min, cur_ts_span, alpha, R, t);
        const bool solution_usable = summary.IsSolutionUsable();
        iterations_out = static_cast<int>(summary.iterations.size());
        solution_usable_out = solution_usable;

        ROS_INFO("Joint Tguess-deskew optimize objective: used=%zu alpha=%.3f rmse=%.4f->%.4f cost=%.6f->%.6f iters=%d status=%s",
                 used, alpha, rmse_before, rmse_after,
                 summary.initial_cost, summary.final_cost,
                 static_cast<int>(summary.iterations.size()),
                 summary.BriefReport().c_str());
        return solution_usable;
    }

bool LidarIntensityORBMatchDual::estimateRigidSVD(const std::vector<cv::Point3f> &P,
                          const std::vector<cv::Point3f> &Q,
                          cv::Mat &R, cv::Mat &t)
{
        if (P.size() < 3)
            return false;
        cv::Point3d meanP(0, 0, 0), meanQ(0, 0, 0);
        for (size_t i = 0; i < P.size(); ++i)
        {
            meanP += cv::Point3d(P[i].x, P[i].y, P[i].z);
            meanQ += cv::Point3d(Q[i].x, Q[i].y, Q[i].z);
        }
        meanP *= 1.0 / P.size();
        meanQ *= 1.0 / P.size();

        cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
        for (size_t i = 0; i < P.size(); ++i)
        {
            cv::Mat p = (cv::Mat_<double>(3, 1) << P[i].x - meanP.x, P[i].y - meanP.y, P[i].z - meanP.z);
            cv::Mat q = (cv::Mat_<double>(1, 3) << Q[i].x - meanQ.x, Q[i].y - meanQ.y, Q[i].z - meanQ.z);
            H += p * q;
        }
        cv::Mat U, S, Vt;
        cv::SVD::compute(H, S, U, Vt);
        R = Vt.t() * U.t();
        if (cv::determinant(R) < 0)
        {
            Vt.row(2) *= -1;
            R = Vt.t() * U.t();
        }
        cv::Mat meanP_m = (cv::Mat_<double>(3, 1) << meanP.x, meanP.y, meanP.z);
        cv::Mat meanQ_m = (cv::Mat_<double>(3, 1) << meanQ.x, meanQ.y, meanQ.z);
        t = meanQ_m - R * meanP_m;
        return true;
    }

void LidarIntensityORBMatchDual::applyRT(const cv::Mat &R, const cv::Mat &t,
                        const cv::Point3f &p, cv::Point3f &q) const
{
        const double *r = (const double *)R.data;
        const double *tv = (const double *)t.data;
        double x = p.x, y = p.y, z = p.z;
        q.x = (float)(r[0] * x + r[1] * y + r[2] * z + tv[0]);
        q.y = (float)(r[3] * x + r[4] * y + r[5] * z + tv[1]);
        q.z = (float)(r[6] * x + r[7] * y + r[8] * z + tv[2]);
    }

std::vector<int> LidarIntensityORBMatchDual::ransac3D(const std::vector<cv::Point3f> &P,
                              const std::vector<cv::Point3f> &Q,
                              double thresh, int iters)
{
        std::vector<int> best_inliers;
        if (P.size() < 3)
            return best_inliers;
        std::uniform_int_distribution<int> uni(0, (int)P.size() - 1);

        for (int it = 0; it < iters; ++it)
        {
            int a = uni(rng_), b = uni(rng_), c = uni(rng_);
            if (a == b || a == c || b == c)
            {
                --it;
                continue;
            }

            std::vector<cv::Point3f> Ps = {P[a], P[b], P[c]};
            std::vector<cv::Point3f> Qs = {Q[a], Q[b], Q[c]};
            cv::Mat Rtmp, ttmp;
            if (!estimateRigidSVD(Ps, Qs, Rtmp, ttmp))
                continue;

            std::vector<int> inliers;
            inliers.reserve(P.size());
            for (size_t i = 0; i < P.size(); ++i)
            {
                cv::Point3f q_est;
                applyRT(Rtmp, ttmp, P[i], q_est);
                if (std::sqrt(dist3D2(q_est, Q[i])) < thresh)
                    inliers.push_back((int)i);
            }
            if (inliers.size() > best_inliers.size())
                best_inliers.swap(inliers);
        }

        ROS_INFO("3D-RANSAC result: kept %zu / %zu (%.1f%%)",
                 best_inliers.size(), P.size(),
                 100.0 * best_inliers.size() / std::max<size_t>(1, P.size()));
        return best_inliers;
    }

std::vector<int> LidarIntensityORBMatchDual::ransac3D_parallel(const std::vector<cv::Point3f> &P,
                                       const std::vector<cv::Point3f> &Q,
                                       double thresh, int iters, int threads)
{
        std::vector<int> best_global;
        if (P.size() < 3)
            return best_global;
        const int N = (int)P.size();
        threads = std::max(1, threads);

#ifdef _OPENMP
#pragma omp parallel num_threads(threads)
#endif
        {
            std::mt19937 rng_local((uint32_t)(0x9e3779b1u ^ (frame_idx_ * 1315423911u)));
#ifdef _OPENMP
            rng_local.discard((uint32_t)omp_get_thread_num() * 7);
#endif
            std::uniform_int_distribution<int> uni(0, N - 1);

            std::vector<int> best_local;

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
            for (int it = 0; it < iters; ++it)
            {
                int a = uni(rng_local), b = uni(rng_local), c = uni(rng_local);
                if (a == b || a == c || b == c)
                {
                    --it;
                    continue;
                }

                std::vector<cv::Point3f> Ps = {P[a], P[b], P[c]};
                std::vector<cv::Point3f> Qs = {Q[a], Q[b], Q[c]};
                cv::Mat Rtmp, ttmp;
                if (!estimateRigidSVD(Ps, Qs, Rtmp, ttmp))
                    continue;

                std::vector<int> inliers;
                inliers.reserve(N);
                for (int i = 0; i < N; ++i)
                {
                    cv::Point3f q_est;
                    applyRT(Rtmp, ttmp, P[i], q_est);
                    float dx = q_est.x - Q[i].x;
                    float dy = q_est.y - Q[i].y;
                    float dz = q_est.z - Q[i].z;
                    if (std::sqrt(dx * dx + dy * dy + dz * dz) < (float)thresh)
                        inliers.push_back(i);
                }
                if (inliers.size() > best_local.size())
                    best_local.swap(inliers);
            }

#ifdef _OPENMP
#pragma omp critical
#endif
            {
                if (best_local.size() > best_global.size())
                    best_global.swap(best_local);
            }
        }

        ROS_INFO("3D-RANSAC(parallel) result: kept %zu / %zu (%.1f%%)",
                 best_global.size(), P.size(),
                 100.0 * best_global.size() / std::max<size_t>(1, P.size()));
        return best_global;
    }

void LidarIntensityORBMatchDual::drawImageLabel(cv::Mat &image_bgr, const std::string &text,
                        int y_offset, const cv::Scalar &color) const
{
        if (text.empty())
            return;

        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const double font_scale = 0.35;
        const int thickness = 1;
        int baseline = 0;
        cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
        cv::Rect bg_rect(8, y_offset + 8, text_size.width + 16, text_size.height + baseline + 12);

        cv::rectangle(image_bgr, bg_rect, cv::Scalar(20, 20, 20), cv::FILLED);
        cv::rectangle(image_bgr, bg_rect, color, 1);
        cv::putText(image_bgr, text,
                    cv::Point(bg_rect.x + 8, bg_rect.y + text_size.height + 3),
                    font, font_scale, color, thickness, cv::LINE_AA);
    }

cv::Mat LidarIntensityORBMatchDual::buildMatchesStackedVisualization(const cv::Mat &prev_img, const cv::Mat &cur_img,
                                             const std::vector<cv::KeyPoint> &prev_kp,
                                             const std::vector<cv::KeyPoint> &cur_kp,
                                             const std::vector<cv::DMatch> &matches,
                                             const std::string &prev_label,
                                             const std::string &cur_label)
{
        cv::Mat prev_color, cur_color;
        cv::cvtColor(prev_img, prev_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(cur_img, cur_color, cv::COLOR_GRAY2BGR);

        int w = std::max(prev_color.cols, cur_color.cols);
        int h = prev_color.rows + cur_color.rows;
        cv::Mat canvas(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
        prev_color.copyTo(canvas(cv::Rect(0, 0, prev_color.cols, prev_color.rows)));
        cur_color.copyTo(canvas(cv::Rect(0, prev_color.rows, cur_color.cols, cur_color.rows)));
        cv::line(canvas, cv::Point(0, prev_color.rows), cv::Point(w - 1, prev_color.rows),
                 cv::Scalar(255, 255, 255), 2, cv::LINE_AA);

        drawImageLabel(canvas, prev_label, 0, cv::Scalar(0, 215, 255));
        drawImageLabel(canvas, cur_label, prev_color.rows, cv::Scalar(80, 255, 80));

        const cv::Scalar match_color(0, 255, 0);
        const int match_radius = std::max(match_vis_radius_, 1);
        const int match_thickness = std::max(match_vis_thickness_, 1);

        for (const auto &m : matches)
        {
            cv::Point2f a = prev_kp[m.queryIdx].pt;
            cv::Point2f b = cur_kp[m.trainIdx].pt + cv::Point2f(0.0f, (float)prev_color.rows);
            cv::circle(canvas, a, match_radius, match_color, match_thickness, cv::LINE_AA);
            cv::circle(canvas, b, match_radius, match_color, match_thickness, cv::LINE_AA);
            cv::line(canvas, a, b, match_color, match_thickness, cv::LINE_AA);
        }
        return canvas;
    }

bool LidarIntensityORBMatchDual::shouldPublishImage(const ros::Publisher &publisher) const
{
        return publish_match_images_ &&
               (latch_published_topics_ || publisher.getNumSubscribers() > 0);
    }

bool LidarIntensityORBMatchDual::shouldPublishRawIntensityImage() const
{
        return publish_raw_intensity_image_ &&
               (latch_published_topics_ || raw_intensity_image_pub_.getNumSubscribers() > 0);
    }

bool LidarIntensityORBMatchDual::shouldPublishEnhancedIntensityImage() const
{
        return publish_enhanced_intensity_image_ &&
               (latch_published_topics_ || enhanced_intensity_image_pub_.getNumSubscribers() > 0);
    }

bool LidarIntensityORBMatchDual::shouldPublishMatchedPointClouds() const
{
        return publish_matched_points_ &&
               (latch_published_topics_ ||
                matchedprev_points_pub_.getNumSubscribers() > 0 ||
                matchedcur_points_pub_.getNumSubscribers() > 0);
    }

void LidarIntensityORBMatchDual::publishVisualizationImage(const cv::Mat &image,
                                   const std_msgs::Header &header,
                                   ros::Publisher &publisher)
{
        if (image.empty())
            return;

        cv::Mat image_bgr = image;
        if (!image_bgr.isContinuous())
            image_bgr = image_bgr.clone();

        sensor_msgs::Image msg;
        msg.header = header;
        msg.height = static_cast<uint32_t>(image_bgr.rows);
        msg.width = static_cast<uint32_t>(image_bgr.cols);
        msg.encoding = "bgr8";
        msg.is_bigendian = false;
        msg.step = static_cast<sensor_msgs::Image::_step_type>(image_bgr.step);

        const size_t bytes = static_cast<size_t>(msg.step) * static_cast<size_t>(msg.height);
        msg.data.assign(image_bgr.datastart, image_bgr.datastart + bytes);
        publisher.publish(msg);
    }

void LidarIntensityORBMatchDual::publishLabeledIntensityImage(const cv::Mat &image, const std::string &label,
                                      const std_msgs::Header &header, ros::Publisher &publisher)
{
        if (image.empty() || image.type() != CV_8UC1)
            return;

        cv::Mat image_bgr;
        cv::cvtColor(image, image_bgr, cv::COLOR_GRAY2BGR);
        drawImageLabel(image_bgr, label, 0, cv::Scalar(80, 255, 80));
        publishVisualizationImage(image_bgr, header, publisher);
    }

void LidarIntensityORBMatchDual::publishRawIntensityImage(const cv::Mat &image, const std::string &label,
                                  const std_msgs::Header &header)
{
        publishLabeledIntensityImage(image, label, header, raw_intensity_image_pub_);
    }

void LidarIntensityORBMatchDual::publishEnhancedIntensityImage(const cv::Mat &image, const std::string &label,
                                       const std_msgs::Header &header)
{
        publishLabeledIntensityImage(image, label, header, enhanced_intensity_image_pub_);
    }

void LidarIntensityORBMatchDual::publishMatchedORBImage(const cv::Mat &image, const std_msgs::Header &header)
{
        publishVisualizationImage(image, header, matchedorb_image_pub_);
    }

void LidarIntensityORBMatchDual::publishMatched2DImage(const cv::Mat &image, const std_msgs::Header &header)
{
        publishVisualizationImage(image, header, matched2d_image_pub_);
    }

void LidarIntensityORBMatchDual::publishMatched3DImage(const cv::Mat &image, const std_msgs::Header &header)
{
        publishVisualizationImage(image, header, matched3d_image_pub_);
    }

sensor_msgs::PointCloud2 LidarIntensityORBMatchDual::buildMatchedPointsCloud(const std::vector<cv::Point3f> &points,
                                                     const std::vector<double> &intensities,
                                                     const std::vector<double> &timestamps,
                                                     const std_msgs::Header &header) const
{
        sensor_msgs::PointCloud2 cloud;
        cloud.header = header;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(points.size());
        cloud.is_bigendian = false;
        cloud.is_dense = false;

        cloud.fields.resize(6);
        auto setField = [&](size_t idx,
                            const std::string &name,
                            uint32_t offset,
                            uint8_t datatype)
        {
            cloud.fields[idx].name = name;
            cloud.fields[idx].offset = offset;
            cloud.fields[idx].datatype = datatype;
            cloud.fields[idx].count = 1;
        };

        setField(0, "x", 0, sensor_msgs::PointField::FLOAT32);
        setField(1, "y", 4, sensor_msgs::PointField::FLOAT32);
        setField(2, "z", 8, sensor_msgs::PointField::FLOAT32);
        setField(3, "intensity", 12, sensor_msgs::PointField::FLOAT32);
        setField(4, "timestamp", 16, sensor_msgs::PointField::FLOAT64);
        setField(5, "match_index", 24, sensor_msgs::PointField::UINT32);

        cloud.point_step = 28;
        cloud.row_step = cloud.point_step * cloud.width;
        cloud.data.assign(static_cast<size_t>(cloud.row_step), 0);

        for (size_t i = 0; i < points.size(); ++i)
        {
            const float x = points[i].x;
            const float y = points[i].y;
            const float z = points[i].z;
            const float intensity =
                (i < intensities.size() && std::isfinite(intensities[i]))
                    ? static_cast<float>(intensities[i])
                    : 0.0f;
            const double timestamp =
                (i < timestamps.size() && std::isfinite(timestamps[i]))
                    ? timestamps[i]
                    : std::numeric_limits<double>::quiet_NaN();
            const uint32_t match_index = static_cast<uint32_t>(i);

            uint8_t *dst = cloud.data.data() + i * cloud.point_step;
            std::memcpy(dst + 0, &x, sizeof(float));
            std::memcpy(dst + 4, &y, sizeof(float));
            std::memcpy(dst + 8, &z, sizeof(float));
            std::memcpy(dst + 12, &intensity, sizeof(float));
            std::memcpy(dst + 16, &timestamp, sizeof(double));
            std::memcpy(dst + 24, &match_index, sizeof(uint32_t));
        }

        return cloud;
    }

void LidarIntensityORBMatchDual::publishMatched3DPointClouds(const std::vector<cv::Point3f> &prev_points,
                                     const std::vector<cv::Point3f> &cur_points,
                                     const std::vector<double> &prev_intensities,
                                     const std::vector<double> &cur_intensities,
                                     const std::vector<double> &prev_timestamps,
                                     const std::vector<double> &cur_timestamps,
                                     const std_msgs::Header &prev_header,
                                     const std_msgs::Header &cur_header)
{
        sensor_msgs::PointCloud2 prev_cloud =
            buildMatchedPointsCloud(prev_points, prev_intensities, prev_timestamps, prev_header);
        sensor_msgs::PointCloud2 cur_cloud =
            buildMatchedPointsCloud(cur_points, cur_intensities, cur_timestamps, cur_header);

        matchedprev_points_pub_.publish(prev_cloud);
        matchedcur_points_pub_.publish(cur_cloud);
    }

bool LidarIntensityORBMatchDual::filterOriginPoints(const sensor_msgs::PointCloud2 &in,
                            sensor_msgs::PointCloud2 &out,
                            size_t &removed_points) const
{
        removed_points = 0;

        if (!filter_origin_points_)
            return false;

        int offset_x = -1, offset_y = -1, offset_z = -1;
        int dtype_x = -1, dtype_y = -1, dtype_z = -1;
        for (const auto &f : in.fields)
        {
            if (f.name == "x")
            {
                offset_x = f.offset;
                dtype_x = f.datatype;
            }
            else if (f.name == "y")
            {
                offset_y = f.offset;
                dtype_y = f.datatype;
            }
            else if (f.name == "z")
            {
                offset_z = f.offset;
                dtype_z = f.datatype;
            }
        }

        if (offset_x < 0 || offset_y < 0 || offset_z < 0 ||
            dtype_x != sensor_msgs::PointField::FLOAT32 ||
            dtype_y != sensor_msgs::PointField::FLOAT32 ||
            dtype_z != sensor_msgs::PointField::FLOAT32)
        {
            ROS_WARN_THROTTLE(1.0, "origin point filter disabled: x/y/z FLOAT32 fields not found.");
            return false;
        }

        const size_t point_step = in.point_step;
        if ((size_t)offset_x + sizeof(float) > point_step ||
            (size_t)offset_y + sizeof(float) > point_step ||
            (size_t)offset_z + sizeof(float) > point_step)
        {
            ROS_WARN_THROTTLE(1.0, "origin point filter disabled: x/y/z offsets exceed point_step.");
            return false;
        }

        const size_t num_points = (size_t)in.width * (size_t)in.height;
        if (num_points == 0 || in.data.empty())
            return false;

        const uint8_t *base_ptr = in.data.data();
        const float eps = std::max(0.0f, origin_filter_eps_);

        std::vector<uint8_t> out_data;
        out_data.reserve(in.data.size());

        for (size_t idx = 0; idx < num_points; ++idx)
        {
            const uint8_t *p = base_ptr + idx * point_step;
            float x = 0.0f, y = 0.0f, z = 0.0f;
            std::memcpy(&x, p + offset_x, sizeof(float));
            std::memcpy(&y, p + offset_y, sizeof(float));
            std::memcpy(&z, p + offset_z, sizeof(float));

            const bool is_origin_point =
                std::isfinite(x) && std::isfinite(y) && std::isfinite(z) &&
                (std::fabs(x) + std::fabs(y) + std::fabs(z) <= eps);

            if (is_origin_point)
            {
                ++removed_points;
                continue;
            }

            const size_t cur_off = out_data.size();
            out_data.resize(cur_off + point_step);
            std::memcpy(out_data.data() + cur_off, p, point_step);
        }

        if (removed_points == 0)
            return false;

        out = in;
        out.data.swap(out_data);
        out.width = static_cast<uint32_t>(num_points - removed_points);
        out.height = 1;
        out.row_step = out.point_step * out.width;
        out.is_dense = false;

        ROS_INFO("[Frame %d] origin point filter: points_in=%zu removed=%zu points_out=%u",
                 frame_idx_, num_points, removed_points, out.width);
        return true;
    }

bool LidarIntensityORBMatchDual::samplePointCloudByRing(const sensor_msgs::PointCloud2 &in,
                                sensor_msgs::PointCloud2 &out)
{
        if (!enable_line_sampling_ ||
            target_lines_ >= source_lines_ ||
            target_lines_ <= 0 ||
            source_lines_ <= 0)
        {
            return false; // no sampling
        }
        if (source_lines_ % target_lines_ != 0)
        {
            ROS_WARN_THROTTLE(1.0,
                              "line sampling disabled: source_lines(%d) %% target_lines(%d) != 0",
                              source_lines_, target_lines_);
            return false;
        }

        // locate the ring field
        int offset_ring = -1;
        int ring_datatype = -1;
        for (const auto &f : in.fields)
        {
            if (f.name == "ring")
            {
                offset_ring = f.offset;
                ring_datatype = f.datatype;
                break;
            }
        }
        if (offset_ring < 0)
        {
            ROS_WARN_THROTTLE(1.0,
                              "line sampling disabled: no 'ring' field in PointCloud2.");
            return false;
        }

        // Optional: drop NaN/placeholder points ((0,0,0)) and near-origin points so downstream LOAM is not disturbed
        bool filter_invalid = sampling_filter_invalid_;
        int offset_x = -1, offset_y = -1, offset_z = -1;
        if (filter_invalid)
        {
            for (const auto &f : in.fields)
            {
                if (f.name == "x")
                    offset_x = f.offset;
                else if (f.name == "y")
                    offset_y = f.offset;
                else if (f.name == "z")
                    offset_z = f.offset;
            }
            if (offset_x < 0 || offset_y < 0 || offset_z < 0)
            {
                ROS_WARN_THROTTLE(1.0, "sampling_filter_invalid enabled but x/y/z offsets not found, disable filtering.");
                filter_invalid = false;
            }
        }

        const size_t num_points = (size_t)in.width * in.height;
        const uint8_t *base_ptr = in.data.data();
        const size_t point_step = in.point_step;

        int step = source_lines_ / target_lines_;
        std::vector<int> ring_to_new(source_lines_, -1);
        for (int i = 0; i < target_lines_; ++i)
        {
            int orig_ring = i * step;
            if (orig_ring >= 0 && orig_ring < source_lines_)
                ring_to_new[orig_ring] = target_lines_ - 1 - i;
        }

        std::vector<uint8_t> out_data;
        out_data.reserve(in.data.size() * target_lines_ / std::max(1, source_lines_) + 256);

        size_t kept_points = 0;
        for (size_t idx = 0; idx < num_points; ++idx)
        {
            const uint8_t *p = base_ptr + idx * point_step;
            int ring = 0;
            if (ring_datatype == sensor_msgs::PointField::UINT16)
            {
                uint16_t r = 0;
                std::memcpy(&r, p + offset_ring, sizeof(uint16_t));
                ring = (int)r;
            }
            else if (ring_datatype == sensor_msgs::PointField::INT16)
            {
                int16_t r = 0;
                std::memcpy(&r, p + offset_ring, sizeof(int16_t));
                ring = (int)r;
            }
            else
            {
                uint16_t r = 0;
                std::memcpy(&r, p + offset_ring, sizeof(uint16_t));
                ring = (int)r;
            }

            if (ring < 0 || ring >= source_lines_)
                continue;
            int new_ring = ring_to_new[ring];
            if (new_ring < 0)
                continue; // this beam is not selected

            if (filter_invalid)
            {
                float x, y, z;
                std::memcpy(&x, p + offset_x, sizeof(float));
                std::memcpy(&y, p + offset_y, sizeof(float));
                std::memcpy(&z, p + offset_z, sizeof(float));

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                    continue;
                if (std::fabs(x) + std::fabs(y) + std::fabs(z) < deskew_placeholder_eps_)
                    continue;
                const float r2 = x * x + y * y + z * z;
                if (r2 < deskew_min_range_ * deskew_min_range_)
                    continue;
            }

            size_t cur_off = out_data.size();
            out_data.resize(cur_off + point_step);
            std::memcpy(out_data.data() + cur_off, p, point_step);

            if (remap_ring_to_compact_)
            {
                uint8_t *pr_out = out_data.data() + cur_off + offset_ring;
                if (ring_datatype == sensor_msgs::PointField::UINT16)
                {
                    uint16_t r = (uint16_t)new_ring;
                    std::memcpy(pr_out, &r, sizeof(uint16_t));
                }
                else if (ring_datatype == sensor_msgs::PointField::INT16)
                {
                    int16_t r = (int16_t)new_ring;
                    std::memcpy(pr_out, &r, sizeof(int16_t));
                }
                else
                {
                    uint16_t r = (uint16_t)new_ring;
                    std::memcpy(pr_out, &r, sizeof(uint16_t));
                }
            }
            ++kept_points;
        }

        if (kept_points == 0)
        {
            ROS_WARN_THROTTLE(1.0, "line sampling produced empty cloud.");
            return false;
        }

        out = in; // copy metadata
        out.data.swap(out_data);
        out.width = kept_points;
        out.height = 1;
        out.row_step = out.point_step * out.width;

        ROS_INFO("[Frame %d] line sampling (publish only): points_in=%zu points_out=%zu",
                 frame_idx_, num_points, kept_points);

        return true;
    }

bool LidarIntensityORBMatchDual::computeCloudTimestampStats(const sensor_msgs::PointCloud2 &cloud,
                                    double &ts_min, double &ts_max,
                                    double &ts_mid, double &ts_span) const
{
        ts_min = std::numeric_limits<double>::infinity();
        ts_max = -std::numeric_limits<double>::infinity();
        ts_mid = std::numeric_limits<double>::quiet_NaN();
        ts_span = 0.0;

        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0 || cloud.data.empty())
            return false;

        int offset_x = -1, offset_y = -1, offset_z = -1, offset_ts = -1;
        int dtype_ts = -1;
        for (const auto &f : cloud.fields)
        {
            if (f.name == "x")
                offset_x = f.offset;
            else if (f.name == "y")
                offset_y = f.offset;
            else if (f.name == "z")
                offset_z = f.offset;
            else if (f.name == "timestamp")
            {
                offset_ts = f.offset;
                dtype_ts = f.datatype;
            }
        }
        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_ts < 0)
            return false;
        if (dtype_ts != sensor_msgs::PointField::FLOAT64)
            return false;

        const size_t point_step = cloud.point_step;
        if ((size_t)offset_x + sizeof(float) > point_step ||
            (size_t)offset_y + sizeof(float) > point_step ||
            (size_t)offset_z + sizeof(float) > point_step ||
            (size_t)offset_ts + sizeof(double) > point_step)
            return false;

        const uint8_t *base_ptr = cloud.data.data();
        const float min_range2 = deskew_min_range_ * deskew_min_range_;
        const float eps_l1 = deskew_placeholder_eps_;

        size_t valid = 0;
        for (size_t idx = 0; idx < num_points; ++idx)
        {
            const uint8_t *p = base_ptr + idx * point_step;

            float x, y, z;
            std::memcpy(&x, p + offset_x, sizeof(float));
            std::memcpy(&y, p + offset_y, sizeof(float));
            std::memcpy(&z, p + offset_z, sizeof(float));
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            if (std::fabs(x) + std::fabs(y) + std::fabs(z) < eps_l1)
                continue;
            if (x * x + y * y + z * z < min_range2)
                continue;

            double ts;
            std::memcpy(&ts, p + offset_ts, sizeof(double));
            if (!std::isfinite(ts))
                continue;

            ts_min = std::min(ts_min, ts);
            ts_max = std::max(ts_max, ts);
            ++valid;
        }

        if (valid < 50 || !(ts_max > ts_min))
            return false;

        ts_span = ts_max - ts_min;
        ts_mid = 0.5 * (ts_min + ts_max);
        return std::isfinite(ts_mid);
    }

void LidarIntensityORBMatchDual::scaleTransformByTime(const cv::Mat &R_in, const cv::Mat &t_in,
                                            double alpha, cv::Mat &R_out, cv::Mat &t_out)
{
        if (!std::isfinite(alpha))
            alpha = 1.0;
        if (std::fabs(alpha - 1.0) < 1e-6)
        {
            R_out = R_in.clone();
            t_out = t_in.clone();
            return;
        }
        cv::Mat rvec;
        cv::Rodrigues(R_in, rvec);
        rvec *= alpha;
        cv::Rodrigues(rvec, R_out);
        t_out = t_in * alpha;
    }

bool LidarIntensityORBMatchDual::deskewPointCloudInPlaceTimestamp(sensor_msgs::PointCloud2 &cloud,
                                          const cv::Mat &R_end_in,
                                          const cv::Mat &t_end_in)
{
        if (!enable_deskew_current_)
            return false;

        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0 || cloud.data.empty())
            return false;

        // Locate the field offsets (note: the AT128 point_step is not necessarily 4/8-byte aligned, so memcpy must be used)
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_ts = -1;
        int dtype_ts = -1;
        for (const auto &f : cloud.fields)
        {
            if (f.name == "x")
                offset_x = f.offset;
            else if (f.name == "y")
                offset_y = f.offset;
            else if (f.name == "z")
                offset_z = f.offset;
            else if (f.name == "timestamp")
            {
                offset_ts = f.offset;
                dtype_ts = f.datatype;
            }
        }
        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_ts < 0)
        {
            ROS_WARN_THROTTLE(1.0, "deskew disabled: missing x/y/z/timestamp field.");
            return false;
        }
        if (dtype_ts != sensor_msgs::PointField::FLOAT64)
        {
            ROS_WARN_THROTTLE(1.0, "deskew disabled: timestamp datatype is not FLOAT64 (datatype=%d).", dtype_ts);
            return false;
        }

        const size_t point_step = cloud.point_step;
        if ((size_t)offset_x + sizeof(float) > point_step ||
            (size_t)offset_y + sizeof(float) > point_step ||
            (size_t)offset_z + sizeof(float) > point_step ||
            (size_t)offset_ts + sizeof(double) > point_step)
        {
            ROS_ERROR("deskewPointCloudInPlaceTimestamp: field offsets exceed point_step (point_step=%zu).", point_step);
            return false;
        }

        uint8_t *base_ptr = cloud.data.data();

        // 1) Collect the timestamp range of the current frame (invalid points ignored)
        double ts_min = std::numeric_limits<double>::infinity();
        double ts_max = -std::numeric_limits<double>::infinity();
        size_t valid_for_span = 0;

        const float min_range2 = deskew_min_range_ * deskew_min_range_;
        const float eps_l1 = deskew_placeholder_eps_;

        for (size_t idx = 0; idx < num_points; ++idx)
        {
            const uint8_t *p = base_ptr + idx * point_step;

            float x, y, z;
            std::memcpy(&x, p + offset_x, sizeof(float));
            std::memcpy(&y, p + offset_y, sizeof(float));
            std::memcpy(&z, p + offset_z, sizeof(float));

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            if (std::fabs(x) + std::fabs(y) + std::fabs(z) < eps_l1)
                continue;
            const float r2 = x * x + y * y + z * z;
            if (r2 < min_range2)
                continue;

            double ts;
            std::memcpy(&ts, p + offset_ts, sizeof(double));
            if (!std::isfinite(ts))
                continue;

            ts_min = std::min(ts_min, ts);
            ts_max = std::max(ts_max, ts);
            ++valid_for_span;
        }

        if (valid_for_span < 50 || !(ts_max > ts_min))
        {
            ROS_WARN_THROTTLE(1.0, "deskew disabled: bad timestamp span (valid=%zu, min=%.6f max=%.6f).",
                              valid_for_span, ts_min, ts_max);
            return false;
        }
        const double ts_span = ts_max - ts_min;

        // 2) Precompute the rotation axis-angle (log(R_end))
        cv::Mat R_end = R_end_in;
        cv::Mat t_end = t_end_in;
        if (deskew_use_inverse_tguess_)
        {
            ROS_WARN_THROTTLE(1.0,
                              "deskew_use_inverse_tguess is deprecated and ignored: deskew always receives T(prev->cur).");
        }

        cv::Mat rvec_end;
        cv::Rodrigues(R_end, rvec_end); // 3x1 (axis * angle)

        const double rx = rvec_end.at<double>(0);
        const double ry = rvec_end.at<double>(1);
        const double rz = rvec_end.at<double>(2);
        const double angle_end = std::sqrt(rx * rx + ry * ry + rz * rz);

        double ax = 0.0, ay = 0.0, az = 0.0;
        if (angle_end > 1e-12)
        {
            ax = rx / angle_end;
            ay = ry / angle_end;
            az = rz / angle_end;
        }

        const double tx = t_end.at<double>(0);
        const double ty = t_end.at<double>(1);
        const double tz = t_end.at<double>(2);

        const float nan_f = std::numeric_limits<float>::quiet_NaN();
        size_t nan_written = 0;

        // 3) Second pass: deskew every point
        for (size_t idx = 0; idx < num_points; ++idx)
        {
            uint8_t *p = base_ptr + idx * point_step;

            float xf, yf, zf;
            std::memcpy(&xf, p + offset_x, sizeof(float));
            std::memcpy(&yf, p + offset_y, sizeof(float));
            std::memcpy(&zf, p + offset_z, sizeof(float));

            if (!std::isfinite(xf) || !std::isfinite(yf) || !std::isfinite(zf) ||
                (std::fabs(xf) + std::fabs(yf) + std::fabs(zf) < eps_l1) ||
                (xf * xf + yf * yf + zf * zf < min_range2))
            {
                // Invalid/placeholder point: write NaN so deskewing does not produce "origin rays"
                std::memcpy(p + offset_x, &nan_f, sizeof(float));
                std::memcpy(p + offset_y, &nan_f, sizeof(float));
                std::memcpy(p + offset_z, &nan_f, sizeof(float));
                ++nan_written;
                continue;
            }

            double ts;
            std::memcpy(&ts, p + offset_ts, sizeof(double));
            if (!std::isfinite(ts))
            {
                std::memcpy(p + offset_x, &nan_f, sizeof(float));
                std::memcpy(p + offset_y, &nan_f, sizeof(float));
                std::memcpy(p + offset_z, &nan_f, sizeof(float));
                ++nan_written;
                continue;
            }

            double s = (ts - ts_min) / ts_span;
            if (s < 0.0)
                s = 0.0;
            if (s > 1.0)
                s = 1.0;

            // Translation interpolation: t(s) = s * t_end
            const double tsx = s * tx;
            const double tsy = s * ty;
            const double tsz = s * tz;

            // Rotation interpolation: R(s) = Exp(s * log(R_end)), via axis-angle -> quaternion
            double w = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
            if (angle_end > 1e-12)
            {
                const double half = 0.5 * (s * angle_end);
                const double sh = std::sin(half);
                w = std::cos(half);
                qx = ax * sh;
                qy = ay * sh;
                qz = az * sh;
            }

            // Subtract the translation first: v = p - t(s)
            double vx = (double)xf - tsx;
            double vy = (double)yf - tsy;
            double vz = (double)zf - tsz;

            // Deskew back to the scan start: p_start = R(s)^T * (p - t(s))
            // i.e. v_rot = q_conj * v * q, with q_conj = (w, -qx, -qy, -qz)
            const double cx = -qx, cy = -qy, cz = -qz;

            // t = 2 * cross(c, v)
            const double tx2 = 2.0 * (cy * vz - cz * vy);
            const double ty2 = 2.0 * (cz * vx - cx * vz);
            const double tz2 = 2.0 * (cx * vy - cy * vx);

            // v' = v + w*t + cross(c, t)
            const double vpx = vx + w * tx2 + (cy * tz2 - cz * ty2);
            const double vpy = vy + w * ty2 + (cz * tx2 - cx * tz2);
            const double vpz = vz + w * tz2 + (cx * ty2 - cy * tx2);

            float xo = (float)vpx;
            float yo = (float)vpy;
            float zo = (float)vpz;

            std::memcpy(p + offset_x, &xo, sizeof(float));
            std::memcpy(p + offset_y, &yo, sizeof(float));
            std::memcpy(p + offset_z, &zo, sizeof(float));
        }

        ROS_INFO("[Frame %d] deskew(current,Tguess): points=%zu nan_written=%zu span=%.6f",
                 frame_idx_, num_points, nan_written, ts_span);

        return true;
    }

void LidarIntensityORBMatchDual::buildIntensityImageAngle(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<float> &pixel_intensity,
        std::vector<double> &pixel_timestamp,
        std::vector<std::vector<PixelPoint>> &pixel_points,
        std::vector<char> &has_point)
{
        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0)
        {
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            pixel_intensity.clear();
            pixel_timestamp.clear();
            pixel_points.clear();
            has_point.clear();
            return;
        }

        int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1, offset_ts = -1;
        int dtype_ts = -1;
        for (const auto &f : cloud.fields)
        {
            if (f.name == "x")
                offset_x = f.offset;
            else if (f.name == "y")
                offset_y = f.offset;
            else if (f.name == "z")
                offset_z = f.offset;
            else if (f.name == "intensity")
                offset_i = f.offset;
            else if (f.name == "timestamp")
            {
                offset_ts = f.offset;
                dtype_ts = f.datatype;
            }
        }

        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_i < 0)
        {
            ROS_ERROR("buildIntensityImageAngle: x/y/z/intensity fields not all found.");
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            pixel_intensity.clear();
            pixel_timestamp.clear();
            pixel_points.clear();
            has_point.clear();
            return;
        }

        const uint8_t *base_ptr = cloud.data.data();
        const size_t point_step = cloud.point_step;

        // ---- project ----

        const int v_res = v_res_;
        const int h_res = h_res_;
        const size_t total_pix = (size_t)v_res * h_res;

        int omp_threads = 1;
#ifdef _OPENMP
        if (enable_parallel_projection_ && enable_multithread_ && num_threads_ > 1)
            omp_threads = num_threads_;
        else
            omp_threads = 1;
#endif

        std::vector<std::vector<PixAcc>> thread_grids(
            (size_t)omp_threads, std::vector<PixAcc>(total_pix));
        std::vector<std::vector<PixelPointRecord>> thread_point_records((size_t)omp_threads);
        if (store_all_pixel_points_)
        {
            const size_t reserve_per_thread =
                std::max<size_t>(1024, num_points / static_cast<size_t>(std::max(1, omp_threads)));
            for (auto &records : thread_point_records)
                records.reserve(reserve_per_thread);
        }

#ifdef _OPENMP
#pragma omp parallel num_threads(omp_threads)
#endif
        {
            int tid = 0;
#ifdef _OPENMP
            tid = omp_get_thread_num();
#endif
            auto &grid = thread_grids[(size_t)tid];
            auto &point_records = thread_point_records[(size_t)tid];

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
            for (int i = 0; i < (int)num_points; ++i)
            {
                if (sample_step_ > 1 && (i % sample_step_ != 0))
                    continue;

                const uint8_t *p_base = base_ptr + (size_t)i * point_step;
                float x, y, z, intensity;
                double timestamp = std::numeric_limits<double>::quiet_NaN();
                std::memcpy(&x, p_base + offset_x, sizeof(float));
                std::memcpy(&y, p_base + offset_y, sizeof(float));
                std::memcpy(&z, p_base + offset_z, sizeof(float));
                std::memcpy(&intensity, p_base + offset_i, sizeof(float));
                if (offset_ts >= 0)
                {
                    if (dtype_ts == sensor_msgs::PointField::FLOAT64)
                    {
                        std::memcpy(&timestamp, p_base + offset_ts, sizeof(double));
                    }
                    else if (dtype_ts == sensor_msgs::PointField::FLOAT32)
                    {
                        float ts_f = 0.0f;
                        std::memcpy(&ts_f, p_base + offset_ts, sizeof(float));
                        timestamp = static_cast<double>(ts_f);
                    }
                }

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    !std::isfinite(intensity) || intensity <= 0.0f)
                {
                    continue;
                }

                double az = std::atan2((double)y, (double)x);
                double r_xy = std::sqrt((double)x * x + (double)y * y);
                double el = std::atan2((double)z, r_xy);

                if (az < min_az_ || az > max_az_ || el < min_el_ || el > max_el_)
                    continue;

                // AT128 packet timestamps increase from +azimuth to -azimuth.
                // Reverse the spatial azimuth axis so image columns follow the
                // physical scan order: left (early) -> right (late).
                double uf = (max_az_ - az) / (max_az_ - min_az_) * (h_res - 1);
                double vf = (max_el_ - el) / (max_el_ - min_el_) * (v_res - 1);

                int u = (int)std::round(uf);
                int v = (int)std::round(vf);
                if (u < 0 || u >= h_res || v < 0 || v >= v_res)
                    continue;

                size_t pid = (size_t)v * h_res + (size_t)u;
                PixAcc &cell = grid[pid];
                if (store_all_pixel_points_)
                    point_records.push_back(PixelPointRecord{pid, PixelPoint{x, y, z, intensity, timestamp}});

                if (!cell.has_point || intensity > cell.max_intensity)
                {
                    cell.max_intensity = intensity;
                    cell.x = x;
                    cell.y = y;
                    cell.z = z;
                    cell.timestamp = timestamp;
                    cell.has_point = true;
                }
            }
        }

        // ---- merge ----

        std::vector<PixAcc> global_grid(total_pix);
        for (int t = 0; t < omp_threads; ++t)
        {
            const auto &g = thread_grids[(size_t)t];
            for (size_t pid = 0; pid < total_pix; ++pid)
            {
                const PixAcc &src = g[pid];
                PixAcc &dst = global_grid[pid];
                if (!src.has_point)
                    continue;
                if (!dst.has_point || src.max_intensity > dst.max_intensity)
                {
                    dst.max_intensity = src.max_intensity;
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.timestamp = src.timestamp;
                    dst.has_point = true;
                }
            }
        }
        materializePixelPointBuckets(total_pix, thread_point_records, pixel_points);

        // ---- output ----
        intensity_f = cv::Mat(v_res, h_res, CV_32F, cv::Scalar(0.0f));
        pixel_x.assign(total_pix, 0.0f);
        pixel_y.assign(total_pix, 0.0f);
        pixel_z.assign(total_pix, 0.0f);
        pixel_intensity.assign(total_pix, 0.0f);
        pixel_timestamp.assign(total_pix, std::numeric_limits<double>::quiet_NaN());
        has_point.assign(total_pix, 0);

        for (size_t pid = 0; pid < total_pix; ++pid)
        {
            const PixAcc &cell = global_grid[pid];
            if (!cell.has_point)
                continue;
            int v = (int)(pid / h_res);
            int u = (int)(pid % h_res);

            intensity_f.at<float>(v, u) = cell.max_intensity;
            pixel_x[pid] = cell.x;
            pixel_y[pid] = cell.y;
            pixel_z[pid] = cell.z;
            pixel_intensity[pid] = cell.max_intensity;
            pixel_timestamp[pid] = cell.timestamp;
            has_point[pid] = 1;
        }
    }

void LidarIntensityORBMatchDual::buildIntensityImageRing(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<float> &pixel_intensity,
        std::vector<double> &pixel_timestamp,
        std::vector<std::vector<PixelPoint>> &pixel_points,
        std::vector<char> &has_point)
{
        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0)
        {
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            pixel_intensity.clear();
            pixel_timestamp.clear();
            pixel_points.clear();
            has_point.clear();
            return;
        }

        int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1, offset_ring = -1, offset_ts = -1;
        int ring_datatype = -1;
        int dtype_ts = -1;

        for (const auto &f : cloud.fields)
        {
            if (f.name == "x")
                offset_x = f.offset;
            else if (f.name == "y")
                offset_y = f.offset;
            else if (f.name == "z")
                offset_z = f.offset;
            else if (f.name == "intensity")
                offset_i = f.offset;
            else if (f.name == "ring")
            {
                offset_ring = f.offset;
                ring_datatype = f.datatype;
            }
            else if (f.name == "timestamp")
            {
                offset_ts = f.offset;
                dtype_ts = f.datatype;
            }
        }

        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_i < 0 || offset_ring < 0)
        {
            ROS_ERROR("buildIntensityImageRing: x/y/z/intensity/ring fields not all found.");
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            pixel_intensity.clear();
            pixel_timestamp.clear();
            pixel_points.clear();
            has_point.clear();
            return;
        }

        const uint8_t *base_ptr = cloud.data.data();
        const size_t point_step = cloud.point_step;

        // ---- project ----

        const int v_res = v_res_;
        const int h_res = h_res_;
        const size_t total_pix = (size_t)v_res * h_res;

        int omp_threads = 1;
#ifdef _OPENMP
        if (enable_parallel_projection_ && enable_multithread_ && num_threads_ > 1)
            omp_threads = num_threads_;
        else
            omp_threads = 1;
#endif

        std::vector<std::vector<PixAcc>> thread_grids(
            (size_t)omp_threads, std::vector<PixAcc>(total_pix));
        std::vector<std::vector<PixelPointRecord>> thread_point_records((size_t)omp_threads);
        if (store_all_pixel_points_)
        {
            const size_t reserve_per_thread =
                std::max<size_t>(1024, num_points / static_cast<size_t>(std::max(1, omp_threads)));
            for (auto &records : thread_point_records)
                records.reserve(reserve_per_thread);
        }

#ifdef _OPENMP
#pragma omp parallel num_threads(omp_threads)
#endif
        {
            int tid = 0;
#ifdef _OPENMP
            tid = omp_get_thread_num();
#endif
            auto &grid = thread_grids[(size_t)tid];
            auto &point_records = thread_point_records[(size_t)tid];

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
            for (int i = 0; i < (int)num_points; ++i)
            {
                if (sample_step_ > 1 && (i % sample_step_ != 0))
                    continue;

                const uint8_t *p_base = base_ptr + (size_t)i * point_step;

                int ring = 0;
                if (ring_datatype == sensor_msgs::PointField::UINT16)
                {
                    const uint16_t *pr = reinterpret_cast<const uint16_t *>(p_base + offset_ring);
                    ring = (int)(*pr);
                }
                else if (ring_datatype == sensor_msgs::PointField::INT16)
                {
                    const int16_t *pr = reinterpret_cast<const int16_t *>(p_base + offset_ring);
                    ring = (int)(*pr);
                }
                else
                {
                    const uint16_t *pr = reinterpret_cast<const uint16_t *>(p_base + offset_ring);
                    ring = (int)(*pr);
                }

                if (ring < 0 || ring >= v_res)
                    continue;

                float x, y, z, intensity;
                double timestamp = std::numeric_limits<double>::quiet_NaN();
                std::memcpy(&x, p_base + offset_x, sizeof(float));
                std::memcpy(&y, p_base + offset_y, sizeof(float));
                std::memcpy(&z, p_base + offset_z, sizeof(float));
                std::memcpy(&intensity, p_base + offset_i, sizeof(float));
                if (offset_ts >= 0)
                {
                    if (dtype_ts == sensor_msgs::PointField::FLOAT64)
                    {
                        std::memcpy(&timestamp, p_base + offset_ts, sizeof(double));
                    }
                    else if (dtype_ts == sensor_msgs::PointField::FLOAT32)
                    {
                        float ts_f = 0.0f;
                        std::memcpy(&ts_f, p_base + offset_ts, sizeof(float));
                        timestamp = static_cast<double>(ts_f);
                    }
                }

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    !std::isfinite(intensity) || intensity <= 0.0f)
                {
                    continue;
                }

                double az = std::atan2((double)y, (double)x);
                if (az < min_az_ || az > max_az_)
                    continue;

                // Keep ring projection horizontally consistent with AT128's
                // timestamp order: +azimuth at the first (left) image column.
                double uf = (max_az_ - az) / (max_az_ - min_az_) * (h_res - 1);
                int u = (int)std::round(uf);
                int v = ring; // row = ring

                if (u < 0 || u >= h_res || v < 0 || v >= v_res)
                    continue;

                size_t pid = (size_t)v * h_res + (size_t)u;
                PixAcc &cell = grid[pid];
                if (store_all_pixel_points_)
                    point_records.push_back(PixelPointRecord{pid, PixelPoint{x, y, z, intensity, timestamp}});

                if (!cell.has_point || intensity > cell.max_intensity)
                {
                    cell.max_intensity = intensity;
                    cell.x = x;
                    cell.y = y;
                    cell.z = z;
                    cell.timestamp = timestamp;
                    cell.has_point = true;
                }
            }
        }

        // ---- merge ----

        std::vector<PixAcc> global_grid(total_pix);
        for (int t = 0; t < omp_threads; ++t)
        {
            const auto &g = thread_grids[(size_t)t];
            for (size_t pid = 0; pid < total_pix; ++pid)
            {
                const PixAcc &src = g[pid];
                PixAcc &dst = global_grid[pid];
                if (!src.has_point)
                    continue;
                if (!dst.has_point || src.max_intensity > dst.max_intensity)
                {
                    dst.max_intensity = src.max_intensity;
                    dst.x = src.x;
                    dst.y = src.y;
                    dst.z = src.z;
                    dst.timestamp = src.timestamp;
                    dst.has_point = true;
                }
            }
        }
        materializePixelPointBuckets(total_pix, thread_point_records, pixel_points);

        // ---- output ----
        intensity_f = cv::Mat(v_res, h_res, CV_32F, cv::Scalar(0.0f));
        pixel_x.assign(total_pix, 0.0f);
        pixel_y.assign(total_pix, 0.0f);
        pixel_z.assign(total_pix, 0.0f);
        pixel_intensity.assign(total_pix, 0.0f);
        pixel_timestamp.assign(total_pix, std::numeric_limits<double>::quiet_NaN());
        has_point.assign(total_pix, 0);

        for (size_t pid = 0; pid < total_pix; ++pid)
        {
            const PixAcc &cell = global_grid[pid];
            if (!cell.has_point)
                continue;
            int v = (int)(pid / h_res);
            int u = (int)(pid % h_res);

            intensity_f.at<float>(v, u) = cell.max_intensity;
            pixel_x[pid] = cell.x;
            pixel_y[pid] = cell.y;
            pixel_z[pid] = cell.z;
            pixel_intensity[pid] = cell.max_intensity;
            pixel_timestamp[pid] = cell.timestamp;
            has_point[pid] = 1;
        }
    }

bool LidarIntensityORBMatchDual::rebuildFeatureCacheFromDeskewedCloud(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &image_out,
        cv::Mat &descriptors_out,
        std::vector<cv::KeyPoint> &keypoints_out,
        std::vector<float> &pixel_x_out,
        std::vector<float> &pixel_y_out,
        std::vector<float> &pixel_z_out,
        std::vector<float> &pixel_intensity_out,
        std::vector<double> &pixel_timestamp_out,
        std::vector<std::vector<PixelPoint>> &pixel_points_out,
        std::vector<char> &has_point_out)
{
        cv::Mat intensity_f;
        if (projection_mode_ == "ring")
        {
            buildIntensityImageRing(cloud, intensity_f,
                                    pixel_x_out, pixel_y_out, pixel_z_out,
                                    pixel_intensity_out, pixel_timestamp_out,
                                    pixel_points_out, has_point_out);
        }
        else
        {
            buildIntensityImageAngle(cloud, intensity_f,
                                     pixel_x_out, pixel_y_out, pixel_z_out,
                                     pixel_intensity_out, pixel_timestamp_out,
                                     pixel_points_out, has_point_out);
        }

        if (intensity_f.empty())
            return false;

        cv::Mat intensity_raw;
        cv::normalize(intensity_f, intensity_raw, 0, 255, cv::NORM_MINMAX, CV_8U);
        image_out = intensity_raw;

        cv::Mat intensity_equalize;
        cv::Mat intensity_clahe;
        if (contrast_mode_ == "clahe" || contrast_mode_ == "equalize")
        {
            cv::equalizeHist(intensity_raw, intensity_equalize);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(intensity_raw, intensity_clahe);
            if (contrast_mode_ == "clahe")
                image_out = intensity_clahe;
            else
                image_out = intensity_equalize;
        }
        if (enable_blur_)
            cv::GaussianBlur(image_out, image_out, cv::Size(3, 3), 0);
        if (enable_bilateral_filter_)
        {
            int bilateral_d = std::max(1, bilateral_d_);
            if ((bilateral_d % 2) == 0)
                ++bilateral_d;
            cv::Mat bilateral_image;
            cv::bilateralFilter(image_out, bilateral_image, bilateral_d,
                                std::max(0.0, bilateral_sigma_color_),
                                std::max(0.0, bilateral_sigma_space_));
            image_out = bilateral_image;
        }

        cv::Ptr<cv::ORB> orb = cv::ORB::create(
            orb_nfeatures_, static_cast<float>(orb_scaleFactor_), orb_nlevels_,
            orb_edgeThreshold_, 0, orb_wta_k_, orb_score_type_,
            orb_patchSize_, orb_fastThreshold_);
        orb->detectAndCompute(image_out, cv::noArray(), keypoints_out, descriptors_out);

        ROS_INFO("[Frame %d] rebuilt corrected feature cache: orb_features=%zu",
                 frame_idx_, keypoints_out.size());
        return true;
    }

void LidarIntensityORBMatchDual::callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
        if (!msg || msg->data.empty())
            return;

        sensor_msgs::PointCloud2 filtered_cloud;
        size_t removed_origin_points = 0;
        const sensor_msgs::PointCloud2 *cloud_in = msg.get();
        if (filterOriginPoints(*msg, filtered_cloud, removed_origin_points))
        {
            cloud_in = &filtered_cloud;
            if (cloud_in->data.empty() || (size_t)cloud_in->width * (size_t)cloud_in->height == 0)
            {
                ROS_WARN("Frame %d skipped: all points were filtered as origin placeholders.", frame_idx_);
                return;
            }
        }

        // 0) Collect the per-point timestamp range of the current frame (for deskew time scaling)
        double cur_ts_min = 0.0, cur_ts_max = 0.0, cur_ts_mid = 0.0, cur_ts_span = 0.0;
        bool cur_ts_ok = computeCloudTimestampStats(*cloud_in, cur_ts_min, cur_ts_max, cur_ts_mid, cur_ts_span);
        double deskew_time_alpha = 1.0;
        bool deskew_time_alpha_valid = false;
        if (deskew_scale_by_time_ && cur_ts_ok && prev_scan_start_valid_)
        {
            const double dt_guess = cur_ts_min - prev_scan_start_ts_; // start-to-start, same unit as timestamp (seconds or nanoseconds)
            if (std::isfinite(dt_guess) && dt_guess > 1e-12 && std::isfinite(cur_ts_span) && cur_ts_span > 0.0)
            {
                deskew_time_alpha = cur_ts_span / dt_guess;
                if (!std::isfinite(deskew_time_alpha))
                    deskew_time_alpha = 1.0;
                if (deskew_time_alpha < deskew_scale_min_)
                    deskew_time_alpha = deskew_scale_min_;
                if (deskew_time_alpha > deskew_scale_max_)
                    deskew_time_alpha = deskew_scale_max_;
                deskew_time_alpha_valid = true;
            }
        }
        else if (!deskew_scale_by_time_)
        {
            deskew_time_alpha_valid = cur_ts_ok;
        }

        // 0) Beam sampling (publish only, not used for projection)
        sensor_msgs::PointCloud2 sampled_cloud;

        // 0.1) Current-frame Tguess (from image matching), used for the predictive constant-velocity deskew of the current frame
        bool got_tguess = false;
        cv::Mat Rfit_this, tfit_this; // cur->prev

        // ---------- 1) Projection: use the filtered current-frame cloud ----------
        cv::Mat intensity_f;
        std::vector<float> pixel_x, pixel_y, pixel_z;
        std::vector<float> pixel_intensity;
        std::vector<double> pixel_timestamp;
        std::vector<std::vector<PixelPoint>> pixel_points;
        std::vector<char> has_point;

        if (projection_mode_ == "ring")
        {
            buildIntensityImageRing(*cloud_in,
                                    intensity_f,
                                    pixel_x, pixel_y, pixel_z,
                                    pixel_intensity, pixel_timestamp, pixel_points, has_point);
        }
        else
        {
            buildIntensityImageAngle(*cloud_in,
                                     intensity_f,
                                     pixel_x, pixel_y, pixel_z,
                                     pixel_intensity, pixel_timestamp, pixel_points, has_point);
        }

        cv::Mat intensity_raw_8u;
        if (!intensity_f.empty())
        {
            cv::normalize(intensity_f, intensity_raw_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
            if (shouldPublishRawIntensityImage())
            {
                const std::string label = show_match_labels_
                                              ? "Current Frame #" + std::to_string(frame_idx_)
                                              : "";
                publishRawIntensityImage(intensity_raw_8u, label, msg->header);
                ROS_INFO("Published raw intensity image: %s", raw_intensity_image_topic_.c_str());
            }
        }
        // ---------- 2) Image enhancement ----------
        cv::Mat intensity_equalize;
        cv::Mat intensity_clahe;
        cv::Mat intensity_bilateral;
        cv::Mat intensity_enh = intensity_raw_8u.clone();
        if (!intensity_raw_8u.empty())
        {
            cv::equalizeHist(intensity_raw_8u, intensity_equalize);
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(intensity_raw_8u, intensity_clahe);

            if (contrast_mode_ == "clahe")
            {
                intensity_enh = intensity_clahe.clone();
            }
            else if (contrast_mode_ == "equalize")
            {
                intensity_enh = intensity_equalize.clone();
            }
        }
        if (enable_blur_ && !intensity_enh.empty())
        {
            cv::GaussianBlur(intensity_enh, intensity_enh, cv::Size(3, 3), 0);
        }
        if (enable_bilateral_filter_ && !intensity_enh.empty())
        {
            int bilateral_d = std::max(1, bilateral_d_);
            if ((bilateral_d % 2) == 0)
                ++bilateral_d;
            const double sigma_color = std::max(0.0, bilateral_sigma_color_);
            const double sigma_space = std::max(0.0, bilateral_sigma_space_);
            cv::bilateralFilter(intensity_enh, intensity_bilateral,
                                bilateral_d, sigma_color, sigma_space);
            intensity_enh = intensity_bilateral.clone();
        }

        if (shouldPublishEnhancedIntensityImage())
        {
            const std::string label = show_match_labels_
                                          ? "Current Frame #" + std::to_string(frame_idx_)
                                          : "";
            publishEnhancedIntensityImage(intensity_enh, label, msg->header);
            ROS_INFO("Published enhanced intensity image: %s", enhanced_intensity_image_topic_.c_str());
        }

        // ---------- 3) ORB ----------

        cv::Ptr<cv::ORB> orb = cv::ORB::create(orb_nfeatures_, (float)orb_scaleFactor_, orb_nlevels_,
                                               orb_edgeThreshold_, 0, orb_wta_k_, orb_score_type_,
                                               orb_patchSize_, orb_fastThreshold_);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        if (!intensity_enh.empty())
        {
            orb->detectAndCompute(intensity_enh, cv::noArray(), keypoints, descriptors);
        }
        else
        {
            ROS_WARN("[Frame %d] intensity image is empty; skip ORB detection", frame_idx_);
        }

        // ---------- 4) Matching + RANSAC ----------
        size_t count_orb_matches = 0, count_2d_inliers = 0, count_3d_inliers = 0;

        if (has_prev_frame_ && !descriptors.empty() && !prev_desc_.empty())
        {
            cv::BFMatcher matcher(cv::NORM_HAMMING);
            std::vector<std::vector<cv::DMatch>> knn_matches;
            matcher.knnMatch(prev_desc_, descriptors, knn_matches, 2);

            std::vector<cv::DMatch> ratio_pass;
            ratio_pass.reserve(knn_matches.size());
            for (const auto &m : knn_matches)
            {
                if (m.size() >= 2 && m[0].distance < ratio_thresh_ * m[1].distance)
                    ratio_pass.push_back(m[0]);
            }

            std::vector<cv::DMatch> dist_pass;
            dist_pass.reserve(ratio_pass.size());
            for (const auto &m : ratio_pass)
            {
                if (m.distance <= hamming_thresh_)
                    dist_pass.push_back(m);
            }
            count_orb_matches = dist_pass.size();

            {
                const bool publish_orb_image = shouldPublishImage(matchedorb_image_pub_);
                const bool need_orb_vis = publish_orb_image;
                const std::string prev_label = show_match_labels_
                                                   ? "Prev Frame #" + std::to_string(frame_idx_ - 1)
                                                   : "";
                const std::string cur_label = show_match_labels_
                                                  ? "Current Frame #" + std::to_string(frame_idx_)
                                                  : "";
                if (need_orb_vis)
                {
                    cv::Mat vis_orb = buildMatchesStackedVisualization(prev_img_, intensity_enh, prev_kp_, keypoints,
                                                                       dist_pass, prev_label, cur_label);
                    if (publish_orb_image)
                    {
                        publishMatchedORBImage(vis_orb, msg->header);
                        ROS_INFO("Published ORB match visualization image: %s (matches=%zu)",
                                 matched_orb_image_topic_.c_str(), dist_pass.size());
                    }

                }
            }

            std::vector<cv::Point2f> pts_prev, pts_cur;
            pts_prev.reserve(dist_pass.size());
            pts_cur.reserve(dist_pass.size());
            for (const auto &m : dist_pass)
            {
                pts_prev.push_back(prev_kp_[m.queryIdx].pt);
                pts_cur.push_back(keypoints[m.trainIdx].pt);
            }

            std::vector<cv::DMatch> matches_2d_inliers;

            if (enable_ransac_2d_ && pts_prev.size() >= 8)
            {
                std::vector<uchar> mask2d;
                cv::Mat H = cv::findHomography(pts_prev, pts_cur,
                                               cv::RANSAC,
                                               ransac_2d_reproj_,
                                               mask2d);
                (void)H;

                matches_2d_inliers.reserve(dist_pass.size());
                for (size_t i = 0; i < dist_pass.size(); ++i)
                {
                    if (i < mask2d.size() && mask2d[i])
                        matches_2d_inliers.push_back(dist_pass[i]);
                }

            }
            else
            {
                matches_2d_inliers = dist_pass;
            }

            count_2d_inliers = matches_2d_inliers.size();

            {
                const bool publish_2d_image = shouldPublishImage(matched2d_image_pub_);
                const bool need_2d_vis = publish_2d_image;
                if (need_2d_vis)
                {
                    const std::string prev_label = show_match_labels_
                                                       ? "Prev Frame #" + std::to_string(frame_idx_ - 1)
                                                       : "";
                    const std::string cur_label = show_match_labels_
                                                      ? "Current Frame #" + std::to_string(frame_idx_)
                                                      : "";
                    cv::Mat vis2d = buildMatchesStackedVisualization(prev_img_, intensity_enh, prev_kp_, keypoints,
                                                                     matches_2d_inliers, prev_label, cur_label);
                    if (publish_2d_image)
                    {
                        publishMatched2DImage(vis2d, msg->header);
                        ROS_INFO("Published 2D visualization image: %s (inliers=%zu)",
                                 matched_2d_image_topic_.c_str(), matches_2d_inliers.size());
                    }

                }
            }

            std::vector<cv::Point3f> P_all, Q_all;
            std::vector<cv::DMatch> matches_for_3d;
            std::vector<double> P_intensity_all, Q_intensity_all;
            std::vector<double> P_timestamp_all, Q_timestamp_all;
            std::vector<std::pair<int, int>> pixel_pair_all;
            P_all.reserve(matches_2d_inliers.size());
            Q_all.reserve(matches_2d_inliers.size());
            P_intensity_all.reserve(matches_2d_inliers.size());
            Q_intensity_all.reserve(matches_2d_inliers.size());
            P_timestamp_all.reserve(matches_2d_inliers.size());
            Q_timestamp_all.reserve(matches_2d_inliers.size());
            pixel_pair_all.reserve(matches_2d_inliers.size());

            for (const auto &m : matches_2d_inliers)
            {
                int u1 = clampValue((int)std::round(prev_kp_[m.queryIdx].pt.x), 0, h_res_ - 1);
                int v1 = clampValue((int)std::round(prev_kp_[m.queryIdx].pt.y), 0, v_res_ - 1);
                int id1 = v1 * h_res_ + u1;

                int u2 = clampValue((int)std::round(keypoints[m.trainIdx].pt.x), 0, h_res_ - 1);
                int v2 = clampValue((int)std::round(keypoints[m.trainIdx].pt.y), 0, v_res_ - 1);
                int id2 = v2 * h_res_ + u2;

                if (id1 >= 0 && id1 < (int)prev_px_.size() && id2 >= 0 && id2 < (int)pixel_x.size())
                {
                    if (prev_has_[id1] && has_point[id2])
                    {
                        P_all.emplace_back(prev_px_[id1], prev_py_[id1], prev_pz_[id1]);
                        Q_all.emplace_back(pixel_x[id2], pixel_y[id2], pixel_z[id2]);
                        P_intensity_all.push_back((id1 < (int)prev_pint_.size()) ? (double)prev_pint_[id1] : 0.0);
                        Q_intensity_all.push_back((id2 < (int)pixel_intensity.size()) ? (double)pixel_intensity[id2] : 0.0);
                        P_timestamp_all.push_back((id1 < (int)prev_pts_.size()) ? prev_pts_[id1] : std::numeric_limits<double>::quiet_NaN());
                        Q_timestamp_all.push_back((id2 < (int)pixel_timestamp.size()) ? pixel_timestamp[id2] : std::numeric_limits<double>::quiet_NaN());
                        pixel_pair_all.emplace_back(id1, id2);
                        matches_for_3d.push_back(m);
                    }
                }
            }
            if (P_all.size() < 3)
            {
                ROS_WARN("Frame %d: not enough 3D correspondences after 2D filtering: %zu",
                         frame_idx_, P_all.size());
            }
            else
            {
                std::vector<int> inlier_idx;
                if (enable_parallel_ransac3d_ && ransac3d_threads_ > 1)
                    inlier_idx = ransac3D_parallel(Q_all, P_all, ransac_3d_thresh_, ransac_3d_iters_, ransac3d_threads_);
                else
                    inlier_idx = ransac3D(Q_all, P_all, ransac_3d_thresh_, ransac_3d_iters_);
                count_3d_inliers = inlier_idx.size();

                std::vector<cv::DMatch> matches_3d_inliers;
                std::vector<cv::Point3f> P_in, Q_in;
                std::vector<double> P_intensity_in, Q_intensity_in;
                std::vector<double> P_timestamp_in, Q_timestamp_in;
                std::vector<std::pair<int, int>> pixel_pair_in;
                for (int idx_in : inlier_idx)
                {
                    if (idx_in >= 0 && idx_in < (int)P_all.size())
                    {
                        P_in.push_back(P_all[(size_t)idx_in]);
                        Q_in.push_back(Q_all[(size_t)idx_in]);
                        P_intensity_in.push_back(P_intensity_all[(size_t)idx_in]);
                        Q_intensity_in.push_back(Q_intensity_all[(size_t)idx_in]);
                        P_timestamp_in.push_back(P_timestamp_all[(size_t)idx_in]);
                        Q_timestamp_in.push_back(Q_timestamp_all[(size_t)idx_in]);
                        pixel_pair_in.push_back(pixel_pair_all[(size_t)idx_in]);
                        matches_3d_inliers.push_back(matches_for_3d[(size_t)idx_in]);
                    }
                }
                bool matched_points_published = false;
                auto publishMatchedPairPoints =
                    [&](const std::vector<cv::Point3f> &prev_points,
                        const std::vector<cv::Point3f> &cur_points,
                        const std::vector<double> &prev_intensities,
                        const std::vector<double> &cur_intensities,
                        const std::vector<double> &prev_timestamps,
                        const std::vector<double> &cur_timestamps,
                        const std::string &source)
                {
                    matched_points_published = true;
                    if (!shouldPublishMatchedPointClouds())
                        return;

                    std_msgs::Header prev_match_header = prev_cloud_header_;
                    if (prev_match_header.frame_id.empty())
                        prev_match_header.frame_id = msg->header.frame_id;
                    // Both matched clouds are published together with the current-frame stamp so RViz/echo can inspect them side by side; the point coordinates are still the original 3D points of their own frames.
                    prev_match_header.stamp = msg->header.stamp;
                    publishMatched3DPointClouds(prev_points, cur_points,
                                                prev_intensities, cur_intensities,
                                                prev_timestamps, cur_timestamps,
                                                prev_match_header, msg->header);
                    ROS_INFO("Published matched 3D point clouds(%s): %s and %s (pairs=%zu)",
                             source.c_str(),
                             matched_prev_points_topic_.c_str(),
                             matched_cur_points_topic_.c_str(),
                             prev_points.size());
                };

                const std::string prev_label = show_match_labels_
                                                   ? "Prev Frame #" + std::to_string(frame_idx_ - 1)
                                                   : "";
                const std::string cur_label = show_match_labels_
                                                  ? "Current Frame #" + std::to_string(frame_idx_)
                                                  : "";
                const bool publish_3d_image = shouldPublishImage(matched3d_image_pub_);
                const bool need_3d_vis = publish_3d_image;
                if (need_3d_vis)
                {
                    cv::Mat vis3d = buildMatchesStackedVisualization(prev_img_, intensity_enh, prev_kp_, keypoints,
                                                                     matches_3d_inliers, prev_label, cur_label);
                    if (publish_3d_image)
                    {
                        publishMatched3DImage(vis3d, msg->header);
                        ROS_INFO("Published 3D visualization image: %s (inliers=%zu)",
                                 matched_3d_image_topic_.c_str(), matches_3d_inliers.size());
                    }

                }

                if (P_in.size() >= 3)
                {
                    cv::Mat Rfit, tfit;
                    // Estimate T(cur->prev) directly so the previous frame is the
                    // fixed reference for both the joint residual and visualization.
                    if (estimateRigidSVD(Q_in, P_in, Rfit, tfit))
                    {
                        if (enable_joint_tguess_deskew_ && cur_ts_ok)
                        {
                            const double stable_rmse_before =
                                computeJointTguessDeskewRmse(P_in, Q_in, Q_timestamp_in,
                                                             cur_ts_min, cur_ts_span,
                                                             deskew_time_alpha, Rfit, tfit);
                            std::vector<cv::Point3f> P_joint, Q_joint;
                            std::vector<double> P_intensity_joint, Q_intensity_joint, P_timestamp_joint, Q_timestamp_joint;
                            size_t joint_pairs_raw = 0;
                            size_t joint_pairs_rejected = 0;
                            double joint_pair_reject_threshold = std::numeric_limits<double>::infinity();
                            size_t joint_pairs = 0;
                            std::string joint_pair_source;
                            if (joint_pixel_match_mode_ == "max_intensity")
                            {
                                P_joint = P_in;
                                Q_joint = Q_in;
                                P_intensity_joint = P_intensity_in;
                                Q_intensity_joint = Q_intensity_in;
                                P_timestamp_joint = P_timestamp_in;
                                Q_timestamp_joint = Q_timestamp_in;
                                joint_pairs_raw = P_joint.size();
                                joint_pairs = P_joint.size();
                                joint_pair_source = "joint_max_intensity";
                            }
                            else
                            {
                                joint_pairs = buildNearestPixelSetCorrespondences(
                                    pixel_pair_in,
                                    prev_pixel_points_, pixel_points,
                                    cur_ts_min, cur_ts_span,
                                    deskew_time_alpha,
                                    Rfit, tfit,
                                    P_joint, Q_joint,
                                    P_intensity_joint, Q_intensity_joint,
                                    P_timestamp_joint, Q_timestamp_joint,
                                    joint_pairs_raw, joint_pairs_rejected,
                                    joint_pair_reject_threshold);
                                joint_pair_source = enable_joint_pair_outlier_rejection_
                                                        ? "joint_pixel_set_nn_filtered"
                                                        : "joint_pixel_set_nn";
                            }
                            if (joint_pixel_match_mode_ == "all")
                            {
                                const std::string joint_pair_thresh_text =
                                    std::isfinite(joint_pair_reject_threshold)
                                        ? (std::to_string(joint_pair_reject_threshold) + " m")
                                        : "disabled";
                                ROS_INFO("Joint 3D pair mode=all, outlier rejection: kept %zu / %zu, rejected=%zu, thresh=%s",
                                         joint_pairs, joint_pairs_raw, joint_pairs_rejected,
                                         joint_pair_thresh_text.c_str());
                            }
                            else
                            {
                                ROS_INFO("Joint 3D pair mode=max_intensity: using 3D-RANSAC inlier max-intensity points, pairs=%zu",
                                         joint_pairs);
                            }

                            if (joint_pairs >= static_cast<size_t>(std::max(3, joint_tguess_min_matches_)))
                            {
                                publishMatchedPairPoints(P_joint, Q_joint,
                                                         P_intensity_joint, Q_intensity_joint,
                                                         P_timestamp_joint, Q_timestamp_joint,
                                                         joint_pair_source);

                                double joint_rmse_before = 0.0, joint_rmse_after = 0.0;
                                size_t joint_used = 0;
                                int joint_iterations = 0;
                                bool joint_solution_usable = false;
                                const cv::Mat Rfit_before_joint = Rfit.clone();
                                const cv::Mat tfit_before_joint = tfit.clone();
                                refineTguessWithJointDeskew(P_joint, Q_joint,
                                                            P_intensity_joint, Q_intensity_joint,
                                                            Q_timestamp_joint,
                                                            cur_ts_min, cur_ts_span,
                                                            deskew_time_alpha,
                                                            Rfit, tfit,
                                                            joint_rmse_before, joint_rmse_after,
                                                            joint_used, joint_iterations,
                                                            joint_solution_usable);

                                const bool joint_rmse_comparable =
                                    std::isfinite(joint_rmse_before) && std::isfinite(joint_rmse_after);
                                const bool accept_joint_solution =
                                    joint_solution_usable &&
                                    (!joint_rmse_comparable || joint_rmse_after <= joint_rmse_before);
                                if (!accept_joint_solution)
                                {
                                    Rfit = Rfit_before_joint.clone();
                                    tfit = tfit_before_joint.clone();
                                    ROS_WARN("Joint Tguess-deskew rejected: usable=%d rmse=%.4f->%.4f; fallback to SVD Tguess",
                                             joint_solution_usable ? 1 : 0,
                                             joint_rmse_before, joint_rmse_after);
                                }

                                const double stable_rmse_after =
                                    computeJointTguessDeskewRmse(P_in, Q_in, Q_timestamp_in,
                                                                 cur_ts_min, cur_ts_span,
                                                                 deskew_time_alpha, Rfit, tfit);

                                ROS_INFO("Joint Tguess-deskew stable RMSE(max-intensity points): %.4f->%.4f; objective RMSE(%s): %.4f->%.4f",
                                         stable_rmse_before, stable_rmse_after,
                                         joint_pair_source.c_str(),
                                         joint_rmse_before, joint_rmse_after);
                            }
                            else
                            {
                                ROS_WARN("Frame %d: joint Tguess-deskew skipped, mode=%s pairs=%zu (< %d), raw=%zu rejected=%zu",
                                         frame_idx_, joint_pair_source.c_str(),
                                         joint_pairs, std::max(3, joint_tguess_min_matches_),
                                         joint_pairs_raw, joint_pairs_rejected);
                            }
                        }

                        // Store and publish the optimized Tguess(cur->prev).
                        got_tguess = true;
                        Rfit_this = Rfit.clone();
                        tfit_this = tfit.clone();

                        double roll = atan2(Rfit.at<double>(2, 1), Rfit.at<double>(2, 2));
                        double pitch = atan2(-Rfit.at<double>(2, 0),
                                             std::sqrt(Rfit.at<double>(2, 1) * Rfit.at<double>(2, 1) + Rfit.at<double>(2, 2) * Rfit.at<double>(2, 2)));
                        double yaw = atan2(Rfit.at<double>(1, 0), Rfit.at<double>(0, 0));

                        double tx = tfit.at<double>(0);
                        double ty = tfit.at<double>(1);
                        double tz = tfit.at<double>(2);

                        ROS_INFO("Tguess frame %d->%d (cur->prev): tx=%.3f ty=%.3f tz=%.3f roll=%.2f pitch=%.2f yaw=%.2f deg",
                                 frame_idx_, frame_idx_ - 1, tx, ty, tz,
                                 roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
                        // if (tguess_pub_.getNumSubscribers() > 0)
                        {
                            nav_msgs::Odometry odom;
                            odom.header.stamp = msg->header.stamp; // same stamp as the current-frame cloud
                            odom.header.frame_id = "tguess_odom";  // parent frame name, user-defined
                            odom.child_frame_id = "tguess_lidar";  // child frame name, user-defined

                            // position
                            odom.pose.pose.position.x = tx;
                            odom.pose.pose.position.y = ty;
                            odom.pose.pose.position.z = tz;

                            // orientation (roll/pitch/yaw are in radians)
                            tf::Quaternion q;
                            q.setRPY(roll, pitch, yaw);
                            odom.pose.pose.orientation.x = q.x();
                            odom.pose.pose.orientation.y = q.y();
                            odom.pose.pose.orientation.z = q.z();
                            odom.pose.pose.orientation.w = q.w();

                            const double ransac3d_ratio_vs_2d =
                                (count_2d_inliers > 0)
                                    ? static_cast<double>(count_3d_inliers) / static_cast<double>(count_2d_inliers)
                                    : 0.0;

                            // covariance[0]/[1] are reused for RANSAC statistics; the other entries keep the prior covariance
                            for (int i = 0; i < 36; ++i)
                                odom.pose.covariance[i] = 0.0;
                            odom.pose.covariance[0] = ransac3d_ratio_vs_2d;
                            odom.pose.covariance[1] = static_cast<double>(count_3d_inliers);
                            odom.pose.covariance[7] = 0.25;  // y
                            odom.pose.covariance[14] = 0.25; // z
                            odom.pose.covariance[21] = 0.05; // roll
                            odom.pose.covariance[28] = 0.05; // pitch
                            odom.pose.covariance[35] = 0.05; // yaw
                            tguess_pub_.publish(odom);
                        }
                    }
                    else
                    {
                        ROS_WARN("estimateRigidSVD failed on final 3D inliers");
                    }
                }

                if (!matched_points_published)
                {
                    publishMatchedPairPoints(P_in, Q_in,
                                             P_intensity_in, Q_intensity_in,
                                             P_timestamp_in, Q_timestamp_in,
                                             "ransac_max_intensity");
                }
            }

        }
        // ---------- 4.5) Use the optimized T(cur->prev) to deskew the current scan ----------
        sensor_msgs::PointCloud2 cloud_for_sampling = *cloud_in; // default: filtered current frame
        sensor_msgs::PointCloud2 deskewed_cloud;
        cv::Mat corrected_cache_img, corrected_cache_desc;
        std::vector<cv::KeyPoint> corrected_cache_kp;
        std::vector<float> corrected_cache_px, corrected_cache_py, corrected_cache_pz, corrected_cache_pint;
        std::vector<double> corrected_cache_pts;
        std::vector<std::vector<PixelPoint>> corrected_cache_pixel_points;
        std::vector<char> corrected_cache_has;
        bool corrected_cache_ready = false;
        if (got_tguess && enable_deskew_current_)
        {
            // Rfit_this/tfit_this are T(cur->prev). Deskew needs the inverse
            // T(prev->cur), which describes the scan-internal forward motion.
            //
            // Note: the Tguess time span is usually inter-frame (e.g. 10 Hz => ~100 ms), while the timestamp span inside a single scan can be shorter (e.g. ~52 ms).
            // Under a constant-velocity model the motion is scaled by the time ratio: alpha = ts_span / dt_guess
            cv::Mat R_use, t_use;
            invertRigidTransform(Rfit_this, tfit_this, R_use, t_use);
            if (deskew_scale_by_time_ && deskew_time_alpha_valid)
            {
                cv::Mat R_scaled, t_scaled;
                scaleTransformByTime(R_use, t_use, deskew_time_alpha, R_scaled, t_scaled);
                R_use = R_scaled;
                t_use = t_scaled;
            }
            const bool deskew_applied =
                deskewPointCloudInPlaceTimestamp(cloud_for_sampling, R_use, t_use);
            if (deskew_applied)
            {
                deskewed_cloud = cloud_for_sampling;
                corrected_cache_ready = rebuildFeatureCacheFromDeskewedCloud(
                    deskewed_cloud,
                    corrected_cache_img, corrected_cache_desc, corrected_cache_kp,
                    corrected_cache_px, corrected_cache_py, corrected_cache_pz,
                    corrected_cache_pint, corrected_cache_pts,
                    corrected_cache_pixel_points, corrected_cache_has);
                if (!corrected_cache_ready)
                    ROS_WARN("[Frame %d] deskew succeeded but corrected feature-cache rebuild failed; using raw cache for the next frame.",
                             frame_idx_);
            }
        }

        const bool publish_sampled_cloud =
            latch_published_topics_ || sampled_pub_.getNumSubscribers() > 0;
        const bool publish_raw_cloud =
            latch_published_topics_ || cloudraw_pub_.getNumSubscribers() > 0;
        const bool publish_deskewed_cloud =
            latch_published_topics_ || deskewedcloud_pub_.getNumSubscribers() > 0;

        if (enable_line_sampling_ && publish_sampled_cloud)
        {
            if (samplePointCloudByRing(cloud_for_sampling, sampled_cloud))
            {
                sampled_cloud.header = msg->header;
                sampled_pub_.publish(sampled_cloud);
            }
        }

        if (publish_raw_cloud)
            cloudraw_pub_.publish(*cloud_in);

        if (publish_deskewed_cloud)
        {
            sensor_msgs::PointCloud2 cloud_to_publish =
                deskewed_cloud.data.empty() ? cloud_for_sampling : deskewed_cloud;
            cloud_to_publish.header = msg->header;
            deskewedcloud_pub_.publish(cloud_to_publish);
        }

        // Update the previous-frame time reference (used for dt_guess of the next frame)
        if (cur_ts_ok)
        {
            prev_scan_start_ts_ = cur_ts_min;
            prev_scan_start_valid_ = true;
        }

        ROS_INFO("[Frame %d] matches: ORB=%zu 2D_inliers=%zu 3D_inliers=%zu",
                 frame_idx_, count_orb_matches, count_2d_inliers, count_3d_inliers);

        // ---------- 6) Update the next-frame reference cache ----------
        // Once a valid deskew is available, the following frame must match
        // against this corrected intensity image and corrected 3D bindings.
        if (corrected_cache_ready)
        {
            prev_img_ = corrected_cache_img.clone();
            prev_desc_ = corrected_cache_desc.clone();
            prev_kp_ = corrected_cache_kp;
            prev_px_ = corrected_cache_px;
            prev_py_ = corrected_cache_py;
            prev_pz_ = corrected_cache_pz;
            prev_pint_ = corrected_cache_pint;
            prev_pts_ = corrected_cache_pts;
            prev_pixel_points_ = corrected_cache_pixel_points;
            prev_has_ = corrected_cache_has;
            ROS_INFO("[Frame %d] next-frame reference uses corrected deskewed cache.", frame_idx_);
        }
        else
        {
            prev_img_ = intensity_enh.clone();
            prev_desc_ = descriptors.clone();
            prev_kp_ = keypoints;
            prev_px_ = pixel_x;
            prev_py_ = pixel_y;
            prev_pz_ = pixel_z;
            prev_pint_ = pixel_intensity;
            prev_pts_ = pixel_timestamp;
            prev_pixel_points_ = pixel_points;
            prev_has_ = has_point;
            ROS_INFO("[Frame %d] next-frame reference uses raw cache (no accepted deskew).", frame_idx_);
        }
        prev_cloud_header_ = msg->header;
        has_prev_frame_ = true;

        frame_idx_++;
    }

}  // namespace LiDAR_Intensity_Assist
