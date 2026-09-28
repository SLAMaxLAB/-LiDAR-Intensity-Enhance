// Class declaration of the intensity-assisted LiDAR odometry front end.
//
// See config/pcd_correction.yaml for the tunable parameters and README.md for
// the processing pipeline and published topics.

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <limits>
#include <random>
#include <string>
#include <vector>

namespace intensity_image_enhance
{

class LidarIntensityORBMatchDual
{

public:
    LidarIntensityORBMatchDual(ros::NodeHandle &nh);

private:
    void loadParameters();
    void initAngleBounds();
    void initRosInterfaces();
    void logConfiguration();

    struct PixelPoint
    {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        float intensity = 0.0f;
        double timestamp = std::numeric_limits<double>::quiet_NaN();
    };

    struct PixAcc
    {
        float max_intensity = 0.0f;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        double timestamp = std::numeric_limits<double>::quiet_NaN();
        bool has_point = false;
    };

    struct PixelPointRecord
    {
        size_t pid = 0;
        PixelPoint point;
    };

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher sampled_pub_;
    ros::Publisher tguess_pub_;
    ros::Publisher cloudraw_pub_;
    ros::Publisher deskewedcloud_pub_;
    ros::Publisher raw_intensity_image_pub_;
    ros::Publisher enhanced_intensity_image_pub_;
    ros::Publisher matchedorb_image_pub_;
    ros::Publisher matched2d_image_pub_;
    ros::Publisher matched3d_image_pub_;
    ros::Publisher matchedprev_points_pub_;
    ros::Publisher matchedcur_points_pub_;

    std::string cloud_topic_;
    std::string sampled_cloud_topic_;
    std::string Tguess_topic_;
    std::string raw_intensity_image_topic_;
    std::string enhanced_intensity_image_topic_;
    std::string matched_orb_image_topic_;
    std::string matched_2d_image_topic_;
    std::string matched_3d_image_topic_;
    std::string matched_prev_points_topic_;
    std::string matched_cur_points_topic_;
    std::string projection_mode_;
    bool show_match_labels_;
    bool publish_match_images_;
    bool publish_raw_intensity_image_;
    bool publish_enhanced_intensity_image_;
    bool publish_matched_points_;
    bool latch_published_topics_;

    int v_res_, h_res_, sample_step_;
    double h_fov_deg_, v_min_deg_, v_max_deg_;
    double min_az_, max_az_, min_el_, max_el_;
    bool filter_origin_points_;
    float origin_filter_eps_;
    bool store_all_pixel_points_;

    // Beam (ring) sampling, publish-only
    bool enable_line_sampling_;
    int source_lines_;
    int target_lines_;
    bool remap_ring_to_compact_;

    // deskew current frame by Tguess (requires per-point timestamp)
    bool enable_deskew_current_;
    bool deskew_use_inverse_tguess_;
    float deskew_min_range_;
    float deskew_placeholder_eps_;
    bool sampling_filter_invalid_;

    // deskew time scaling helpers
    bool deskew_scale_by_time_;
    double deskew_scale_min_;
    double deskew_scale_max_;

    // per-frame scan start timestamp (from per-point 'timestamp' field)
    double prev_scan_start_ts_ = std::numeric_limits<double>::quiet_NaN();
    bool prev_scan_start_valid_ = false;

    // ORB & RANSAC parameters
    float ratio_thresh_;
    int hamming_thresh_;
    double ransac_2d_reproj_;
    double ransac_3d_thresh_;
    int ransac_3d_iters_;

    bool enable_blur_;
    bool enable_bilateral_filter_;
    int bilateral_d_;
    double bilateral_sigma_color_;
    double bilateral_sigma_space_;
    std::string contrast_mode_;
    int match_vis_radius_;
    int match_vis_thickness_;
    bool enable_joint_tguess_deskew_;
    std::string joint_pixel_match_mode_;
    int joint_tguess_max_iters_;
    int joint_tguess_min_matches_;
    double joint_tguess_huber_delta_;
    double joint_tguess_intensity_weight_scale_;
    int joint_tguess_num_threads_;
    bool enable_joint_pair_outlier_rejection_;
    double joint_pair_outlier_abs_thresh_;
    double joint_pair_outlier_mad_k_;
    double joint_pair_outlier_min_thresh_;

    int orb_nfeatures_, orb_nlevels_, orb_edgeThreshold_, orb_patchSize_, orb_fastThreshold_, orb_wta_k_;
    double orb_scaleFactor_;
    cv::ORB::ScoreType orb_score_type_;

    bool enable_ransac_2d_;
    bool enable_multithread_;
    int num_threads_;
    bool enable_parallel_projection_;
    bool enable_parallel_ransac3d_;
    int ransac3d_threads_;

    int frame_idx_;
    bool has_prev_frame_;
    cv::Mat prev_img_;
    cv::Mat prev_desc_;
    std::vector<cv::KeyPoint> prev_kp_;
    std::vector<float> prev_px_, prev_py_, prev_pz_;
    std::vector<float> prev_pint_;
    std::vector<double> prev_pts_;
    std::vector<std::vector<PixelPoint>> prev_pixel_points_;
    std::vector<char> prev_has_;
    std_msgs::Header prev_cloud_header_;

    std::mt19937 rng_;

    struct JointTguessDeskewResidual
    {
        JointTguessDeskewResidual(const cv::Point3f &p_prev,
                                  const cv::Point3f &p_cur,
                                  double s_cur,
                                  double alpha,
                                  double sqrt_weight)
            : px(p_prev.x), py(p_prev.y), pz(p_prev.z),
              qx(p_cur.x), qy(p_cur.y), qz(p_cur.z),
              s(s_cur), a(alpha), sw(sqrt_weight)
        {
        }

        template <typename T>
        bool operator()(const T *const pose, T *residuals) const
        {
            const T p[3] = {T(px), T(py), T(pz)};
            const T q[3] = {T(qx), T(qy), T(qz)};

            // pose is T(cur->prev). Its inverse provides the prev->cur
            // scan motion used to deskew the current-frame measurement.
            const T neg_pose[3] = {-pose[0], -pose[1], -pose[2]};
            const T t_cur_to_prev[3] = {pose[3], pose[4], pose[5]};
            T rotated_translation[3];
            ceres::AngleAxisRotatePoint(neg_pose, t_cur_to_prev, rotated_translation);
            const T t_prev_to_cur[3] = {
                -rotated_translation[0], -rotated_translation[1], -rotated_translation[2]};

            T q_minus_t[3] = {
                q[0] - T(s * a) * t_prev_to_cur[0],
                q[1] - T(s * a) * t_prev_to_cur[1],
                q[2] - T(s * a) * t_prev_to_cur[2],
            };

            T r_deskew[3] = {
                T(s * a) * pose[0],
                T(s * a) * pose[1],
                T(s * a) * pose[2],
            };
            T q_deskew[3];
            ceres::AngleAxisRotatePoint(r_deskew, q_minus_t, q_deskew);

            T q_in_prev[3];
            ceres::AngleAxisRotatePoint(pose, q_deskew, q_in_prev);
            q_in_prev[0] += pose[3];
            q_in_prev[1] += pose[4];
            q_in_prev[2] += pose[5];

            residuals[0] = T(sw) * (q_in_prev[0] - p[0]);
            residuals[1] = T(sw) * (q_in_prev[1] - p[1]);
            residuals[2] = T(sw) * (q_in_prev[2] - p[2]);
            return true;
        }

        double px, py, pz;
        double qx, qy, qz;
        double s;
        double a;
        double sw;
    };

    cv::Point3d deskewFeaturePointByPose(const cv::Point3f &q,
                                         double s,
                                         double alpha,
                                         const cv::Mat &R,
                                         const cv::Mat &t) const;

    static void invertRigidTransform(const cv::Mat &R,
                                     const cv::Mat &t,
                                     cv::Mat &R_inv,
                                     cv::Mat &t_inv);

    cv::Point3d mapCurrentFeatureToPreviousReference(const cv::Point3f &q,
                                                      double s,
                                                      double alpha,
                                                      const cv::Mat &R_cur_to_prev,
                                                      const cv::Mat &t_cur_to_prev,
                                                      const cv::Mat &R_prev_to_cur,
                                                      const cv::Mat &t_prev_to_cur) const;

    static cv::Point3f toPoint3f(const PixelPoint &p);

    void materializePixelPointBuckets(
        size_t total_pix,
        const std::vector<std::vector<PixelPointRecord>> &thread_point_records,
        std::vector<std::vector<PixelPoint>> &pixel_points) const;

    static double medianValue(std::vector<double> values);

    size_t buildNearestPixelSetCorrespondences(
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
        double &reject_threshold_out) const;

    double computeJointTguessDeskewRmse(const std::vector<cv::Point3f> &P,
                                        const std::vector<cv::Point3f> &Q,
                                        const std::vector<double> &Q_ts,
                                        double cur_ts_min,
                                        double cur_ts_span,
                                        double alpha,
                                        const cv::Mat &R,
                                        const cv::Mat &t) const;

    bool refineTguessWithJointDeskew(const std::vector<cv::Point3f> &P,
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
                                     bool &solution_usable_out) const;

    // ---------- 3D rigid transform estimation ----------
    bool estimateRigidSVD(const std::vector<cv::Point3f> &P,
                          const std::vector<cv::Point3f> &Q,
                          cv::Mat &R, cv::Mat &t);

    void applyRT(const cv::Mat &R, const cv::Mat &t,
                        const cv::Point3f &p, cv::Point3f &q) const;

    std::vector<int> ransac3D(const std::vector<cv::Point3f> &P,
                              const std::vector<cv::Point3f> &Q,
                              double thresh, int iters);

    std::vector<int> ransac3D_parallel(const std::vector<cv::Point3f> &P,
                                       const std::vector<cv::Point3f> &Q,
                                       double thresh, int iters, int threads);

    void drawImageLabel(cv::Mat &image_bgr, const std::string &text,
                        int y_offset, const cv::Scalar &color) const;

    cv::Mat buildMatchesStackedVisualization(const cv::Mat &prev_img, const cv::Mat &cur_img,
                                             const std::vector<cv::KeyPoint> &prev_kp,
                                             const std::vector<cv::KeyPoint> &cur_kp,
                                             const std::vector<cv::DMatch> &matches,
                                             const std::string &prev_label,
                                             const std::string &cur_label);

    bool shouldPublishImage(const ros::Publisher &publisher) const;

    bool shouldPublishRawIntensityImage() const;

    bool shouldPublishEnhancedIntensityImage() const;

    bool shouldPublishMatchedPointClouds() const;

    void publishVisualizationImage(const cv::Mat &image,
                                   const std_msgs::Header &header,
                                   ros::Publisher &publisher);

    void publishLabeledIntensityImage(const cv::Mat &image, const std::string &label,
                                      const std_msgs::Header &header, ros::Publisher &publisher);

    void publishRawIntensityImage(const cv::Mat &image, const std::string &label,
                                  const std_msgs::Header &header);

    void publishEnhancedIntensityImage(const cv::Mat &image, const std::string &label,
                                       const std_msgs::Header &header);

    void publishMatchedORBImage(const cv::Mat &image, const std_msgs::Header &header);

    void publishMatched2DImage(const cv::Mat &image, const std_msgs::Header &header);

    void publishMatched3DImage(const cv::Mat &image, const std_msgs::Header &header);

    sensor_msgs::PointCloud2 buildMatchedPointsCloud(const std::vector<cv::Point3f> &points,
                                                     const std::vector<double> &intensities,
                                                     const std::vector<double> &timestamps,
                                                     const std_msgs::Header &header) const;

    void publishMatched3DPointClouds(const std::vector<cv::Point3f> &prev_points,
                                     const std::vector<cv::Point3f> &cur_points,
                                     const std::vector<double> &prev_intensities,
                                     const std::vector<double> &cur_intensities,
                                     const std::vector<double> &prev_timestamps,
                                     const std::vector<double> &cur_timestamps,
                                     const std_msgs::Header &prev_header,
                                     const std_msgs::Header &cur_header);

    bool filterOriginPoints(const sensor_msgs::PointCloud2 &in,
                            sensor_msgs::PointCloud2 &out,
                            size_t &removed_points) const;

    // ---------- Beam sampling: select a subset of rings and emit a new cloud (publish only) ----------
    bool samplePointCloudByRing(const sensor_msgs::PointCloud2 &in,
                                sensor_msgs::PointCloud2 &out);

    // ---------- Timestamp statistics (used for deskew time scaling) ----------
    // Collects ts_min/ts_max from the per-point "timestamp" (float64) field of a PointCloud2, plus mid/span.
    // The same filtering rules as the deskew path apply: skip NaN, (0,0,0) placeholders, and near-origin points.
    bool computeCloudTimestampStats(const sensor_msgs::PointCloud2 &cloud,
                                    double &ts_min, double &ts_max,
                                    double &ts_mid, double &ts_span) const;

    // ---------- SE(3) time scaling: scale an inter-frame Tguess down to the intra-scan motion ----------
    // Under a constant-velocity assumption the motion is Twist * dt, so log(SE(3)) scales linearly with time.
    // Approximation used here: R_scaled = Exp(alpha * log(R)), t_scaled = alpha * t
    static inline void scaleTransformByTime(const cv::Mat &R_in, const cv::Mat &t_in,
                                            double alpha, cv::Mat &R_out, cv::Mat &t_out);

    // ---------- Point-cloud deskewing: per-point timestamp + Tguess (constant-velocity model) ----------
    // Notes:
    //  - requires the PointCloud2 fields x/y/z/timestamp (timestamp as float64)
    //  - Tguess (R_end, t_end) approximates the start->end motion over one scan period
    //  - every point is interpolated by s = (ts - ts_min) / (ts_max - ts_min) and compensated back to the scan start
    //
    // Deskewing rewrites x/y/z in place inside cloud.data; all other fields are left untouched.
    bool deskewPointCloudInPlaceTimestamp(sensor_msgs::PointCloud2 &cloud,
                                          const cv::Mat &R_end_in,
                                          const cv::Mat &t_end_in);

    // ---------- Projection mode 1: angle (az + el) ----------
    void buildIntensityImageAngle(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<float> &pixel_intensity,
        std::vector<double> &pixel_timestamp,
        std::vector<std::vector<PixelPoint>> &pixel_points,
        std::vector<char> &has_point);

    // ---------- Projection mode 2: ring + azimuth ----------
    void buildIntensityImageRing(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<float> &pixel_intensity,
        std::vector<double> &pixel_timestamp,
        std::vector<std::vector<PixelPoint>> &pixel_points,
        std::vector<char> &has_point);

    bool rebuildFeatureCacheFromDeskewedCloud(
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
        std::vector<char> &has_point_out);

    // ---------- Callback ----------
    void callback(const sensor_msgs::PointCloud2ConstPtr &msg);

};

}  // namespace intensity_image_enhance
