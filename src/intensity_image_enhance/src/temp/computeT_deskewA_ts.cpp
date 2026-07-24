// lidar_intensity_orb_match_dual_sampling_pubonly.cpp
//
// 两种投影模式可选：
//  1) angle:  行 = elevation 角，列 = azimuth 角
//  2) ring :  行 = ring 线束编号，列 = azimuth 角
//
// 新增：
//  - 对原始 128 线点云按线束均匀采样到 128/64/32/16（参数可选）
//  - 只发布采样后的点云，不参与强度图投影
//
// 强度图投影：
//  - 始终使用原始 128 线点云（回调里收到的 msg）进行投影
//  - 剔除 intensity <= 0 的点
//  - 一个像素内取强度最大的点
//  - 点云读取使用裸指针 + offset
//  - 投影阶段拆分：offset / project / merge / normalize + sampling，各自耗时输出
//
// 后续：图像增强 + ORB + 匹配 + 2D/3D-RANSAC + 刚体估计 + time_log
//
// 主要参数（ROS param，可通过 launch 设置）:
//
//  ~projection_mode            : "angle" 或 "ring"（默认 "angle"）
//  ~output_dir                 : 输出目录，默认 "./lidar_output"
//  ~cloud_topic                : 点云话题，默认 "/lidar_points"
//  ~sampled_cloud_topic        : 采样后点云话题，默认 "sampled_points"
//  ~v_res                      : 垂直分辨率（行数），默认 128
//  ~h_res                      : 水平分辨率（列数），默认 500
//  ~h_fov_deg                  : 水平 FOV，默认 120 度
//  ~v_min_deg, ~v_max_deg      : 垂直角范围（angle 模式用），默认 -12.5 ~ 12.9
//  ~sample_step                : 点云采样步长（按点索引采样），默认 1（不采样）
//
//  ~enable_line_sampling       : 是否启用按 ring 采样并发布，默认 false
//  ~source_lines               : 原始线数，默认 128
//  ~target_lines               : 目标线数，默认 128（建议设为 128/64/32/16）
//  ~remap_ring_to_compact      : 是否把被选中的 ring 映射到 [0, target_lines-1]，默认 true
//
//  ~ratio_thresh               : ORB 比率阈值，默认 0.75
//  ~hamming_thresh             : Hamming 距离阈值，默认 50
//  ~ransac_2d_reproj           : 2D RANSAC 重投影误差阈，默认 3 像素
//  ~ransac_3d_thresh           : 3D RANSAC 距离阈，默认 0.2 m
//  ~ransac_3d_iters            : 3D RANSAC 迭代次数，默认 100
//
//  ~enable_blur                : 是否高斯模糊，默认 true
//  ~contrast_mode              : "none" / "equalize" / "clahe"，默认 "clahe"
//  ~orb_vis_radius             : ORB 可视化点半径，默认 2
//
//  ~enable_ransac_2d           : 是否启用 2D-RANSAC，默认 true
//  ~enable_multithread         : 是否启用多线程（OpenCV+OMP），默认 true
//  ~num_threads                : 默认线程数，默认 8
//  ~enable_parallel_projection : 投影阶段是否并行，默认 true
//  ~enable_parallel_ransac3d   : 3D-RANSAC 是否并行，默认 true
//  ~ransac3d_threads           : 3D-RANSAC 线程数，默认 = num_threads
//
// 输出：
//  intensity_raw_#.png      : 原始强度图
//  intensity_enh_#.png      : 增强强度图
//  orb_vis_#.png            : ORB 关键点可视化
//  match_2d_#.png           : 2D 匹配可视化
//  match_3d_#.png           : 3D 内点匹配可视化
//  matches_3d_#.txt         : 3D 对应点列表
//  all_Tguess.txt           : 每帧估计的 T_car（tx ty tz roll pitch yaw）
//  time_log.csv             : 每帧各阶段耗时（ms）
//
// 注意：需要点云包含字段 x,y,z,intensity，若使用 ring 模式或线束采样，还需要 ring 字段。

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <random>
#include <fstream>
#include <iomanip>
#include <limits>
#include <cstring>

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

template <typename T>
inline T clampValue(T v, T lo, T hi)
{
    return (v < lo) ? lo : (v > hi ? hi : v);
}

inline double dist3D2(const cv::Point3f &a, const cv::Point3f &b)
{
    double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

class LidarIntensityORBMatchDual
{
public:
    LidarIntensityORBMatchDual(ros::NodeHandle &nh)
        : nh_(nh), frame_idx_(0), has_prev_frame_(false), has_prev_cloud_msg_(false)
    {
        // -------- 参数读取 --------
        nh_.param<std::string>("projection_mode", projection_mode_, std::string("ring")); // "angle" 或 "ring"
        nh_.param<std::string>("output_dir", output_dir_, std::string("./lidar_output"));
        nh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/lidar_points"));
        nh_.param<std::string>("sampled_cloud_topic", sampled_cloud_topic_, std::string("/sampled_points"));
        nh_.param<std::string>("Tguess_topic", Tguess_topic_, std::string("/Tguess"));

        // ---------- deskew (LOAM 匀速模型) ----------
        nh_.param<bool>("enable_deskew", enable_deskew_, false);
        nh_.param<std::string>("deskewed_cloud_topic", deskewed_cloud_topic_, std::string("/deskew_points"));
        nh_.param<int>("deskew_min_inliers", deskew_min_inliers_, 15);
        nh_.param<double>("deskew_max_trans", deskew_max_trans_, 3.0);
        nh_.param<double>("deskew_max_rot_deg", deskew_max_rot_deg_, 20.0);

        nh_.param<int>("v_res", v_res_, 128);
        nh_.param<int>("h_res", h_res_, 500);
        nh_.param<double>("h_fov_deg", h_fov_deg_, 120.0);
        nh_.param<double>("v_min_deg", v_min_deg_, -12.5);
        nh_.param<double>("v_max_deg", v_max_deg_, 12.9);
        nh_.param<int>("sample_step", sample_step_, 1);

        nh_.param<bool>("enable_line_sampling", enable_line_sampling_, true);
        nh_.param<int>("source_lines", source_lines_, 128);
        nh_.param<int>("target_lines", target_lines_, 32);
        nh_.param<bool>("remap_ring_to_compact", remap_ring_to_compact_, true);

        nh_.param<float>("ratio_thresh", ratio_thresh_, 0.75f);
        nh_.param<int>("hamming_thresh", hamming_thresh_, 50);
        nh_.param<double>("ransac_2d_reproj", ransac_2d_reproj_, 3.0);
        nh_.param<double>("ransac_3d_thresh", ransac_3d_thresh_, 0.1);
        nh_.param<int>("ransac_3d_iters", ransac_3d_iters_, 100);

        nh_.param<bool>("enable_blur", enable_blur_, true);
        nh_.param<std::string>("contrast_mode", contrast_mode_, std::string("clahe"));
        nh_.param<int>("orb_vis_radius", orb_vis_radius_, 2);

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

        // 创建输出目录
        if (mkdir(output_dir_.c_str(), 0777) && errno != EEXIST)
        {
            ROS_WARN("mkdir failed or already exists: %s", output_dir_.c_str());
        }

        // 角度边界（弧度）
        min_az_ = -h_fov_deg_ / 2.0 * M_PI / 180.0;
        max_az_ = h_fov_deg_ / 2.0 * M_PI / 180.0;
        min_el_ = v_min_deg_ * M_PI / 180.0;
        max_el_ = v_max_deg_ * M_PI / 180.0;

        // 订阅和发布
        sub_ = nh_.subscribe(cloud_topic_, 1, &LidarIntensityORBMatchDual::callback, this);
        sampled_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(sampled_cloud_topic_, 1);
        tguess_pub_ = nh_.advertise<nav_msgs::Odometry>(Tguess_topic_, 1);
        deskew_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(deskewed_cloud_topic_, 1);
        cloudraw_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_raw", 1);
        // RNG
        rng_.seed(std::random_device{}());

        // OpenCV 线程设置
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

        // time_log.csv
        std::ofstream tlog(output_dir_ + "/time_log.csv", std::ios::app);
        if (tlog.tellp() == 0)
        {
            tlog << "frame,sample_ms,proj_ms,enh_ms,orb_ms,match_ms,ransac2d_ms,ransac3d_ms,total_ms\n";
        }
        tlog.close();

        ROS_INFO("Node initialized. Output dir: %s", output_dir_.c_str());
        ROS_INFO("Image size: %d x %d ; h_fov: %.2f deg ; v_range: %.2f ~ %.2f deg",
                 v_res_, h_res_, h_fov_deg_, v_min_deg_, v_max_deg_);
        ROS_INFO("Projection mode = %s (\"angle\" or \"ring\")",
                 projection_mode_.c_str());
        ROS_INFO("Line sampling (publish only): %s, source_lines=%d target_lines=%d remap=%s",
                 enable_line_sampling_ ? "ENABLED" : "DISABLED",
                 source_lines_, target_lines_, remap_ring_to_compact_ ? "true" : "false");
        ROS_INFO("enable_deskew: %s", enable_deskew_ ? "true" : "false");
    }

private:
    struct PixAcc
    {
        float max_intensity = 0.0f;
        float x = 0.0f, y = 0.0f, z = 0.0f;
        bool has_point = false;
    };

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher sampled_pub_;
    ros::Publisher tguess_pub_;

    ros::Publisher deskew_pub_;
    ros::Publisher cloudraw_pub_;
    std::string output_dir_;
    std::string cloud_topic_;
    std::string sampled_cloud_topic_;
    std::string Tguess_topic_;
    std::string deskewed_cloud_topic_;
    std::string projection_mode_;

    int v_res_, h_res_, sample_step_;
    double h_fov_deg_, v_min_deg_, v_max_deg_;
    double min_az_, max_az_, min_el_, max_el_;

    // 线束采样相关（仅用于发布）
    bool enable_line_sampling_;
    int source_lines_;
    int target_lines_;
    bool remap_ring_to_compact_;

    // deskew（匀速模型）
    bool enable_deskew_;
    int deskew_min_inliers_;
    double deskew_max_trans_;
    double deskew_max_rot_deg_;

    // ORB & RANSAC 参数
    float ratio_thresh_;
    int hamming_thresh_;
    double ransac_2d_reproj_;
    double ransac_3d_thresh_;
    int ransac_3d_iters_;

    bool enable_blur_;
    std::string contrast_mode_;
    int orb_vis_radius_;

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
    std::vector<char> prev_has_;

    // 缓存上一帧原始点云（用于方案A：一帧延迟 deskew）
    sensor_msgs::PointCloud2 prev_cloud_msg_;
    sensor_msgs::PointCloud2 deskewed_cloud_msg_;
    bool has_prev_cloud_msg_;

    std::mt19937 rng_;

    // ---------- 3D 刚体估计 ----------
    bool estimateRigidSVD(const std::vector<cv::Point3f> &P,
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

    inline void applyRT(const cv::Mat &R, const cv::Mat &t,
                        const cv::Point3f &p, cv::Point3f &q) const
    {
        const double *r = (const double *)R.data;
        const double *tv = (const double *)t.data;
        double x = p.x, y = p.y, z = p.z;
        q.x = (float)(r[0] * x + r[1] * y + r[2] * z + tv[0]);
        q.y = (float)(r[3] * x + r[4] * y + r[5] * z + tv[1]);
        q.z = (float)(r[6] * x + r[7] * y + r[8] * z + tv[2]);
    }

    std::vector<int> ransac3D(const std::vector<cv::Point3f> &P,
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

    std::vector<int> ransac3D_parallel(const std::vector<cv::Point3f> &P,
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

    void visualizeMatchesStacked(const cv::Mat &prev_img, const cv::Mat &cur_img,
                                 const std::vector<cv::KeyPoint> &prev_kp,
                                 const std::vector<cv::KeyPoint> &cur_kp,
                                 const std::vector<cv::DMatch> &matches,
                                 const std::string &path)
    {
        cv::Mat prev_color, cur_color;
        cv::cvtColor(prev_img, prev_color, cv::COLOR_GRAY2BGR);
        cv::cvtColor(cur_img, cur_color, cv::COLOR_GRAY2BGR);

        int w = std::max(prev_color.cols, cur_color.cols);
        int h = prev_color.rows + cur_color.rows;
        cv::Mat canvas(h, w, CV_8UC3, cv::Scalar(0, 0, 0));
        prev_color.copyTo(canvas(cv::Rect(0, 0, prev_color.cols, prev_color.rows)));
        cur_color.copyTo(canvas(cv::Rect(0, prev_color.rows, cur_color.cols, cur_color.rows)));

        std::mt19937 rng_local(frame_idx_);
        std::uniform_int_distribution<int> ud(0, 255);

        for (const auto &m : matches)
        {
            cv::Point2f a = prev_kp[m.queryIdx].pt;
            cv::Point2f b = cur_kp[m.trainIdx].pt + cv::Point2f(0.0f, (float)prev_color.rows);
            cv::Scalar color(ud(rng_local), ud(rng_local), ud(rng_local));
            cv::circle(canvas, a, 2, color, -1);
            cv::circle(canvas, b, 2, color, -1);
            cv::line(canvas, a, b, color, 1, cv::LINE_AA);
        }
        cv::imwrite(path, canvas);
    }

    // ---------- 线束采样：按 ring 选择部分线束并输出新点云（只用于发布） ----------
    bool samplePointCloudByRing(const sensor_msgs::PointCloud2 &in,
                                sensor_msgs::PointCloud2 &out,
                                double &t_sample_ms)
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        t_sample_ms = 0.0;

        if (!enable_line_sampling_ ||
            target_lines_ >= source_lines_ ||
            target_lines_ <= 0 ||
            source_lines_ <= 0)
        {
            return false; // 不采样
        }
        if (source_lines_ % target_lines_ != 0)
        {
            ROS_WARN_THROTTLE(1.0,
                              "line sampling disabled: source_lines(%d) %% target_lines(%d) != 0",
                              source_lines_, target_lines_);
            return false;
        }

        // 找 ring 字段
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
                const uint16_t *pr = reinterpret_cast<const uint16_t *>(p + offset_ring);
                ring = (int)(*pr);
            }
            else if (ring_datatype == sensor_msgs::PointField::INT16)
            {
                const int16_t *pr = reinterpret_cast<const int16_t *>(p + offset_ring);
                ring = (int)(*pr);
            }
            else
            {
                const uint16_t *pr = reinterpret_cast<const uint16_t *>(p + offset_ring);
                ring = (int)(*pr);
            }

            if (ring < 0 || ring >= source_lines_)
                continue;
            int new_ring = ring_to_new[ring];
            if (new_ring < 0)
                continue; // 该线束未被选中

            size_t cur_off = out_data.size();
            out_data.resize(cur_off + point_step);
            std::memcpy(out_data.data() + cur_off, p, point_step);

            if (remap_ring_to_compact_)
            {
                uint8_t *pr_out = out_data.data() + cur_off + offset_ring;
                if (ring_datatype == sensor_msgs::PointField::UINT16)
                {
                    *reinterpret_cast<uint16_t *>(pr_out) = (uint16_t)new_ring;
                }
                else if (ring_datatype == sensor_msgs::PointField::INT16)
                {
                    *reinterpret_cast<int16_t *>(pr_out) = (int16_t)new_ring;
                }
                else
                {
                    *reinterpret_cast<uint16_t *>(pr_out) = (uint16_t)new_ring;
                }
            }
            ++kept_points;
        }

        auto t1 = std::chrono::high_resolution_clock::now();
        t_sample_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        if (kept_points == 0)
        {
            ROS_WARN_THROTTLE(1.0, "line sampling produced empty cloud.");
            return false;
        }

        out = in; // 拷贝元信息
        out.data.swap(out_data);
        out.width = kept_points;
        out.height = 1;
        out.row_step = out.point_step * out.width;

        ROS_INFO("[Frame %d] line sampling (publish only): points_in=%zu points_out=%zu, time=%.3f ms",
                 frame_idx_, num_points, kept_points, t_sample_ms);

        return true;
    }

    // ---------- 投影模式 1：angle（az + el） ----------
    void buildIntensityImageAngle(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<char> &has_point,
        double &t_offset_ms,
        double &t_project_ms,
        double &t_merge_ms)
    {
        auto t_off_start = std::chrono::high_resolution_clock::now();

        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0)
        {
            t_offset_ms = t_project_ms = t_merge_ms = 0.0;
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            has_point.clear();
            return;
        }

        int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1;
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
        }

        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_i < 0)
        {
            ROS_ERROR("buildIntensityImageAngle: x/y/z/intensity fields not all found.");
            t_offset_ms = t_project_ms = t_merge_ms = 0.0;
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            has_point.clear();
            return;
        }

        const uint8_t *base_ptr = cloud.data.data();
        const size_t point_step = cloud.point_step;

        auto t_off_end = std::chrono::high_resolution_clock::now();
        t_offset_ms = std::chrono::duration<double, std::milli>(t_off_end - t_off_start).count();

        // ---- project ----
        auto t_proj_start = std::chrono::high_resolution_clock::now();

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

#ifdef _OPENMP
#pragma omp parallel num_threads(omp_threads)
#endif
        {
            int tid = 0;
#ifdef _OPENMP
            tid = omp_get_thread_num();
#endif
            auto &grid = thread_grids[(size_t)tid];

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
            for (int i = 0; i < (int)num_points; ++i)
            {
                if (sample_step_ > 1 && (i % sample_step_ != 0))
                    continue;

                const uint8_t *p_base = base_ptr + (size_t)i * point_step;
                const float *px = reinterpret_cast<const float *>(p_base + offset_x);
                const float *py = reinterpret_cast<const float *>(p_base + offset_y);
                const float *pz = reinterpret_cast<const float *>(p_base + offset_z);
                const float *pi = reinterpret_cast<const float *>(p_base + offset_i);

                float x = *px;
                float y = *py;
                float z = *pz;
                float intensity = *pi;

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

                double uf = (az - min_az_) / (max_az_ - min_az_) * (h_res - 1);
                double vf = (max_el_ - el) / (max_el_ - min_el_) * (v_res - 1);

                int u = (int)std::round(uf);
                int v = (int)std::round(vf);
                if (u < 0 || u >= h_res || v < 0 || v >= v_res)
                    continue;

                size_t pid = (size_t)v * h_res + (size_t)u;
                PixAcc &cell = grid[pid];

                if (!cell.has_point || intensity > cell.max_intensity)
                {
                    cell.max_intensity = intensity;
                    cell.x = x;
                    cell.y = y;
                    cell.z = z;
                    cell.has_point = true;
                }
            }
        }

        auto t_proj_end = std::chrono::high_resolution_clock::now();
        t_project_ms = std::chrono::duration<double, std::milli>(t_proj_end - t_proj_start).count();

        // ---- merge ----
        auto t_merge_start = std::chrono::high_resolution_clock::now();

        std::vector<PixAcc> global_grid(total_pix);
        for (int t = 0; t < omp_threads; ++t)
        {
            const auto &g = thread_grids[(size_t)t];
            for (size_t pid = 0; pid < total_pix; ++pid)
            {
                const PixAcc &src = g[pid];
                PixAcc &dst = global_grid[pid];
                if (src.has_point && (!dst.has_point || src.max_intensity > dst.max_intensity))
                    dst = src;
            }
        }

        auto t_merge_end = std::chrono::high_resolution_clock::now();
        t_merge_ms = std::chrono::duration<double, std::milli>(t_merge_end - t_merge_start).count();

        // ---- output ----
        intensity_f = cv::Mat(v_res, h_res, CV_32F, cv::Scalar(0.0f));
        pixel_x.assign(total_pix, 0.0f);
        pixel_y.assign(total_pix, 0.0f);
        pixel_z.assign(total_pix, 0.0f);
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
            has_point[pid] = 1;
        }
    }

    // ---------- 投影模式 2：ring + azimuth ----------
    void buildIntensityImageRing(
        const sensor_msgs::PointCloud2 &cloud,
        cv::Mat &intensity_f,
        std::vector<float> &pixel_x,
        std::vector<float> &pixel_y,
        std::vector<float> &pixel_z,
        std::vector<char> &has_point,
        double &t_offset_ms,
        double &t_project_ms,
        double &t_merge_ms)
    {
        auto t_off_start = std::chrono::high_resolution_clock::now();

        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0)
        {
            t_offset_ms = t_project_ms = t_merge_ms = 0.0;
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            has_point.clear();
            return;
        }

        int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1, offset_ring = -1;
        int ring_datatype = -1;

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
        }

        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_i < 0 || offset_ring < 0)
        {
            ROS_ERROR("buildIntensityImageRing: x/y/z/intensity/ring fields not all found.");
            t_offset_ms = t_project_ms = t_merge_ms = 0.0;
            intensity_f.release();
            pixel_x.clear();
            pixel_y.clear();
            pixel_z.clear();
            has_point.clear();
            return;
        }

        const uint8_t *base_ptr = cloud.data.data();
        const size_t point_step = cloud.point_step;

        auto t_off_end = std::chrono::high_resolution_clock::now();
        t_offset_ms = std::chrono::duration<double, std::milli>(t_off_end - t_off_start).count();

        // ---- project ----
        auto t_proj_start = std::chrono::high_resolution_clock::now();

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

#ifdef _OPENMP
#pragma omp parallel num_threads(omp_threads)
#endif
        {
            int tid = 0;
#ifdef _OPENMP
            tid = omp_get_thread_num();
#endif
            auto &grid = thread_grids[(size_t)tid];

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

                const float *px = reinterpret_cast<const float *>(p_base + offset_x);
                const float *py = reinterpret_cast<const float *>(p_base + offset_y);
                const float *pz = reinterpret_cast<const float *>(p_base + offset_z);
                const float *pi = reinterpret_cast<const float *>(p_base + offset_i);

                float x = *px;
                float y = *py;
                float z = *pz;
                float intensity = *pi;

                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    !std::isfinite(intensity) || intensity <= 0.0f)
                {
                    continue;
                }

                double az = std::atan2((double)y, (double)x);
                if (az < min_az_ || az > max_az_)
                    continue;

                double uf = (az - min_az_) / (max_az_ - min_az_) * (h_res - 1);
                int u = (int)std::round(uf);
                int v = ring; // 行 = ring

                if (u < 0 || u >= h_res || v < 0 || v >= v_res)
                    continue;

                size_t pid = (size_t)v * h_res + (size_t)u;
                PixAcc &cell = grid[pid];

                if (!cell.has_point || intensity > cell.max_intensity)
                {
                    cell.max_intensity = intensity;
                    cell.x = x;
                    cell.y = y;
                    cell.z = z;
                    cell.has_point = true;
                }
            }
        }

        auto t_proj_end = std::chrono::high_resolution_clock::now();
        t_project_ms = std::chrono::duration<double, std::milli>(t_proj_end - t_proj_start).count();

        // ---- merge ----
        auto t_merge_start = std::chrono::high_resolution_clock::now();

        std::vector<PixAcc> global_grid(total_pix);
        for (int t = 0; t < omp_threads; ++t)
        {
            const auto &g = thread_grids[(size_t)t];
            for (size_t pid = 0; pid < total_pix; ++pid)
            {
                const PixAcc &src = g[pid];
                PixAcc &dst = global_grid[pid];
                if (src.has_point && (!dst.has_point || src.max_intensity > dst.max_intensity))
                    dst = src;
            }
        }

        auto t_merge_end = std::chrono::high_resolution_clock::now();
        t_merge_ms = std::chrono::duration<double, std::milli>(t_merge_end - t_merge_start).count();

        // ---- output ----
        intensity_f = cv::Mat(v_res, h_res, CV_32F, cv::Scalar(0.0f));
        pixel_x.assign(total_pix, 0.0f);
        pixel_y.assign(total_pix, 0.0f);
        pixel_z.assign(total_pix, 0.0f);
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
            has_point[pid] = 1;
        }
    }

    // ---------- deskew: 参考 LOAM 匀速模型，把点云矫正到扫描起始时刻 ----------
    // 约定：R_end,t_end 表示 start->end 的相对位姿（点从 start 坐标变到 end 坐标）:
    //       p_end = R_end * p_start + t_end
    //
    // 对扫描内任意时刻 s∈[0,1]，做线性插值：
    //   roll(s)=s*roll_end, pitch(s)=s*pitch_end, yaw(s)=s*yaw_end
    //   t(s)=s*t_end
    // 然后把点从“采样时刻坐标系”变回 start：
    //   p_start = R(s)^T * (p_raw - t(s))
    //
    // 说明：这是 LOAM 常用的近似（Euler 角线性 + 平移线性），不是严格的 SE(3) exp 插值，
    //      但计算量小，通常足够用于 deskew。
    inline void rpyToMatrixZYX(double roll, double pitch, double yaw, double R[9]) const
    {
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        double cr = std::cos(roll), sr = std::sin(roll);
        double cp = std::cos(pitch), sp = std::sin(pitch);
        double cy = std::cos(yaw), sy = std::sin(yaw);

        R[0] = cy * cp;
        R[1] = cy * sp * sr - sy * cr;
        R[2] = cy * sp * cr + sy * sr;

        R[3] = sy * cp;
        R[4] = sy * sp * sr + cy * cr;
        R[5] = sy * sp * cr - cy * sr;

        R[6] = -sp;
        R[7] = cp * sr;
        R[8] = cp * cr;
    }

    inline void rotMatToRPYZYX(const cv::Mat &R, double &roll, double &pitch, double &yaw) const
    {
        // 与上面的 ZYX 顺序一致
        roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
        pitch = std::atan2(-R.at<double>(2, 0),
                           std::sqrt(R.at<double>(2, 1) * R.at<double>(2, 1) + R.at<double>(2, 2) * R.at<double>(2, 2)));
        yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
    }

    // ---------- deskew: 使用每点 timestamp 计算 s（最推荐，适合 AT128 这类按 ring 排列的非组织化点云） ----------
    // 要求点云包含字段：x,y,z,timestamp（ring/intensity 可有可无）
    // s = (ts - ts_min) / (ts_max - ts_min)
    // 其余补偿公式与 LOAM 匀速模型一致：
    //   p_start = R(s)^T * (p_raw - t(s))
    // 其中 R(s),t(s) 采用 end 时刻的 rpy/t 做线性插值（LOAM 常用近似）
    bool deskewPointCloudInPlaceTimestamp(sensor_msgs::PointCloud2 &cloud,
                                          const cv::Mat &R_end,
                                          const cv::Mat &t_end) const
    {
        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0 || cloud.data.empty())
            return false;

        // 找 x/y/z/timestamp offset
        int offset_x = -1, offset_y = -1, offset_z = -1, offset_ts = -1;
        for (const auto &f : cloud.fields)
        {
            if (f.name == "x")
                offset_x = f.offset;
            else if (f.name == "y")
                offset_y = f.offset;
            else if (f.name == "z")
                offset_z = f.offset;
            else if (f.name == "timestamp")
                offset_ts = f.offset;
        }
        if (offset_x < 0 || offset_y < 0 || offset_z < 0 || offset_ts < 0)
        {
            // 没有 timestamp 就无法用该方法
            return false;
        }

        if (R_end.empty() || t_end.empty() ||
            R_end.rows != 3 || R_end.cols != 3 ||
            t_end.rows != 3 || t_end.cols != 1)
        {
            ROS_ERROR("deskewPointCloudInPlaceTimestamp: invalid R_end/t_end shape.");
            return false;
        }

        const size_t point_step = cloud.point_step;
        uint8_t *base_ptr = cloud.data.data();

        // offset 边界检查，避免越界读写
        if ((size_t)offset_x + sizeof(float) > point_step ||
            (size_t)offset_y + sizeof(float) > point_step ||
            (size_t)offset_z + sizeof(float) > point_step ||
            (size_t)offset_ts + sizeof(double) > point_step)
        {
            ROS_ERROR("deskewPointCloudInPlaceTimestamp: field offsets exceed point_step (point_step=%zu)", point_step);
            return false;
        }

        // NOTE:
        // 一些 Hesai/ROS 驱动会用 (0,0,0) 作为“无效点/空回波”的占位。
        // 原始点云里这些点全部叠在原点，视觉上不明显；一旦做 deskew（按 s 插值平移），
        // 这些原点占位会被展开成一束从原点出发的“密集射线”（典型伪影，与你描述一致）。
        // 处理策略：
        //  1) 计算 ts_min/ts_max 时忽略占位点，避免它们污染时间跨度。
        //  2) deskew 时将占位点写成 NaN（RViz/PCL 通常会忽略 NaN），避免产生射线。
        const float kMinNorm1 = 1e-6f; // |x|+|y|+|z| < 该阈值视为占位点
        const float kNaN = std::numeric_limits<float>::quiet_NaN();

        // 1) 统计本帧 ts_min/ts_max（用 memcpy 避免未对齐 double 读取 UB）
        double ts_min = std::numeric_limits<double>::infinity();
        double ts_max = -std::numeric_limits<double>::infinity();

        size_t skipped_placeholder_for_span = 0;
        size_t skipped_nonfinite_for_span = 0;
        for (size_t i = 0; i < num_points; ++i)
        {
            uint8_t *p_base = base_ptr + i * point_step;
            // 忽略无效占位点（避免它们的 timestamp 影响 ts_span）
            float x, y, z;
            std::memcpy(&x, p_base + offset_x, sizeof(float));
            std::memcpy(&y, p_base + offset_y, sizeof(float));
            std::memcpy(&z, p_base + offset_z, sizeof(float));
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                skipped_nonfinite_for_span++;
                continue;
            }
            if (std::fabs(x) + std::fabs(y) + std::fabs(z) < kMinNorm1)
            {
                skipped_placeholder_for_span++;
                continue;
            }

            double ts;
            std::memcpy(&ts, p_base + offset_ts, sizeof(double));
            if (!std::isfinite(ts))
                continue;
            if (ts < ts_min)
                ts_min = ts;
            if (ts > ts_max)
                ts_max = ts;
        }

        if (!(ts_max > ts_min))
        {
            ROS_WARN_THROTTLE(1.0, "deskewPointCloudInPlaceTimestamp: invalid timestamp span (min=%.6f max=%.6f).", ts_min, ts_max);
            return false;
        }

        const double ts_span = ts_max - ts_min;

        // 2) 提取 end 时刻的 rpy + t
        double roll_end = 0.0, pitch_end = 0.0, yaw_end = 0.0;
        rotMatToRPYZYX(R_end, roll_end, pitch_end, yaw_end);

        const double tx_end = t_end.at<double>(0);
        const double ty_end = t_end.at<double>(1);
        const double tz_end = t_end.at<double>(2);

        // 3) 遍历每个点，按 timestamp 比例 s 插值并补偿
        size_t nan_written = 0;
        for (size_t i = 0; i < num_points; ++i)
        {
            uint8_t *p_base = base_ptr + i * point_step;

            float x, y, z;
            std::memcpy(&x, p_base + offset_x, sizeof(float));
            std::memcpy(&y, p_base + offset_y, sizeof(float));
            std::memcpy(&z, p_base + offset_z, sizeof(float));

            // 无效点：写 NaN，避免被 deskew 展开成“从原点出发的密集射线”
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                (std::fabs(x) + std::fabs(y) + std::fabs(z) < kMinNorm1))
            {
                std::memcpy(p_base + offset_x, &kNaN, sizeof(float));
                std::memcpy(p_base + offset_y, &kNaN, sizeof(float));
                std::memcpy(p_base + offset_z, &kNaN, sizeof(float));
                nan_written++;
                continue;
            }

            double ts;
            std::memcpy(&ts, p_base + offset_ts, sizeof(double));
            if (!std::isfinite(ts))
                continue;

            double s = (ts - ts_min) / ts_span;
            s = clampValue(s, 0.0, 1.0);

            // 插值（LOAM 匀速近似）
            double roll_s = s * roll_end;
            double pitch_s = s * pitch_end;
            double yaw_s = s * yaw_end;

            double tx_s = s * tx_end;
            double ty_s = s * ty_end;
            double tz_s = s * tz_end;

            double R_s[9];
            rpyToMatrixZYX(roll_s, pitch_s, yaw_s, R_s);

            // p_start = R(s)^T * (p_raw - t(s))
            double x1 = (double)x - tx_s;
            double y1 = (double)y - ty_s;
            double z1 = (double)z - tz_s;

            double x_new = R_s[0] * x1 + R_s[3] * y1 + R_s[6] * z1;
            double y_new = R_s[1] * x1 + R_s[4] * y1 + R_s[7] * z1;
            double z_new = R_s[2] * x1 + R_s[5] * y1 + R_s[8] * z1;

            float xf = (float)x_new;
            float yf = (float)y_new;
            float zf = (float)z_new;
            std::memcpy(p_base + offset_x, &xf, sizeof(float));
            std::memcpy(p_base + offset_y, &yf, sizeof(float));
            std::memcpy(p_base + offset_z, &zf, sizeof(float));
        }
        ROS_DEBUG_THROTTLE(1.0,
                           "deskew(timestamp): points=%zu, nan_written=%zu, skipped_placeholder_for_span=%zu, skipped_nonfinite_for_span=%zu, ts_span=%.6f",
                           num_points, nan_written, skipped_placeholder_for_span, skipped_nonfinite_for_span, ts_span);
        return true;
    }

    bool deskewPointCloudInPlaceLOAM(sensor_msgs::PointCloud2 &cloud,
                                     const cv::Mat &R_end,
                                     const cv::Mat &t_end) const
    {
        const size_t num_points = (size_t)cloud.width * cloud.height;
        if (num_points == 0 || cloud.data.empty())
            return false;

        // 找 x/y/z offset
        int offset_x = -1, offset_y = -1, offset_z = -1;
        for (const auto &f : cloud.fields)
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
            ROS_ERROR("deskewPointCloudInPlaceLOAM: x/y/z fields not found.");
            return false;
        }

        if (R_end.empty() || t_end.empty() ||
            R_end.rows != 3 || R_end.cols != 3 ||
            t_end.rows != 3 || t_end.cols != 1)
        {
            ROS_ERROR("deskewPointCloudInPlaceLOAM: invalid R_end/t_end shape.");
            return false;
        }

        const size_t point_step = cloud.point_step;
        uint8_t *base_ptr = cloud.data.data();

        // 1) 计算 startOri / endOri（用第一点&最后一点的水平角，LOAM 同款）
        bool found_start = false, found_end = false;
        double startOri = 0.0, endOri = 0.0;

        for (size_t i = 0; i < num_points; ++i)
        {
            uint8_t *p_base = base_ptr + i * point_step;
            float x = *reinterpret_cast<float *>(p_base + offset_x);
            float y = *reinterpret_cast<float *>(p_base + offset_y);
            if (!std::isfinite(x) || !std::isfinite(y))
                continue;
            if (std::fabs(x) + std::fabs(y) < 1e-6)
                continue;
            startOri = -std::atan2((double)y, (double)x);
            found_start = true;
            break;
        }

        for (size_t k = 0; k < num_points; ++k)
        {
            size_t i = num_points - 1 - k;
            uint8_t *p_base = base_ptr + i * point_step;
            float x = *reinterpret_cast<float *>(p_base + offset_x);
            float y = *reinterpret_cast<float *>(p_base + offset_y);
            if (!std::isfinite(x) || !std::isfinite(y))
                continue;
            if (std::fabs(x) + std::fabs(y) < 1e-6)
                continue;
            endOri = -std::atan2((double)y, (double)x);
            found_end = true;
            break;
        }

        if (!found_start || !found_end)
        {
            ROS_WARN_THROTTLE(1.0, "deskewPointCloudInPlaceLOAM: cannot find valid start/end ori.");
            return false;
        }

        endOri += 2.0 * M_PI;
        if (endOri - startOri > 3.0 * M_PI)
            endOri -= 2.0 * M_PI;
        else if (endOri - startOri < M_PI)
            endOri += 2.0 * M_PI;

        double oriSpan = endOri - startOri;
        if (!(oriSpan > 1e-3))
        {
            ROS_WARN_THROTTLE(1.0, "deskewPointCloudInPlaceLOAM: oriSpan too small (%.6f).", oriSpan);
            return false;
        }

        // 2) 提取 end 时刻的 rpy + t
        double roll_end = 0.0, pitch_end = 0.0, yaw_end = 0.0;
        rotMatToRPYZYX(R_end, roll_end, pitch_end, yaw_end);

        const double tx_end = t_end.at<double>(0);
        const double ty_end = t_end.at<double>(1);
        const double tz_end = t_end.at<double>(2);

        // 3) 遍历每个点，按 s 插值并补偿
        bool halfPassed = false;

        for (size_t i = 0; i < num_points; ++i)
        {
            uint8_t *p_base = base_ptr + i * point_step;

            float *px = reinterpret_cast<float *>(p_base + offset_x);
            float *py = reinterpret_cast<float *>(p_base + offset_y);
            float *pz = reinterpret_cast<float *>(p_base + offset_z);

            float x = *px;
            float y = *py;
            float z = *pz;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            if (std::fabs(x) + std::fabs(y) < 1e-6)
                continue;

            double ori = -std::atan2((double)y, (double)x);

            // LOAM 的“角度连续化”逻辑：确保 ori 随点序单调
            if (!halfPassed)
            {
                if (ori < startOri - M_PI / 2)
                    ori += 2.0 * M_PI;
                else if (ori > startOri + 3.0 * M_PI / 2)
                    ori -= 2.0 * M_PI;

                if (ori - startOri > M_PI)
                    halfPassed = true;
            }
            else
            {
                ori += 2.0 * M_PI;
                if (ori < endOri - 3.0 * M_PI / 2)
                    ori += 2.0 * M_PI;
                else if (ori > endOri + M_PI / 2)
                    ori -= 2.0 * M_PI;
            }

            double s = (ori - startOri) / oriSpan;
            s = clampValue(s, 0.0, 1.0);

            // 插值
            double roll_s = s * roll_end;
            double pitch_s = s * pitch_end;
            double yaw_s = s * yaw_end;

            double tx_s = s * tx_end;
            double ty_s = s * ty_end;
            double tz_s = s * tz_end;

            double R_s[9];
            rpyToMatrixZYX(roll_s, pitch_s, yaw_s, R_s);

            // p_start = R(s)^T * (p_raw - t(s))
            double x1 = (double)x - tx_s;
            double y1 = (double)y - ty_s;
            double z1 = (double)z - tz_s;

            double x_new = R_s[0] * x1 + R_s[3] * y1 + R_s[6] * z1;
            double y_new = R_s[1] * x1 + R_s[4] * y1 + R_s[7] * z1;
            double z_new = R_s[2] * x1 + R_s[5] * y1 + R_s[8] * z1;

            *px = (float)x_new;
            *py = (float)y_new;
            *pz = (float)z_new;
        }

        return true;
    }

    // ---------- 回调 ----------
    void callback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {

        if (!msg || msg->data.empty())
            return;
        prev_cloud_msg_ = *msg;
        auto t_total_start = std::chrono::high_resolution_clock::now();

        // 0) 线束采样（只用于发布，不用于投影）
        double t_sample_ms = 0.0;
        sensor_msgs::PointCloud2 sampled_cloud;

        // ---------- 1) 投影：始终使用原始点云 msg ----------
        double t_off_ms = 0.0, t_proj_inner_ms = 0.0, t_merge_ms = 0.0, t_norm_ms = 0.0;

        auto t_proj_start = std::chrono::high_resolution_clock::now();

        cv::Mat intensity_f;
        std::vector<float> pixel_x, pixel_y, pixel_z;
        std::vector<char> has_point;

        if (projection_mode_ == "ring")
        {
            buildIntensityImageRing(*msg,
                                    intensity_f,
                                    pixel_x, pixel_y, pixel_z, has_point,
                                    t_off_ms, t_proj_inner_ms, t_merge_ms);
        }
        else
        {
            buildIntensityImageAngle(*msg,
                                     intensity_f,
                                     pixel_x, pixel_y, pixel_z, has_point,
                                     t_off_ms, t_proj_inner_ms, t_merge_ms);
        }

        cv::Mat intensity_raw_8u;
        auto t_norm_start = std::chrono::high_resolution_clock::now();
        if (!intensity_f.empty())
        {
            cv::normalize(intensity_f, intensity_raw_8u, 0, 255, cv::NORM_MINMAX, CV_8U);
            std::string raw_path = output_dir_ + "/intensity_raw_" + std::to_string(frame_idx_) + ".png";
            // cv::imwrite(raw_path, intensity_raw_8u);
        }
        auto t_norm_end = std::chrono::high_resolution_clock::now();
        t_norm_ms = std::chrono::duration<double, std::milli>(t_norm_end - t_norm_start).count();

        auto t_proj_end = std::chrono::high_resolution_clock::now();
        double t_proj_ms = std::chrono::duration<double, std::milli>(t_proj_end - t_proj_start).count();

        ROS_INFO("[Frame %d] Projection(%s) breakdown (ms): sample=%.3f offset=%.3f project=%.3f merge=%.3f normalize=%.3f total=%.3f",
                 frame_idx_, projection_mode_.c_str(),
                 t_sample_ms, t_off_ms, t_proj_inner_ms, t_merge_ms, t_norm_ms, t_proj_ms);

        // ---------- 2) 图像增强 ----------
        auto t_enh_start = std::chrono::high_resolution_clock::now();

        cv::Mat intensity_enh = intensity_raw_8u.clone();
        if (contrast_mode_ == "clahe")
        {
            cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
            clahe->apply(intensity_raw_8u, intensity_enh);
        }
        else if (contrast_mode_ == "equalize")
        {
            cv::equalizeHist(intensity_raw_8u, intensity_enh);
        }
        if (enable_blur_)
        {
            cv::GaussianBlur(intensity_enh, intensity_enh, cv::Size(3, 3), 0);
        }

        std::string enh_path = output_dir_ + "/intensity_enh_" + std::to_string(frame_idx_) + ".png";
        // cv::imwrite(enh_path, intensity_enh);

        auto t_enh_end = std::chrono::high_resolution_clock::now();
        double t_enh_ms = std::chrono::duration<double, std::milli>(t_enh_end - t_enh_start).count();

        // ---------- 3) ORB ----------
        auto t_orb_start = std::chrono::high_resolution_clock::now();

        cv::Ptr<cv::ORB> orb = cv::ORB::create(orb_nfeatures_, (float)orb_scaleFactor_, orb_nlevels_,
                                               orb_edgeThreshold_, 0, orb_wta_k_, orb_score_type_,
                                               orb_patchSize_, orb_fastThreshold_);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(intensity_enh, cv::noArray(), keypoints, descriptors);

        cv::Mat orb_vis;
        cv::cvtColor(intensity_enh, orb_vis, cv::COLOR_GRAY2BGR);
        for (const auto &kp : keypoints)
        {
            cv::circle(orb_vis, kp.pt, orb_vis_radius_, cv::Scalar(0, 255, 0), -1);
        }
        // cv::imwrite(output_dir_ + "/orb_vis_" + std::to_string(frame_idx_) + ".png", orb_vis);

        auto t_orb_end = std::chrono::high_resolution_clock::now();
        double t_orb_ms = std::chrono::duration<double, std::milli>(t_orb_end - t_orb_start).count();

        // ---------- 4) 匹配 + RANSAC ----------
        double t_match_ms = 0.0, t_ransac2d_ms = 0.0, t_ransac3d_ms = 0.0;
        size_t count_2d_inliers = 0, count_3d_inliers = 0;

        if (has_prev_frame_ && !descriptors.empty() && !prev_desc_.empty())
        {
            auto t_match_start = std::chrono::high_resolution_clock::now();

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

            auto t_match_end1 = std::chrono::high_resolution_clock::now();
            t_match_ms = std::chrono::duration<double, std::milli>(t_match_end1 - t_match_start).count();

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
                auto t_r2_start = std::chrono::high_resolution_clock::now();

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

                auto t_r2_end = std::chrono::high_resolution_clock::now();
                t_ransac2d_ms = std::chrono::duration<double, std::milli>(t_r2_end - t_r2_start).count();
            }
            else
            {
                matches_2d_inliers = dist_pass;
                t_ransac2d_ms = 0.0;
            }

            count_2d_inliers = matches_2d_inliers.size();

            std::string vis2d_path = output_dir_ + "/match_2d_" + std::to_string(frame_idx_) + ".png";
            // visualizeMatchesStacked(prev_img_, intensity_enh, prev_kp_, keypoints, matches_2d_inliers, vis2d_path);
            ROS_INFO("Saved 2D visualization: %s (inliers=%zu)", vis2d_path.c_str(), matches_2d_inliers.size());

            std::vector<cv::Point3f> P_all, Q_all;
            std::vector<cv::DMatch> matches_for_3d;
            P_all.reserve(matches_2d_inliers.size());
            Q_all.reserve(matches_2d_inliers.size());

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
                        matches_for_3d.push_back(m);
                    }
                }
            }

            if (P_all.size() < 3)
            {
                ROS_WARN("Frame %d: not enough 3D correspondences after 2D filtering: %zu",
                         frame_idx_, P_all.size());
                std::ofstream ofs(output_dir_ + "/matches_3d_" + std::to_string(frame_idx_) + ".txt", std::ios::trunc);
                ofs.close();
            }
            else
            {
                auto t_r3_start = std::chrono::high_resolution_clock::now();
                std::vector<int> inlier_idx;
                if (enable_parallel_ransac3d_ && ransac3d_threads_ > 1)
                    inlier_idx = ransac3D_parallel(P_all, Q_all, ransac_3d_thresh_, ransac_3d_iters_, ransac3d_threads_);
                else
                    inlier_idx = ransac3D(P_all, Q_all, ransac_3d_thresh_, ransac_3d_iters_);
                auto t_r3_end = std::chrono::high_resolution_clock::now();
                t_ransac3d_ms = std::chrono::duration<double, std::milli>(t_r3_end - t_r3_start).count();

                count_3d_inliers = inlier_idx.size();

                std::vector<cv::DMatch> matches_3d_inliers;
                std::vector<cv::Point3f> P_in, Q_in;
                std::ofstream ofs3d(output_dir_ + "/matches_3d_" + std::to_string(frame_idx_) + ".txt", std::ios::trunc);
                for (int idx_in : inlier_idx)
                {
                    if (idx_in >= 0 && idx_in < (int)P_all.size())
                    {
                        P_in.push_back(P_all[(size_t)idx_in]);
                        Q_in.push_back(Q_all[(size_t)idx_in]);
                        matches_3d_inliers.push_back(matches_for_3d[(size_t)idx_in]);
                        ofs3d << P_all[(size_t)idx_in].x << " " << P_all[(size_t)idx_in].y << " " << P_all[(size_t)idx_in].z << " "
                              << Q_all[(size_t)idx_in].x << " " << Q_all[(size_t)idx_in].y << " " << Q_all[(size_t)idx_in].z << "\n";
                    }
                }
                ofs3d.close();
                ROS_INFO("Saved %zu 3D matches => %s",
                         P_in.size(), (output_dir_ + "/matches_3d_" + std::to_string(frame_idx_) + ".txt").c_str());

                std::string vis3d_path = output_dir_ + "/match_3d_" + std::to_string(frame_idx_) + ".png";
                visualizeMatchesStacked(prev_img_, intensity_enh, prev_kp_, keypoints, matches_3d_inliers, vis3d_path);
                ROS_INFO("Saved 3D visualization: %s (inliers=%zu)",
                         vis3d_path.c_str(), matches_3d_inliers.size());

                if (P_in.size() >= 3)
                {
                    cv::Mat Rfit, tfit;
                    if (estimateRigidSVD(P_in, Q_in, Rfit, tfit))
                    {

                        // ---- 方案A：一帧延迟 deskew ----
                        // 用本次估计到的 T(prev->cur) = (Rfit, tfit) 去矫正“上一帧原始点云” prev_cloud_msg_，
                        // 把上一帧扫描内所有点统一变换到上一帧扫描起始坐标系（LOAM 匀速模型）。
                        if (enable_deskew_ && has_prev_cloud_msg_)
                        {
                            // 质量门限：3D 内点数 + 运动幅度
                            if ((int)count_3d_inliers >= deskew_min_inliers_)
                            {
                                double tx_tmp = tfit.at<double>(0);
                                double ty_tmp = tfit.at<double>(1);
                                double tz_tmp = tfit.at<double>(2);
                                double trans_norm = std::sqrt(tx_tmp * tx_tmp + ty_tmp * ty_tmp + tz_tmp * tz_tmp);

                                double tr = Rfit.at<double>(0, 0) + Rfit.at<double>(1, 1) + Rfit.at<double>(2, 2);
                                double c = clampValue((tr - 1.0) / 2.0, -1.0, 1.0);
                                double rot_deg = std::acos(c) * 180.0 / M_PI;

                                if (trans_norm <= deskew_max_trans_ && rot_deg <= deskew_max_rot_deg_)
                                {
                                    sensor_msgs::PointCloud2 deskew_cloud = prev_cloud_msg_;
                                    bool ok_deskew = deskewPointCloudInPlaceTimestamp(deskew_cloud, Rfit, tfit);
                                    if (!ok_deskew)
                                    {
                                        // fallback：没有 timestamp 时使用 LOAM 的角度推时间（要求点序近似时间序）
                                        ok_deskew = deskewPointCloudInPlaceLOAM(deskew_cloud, Rfit, tfit);
                                    }
                                    if (ok_deskew)
                                    {
                                        // deskew_cloud 的 stamp / frame_id 直接沿用上一帧
                                        deskew_pub_.publish(deskew_cloud);
                                        deskewed_cloud_msg_ = deskew_cloud;
                                        cloudraw_pub_.publish(prev_cloud_msg_);
                                        ROS_INFO("Deskew published for frame %d using T %d->%d (inliers=%zu, |t|=%.3f, rot=%.2f deg)",
                                                 frame_idx_ - 1, frame_idx_ - 1, frame_idx_,
                                                 (size_t)count_3d_inliers, trans_norm, rot_deg);
                                    }
                                    else
                                    {
                                        ROS_WARN("Deskew failed (timestamp & LOAM fallback both failed).");
                                    }
                                }
                                else
                                {
                                    ROS_WARN("Skip deskew: motion too large |t|=%.3f(max=%.3f) rot=%.2f(max=%.2f)",
                                             trans_norm, deskew_max_trans_, rot_deg, deskew_max_rot_deg_);
                                }
                            }
                            else
                            {
                                deskewed_cloud_msg_ = prev_cloud_msg_;
                                ROS_WARN("Skip deskew: 3D inliers too few (%zu < %d)",
                                         (size_t)count_3d_inliers, deskew_min_inliers_);
                            }
                        }

                        cv::Mat R_inv = Rfit.t();
                        cv::Mat t_inv = -R_inv * tfit;

                        double roll = atan2(R_inv.at<double>(2, 1), R_inv.at<double>(2, 2));
                        double pitch = atan2(-R_inv.at<double>(2, 0),
                                             std::sqrt(R_inv.at<double>(2, 1) * R_inv.at<double>(2, 1) + R_inv.at<double>(2, 2) * R_inv.at<double>(2, 2)));
                        double yaw = atan2(R_inv.at<double>(1, 0), R_inv.at<double>(0, 0));

                        double tx = t_inv.at<double>(0);
                        double ty = t_inv.at<double>(1);
                        double tz = t_inv.at<double>(2);

                        std::ofstream fout(output_dir_ + "/all_Tguess.txt", std::ios::app);
                        fout << frame_idx_ - 1 << " "
                             << tx << " " << ty << " " << tz << " "
                             << (roll * 180.0 / M_PI) << " "
                             << (pitch * 180.0 / M_PI) << " "
                             << (yaw * 180.0 / M_PI) << "\n";
                        fout.close();

                        ROS_INFO("Tguess frame %d->%d: tx=%.3f ty=%.3f tz=%.3f roll=%.2f pitch=%.2f yaw=%.2f deg",
                                 frame_idx_ - 1, frame_idx_, tx, ty, tz,
                                 roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);
                        if (tguess_pub_.getNumSubscribers() > 0)
                        {
                            nav_msgs::Odometry odom;
                            odom.header.stamp = msg->header.stamp; // ★ 和当前帧点云同一个 stamp
                            odom.header.frame_id = "tguess_odom";  // 父坐标系名字，自定义
                            odom.child_frame_id = "tguess_lidar";  // 子坐标系名字，自定义

                            // 位置
                            odom.pose.pose.position.x = tx;
                            odom.pose.pose.position.y = ty;
                            odom.pose.pose.position.z = tz;

                            // 姿态（roll/pitch/yaw 是弧度）
                            tf::Quaternion q;
                            q.setRPY(roll, pitch, yaw);
                            odom.pose.pose.orientation.x = q.x();
                            odom.pose.pose.orientation.y = q.y();
                            odom.pose.pose.orientation.z = q.z();
                            odom.pose.pose.orientation.w = q.w();

                            // 协方差可以先给个比较大的值，表示只是“先验”
                            for (int i = 0; i < 36; ++i)
                            {

                                odom.pose.covariance[i] = 0.0;
                            }
                            odom.pose.covariance[0] = P_in.size(); // x
                            odom.pose.covariance[7] = 0.25;        // y
                            odom.pose.covariance[14] = 0.25;       // z
                            odom.pose.covariance[21] = 0.05;       // roll
                            odom.pose.covariance[28] = 0.05;       // pitch
                            odom.pose.covariance[35] = 0.05;       // yaw
                            std::cout << "frame-Tguess_time:" << frame_idx_ << "-" << odom.header.stamp << std::endl;
                            tguess_pub_.publish(odom);
                        }
                    }
                    else
                    {
                        ROS_WARN("estimateRigidSVD failed on final 3D inliers");
                    }
                }
            }

            auto t_match_end = std::chrono::high_resolution_clock::now();
            t_match_ms = std::chrono::duration<double, std::milli>(t_match_end - t_match_start).count();
        }
        if (enable_line_sampling_)
        {
            sensor_msgs::PointCloud2 cloud_in;
            if (enable_deskew_)
            {
                cloud_in = deskewed_cloud_msg_;
            }
            else
            {
                cloud_in = *msg;
            }
            if (samplePointCloudByRing(cloud_in, sampled_cloud, t_sample_ms))
            {
                if (sampled_pub_.getNumSubscribers() > 0)
                {
                    sampled_cloud.header = msg->header;
                    sampled_pub_.publish(sampled_cloud);
                }
                std::cout << "frame-cloud_time:" << frame_idx_ << "-" << sampled_cloud.header.stamp << std::endl;
            }
        }
        // ---------- 5) time_log ----------
        auto t_total_end = std::chrono::high_resolution_clock::now();
        double t_total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

        {
            std::ofstream tlog(output_dir_ + "/time_log.csv", std::ios::app);
            tlog << frame_idx_ << "," << std::fixed << std::setprecision(3)
                 << t_sample_ms << "," << t_proj_ms << "," << t_enh_ms << "," << t_orb_ms << ","
                 << t_match_ms << "," << t_ransac2d_ms << "," << t_ransac3d_ms << "," << t_total_ms << "\n";
            tlog.close();
        }

        ROS_INFO("[Frame %d] times(ms): sample=%.1f proj=%.1f enh=%.1f orb=%.1f match=%.1f r2d=%.1f r3d=%.1f total=%.1f",
                 frame_idx_, t_sample_ms, t_proj_ms, t_enh_ms, t_orb_ms,
                 t_match_ms, t_ransac2d_ms, t_ransac3d_ms, t_total_ms);
        ROS_INFO("[Frame %d] matches: 2D_inliers=%zu 3D_inliers=%zu",
                 frame_idx_, (size_t)count_2d_inliers, (size_t)count_3d_inliers);

        // ---------- 6) 更新上一帧缓存 ----------
        prev_img_ = intensity_enh.clone();
        prev_desc_ = descriptors.clone();
        prev_kp_ = keypoints;
        prev_px_ = pixel_x;
        prev_py_ = pixel_y;
        prev_pz_ = pixel_z;
        prev_has_ = has_point;
        has_prev_frame_ = true;

        // 缓存当前帧原始点云（用于下一次：deskew 当前帧）

        has_prev_cloud_msg_ = true;

        frame_idx_++;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_intensity_orb_match_dual_sampling_pubonly");
    ros::NodeHandle nh("~");
    LidarIntensityORBMatchDual node(nh);
    ros::spin();
    return 0;
}
