// projection_node.cpp
//
// 测试节点：订阅 128x2500 点云，投影为强度图（多线程），保存 PNG。
// 重点是投影部分：利用点云结构：128 线 × 每线 2500 点，按行=线束直接映射。
// 依赖：roscpp, sensor_msgs, OpenCV, (可选)OpenMP。

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>

#include <vector>
#include <string>
#include <cmath>
#include <chrono>
#include <limits>
#include <cstdio> // std::snprintf

#ifdef _OPENMP
#include <omp.h>
#endif

// 用角度投影，将点云映射为强度图（v_res_ x h_res_）
// - 剔除 intensity <= 0 的点
// - 一个像素内取强度最大的点
// - 多线程 + 线程本地 grid
// - 输出各阶段耗时（ms）
struct PixAccAngle
{
    float max_intensity = 0.0f;
    float x = 0.0f, y = 0.0f, z = 0.0f;
    bool has_point = false;
};

void buildIntensityImageAngleFast(
    const sensor_msgs::PointCloud2ConstPtr &msg,
    cv::Mat &intensity_f,        // 输出：float 强度图
    cv::Mat &intensity_8u,       // 输出：8U 强度图（已归一化，可直接保存）
    std::vector<float> &pixel_x, // 输出：像素对应的 3D 点
    std::vector<float> &pixel_y,
    std::vector<float> &pixel_z,
    std::vector<char> &has_point, // 输出：像素是否有点
    double &t_offset_ms,
    double &t_project_ms,
    double &t_merge_ms,
    double &t_norm_ms,
    double &t_total_ms)
{
    auto t_total_start = std::chrono::high_resolution_clock::now();

    if (!msg || msg->data.empty())
    {
        ROS_WARN("buildIntensityImageAngleFast: empty point cloud.");
        intensity_f.release();
        intensity_8u.release();
        pixel_x.clear();
        pixel_y.clear();
        pixel_z.clear();
        has_point.clear();
        t_offset_ms = t_project_ms = t_merge_ms = t_norm_ms = t_total_ms = 0.0;
        return;
    }

    const int v_res = v_res_;
    const int h_res = h_res_;
    const size_t total_pix = (size_t)v_res * h_res;
    const size_t num_points = (size_t)msg->width * msg->height;

    // ---------- 1) 解析字段偏移 ----------
    auto t_off_start = std::chrono::high_resolution_clock::now();

    int offset_x = -1, offset_y = -1, offset_z = -1, offset_i = -1;
    for (const auto &f : msg->fields)
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
        ROS_ERROR("buildIntensityImageAngleFast: x/y/z/intensity fields not all found.");
        t_offset_ms = t_project_ms = t_merge_ms = t_norm_ms = t_total_ms = 0.0;
        return;
    }

    const uint8_t *base_ptr = msg->data.data();
    const size_t point_step = msg->point_step;

    auto t_off_end = std::chrono::high_resolution_clock::now();
    t_offset_ms = std::chrono::duration<double, std::milli>(t_off_end - t_off_start).count();

    // ---------- 2) 多线程投影：构建线程本地 grids ----------
    auto t_proj_start = std::chrono::high_resolution_clock::now();

    int omp_threads = 1;
#ifdef _OPENMP
    omp_threads = omp_get_max_threads();
#endif

    std::vector<std::vector<PixAccAngle>> thread_grids(
        (size_t)omp_threads, std::vector<PixAccAngle>(total_pix));

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
            const uint8_t *p_base = base_ptr + (size_t)i * point_step;

            const float *px = reinterpret_cast<const float *>(p_base + offset_x);
            const float *py = reinterpret_cast<const float *>(p_base + offset_y);
            const float *pz = reinterpret_cast<const float *>(p_base + offset_z);
            const float *pi = reinterpret_cast<const float *>(p_base + offset_i);

            float x = *px;
            float y = *py;
            float z = *pz;
            float intensity = *pi;

            // 剔除 intensity <= 0 和无效值
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                !std::isfinite(intensity) || intensity <= 0.0f)
            {
                continue;
            }

            // 计算方位角 & 俯仰角（弧度）
            double az = std::atan2((double)y, (double)x);
            double r_xy = std::sqrt((double)x * x + (double)y * y);
            double el = std::atan2((double)z, r_xy);

            // FOV 裁剪（用你类里的 min_az_/max_az_/min_el_/max_el_）
            if (az < min_az_ || az > max_az_ || el < min_el_ || el > max_el_)
                continue;

            // 映射到像素坐标
            double uf = (az - min_az_) / (max_az_ - min_az_) * (h_res - 1);
            double vf = (max_el_ - el) / (max_el_ - min_el_) * (v_res - 1); // 上大下小

            int u = (int)std::round(uf);
            int v = (int)std::round(vf);
            if (u < 0 || u >= h_res || v < 0 || v >= v_res)
                continue;

            size_t pid = (size_t)v * h_res + (size_t)u;
            PixAccAngle &cell = grid[pid];

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

    // ---------- 3) 归并线程本地 grids ----------
    auto t_merge_start = std::chrono::high_resolution_clock::now();

    std::vector<PixAccAngle> global_grid(total_pix);

    for (int t = 0; t < omp_threads; ++t)
    {
        const auto &g = thread_grids[(size_t)t];
        for (size_t pid = 0; pid < total_pix; ++pid)
        {
            const PixAccAngle &src = g[pid];
            PixAccAngle &dst = global_grid[pid];
            if (src.has_point && (!dst.has_point || src.max_intensity > dst.max_intensity))
                dst = src;
        }
    }

    auto t_merge_end = std::chrono::high_resolution_clock::now();
    t_merge_ms = std::chrono::duration<double, std::milli>(t_merge_end - t_merge_start).count();

    // ---------- 4) 填充强度图 & 像素->3D 映射 ----------
    intensity_f = cv::Mat(v_res, h_res, CV_32F, cv::Scalar(0.0f));
    pixel_x.assign(total_pix, 0.0f);
    pixel_y.assign(total_pix, 0.0f);
    pixel_z.assign(total_pix, 0.0f);
    has_point.assign(total_pix, 0);

    for (size_t pid = 0; pid < total_pix; ++pid)
    {
        const PixAccAngle &cell = global_grid[pid];
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

    // ---------- 5) 归一化 + 保存 8U 图像 ----------
    auto t_norm_start = std::chrono::high_resolution_clock::now();

    cv::normalize(intensity_f, intensity_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

    char fname[256];
    std::snprintf(fname, sizeof(fname), "/intensity_angle_%06d.png", frame_idx_);
    std::string path = output_dir_ + std::string(fname);
    if (!cv::imwrite(path, intensity_8u))
    {
        ROS_WARN("Failed to write intensity image: %s", path.c_str());
    }

    auto t_norm_end = std::chrono::high_resolution_clock::now();
    t_norm_ms = std::chrono::duration<double, std::milli>(t_norm_end - t_norm_start).count();

    // ---------- 6) 总耗时 ----------
    auto t_total_end = std::chrono::high_resolution_clock::now();
    t_total_ms = std::chrono::duration<double, std::milli>(t_total_end - t_total_start).count();

    ROS_INFO("[Frame %d] angle projection time breakdown (ms):", frame_idx_);
    ROS_INFO("  offsets   = %.3f", t_offset_ms);
    ROS_INFO("  project   = %.3f", t_project_ms);
    ROS_INFO("  merge     = %.3f", t_merge_ms);
    ROS_INFO("  normalize = %.3f", t_norm_ms);
    ROS_INFO("  total     = %.3f", t_total_ms);
    ROS_INFO("  image     = %dx%d -> %s", v_res, h_res, path.c_str());
}

// 一个简单的测试节点：订阅点云、调用投影函数
class ProjectionTester
{
public:
    ProjectionTester(ros::NodeHandle &nh)
        : nh_(nh), frame_idx_(0)
    {
        nh_.param<std::string>("output_dir", output_dir_, std::string("/home/lb/Piont_cloudToImage/lidar_output"));
        nh_.param<std::string>("cloud_topic", cloud_topic_, std::string("/lidar_points"));

        // 创建输出目录（简单处理：让系统自己建父目录，或者你自己预先 mkdir）
        ROS_INFO("ProjectionTester: output_dir=%s, cloud_topic=%s",
                 output_dir_.c_str(), cloud_topic_.c_str());

        sub_ = nh_.subscribe(cloud_topic_, 1, &ProjectionTester::cloudCallback, this);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        cv::Mat intensity_f;
        cv::Mat intensity_raw_8u;
        std::vector<float> pixel_x, pixel_y, pixel_z;
        std::vector<char> has_point;

        double t_off_ms, t_proj_ms, t_merge_ms, t_norm_ms, t_total_ms;

        buildIntensityImageAngleFast(msg,
                                     intensity_f,
                                     intensity_raw_8u,
                                     pixel_x, pixel_y, pixel_z, has_point,
                                     t_off_ms, t_proj_ms, t_merge_ms, t_norm_ms, t_total_ms);

        // 后面照旧：用 intensity_raw_8u 做增强 + ORB

        frame_idx_++;
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::string output_dir_;
    std::string cloud_topic_;
    int frame_idx_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_projection_128x2500_test_node");
    ros::NodeHandle nh("~");

    ProjectionTester node(nh);

    ros::spin();
    return 0;
}
