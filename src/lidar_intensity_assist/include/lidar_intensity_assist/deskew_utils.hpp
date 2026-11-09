#pragma once

#include <ceres/rotation.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

namespace lidar_intensity_assist
{
namespace deskew
{

// T maps scan-end coordinates into scan-start coordinates. A constant body
// velocity gives T(k) = Exp(k Log(T)), including the translation/rotation coupling.
template <typename T>
void translationToTwist(const T *omega, const T *translation, T *velocity)
{
    using std::cos;
    using std::sin;
    using std::sqrt;
    const T theta2 = omega[0] * omega[0] + omega[1] * omega[1] + omega[2] * omega[2];
    T coefficient;
    if (theta2 > T(1e-8))
    {
        const T theta = sqrt(theta2);
        coefficient = (T(1) - T(0.5) * theta * cos(T(0.5) * theta) / sin(T(0.5) * theta)) / theta2;
    }
    else
    {
        coefficient = T(1.0 / 12.0) + theta2 / T(720.0) + theta2 * theta2 / T(30240.0);
    }
    T cross[3], cross2[3];
    ceres::CrossProduct(omega, translation, cross);
    ceres::CrossProduct(omega, cross, cross2);
    for (int i = 0; i < 3; ++i)
        velocity[i] = translation[i] - T(0.5) * cross[i] + coefficient * cross2[i];
}

template <typename T>
void transformWithTwist(const T *omega, const T *velocity, double fraction,
                        const T *point, T *output)
{
    using std::cos;
    using std::sin;
    using std::sqrt;
    T scaled_omega[3], scaled_velocity[3];
    for (int i = 0; i < 3; ++i)
    {
        scaled_omega[i] = T(fraction) * omega[i];
        scaled_velocity[i] = T(fraction) * velocity[i];
    }
    const T theta2 = scaled_omega[0] * scaled_omega[0] +
                     scaled_omega[1] * scaled_omega[1] +
                     scaled_omega[2] * scaled_omega[2];
    T a, b;
    if (theta2 > T(1e-8))
    {
        const T theta = sqrt(theta2);
        a = (T(1) - cos(theta)) / theta2;
        b = (theta - sin(theta)) / (theta2 * theta);
    }
    else
    {
        a = T(0.5) - theta2 / T(24.0) + theta2 * theta2 / T(720.0);
        b = T(1.0 / 6.0) - theta2 / T(120.0) + theta2 * theta2 / T(5040.0);
    }
    T cross[3], cross2[3];
    ceres::CrossProduct(scaled_omega, scaled_velocity, cross);
    ceres::CrossProduct(scaled_omega, cross, cross2);
    ceres::AngleAxisRotatePoint(scaled_omega, point, output);
    for (int i = 0; i < 3; ++i)
        output[i] += scaled_velocity[i] + a * cross[i] + b * cross2[i];
}

template <typename T>
void transformAtFraction(const T *omega, const T *translation, double fraction,
                         const T *point, T *output)
{
    T velocity[3];
    translationToTwist(omega, translation, velocity);
    transformWithTwist(omega, velocity, fraction, point, output);
}

// Bounds reject unreliable timing; changing the measured ratio invents motion.
inline bool computeTimeScale(double scan_span, double frame_interval,
                             double min_ratio, double max_ratio, double &alpha)
{
    alpha = std::numeric_limits<double>::quiet_NaN();
    if (!std::isfinite(scan_span) || !std::isfinite(frame_interval) ||
        scan_span <= 0.0 || frame_interval <= 1e-12)
        return false;
    alpha = scan_span / frame_interval;
    return std::isfinite(alpha) && alpha > 0.0 && alpha >= min_ratio && alpha <= max_ratio;
}

struct CloudFields
{
    int x = -1, y = -1, z = -1, timestamp = -1;
};

inline bool findCloudFields(const sensor_msgs::msg::PointCloud2 &cloud, CloudFields &fields)
{
    fields = CloudFields();
    if (cloud.is_bigendian || cloud.width == 0 || cloud.height == 0 || cloud.point_step == 0)
        return false;
    for (const auto &field : cloud.fields)
    {
        int *offset = nullptr;
        uint8_t expected = sensor_msgs::msg::PointField::FLOAT32;
        if (field.name == "x") offset = &fields.x;
        else if (field.name == "y") offset = &fields.y;
        else if (field.name == "z") offset = &fields.z;
        else if (field.name == "timestamp")
        {
            offset = &fields.timestamp;
            expected = sensor_msgs::msg::PointField::FLOAT64;
        }
        if (offset)
        {
            const size_t bytes = expected == sensor_msgs::msg::PointField::FLOAT64 ? 8 : 4;
            if (field.datatype != expected || field.count != 1 ||
                static_cast<size_t>(field.offset) + bytes > cloud.point_step)
                return false;
            *offset = static_cast<int>(field.offset);
        }
    }
    const uint64_t row_bytes = uint64_t(cloud.width) * cloud.point_step;
    const uint64_t required = uint64_t(cloud.height - 1) * cloud.row_step + row_bytes;
    return fields.x >= 0 && fields.y >= 0 && fields.z >= 0 && fields.timestamp >= 0 &&
           cloud.row_step >= row_bytes && required <= cloud.data.size();
}

inline size_t pointOffset(const sensor_msgs::msg::PointCloud2 &cloud, size_t index)
{
    return (index / cloud.width) * cloud.row_step + (index % cloud.width) * cloud.point_step;
}

inline bool readValidPoint(const uint8_t *data, const CloudFields &fields,
                           double min_range_squared, double placeholder_eps,
                           double *point, double &timestamp)
{
    const int offsets[3] = {fields.x, fields.y, fields.z};
    for (int i = 0; i < 3; ++i)
    {
        float value;
        std::memcpy(&value, data + offsets[i], sizeof(value));
        point[i] = value;
        if (!std::isfinite(point[i])) return false;
    }
    std::memcpy(&timestamp, data + fields.timestamp, sizeof(timestamp));
    return std::isfinite(timestamp) &&
           std::fabs(point[0]) + std::fabs(point[1]) + std::fabs(point[2]) >= placeholder_eps &&
           point[0] * point[0] + point[1] * point[1] + point[2] * point[2] >= min_range_squared;
}

struct TimestampStats
{
    double min = std::numeric_limits<double>::infinity();
    double max = -std::numeric_limits<double>::infinity();
    size_t valid_points = 0;
};

inline bool timestampStats(const sensor_msgs::msg::PointCloud2 &cloud, double min_range,
                           double placeholder_eps, TimestampStats &stats)
{
    stats = TimestampStats();
    CloudFields fields;
    if (!findCloudFields(cloud, fields)) return false;
    const size_t count = size_t(cloud.width) * cloud.height;
    for (size_t i = 0; i < count; ++i)
    {
        double point[3], timestamp;
        if (!readValidPoint(cloud.data.data() + pointOffset(cloud, i), fields,
                            min_range * min_range, placeholder_eps, point, timestamp))
            continue;
        stats.min = std::min(stats.min, timestamp);
        stats.max = std::max(stats.max, timestamp);
        ++stats.valid_points;
    }
    return stats.valid_points >= 50 && stats.max > stats.min && std::isfinite(stats.max - stats.min);
}

// omega/translation describe the total scan-end pose in the scan-start frame.
// ring, timestamps, point order, padding and headers are preserved.
inline bool cloudToStart(sensor_msgs::msg::PointCloud2 &cloud, const double *omega,
                         const double *translation, double min_range,
                         double placeholder_eps, size_t &invalid_points)
{
    invalid_points = 0;
    for (int i = 0; i < 3; ++i)
        if (!std::isfinite(omega[i]) || !std::isfinite(translation[i])) return false;
    TimestampStats stats;
    CloudFields fields;
    if (!timestampStats(cloud, min_range, placeholder_eps, stats) || !findCloudFields(cloud, fields))
        return false;
    double velocity[3];
    translationToTwist(omega, translation, velocity);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const int offsets[3] = {fields.x, fields.y, fields.z};
    const size_t count = size_t(cloud.width) * cloud.height;
    for (size_t i = 0; i < count; ++i)
    {
        uint8_t *data = cloud.data.data() + pointOffset(cloud, i);
        double point[3], timestamp, corrected[3];
        if (!readValidPoint(data, fields, min_range * min_range, placeholder_eps, point, timestamp))
        {
            for (int j = 0; j < 3; ++j) std::memcpy(data + offsets[j], &nan, sizeof(nan));
            ++invalid_points;
            continue;
        }
        const double fraction = (timestamp - stats.min) / (stats.max - stats.min);
        transformWithTwist(omega, velocity, fraction, point, corrected);
        for (int j = 0; j < 3; ++j)
        {
            const float value = static_cast<float>(corrected[j]);
            std::memcpy(data + offsets[j], &value, sizeof(value));
        }
    }
    if (invalid_points) cloud.is_dense = false;
    return true;
}

}  // namespace deskew
}  // namespace lidar_intensity_assist
