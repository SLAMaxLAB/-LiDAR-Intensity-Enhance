#pragma once

#include <cmath>
#include <opencv2/core.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace intensity_image_enhance
{

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

}  // namespace intensity_image_enhance
