#ifndef ALIAS_HPP
#define ALIAS_HPP

#include "opencv2/opencv.hpp"

using Descriptors = cv::Mat;
using Transform2D = cv::Matx<float, 3, 3>;
using Transform3D = cv::Matx<float, 4, 4>;
using Image       = cv::Mat;
using Vec3        = cv::Matx<float, 3, 1>;
using Vec2        = cv::Matx<float, 2, 1>;

inline double length(const Vec3 &vec3) {
    return sqrt(vec3(0, 0) * vec3(0, 0) + vec3(1, 0) * vec3(1, 0) + vec3(2, 0) * vec3(2, 0));
}
inline double length(const Vec2 &vec2) {
    return sqrt(vec2(0, 0) * vec2(0, 0) + vec2(1, 0) * vec2(1, 0));
}



#endif// ALIAS_HPP
