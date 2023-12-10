//
// Created by mojiw on 2023/12/6.
//

#ifndef INTERFACES_HPP
#define INTERFACES_HPP
#include <opencv2/core/types.hpp>

struct Direction {
    const double pitch;
    const double yaw;

    [[nodiscard]] auto operator+(const Direction &other) const -> Direction { return {pitch + other.pitch, yaw + other.yaw}; }
    [[nodiscard]] auto operator-(const Direction &other) const -> Direction { return {pitch - other.pitch, yaw - other.yaw}; }
    [[nodiscard]] auto distance2() const -> double { return pow(pitch, 2) + pow(yaw, 2); }
    [[nodiscard]] auto distance() const -> double { return sqrt(distance2()); }
};


struct RuneDirection {
    const bool      null;
    const Direction target;
    const Direction centre;
};

struct BallisticsInput {
    const cv::Point2f target;
    const cv::Point2f centre;
};
#endif// INTERFACES_HPP
