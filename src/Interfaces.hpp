//
// Created by mojiw on 2023/12/6.
//

#ifndef INTERFACES_HPP
#define INTERFACES_HPP
#include <opencv2/core/types.hpp>

struct Direction {
    const double pitch;
    const double yaw;
};


struct DetectorReturn {
    const bool      null;
    const Direction target;
    const Direction centre;
};

struct BallisticsInput {
    const cv::Point2f target;
    const cv::Point2f centre;
};
#endif// INTERFACES_HPP
