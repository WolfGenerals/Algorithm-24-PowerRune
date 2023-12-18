#ifndef PLANE_HPP
#define PLANE_HPP
#include <opencv2/core/types.hpp>
#include "../Interfaces.hpp"


struct Plane {
    double distanse;
    double yaw;

    [[nodiscard]] cv::Point2d intersect(Direction dir) const {
        return {
            (distanse * std::cos(yaw + dir.yaw)),
            (distanse * std::sin(yaw + dir.yaw))
        };
    }
    [[nodiscard]] Direction direction(const cv::Point2d &point) const {
        return {
            std::atan2(point.y, point.x) - yaw,
            std::atan2(point.y, point.x) - yaw
        };
    }
};



#endif //PLANE_HPP
