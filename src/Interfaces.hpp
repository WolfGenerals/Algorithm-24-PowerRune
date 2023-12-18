#ifndef INTERFACES_HPP
#define INTERFACES_HPP
#include <opencv2/core/types.hpp>
#include <valarray>

struct Direction {
    double pitch;
    double yaw;

    [[nodiscard]] auto operator+(const Direction &other) const -> Direction { return {pitch + other.pitch, yaw + other.yaw}; }
    [[nodiscard]] auto operator-(const Direction &other) const -> Direction { return {pitch - other.pitch, yaw - other.yaw}; }

    [[nodiscard]] double angle() const {
        return std::acos(std::cos(pitch) * std::cos(yaw));
    }
};
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
template<typename T>
struct Rune {
    T target;
    T centre;
};
#endif// INTERFACES_HPP
