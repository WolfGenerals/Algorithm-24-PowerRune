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
                (distanse * -std::tan(dir.yaw - yaw)),
                (distanse / std::cos(dir.yaw - yaw) / std::tan(dir.pitch))};
    }
    [[nodiscard]] Direction direction(const cv::Point2d &point) const {
        const double yaw_   = -std::atan2(point.x, distanse);
        const double pitch_ = std::atan2(point.y, distanse/std::cos(yaw_));
        return {pitch_, yaw_+yaw};
    }
};
template<typename T>
struct Rune {
    T target;
    T centre;
};
#endif// INTERFACES_HPP
