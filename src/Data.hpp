#ifndef INTERFACES_HPP
#define INTERFACES_HPP
#include <opencv2/core/types.hpp>
#include <valarray>


struct Angle {
    double radian;

    Angle(const double &radian = 0)
        : radian(radian) {
        if (this->radian >= M_PI)
            this->radian = fmod(this->radian, 2 * M_PI);
        else if (this->radian < -M_PI)
            this->radian = fmod(this->radian, 2 * M_PI) + 2 * M_PI;
    }

    [[nodiscard]] auto operator+(const Angle &other) const -> Angle { return {radian + other.radian}; }
    [[nodiscard]] auto operator-(const Angle &other) const -> Angle { return {radian - other.radian}; }
    [[nodiscard]] auto operator*(const double &other) const -> Angle { return {radian * other}; }
    [[nodiscard]] auto operator/(const double &other) const -> Angle { return {radian / other}; }
    [[nodiscard]] auto operator==(const Angle &other) const -> bool { return radian == other.radian; }

    operator double() const { return radian; }
};


struct Direction {
    Angle pitch;
    Angle yaw;

    [[nodiscard]] auto operator+(const Direction &other) const -> Direction { return {pitch + other.pitch, yaw + other.yaw}; }
    [[nodiscard]] auto operator-(const Direction &other) const -> Direction { return {pitch - other.pitch, yaw - other.yaw}; }

    [[nodiscard]] double angle() const {
        return std::acos(std::cos(pitch) * std::cos(yaw));
    }
};
struct Plane {
    double distanse;
    Angle  yaw;

    [[nodiscard]] cv::Point2d intersect(Direction dir) const {
        return {
                (distanse * -std::tan(dir.yaw - yaw)),
                (distanse / std::cos(dir.yaw - yaw) / std::tan(dir.pitch))};
    }
    [[nodiscard]] Direction direction(const cv::Point2d &point) const {
        const double yaw_   = -std::atan2(point.x, distanse);
        const double pitch_ = std::atan2(point.y, distanse / std::cos(yaw_));
        return {pitch_, yaw_ + yaw};
    }
};
template<typename T>
struct Rune {
    T target;
    T centre;
};
#endif// INTERFACES_HPP
