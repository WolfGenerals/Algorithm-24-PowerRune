#ifndef INTERFACES_HPP
#define INTERFACES_HPP
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

template<typename T>
struct Rune {
    T target;
    T centre;
};
#endif// INTERFACES_HPP
