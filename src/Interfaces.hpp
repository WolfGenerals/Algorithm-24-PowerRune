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
};

template<typename T>
struct Rune {
    const T target;
    const T centre;
};
#endif// INTERFACES_HPP
