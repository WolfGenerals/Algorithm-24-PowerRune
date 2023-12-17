#ifndef INTERFACES_HPP
#define INTERFACES_HPP

struct Direction {
    double pitch;
    double yaw;

    [[nodiscard]] auto operator+(const Direction &other) const -> Direction { return {pitch + other.pitch, yaw + other.yaw}; }
    [[nodiscard]] auto operator-(const Direction &other) const -> Direction { return {pitch - other.pitch, yaw - other.yaw}; }
};

template<typename T>
struct Rune {
    T target;
    T centre;
};
#endif// INTERFACES_HPP
