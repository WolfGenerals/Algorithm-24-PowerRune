//
// Created by mojiw on 2023/12/10.
//

#ifndef FILTER_HPP
#define FILTER_HPP
#include "../Interfaces.hpp"


#include "../Stabilizer.hpp"
#include <optional>
/**
 * @class AbsoluteDirectionConverter
 * @brief 用于给定符在云台坐标系的方向转换至车体坐标系。
 */
class AbsoluteDirectionConverter {
    int    historySize;
    double excludedThreshold;

public:
    explicit AbsoluteDirectionConverter(const int historySize=10, const double excludedThreshold=3)
        : historySize(historySize),
          excludedThreshold(excludedThreshold) {}

private:
    StabilizedDouble centerPitch{historySize, excludedThreshold};
    StabilizedDouble centerYaw{historySize, excludedThreshold};

public:
    /**
     * @brief 云台方向。
     */
    Direction gimbal{};

    /**
     * @brief 从云台方向转换至车体坐标系，并检查方向偏差。
     * @param source 云台坐标系的符。
     * @return 转换后的符，如果偏差超出可接受范围则返回空。
     */
    [[nodiscard]] std::optional<Rune<Direction>> convertToVehicle(const Rune<Direction> &source) {
        centerPitch = source.centre.pitch - gimbal.pitch;
        centerYaw   = source.centre.yaw - gimbal.yaw;

        if (!centerPitch.valid() || !centerYaw.valid())
            return std::nullopt;

        return Rune<Direction>{source.target - gimbal, {centerPitch, centerYaw}};
    };
};

#endif//FILTER_HPP
