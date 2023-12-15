//
// Created by mojiw on 2023/12/10.
//

#ifndef FILTER_HPP
#define FILTER_HPP
#include "../Interfaces.hpp"


#include <optional>
#include <queue>

/**
 * @class AbsoluteDirectionConverter
 * @brief 用于给定符在云台坐标系的方向转换至车体坐标系。
 */
class AbsoluteDirectionConverter {
    /**
     * @brief 最大历史记录数。
     */
    constexpr int         maxHistory = 30;
    /**
     * @brief 中心点队列。
     */
    std::queue<Direction> centres{};

    /**
     * @brief 计算方向的平均值。
     * @return 平均方向。
     */
    [[nodiscard]] Direction average() const;
    /**
     * @brief 计算方向的方差。
     * @return 方向的方差。
     */
    [[nodiscard]] double    variance() const;

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
    [[nodiscard]] std::optional<Rune<Direction>> convertToVehicle(const Rune<Direction> &source);
};

#endif//FILTER_HPP
