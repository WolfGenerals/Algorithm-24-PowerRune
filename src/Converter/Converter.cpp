//
// Created by mojiw on 2023/12/10.
//

#include "Converter.hpp"


#include <numeric>

using namespace std;

// 计算平均方向
Direction AbsoluteDirectionConverter::average() const {
    double pitchSum = 0;
    double yawSum   = 0;
    for (const auto &[pitch, yaw]: centres) {// 遍历中心点列表
        pitchSum += pitch;                   // 累加俯仰角的值
        yawSum += yaw;                       // 累加偏航角的值
    }
    return {pitchSum / centres.size(), yawSum / centres.size()};// NOLINT(*-narrowing-conversions)
}

// 计算方差
double AbsoluteDirectionConverter::variance() const {
    double distance2Sum = 0;                                         // 方差累加和
    for (const auto &[pitch, yaw]: centres)                          // 遍历中心点列表
        distance2Sum += pitch * pitch + yaw * yaw;                   // 计算俯仰角和偏航角的平方和
    const auto [pitch, yaw] = average();                             // 获取平均方向
    return distance2Sum / centres.size() - pitch * pitch - yaw * yaw;// NOLINT(*-narrowing-conversions)
}

std::optional<Rune<Direction>> AbsoluteDirectionConverter::convertToVehicle(const Rune<Direction> &source) {
    const Direction target = source.target - gimbal;// 计算目标方向
    const Direction centre = source.centre - gimbal;// 计算中心方向

    centres.push_back(centre);// 将中心方向添加到中心点列表
    if (centres.size() > maxHistory)
        centres.pop_front();// 如果中心点列表过大，则移除最旧的一个方向

    const auto   averageDirection = average();// 计算平均方向
    const double deviation =
            (centre.pitch - averageDirection.pitch) * (centre.pitch - averageDirection.pitch) +
            (centre.yaw - averageDirection.yaw) * (centre.yaw - averageDirection.yaw);// 计算中心方向与平均方向的偏差

    if (deviation > 9 * variance()) // 如果偏差超过允许范围，则返回空
        return nullopt;
    
    return Rune<Direction>{target, averageDirection}; // 返回转换后的方向
}

