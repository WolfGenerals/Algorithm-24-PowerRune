//
// Created by mojiw on 2023/12/10.
//

#include "Converter.hpp"


#include <numeric>

using namespace std;

auto Converter::average() const -> Direction {
    double pitchSum = 0;
    double yawSum   = 0;
    for (const auto &[pitch, yaw]: centres) {
        pitchSum += pitch;
        yawSum += yaw;
    }
    return {pitchSum / centres.size(), yawSum / centres.size()};// NOLINT(*-narrowing-conversions)
}
auto Converter::variance() const -> double {
    double distance2Sum = 0;
    for (const auto &centre: centres)
        distance2Sum += centre.distance2();
    return sqrt(distance2Sum / centres.size() - average().distance2());// NOLINT(*-narrowing-conversions)
}
auto Converter::from(const RuneDirection &source) -> RuneDirection {
    const Direction target = source.target - gimbal;
    const Direction centre = source.centre - gimbal;

    if ((centre - average()).distance() > 3 * variance())
        return {true};

    centres.push_back(centre);
    if (centres.size() > maxCache)
        centres.pop_front();

    return {false, target, average()};
}