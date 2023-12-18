#ifndef FILTER_HPP
#define FILTER_HPP
#include "../Interfaces.hpp"


#include "../Stabilizer.hpp"
#include <opencv2/core/types.hpp>
#include <optional>

class PixelToDirection {
    /**
     *@brief 镜头焦距（像素）
     */
    const double focalLength;
    const int    width;
    const int    height;

    const int    historySize;
    const double excludedThreshold;


public:
    explicit PixelToDirection(const double focalLength, int width, int height, const int historySize = 50, const double excludedThreshold = 3)
        : focalLength(focalLength),
          width(width),
          height(height),
          historySize(historySize),
          excludedThreshold(excludedThreshold) {}

private:
    StabilizedDouble centerPitch{historySize, excludedThreshold};
    StabilizedDouble centerYaw{historySize, excludedThreshold};


    [[nodiscard]] Direction pointToDirection(const cv::Point2d &point) const {
        return Direction{atan2(point.y - height / 2.0f, focalLength), -atan2(point.x - width / 2.0f, focalLength)} - gimbal;
    }

public:
    /**
     * @brief 云台方向。
     */
    Direction gimbal{};

    [[nodiscard]] std::optional<Rune<Direction>> operator()(const Rune<cv::Point2d> &source) {
        const Direction target = pointToDirection(source.target);
        const Direction centre = pointToDirection(source.centre);


        centerPitch = centre.pitch;
        centerYaw   = centre.yaw;

        if (!centerPitch.valid() || !centerYaw.valid())
            return std::nullopt;

        return Rune<Direction>{target, {centerPitch, centerYaw}};
    };
};

#endif//FILTER_HPP
