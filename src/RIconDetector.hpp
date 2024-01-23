#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <optional>

#include "opencv2/opencv.hpp"

#include "Configuration.hpp"
#include "PowerRune.hpp"


class RIconDetector {
public:
    const Configuration& config;

    auto detect(const cv::Mat& binary) -> std::optional<RIcon> {
        //发现轮廓
        std::vector<std::vector<cv::Point>> contours;
        findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto& contour: contours) {
            //计算外接旋转矩形
            const cv::RotatedRect rect = minAreaRect(contour);
            //判断是否是R标
            //长宽比在小于一定值时认为是R标
            if (rect.size.width / rect.size.height > config.R标最大长宽比()
                || rect.size.height / rect.size.width > config.R标最大长宽比())
                continue;
            // 基于填充率
            if (contourArea(contour) < rect.size.height * rect.size.width * config.R标最小填充率())
                continue;
            // 基于半径
            const float range = (rect.size.height + rect.size.width) / 2;
            if (range < config.R标最小半径() || range > config.R标最大半径())
                continue;

            return RIcon{rect.center, rect.size.height / 2};
        };
        return std::nullopt;
    };
};
#endif //DETECTOR_HPP
