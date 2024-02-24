#ifndef RUNEDETECTOR_HPP
#define RUNEDETECTOR_HPP

#include <vector>
#include "opencv2/opencv.hpp"
#include "Configuration.hpp"
#include "PowerRune.hpp"


class FanBladeDetector {
public:
    const Configuration& config;

    auto detect(const cv::Point2d& center, const cv::Mat& image) -> std::vector<FanBlade> {
        std::vector<FanBlade> result;
        //寻找轮廓的外接旋转矩形
        std::vector<std::vector<cv::Point>> contours;
        findContours(image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (auto& contour: contours) {
            //计算外接旋转矩形
            const cv::RotatedRect rect = minAreaRect(contour);
            //判断是否是扇叶外缘
            //面积
            if (contourArea(contour) < config.扇叶外缘最小面积())
                continue;
            //计算靶心
            result.push_back(FanBlade{(static_cast<cv::Point2d>(rect.center) - center) * config.外缘靶心半径比(), RuneState::INACTIVE});
        }
        return result;
    }
};


#endif //RUNEDETECTOR_HPP
