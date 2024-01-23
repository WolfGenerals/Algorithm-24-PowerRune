#ifndef RUNEMASKER_HPP
#define RUNEMASKER_HPP
#include "Configuration.hpp"
#include "PowerRune.hpp"

#include "opencv2/opencv.hpp"


class Masker {
public:
    const Configuration& config;

    auto mask(const cv::Point2d& center, const cv::Mat& image) -> cv::Mat {
        cv::Mat result = image;
        circle(result, center, config.内圈半径(), 0, -1);
        circle(result, center, config.外圈半径()+500, 0, 1000);
        return result;
    }
};


#endif //RUNEMASKER_HPP
