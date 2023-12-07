//
// Created by mojiw on 2023/12/2.
//

#ifndef ARM_HPP
#define ARM_HPP
#include "Alias.hpp"
#include "opencv2/opencv.hpp"
#include <yaml-cpp/node/node.h>

/**
 * \brief 能量机关的臂
 */
class Sample {
public:
    /**\brief 旋转中心 */
    const cv::Point2f centre;
    /**\brief 打击点 */
    const cv::Point2f target;

    [[nodiscard]] static auto of(YAML::Node node) -> Sample;

    [[nodiscard]] auto transform(const Transform &transform) const -> Sample;
    [[nodiscard]] auto showOn(const Image &image) const -> Image;
};

#endif //ARM_HPP
