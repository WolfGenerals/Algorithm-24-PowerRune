//
// Created by mojiw on 2023/12/2.
//

#ifndef ARM_HPP
#define ARM_HPP
#include "../Alias.hpp"
#include "opencv2/opencv.hpp"


/**
 * \brief 能量机关的臂
 */
class Arm {
public:
    /**\brief 旋转中心 */
    const cv::Point2f centre;
    /**\brief 打击点 */
    const cv::Point2f target;

    [[nodiscard]] auto transform(const Transform &transform) const -> Arm;
    [[nodiscard]] auto showOn(const Image &image) const -> Image;
};


#endif //ARM_HPP
