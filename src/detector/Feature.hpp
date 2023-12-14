//
// Created by mojiw on 2023/12/5.

#ifndef FEATUREMATCH_HPP
#define FEATUREMATCH_HPP

#include "Alias.hpp"
#include "opencv2/opencv.hpp"

/**
 * \brief 图像的特征
 */
struct Feature {
    const std::vector<cv::KeyPoint> keyPoints; /**< 关键点集合 */
    const Descriptors descriptors; /**< 描述子 */

    /**
     * \brief 根据给定的图像生成特征
     * \param image 输入图像
     * \return 生成的特征
     */
    static auto of(const Image &image) -> Feature;
};

#endif//FEATUREMATCH_HPP
