//
// Created by mojiw on 2023/12/5.

#ifndef FEATUREMATCH_HPP
#define FEATUREMATCH_HPP

#include "Alias.hpp"
#include "opencv2/opencv.hpp"

/**
 * \brief 图像的特征点
 */
struct Feature {
    const std::vector<cv::KeyPoint> keyPoints;
    const Descriptors               descriptors;

    /**
     * \brief 计算特征点
     * \param image 源图像
     * \return 图像的特征点
     */
    static auto of(const Image &image) -> Feature;
};

#endif//FEATUREMATCH_HPP
