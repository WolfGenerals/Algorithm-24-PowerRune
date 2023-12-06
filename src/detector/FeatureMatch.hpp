//
// Created by mojiw on 2023/12/5.

#ifndef FEATUREMATCH_HPP
#define FEATUREMATCH_HPP

#include "../Alias.hpp"
#include "opencv2/opencv.hpp"

/**
 * \brief 图像的特征点
 */
class Feature {
public:
    const std::vector<cv::KeyPoint> keyPoints;
    const Descriptors                   descriptors;
    const Image                   image;

    /**
     * \brief 计算特征点
     * \param image 源图像
     * \return 图像的特征点
     */
    static auto of(const Image &image) -> Feature;

    [[nodiscard]] auto show() const -> Image;
};

/**
 * \brief 两个特征间的匹配关系
 */
class Matches {
public:
    const Feature                &reference;
    const Feature                &actual;
    const std::vector<cv::DMatch> matches;

    /**
     * \brief 计算特征间的匹配
     * \param reference 参考图像的特征
     * \param actual 实际图像的特征
     * \return 两个特征间的匹配关系
     */
    static auto between(const Feature &reference, const Feature &actual) -> Matches;

    /**
     * \brief 计算透视变换
     * \return 从参考图像到实际图像的透视变换
     */
    [[nodiscard]] auto transform() const -> Transform;
    [[nodiscard]] auto show() const -> Image;
};

#endif//FEATUREMATCH_HPP
