//
// Created by mojiw on 2023/12/11.

#ifndef API_HPP
#define API_HPP
#include "../Interfaces.hpp"
#include "Alias.hpp"
#include "Feature.hpp"


#include <optional>

/**
 * \brief 获取摄像头中符的坐标
 * \param image 摄像头输入
 * \param sampleImage 样本特征
 * \param sampleRune 样本符
 * \return 摄像头中符的坐标
 */
auto runeFromCamera(
        const Image             &image,
        const Feature           &sampleImage,
        const Rune<cv::Point2f> &sampleRune)
        -> std::optional<Rune<cv::Point2f>>;

#endif//API_HPP
