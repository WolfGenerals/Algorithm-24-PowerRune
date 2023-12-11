//
// Created by mojiw on 2023/12/11.
//

#ifndef MATCHES_HPP
#define MATCHES_HPP
#include "Feature.hpp"


#include <optional>
#include <vector>


/**
 * \brief 两个特征间的匹配关系
 */
struct Matches {
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
    [[nodiscard]] auto transform() const -> std::optional<Transform>;
};


#endif //MATCHES_HPP
