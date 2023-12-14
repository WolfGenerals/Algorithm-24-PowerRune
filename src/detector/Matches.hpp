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
    const Feature                &reference;    /**< 参考特征 */
    const Feature                &actual;       /**< 实际特征 */
    const std::vector<cv::DMatch> matches;       /**< 匹配关系 */


    /**
     * \brief 获取两个特征之间的匹配关系
     *
     * \param reference 参考特征
     * \param actual 实际特征
     * \return 两个特征间的匹配关系
     */
    static auto between(const Feature &reference, const Feature &actual) -> Matches;

    /**
     * \brief 获取变换矩阵
     *
     * \return 从参考特征到实际特征的透视变换
     */
    [[nodiscard]] auto transform() const -> std::optional<Transform>;
};


#endif //MATCHES_HPP
