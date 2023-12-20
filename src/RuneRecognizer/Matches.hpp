#ifndef MATCHES_HPP
#define MATCHES_HPP
#include "Feature.hpp"


#include <optional>
#include <vector>

static inline const cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create();

/**
 * \brief 两个特征间的匹配关系
 */
struct Matches {
    const Feature                &reference; /**< 参考特征 */
    const Feature                &actual;    /**< 实际特征 */
    const std::vector<cv::DMatch> matches;   /**< 匹配关系 */

    /**
     * \brief 获取两个特征之间的匹配关系
     *
     * \param reference 参考特征
     * \param actual 实际特征
     * \return 两个特征间的匹配关系
     */
    [[nodiscard]] static auto between(const Feature &reference, const Feature &actual) -> Matches {
        std::vector<cv::DMatch>              goodMatch;// 存储好的匹配关系
        std::vector<std::vector<cv::DMatch>> matches;  // 存储所有匹配关系

        // 使用matcher进行knnMatch，得到所有匹配关系
        matcher->knnMatch(actual.descriptors, reference.descriptors, matches, 2);

        // 遍历所有匹配关系
        for (std::vector<cv::DMatch> &match: matches)
            // 如果当前匹配关系的第一项距离小于0.85乘以第二项的距离，则认为这是一个好的匹配关系
            if (match[0].distance < 0.85 * match[1].distance)
                // 将好的匹配关系添加到goodMatch中
                goodMatch.push_back(match[0]);


        // 返回好的匹配关系以及参考特征和实际特征
        return {reference, actual, move(goodMatch)};
    }

    [[nodiscard]] auto transform() const -> std::optional<Transform2D> {
        // 如果匹配关系数量小于4，则返回空
        if (matches.size() < 4) return std::nullopt;

        std::vector<Vec2> ref;// 存储参考特征点的坐标
        std::vector<Vec2> act;// 存储实际特征点的坐标

        // 遍历所有匹配关系，获取参考特征点和实际特征点的坐标
        for (const cv::DMatch &match: matches) {
            ref.push_back(reference.keyPoints[match.trainIdx].pt);
            act.push_back(actual.keyPoints[match.queryIdx].pt);
        }

        // 使用findHomography函数，通过最小二乘法估计单应性矩阵
        cv::Mat transform = findHomography(ref, act, cv::RANSAC);
        // 如果单应性矩阵为空，则返回空
        if ((transform).empty())
            return std::nullopt;

        // 返回旋转变换和平移向量
        return transform;
    }
};


#endif //MATCHES_HPP
