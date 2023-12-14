//
// Created by mojiw on 2023/12/11.
//

#include "Matches.hpp"

using namespace std;
using namespace cv;

// 创建一个BFMatcher对象matcher，用于特征匹配
const Ptr<DescriptorMatcher> matcher = BFMatcher::create();

// 判断两份特征之间的匹配关系
auto Matches::between(const Feature &reference, const Feature &actual) -> Matches {
    vector<DMatch> goodMatch; // 存储好的匹配关系
    vector<vector<DMatch>> matches; // 存储所有匹配关系

    // 使用matcher进行knnMatch，得到所有匹配关系
    matcher->knnMatch(actual.descriptors, reference.descriptors, matches, 2);

    // 遍历所有匹配关系
    for (vector<DMatch> &match: matches)
        // 如果当前匹配关系的第一项距离小于0.85乘以第二项的距离，则认为这是一个好的匹配关系
        if (match[0].distance < 0.85 * match[1].distance)
            // 将好的匹配关系添加到goodMatch中
            goodMatch.push_back(match[0]);

    // 返回好的匹配关系以及参考特征和实际特征
    return {reference, actual, move(goodMatch)};
}

// 获取旋转变换和平移向量
auto Matches::transform() const -> std::optional<Transform> {
    // 如果匹配关系数量小于4，则返回空
    if (matches.size() < 4) return nullopt;

    vector<Point2f> ref; // 存储参考特征点的坐标
    vector<Point2f> act; // 存储实际特征点的坐标

    // 遍历所有匹配关系，获取参考特征点和实际特征点的坐标
    for (const DMatch &match: matches) {
        ref.push_back(reference.keyPoints[match.trainIdx].pt);
        act.push_back(actual.keyPoints[match.queryIdx].pt);
    }

    // 使用findHomography函数，通过最小二乘法估计单应性矩阵
    Transform transform = findHomography(ref, act, RANSAC);
    // 如果单应性矩阵为空，则返回空
    if (transform.empty()) return nullopt;

    // 返回旋转变换和平移向量
    return transform;
}

