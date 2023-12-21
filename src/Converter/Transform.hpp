#ifndef TRANSFORM_HPP
#define TRANSFORM_HPP

#include "../RuneRecognizer/Matches.hpp"


struct Transform2D {
    cv::Matx33f matrix;

    explicit Transform2D(const cv::Matx33f& matrix)
        : matrix(matrix) {}

    Vec2 operator*(const Vec2& vec) const {
        return {
                    matrix(0, 0) * vec(0) + matrix(0, 1) * vec(1) + matrix(0, 2),
                    matrix(1, 0) * vec(0) + matrix(1, 1) * vec(1) + matrix(1, 2)
                };
    }

    std::vector<Vec2> operator*(const std::vector<Vec2>& vecs) const {
        std::vector<Vec2> result;
        perspectiveTransform(vecs, result, matrix);

        return result;
    }

    [[nodiscard]] static std::optional<Transform2D> fit(const Matches& matches) {
        // 如果匹配关系数量小于4，则返回空
        if (matches.matches.size() < 4) return std::nullopt;

        std::vector<Vec2> ref; // 存储参考特征点的坐标
        std::vector<Vec2> act; // 存储实际特征点的坐标

        // 遍历所有匹配关系，获取参考特征点和实际特征点的坐标
        for (const cv::DMatch& match: matches.matches) {
            ref.push_back(matches.reference.keyPoints[match.trainIdx].pt);
            act.push_back(matches.actual.keyPoints[match.queryIdx].pt);
        }

        // 使用findHomography函数，通过最小二乘法估计单应性矩阵
        const cv::Mat transform = findHomography(ref, act, cv::RANSAC);
        // 如果单应性矩阵为空，则返回空
        if ((transform).empty())
            return std::nullopt;

        // 返回旋转变换和平移向量
        return {Transform2D{transform}};
    }
};


struct Transform3D {
    cv::Matx33f rmat;
    Vec3        tvec;

    [[nodiscard]] static Transform3D fit(
        const std::vector<Vec3>&     worldPoints,
        const std::vector<Vec2>&     imagePoints,
        const cv::Matx33f&           cameraMatrix,
        const cv::Matx<float, 1, 5>& distCoeffs
    ) {
        Vec3 rvec;
        cv::Matx33f rmat;
        Vec3        tvec;
        solvePnP(worldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);
        Rodrigues(rvec, rmat);

        return {rmat, tvec};
    }

    Vec3 operator*(const Vec3& vec) const {
        return {
                    rmat(2, 0) * vec(0) + rmat(2, 1) * vec(1) + rmat(2, 2) * vec(2) + tvec(2),
                    rmat(0, 0) * vec(0) + rmat(0, 1) * vec(1) + rmat(0, 2) * vec(2) + tvec(0),
                    rmat(1, 0) * vec(0) + rmat(1, 1) * vec(1) + rmat(1, 2) * vec(2) + tvec(1)
                };
    }

    std::vector<Vec3> operator*(const std::vector<Vec3>& vecs) const {
        std::vector<Vec3> result;
        for (const Vec3& vec: vecs) { result.push_back(*this * vec); }
        return result;
    }
};


#endif //TRANSFORM_HPP
