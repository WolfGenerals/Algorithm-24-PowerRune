//
// Created by mojiw on 2023/12/11.
//

#include "Feature.hpp"
#include "Matches.hpp"
#include "RuneRecognizer.hpp"

using namespace std;
using namespace cv;

auto RuneRecognizer::fromCamera(const Image &image) const -> std::optional<Rune<cv::Point2f>> {
    const std::vector untransformedPoints{sampleRune.target, sampleRune.centre};

    // 获取图像的特征
    const auto feature   = Feature::of(image);
    // 在样本图像和特征点之间进行匹配
    const auto matches   = Matches::between(sampleFeature, feature);
    // 根据匹配结果获取变换矩阵
    const auto transform = matches.transform();


    // 确保变换矩阵存在
    if (!transform)
        return std::nullopt;

    // 应用透视变换
    perspectiveTransform(untransformedPoints, untransformedPoints, transform.value());

    // 返回检测到的符文位置
    return Rune<Point2f>{untransformedPoints[0], untransformedPoints[1]};
}
