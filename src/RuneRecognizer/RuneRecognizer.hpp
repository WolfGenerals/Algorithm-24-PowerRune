
#ifndef API_HPP
#define API_HPP
#include "../Interfaces.hpp"
#include "Alias.hpp"
#include "Feature.hpp"
#include "Matches.hpp"


#include <optional>


/**
 * @class RuneRecognizer
 * @brief 用于从相机图像中识别符文的类
 */
class RuneRecognizer {
public:
    /**
     * @brief 用于识别的样本图像
     */
    const Feature& sampleFeature;

    /**
     * @brief 用于识别的样本符文
     */
    const Rune<cv::Point2f> &sampleRune;

    /**
     * @brief 从相机图像中识别符文
     * @param image 用于识别的相机图像
     * @return 如果成功识别符文，则返回识别的符文位置的std::optional对象，否则返回std::nullopt
     */
    [[nodiscard]] auto fromCamera(const Image& image) const -> std::optional<Rune<cv::Point2d>>{
        const std::vector untransformedPoints{sampleRune.target, sampleRune.centre};
        // 获取图像的特征
        const auto        feature   = Feature::of(image);
        // 在样本图像和特征点之间进行匹配
        const auto        matches   = Matches::between(sampleFeature, feature);
        // 根据匹配结果获取变换矩阵
        const auto        transform = matches.transform();


        // 确保变换矩阵存在
        if (!transform)
            return std::nullopt;

        // 应用透视变换
        perspectiveTransform(untransformedPoints, untransformedPoints, transform.value());
        return Rune<cv::Point2d>{untransformedPoints[0], untransformedPoints[1]};
       }

};

#endif//API_HPP
