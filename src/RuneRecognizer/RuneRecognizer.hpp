//
// Created by mojiw on 2023/12/11.

#ifndef API_HPP
#define API_HPP
#include "../Interfaces.hpp"
#include "Alias.hpp"
#include "Feature.hpp"


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
     *@brief 镜头焦距（像素）
     */
    const double       focalLength;
    /**
     * @brief 从相机图像中识别符文
     * @param image 用于识别的相机图像
     * @return 如果成功识别符文，则返回识别的符文位置的std::optional对象，否则返回std::nullopt
     */
    [[nodiscard]] auto fromCamera(const Image& image) const -> std::optional<Rune<Direction>>;
};

#endif//API_HPP
