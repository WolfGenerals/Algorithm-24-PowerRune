#ifndef FEATUREMATCH_HPP
#define FEATUREMATCH_HPP

#include "../Alias.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"

static inline const cv::Ptr<cv::Feature2D> detector = cv::ORB::create(500, 2, 1, 31, 0, 2, cv::ORB::HARRIS_SCORE, 127, 20);
static inline const cv::Ptr<cv::Feature2D> computer = cv::xfeatures2d::BEBLID::create(0.75f);

/**
 * \brief 图像的特征
 */
struct Feature {
    const std::vector<cv::KeyPoint> keyPoints;   /**< 关键点集合 */
    const Descriptors               descriptors; /**< 描述子 */

    /**
     * \brief 根据给定的图像生成特征
     * \param image 输入图像
     * \return 生成的特征
     */
    [[nodiscard]] static auto of(const Image &image) -> Feature {
        if (image.empty())
            throw std::runtime_error("图像为空");
        if (image.channels() != 1)
            throw std::runtime_error("图片必须是灰度图");

        std::vector<cv::KeyPoint> keyPoints;
        Descriptors               descriptors;

        // 使用detector对象的detectAndCompute函数来检测图像中的特征点和计算描述符
        detector->detect(image, keyPoints);
        computer->compute(image, keyPoints, descriptors);
        // detector->detectAndCompute(image, cv::noArray(), keyPoints, descriptors);

        return {move(keyPoints), std::move(descriptors)};
    }
};

#endif//FEATUREMATCH_HPP
