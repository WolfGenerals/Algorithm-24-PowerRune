//
// Created by mojiw on 2023/12/5.
//

#include "Feature.hpp"

using namespace std;
using namespace cv;

const Ptr<Feature2D> detector = ORB::create(500, 2, 1, 31, 0, 2, ORB::HARRIS_SCORE, 127, 20);

// 定义Feature类的of函数，接收一个Image类型的参数并返回一个Feature类型的值
auto Feature::of(const Image &image) -> Feature {
    if (image.empty())
        throw runtime_error("图像为空");
    if (image.channels()!= 1)
        throw runtime_error("图片必须是灰度图");

    vector<KeyPoint> keyPoints;
    Descriptors      descriptors;

    // 使用detector对象的detectAndCompute函数来检测图像中的特征点和计算描述符
    detector->detectAndCompute(image, noArray(), keyPoints, descriptors);

    return {move(keyPoints), move(descriptors)};
}
