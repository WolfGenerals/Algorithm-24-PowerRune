#include "../../src/RuneRecognizer/Feature.hpp"
#include <gtest/gtest.h>

using namespace cv;
using namespace std;

//测试特征提取
TEST(FeatureTest, 正常提取) {
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);
    const auto [keyPoints, descriptors] = Feature::of(image);
    ASSERT_FALSE(keyPoints.empty());
    ASSERT_FALSE(descriptors.empty());
}

//空图像测试
TEST(FeatureTest, 空图像_抛出异常) {
    const auto empty = Mat();// Initialize invalid image
    ASSERT_THROW(Feature::of(empty), std::runtime_error);
}

//错误的通道数量测试
TEST(FeatureTest, 错误的通道数量_抛出异常) {
    const Image empty = Mat::zeros(100, 100, CV_8UC3);// Initialize invalid image
    ASSERT_THROW(Feature::of(empty), std::runtime_error);
}