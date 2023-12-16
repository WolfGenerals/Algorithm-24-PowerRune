#include "../../src/RuneRecognizer/RuneRecognizer.hpp"
#include "../../src/RuneRecognizer/Alias.hpp"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>


using namespace cv;


TEST(RuneRecognizerTest, 有特征点的图像_返回非空值) {
    // Arrange
    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);

    const Feature        sampleFeature = Feature::of(image);
    const Rune<Point2f>  sampleRune    = {Point{0, 0}, Point{1, 1}};
    const RuneRecognizer runeRecognizer{sampleFeature, sampleRune, 1};

    // Act
    const auto result = runeRecognizer.fromCamera(image);

    // Assert
    ASSERT_TRUE(result.has_value());
}

TEST(RuneRecognizerTest, 无特征点的图像_返回空值) {
    // Arrange
    const Image empty = Mat::zeros(200, 200, CV_8UC1);// Initialize invalid image

    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);

    const Feature       sampleFeature = Feature::of(image);
    const Rune<Point2f> sampleRune    = {Point{0, 0}, Point{1, 1}};
    RuneRecognizer      runeRecognizer{sampleFeature, sampleRune,1};

    // Act
    auto result = runeRecognizer.fromCamera(empty);

    // Assert
    ASSERT_FALSE(result.has_value());
}

TEST(RuneRecognizerTest, 错误的通道数量_抛出异常) {
    // Arrange
    const Image empty = Mat::zeros(1, 1, CV_8UC3);// Initialize invalid image

    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);


    const Feature        sampleFeature = Feature::of(image);
    const Rune<Point2f>  sampleRune    = {Point{0, 0}, Point{1, 1},1};
    const RuneRecognizer runeRecognizer{sampleFeature, sampleRune};

    // Act
    ASSERT_THROW(runeRecognizer.fromCamera(empty), std::runtime_error);
}

TEST(RuneRecognizerTest, 空图像_抛出异常) {
    // Arrange
    const auto empty = Mat();// Initialize invalid image

    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);


    const Feature        sampleFeature = Feature::of(image);
    const Rune<Point2f>  sampleRune    = {Point{0, 0}, Point{1, 1},1};
    const RuneRecognizer runeRecognizer{sampleFeature, sampleRune};

    // Act
    ASSERT_THROW(runeRecognizer.fromCamera(empty), std::runtime_error);
}
