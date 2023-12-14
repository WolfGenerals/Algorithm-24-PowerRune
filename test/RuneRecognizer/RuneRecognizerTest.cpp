#include "../../src/RuneRecognizer/RuneRecognizer.hpp"
#include "../../src/RuneRecognizer/Alias.hpp"
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>


using namespace cv;


TEST(RuneRecognizerTest, RuneFromCamera_ValidImage_ReturnsRune) {
    // Arrange
    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());

    const Feature        sampleFeature = Feature::of(image);
    const Rune<Point2f>  sampleRune    = {Point{0, 0}, Point{1, 1}};
    const RuneRecognizer runeRecognizer{sampleFeature, sampleRune};

    // Act
    const auto result = runeRecognizer.fromCamera(image);

    // Assert
    ASSERT_TRUE(result.has_value());
}

TEST(RuneRecognizerTest, RuneFromCamera_InvalidImage_ReturnsNoRune) {
    // Arrange
    const Image empty = Mat::zeros(1, 1, CV_8UC3);// Initialize invalid image

    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());

    const Feature       sampleFeature = Feature::of(image);
    const Rune<Point2f> sampleRune    = {Point{0, 0}, Point{1, 1}};
    RuneRecognizer      runeRecognizer{sampleFeature, sampleRune};

    // Act
    auto result = runeRecognizer.fromCamera(empty);

    // Assert
    ASSERT_FALSE(result.has_value());
}
