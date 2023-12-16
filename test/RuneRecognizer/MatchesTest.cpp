#include "../../src/RuneRecognizer/Matches.hpp"
#include "gtest/gtest.h"

using namespace cv;
using namespace std;

TEST(MatchesTest, 正常匹配) {
    // Arrange
    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);

    const Feature feature = Feature::of(image);

    const Matches matches = Matches::between(feature, feature);
    ASSERT_TRUE(matches.transform().has_value());
}

//无法匹配，返回nullopt
TEST(MatchesTest, 无法匹配_返回空值) {
    // Arrange
    // Initialize valid image
    VideoCapture videoCapture("https://img-blog.csdnimg.cn/fb28c54943314e249f5814852300d6fc.png");
    Image        image;
    videoCapture.read(image);
    ASSERT_FALSE(image.empty());
    cvtColor(image, image, COLOR_BGR2GRAY);

    const Matches matches = Matches::between(Feature::of(image), Feature::of(Mat::zeros(100, 100, CV_8UC1)));
    ASSERT_FALSE(matches.transform().has_value());
}