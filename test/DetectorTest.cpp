#include "..\src\detector\Detector.hpp"
#include "gtest/gtest.h"

TEST(DetectorTest, Init) {
    Detector(Image(), cv::Point(0, 0), cv::Point(0, 0), 0, 0);
}

TEST(DetectorTest, Call) {
    const Detector detector(Image(100, 100, CV_8UC3), cv::Point(0, 0), cv::Point(0, 0), 10, 10);
    auto           directionOf = detector.directionOf(Image(100, 100, CV_8UC3));
}
