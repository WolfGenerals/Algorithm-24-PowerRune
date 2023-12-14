#include "..\src\detector\RuneRecognizer.hpp"
#include "gtest/gtest.h"
#include <opencv2/core/mat.hpp>

TEST(DetectorTest, Init) {
    RuneFromCamera(cv::Mat::zeros(1, 1, CV_8UC3), {{1, 1}, {1, 1}});
}

TEST(DetectorTest, Call) {
    const auto runeFromCamera = RuneFromCamera(cv::Mat::zeros(1, 1, CV_8UC3),
                                               {{1, 1}, {1, 1}});
    auto       fromCamera =
            runeFromCamera(cv::Mat::zeros(1, 1, CV_8UC3));
}
