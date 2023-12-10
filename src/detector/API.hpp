//
// Created by mojiw on 2023/12/7.
//

#ifndef API_HPP
#define API_HPP
#include "../Interfaces.hpp"
#include "Alias.hpp"
#include "FeatureMatch.hpp"
#include "Sample.hpp"

class Detector {
    Sample  sample;
    Feature featureSample;
    double  xAngle{};
    double  yAngle{};

public:
    Detector(const Image &sampleImage, cv::Point2f centre, cv::Point2f target, double xAngle, double yAngle);

    [[nodiscard]] auto directionOf(const Image &image) const -> DetectorReturn;
};

#endif//API_HPP
