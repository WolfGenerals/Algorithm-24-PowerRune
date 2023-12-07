//
// Created by mojiw on 2023/12/7.
//

#include "API.hpp"
#include "FeatureMatch.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;


Detector::Detector(const Image &sampleImage, const Point2f centre, const Point2f target, const double xAngle, const double yAngle)
    : sample({centre, target}),
      featureSample(Feature::of(sampleImage)),
      xAngle(xAngle), yAngle(yAngle) {}

auto Detector::directionOf(const Image &image) const -> Arm {
    Image img;
    cvtColor(image, img, COLOR_BGR2GRAY);

    const Feature feature = Feature::of(image);
    const Matches matches = Matches::between(featureSample, feature);
    auto [centre, target] = sample.transform(matches.transform());
    return {{(img.rows / 2.0 - target.y) / image.rows * yAngle,
             (img.cols / 2.0 - target.x) / image.cols * xAngle},

            {(img.rows / 2.0 - centre.y) / image.rows * yAngle,
             (img.cols / 2.0 - centre.x) / image.cols * xAngle}};
}