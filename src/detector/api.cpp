//
// Created by mojiw on 2023/12/11.
//

#include "api.hpp"
#include "Feature.hpp"
#include "Matches.hpp"

using namespace std;
using namespace cv;

auto runeFromCamera(const Image &image, const Feature &sampleImage, const Rune<Point2f> &sampleRune) -> std::optional<Rune<cv::Point2f>> {
    const vector points{sampleRune.target, sampleRune.centre};

    const auto feature   = Feature::of(image);
    const auto matches   = Matches::between(sampleImage, feature);
    const auto transform = matches.transform();

    if (!transform) return nullopt;

    perspectiveTransform(points, points, transform.value());
    return Rune{points[0], points[1]};
}