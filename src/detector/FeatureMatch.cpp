//
// Created by mojiw on 2023/12/5.
//

#include "FeatureMatch.hpp"

using namespace std;
using namespace cv;

const Ptr<Feature2D> detector   = ORB::create(500, 2, 1, 31, 0, 2, ORB::HARRIS_SCORE, 127, 20);
const Ptr<Feature2D> descriptor = detector;

auto Feature::of(const Image &image) -> Feature {
    auto keyPoints   = vector<KeyPoint>();
    auto descriptors = Descriptors();

    descriptor->detectAndCompute(image, noArray(), keyPoints, descriptors);

    return {move(keyPoints), move(descriptors)};
}


const Ptr<DescriptorMatcher> matcher = BFMatcher::create();

auto Matches::between(const Feature &reference, const Feature &actual) -> Matches {
    vector<DMatch>         goodMatch;
    vector<vector<DMatch>> matches;

    matcher->knnMatch(actual.descriptors, reference.descriptors, matches, 2);

    for (vector<DMatch> &match: matches)
        if (match[0].distance < 0.85 * match[1].distance)
            goodMatch.push_back(match[0]);

    return {reference, actual, move(goodMatch)};
}

auto Matches::transform() const -> Transform {
    if (matches.size() < 4) return {};

    auto ref = vector<Point2f>();
    auto act = vector<Point2f>();

    for (const DMatch &match: matches) {
        ref.push_back(reference.keyPoints[match.trainIdx].pt);
        act.push_back(actual.keyPoints[match.queryIdx].pt);
    }

    return findHomography(ref, act, RANSAC);
}
