//
// Created by mojiw on 2023/12/11.
//

#include "Matches.hpp"

using namespace std;
using namespace cv;

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

auto Matches::transform() const -> std::optional<Transform> {
    if (matches.size() < 4) return nullopt;

    vector<Point2f> ref;
    vector<Point2f> act;

    for (const DMatch &match: matches) {
        ref.push_back(reference.keyPoints[match.trainIdx].pt);
        act.push_back(actual.keyPoints[match.queryIdx].pt);
    }

    Transform transform = findHomography(ref, act, RANSAC);
    if (transform.empty()) return nullopt;

    return transform;
}
