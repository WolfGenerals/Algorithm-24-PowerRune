//
// Created by mojiw on 2023/12/5.
//

#include "Feature.hpp"

using namespace std;
using namespace cv;

const Ptr<Feature2D> detector   = ORB::create(500, 2, 1, 31, 0, 2, ORB::HARRIS_SCORE, 127, 20);
const Ptr<Feature2D> descriptor = detector;

auto Feature::of(const Image &image) -> Feature {
    vector<KeyPoint> keyPoints;
    Descriptors      descriptors;

    descriptor->detectAndCompute(image, noArray(), keyPoints, descriptors);

    return {move(keyPoints), move(descriptors)};
}
