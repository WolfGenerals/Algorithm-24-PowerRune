//
// Created by mojiw on 2023/12/2.
//

#include "Sample.hpp"

using namespace std;
using namespace cv;

auto Sample::of(YAML::Node node) -> Sample {
    return {{node["centre"]["x"].as<float>(), node["centre"]["y"].as<float>()},
            {node["target"]["x"].as<float>(), node["target"]["y"].as<float>()}};
}
auto Sample::transform(const Transform &transform) const -> Sample {
    if (transform.empty())
        return *this;
    vector<Point2f> in;
    vector<Point2f> out;
    in.push_back(centre);
    in.push_back(target);
    perspectiveTransform(in, out, transform);
    return {out[0], out[1]};
}

auto Sample::showOn(const Image &image) const -> Image {
    Image out = image.clone();
    circle(out, target, 9, Scalar(255, 255, 255), 4);
    circle(out, target, 7, Scalar(0, 255, 0), 4);
    circle(out, centre, 11, Scalar(255, 255, 255), 4);
    circle(out, centre, 10, Scalar(0, 255, 0), 4);
    return move(out);
}