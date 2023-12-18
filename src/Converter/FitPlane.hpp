#ifndef FITPLANE_HPP
#define FITPLANE_HPP
#include "../Stabilizer.hpp"
#include "..\Data.hpp"

#include <opencv2/core/types.hpp>


class FitPlane {
    const int    historySize;
    const double excludedThreshold;

    const double     radius;
    StabilizedDouble distanse{historySize, excludedThreshold};

public:
    FitPlane(const double radius, const int historySize, const double excludedThreshold)
        : historySize(historySize),
          excludedThreshold(excludedThreshold),
          radius(radius) {}

    Plane operator()(const Rune<Direction> &rune) {
        distanse = radius / std::tan((rune.target - rune.centre).angle());
        return Plane{distanse, rune.centre.yaw};
    }
};


#endif//FITPLANE_HPP
