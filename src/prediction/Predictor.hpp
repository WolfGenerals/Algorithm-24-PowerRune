#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP
#include "../Stabilizer.hpp"


#include <opencv2/core/types.hpp>
#include <opencv2/video/tracking.hpp>


class Predictor {
public:
    virtual ~Predictor() = default;

    virtual cv::Point2d update(const cv::Point2d &current, int64 timestamp) = 0;
};


class Small final : public Predictor {
    int    historySize;
    double excludedThreshold;

    StabilizedDouble initialX{historySize, excludedThreshold};
    StabilizedDouble initialY{historySize, excludedThreshold};
    double           omega;

public:
    Small(const double omega, const int historySize, const double excludedThreshold)
        : historySize(historySize),
          excludedThreshold(excludedThreshold),
          omega(omega) {}

    cv::Point2d update(const cv::Point2d &current, int64 timestamp) override {
        initialX = current.x;
    }
};

class Large final : public Predictor {
public:
    cv::Point2d update(const cv::Point2d &current, int64 timestamp) override;
};


#endif//PREDICTOR_HPP
