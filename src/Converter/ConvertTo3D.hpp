#ifndef FILTER_HPP
#define FILTER_HPP


#include "../RuneRecognizer/Matches.hpp"


#include <opencv2/calib3d.hpp>


class ConvertTo3D {
public:
    const std::vector<Vec3>     &sourceWorldPoints;
    const std::vector<Vec2>     &sourceImagePoints;
    const cv::Matx33f           &cameraMatrix;
    const cv::Matx<float, 1, 5> &distCoeffs;

    [[nodiscard]] std::tuple<cv::Matx33f, Vec3> converter(const Transform2D& transform) const {
        Vec3              rvec;
        Vec3              tvec;
        cv::Matx33f       rmat;
        std::vector<Vec2> imagePoints;
        perspectiveTransform(sourceImagePoints, imagePoints, transform);
        solvePnP(sourceWorldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);
        Rodrigues(rvec, rmat);

        return {rmat, tvec};
    }
};

#endif//FILTER_HPP
