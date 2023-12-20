#ifndef FILTER_HPP
#define FILTER_HPP


#include "../RuneRecognizer/Matches.hpp"


#include <opencv2/calib3d.hpp>

class ConvertTo3D {
public:
    const std::vector<Vec3>     sourceWorldPoints;
    const std::vector<Vec2>     sourceImagePoints;
    const cv::Matx33f           cameraMatrix;
    const cv::Matx<float, 1, 5> distCoeffs;

    [[nodiscard]] auto converter(const Transform2D &transform) const {
        Vec3              rvec;
        Vec3              tvec;
        std::vector<Vec2> imagePoints;
        perspectiveTransform(sourceImagePoints, imagePoints, transform);
        solvePnP(sourceWorldPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, false);

        return [rvec, tvec](const Vec3 &point) -> Vec3 {
            //计算变换后的坐标
            cv::Matx33f rmat;
            Rodrigues(rvec, rmat);
            return rmat * point + tvec;
        };
    }
};

#endif//FILTER_HPP
