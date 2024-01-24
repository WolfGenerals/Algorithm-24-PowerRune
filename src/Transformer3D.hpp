#ifndef DIRECTIONCALCULATOR_HPP
#define DIRECTIONCALCULATOR_HPP


class Transformer3D {
public:
    const Configuration& config;

    cv::Point3d direction(const cv::Point2d& point_px) const {
        const cv::Matx33d            cameraMatrix = config.cameraMatrix();
        cv::Point3d                  direction{(cameraMatrix(0, 0) + cameraMatrix(1, 1)) / 2, 0, 0};
        direction.y = -(point_px.x - cameraMatrix(0, 2));
        direction.z = -(point_px.y - cameraMatrix(1, 2)) / direction.x;
        // 归一化
        direction /= direction.x;
        return direction;
    }

    double distance(const double lenght_px,const double length_m) const {
        const cv::Matx33d            cameraMatrix = config.cameraMatrix();
        return (cameraMatrix(0, 0) + cameraMatrix(1, 1)) / 2 * length_m / lenght_px;
    }
};


#endif //DIRECTIONCALCULATOR_HPP
