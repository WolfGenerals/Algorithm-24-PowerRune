#ifndef DIRECTIONCALCULATOR_HPP
#define DIRECTIONCALCULATOR_HPP


class Transformer3D {
public:
    const Configuration& config;

    cv::Point3d direction(const cv::Point2d& point_px) const {
        // 相机参数, 保持最新
        const cv::Matx33d& cameraMatrix = config.cameraMatrix();
        const double fx = cameraMatrix(0, 0);
        const double fy = cameraMatrix(1, 1);
        const double cx = cameraMatrix(0, 2);
        const double cy = cameraMatrix(1, 2);
        // 判断fx和fy是否为0
        if (fx == 0 || fy == 0) {
            return {0, 0, 0};
        }
        // 2d图像：X向右，Y向下
        // 3d空间：x轴是前方，Y轴向左，z轴是垂直于XY平面向上
        return{
                    1.0f,
                    -(point_px.x - cx) / fx,
                    -(point_px.y - cy) / fy,
                };
    }

    double distance(const double length_px, const double length_m) const {
        // 入参检查
        if (length_px <= 0 || length_m <= 0) {
            return 0;
        }
        // 相机参数, 保持最新
        const cv::Matx33d& cameraMatrix = config.cameraMatrix();
        const double fx = cameraMatrix(0, 0);
        const double fy = cameraMatrix(1, 1);
        return (fx+fy) / 2 * length_m / length_px;
    }
};


#endif //DIRECTIONCALCULATOR_HPP
