#ifndef CONFIGURATION_HPP
#define CONFIGURATION_HPP


class Configuration {
    const rclcpp::Node&                                           node;
    cv::Matx33d                                                   cameraMatrix_;
    cv::Matx<double, 1, 5>                                        distCoeffs_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cameraInfo_subscriber;

public:
    [[nodiscard]] bool   enable() const { return node.get_parameter("enable").as_bool(); }
    [[nodiscard]] bool   DEBUG() const { return node.get_parameter("DEBUG").as_bool(); }
    [[nodiscard]] int    二值化阈值() const { return node.get_parameter("二值化阈值").as_int(); };
    [[nodiscard]] double R标最大长宽比() const { return node.get_parameter("R标最大长宽比").as_double(); }
    [[nodiscard]] double R标最小填充率() const { return node.get_parameter("R标最小填充率").as_double(); }
    [[nodiscard]] double R标最小半径() const { return node.get_parameter("R标最小半径").as_double(); }
    [[nodiscard]] double R标最大半径() const { return node.get_parameter("R标最大半径").as_double(); }
    [[nodiscard]] int    内圈半径() const { return node.get_parameter("内圈半径").as_int(); }
    [[nodiscard]] int    外圈半径() const { return node.get_parameter("外圈半径").as_int(); }
    [[nodiscard]] int    扇叶外缘最小面积() const { return node.get_parameter("扇叶外缘最小面积").as_int(); }
    [[nodiscard]] double 外缘靶心半径比() const { return node.get_parameter("轮廓靶心半径比").as_double(); }
    [[nodiscard]] double 命中延迟_秒() const { return node.get_parameter("命中延迟_秒").as_double(); }

    [[nodiscard]] cv::Matx33d            cameraMatrix() const { return cameraMatrix_; }
    [[nodiscard]] cv::Matx<double, 1, 5> distCoeffs() const { return distCoeffs_; }

    explicit Configuration(rclcpp::Node& node) : node(node) {
        node.declare_parameter("enable", true);
        node.declare_parameter("DEBUG", false);
        node.declare_parameter("二值化阈值", 200);
        node.declare_parameter("R标最大长宽比", 1.1);
        node.declare_parameter("R标最小填充率", 0.7);
        node.declare_parameter("R标最小半径", 10.0);
        node.declare_parameter("R标最大半径", 20.0);
        node.declare_parameter("内圈半径", 160);
        node.declare_parameter("外圈半径", 220);
        node.declare_parameter("扇叶外缘最小面积", 100);
        node.declare_parameter("轮廓靶心半径比", 0.85);
        node.declare_parameter("命中延迟_秒", 0.2);

        cameraInfo_subscriber = node.create_subscription<sensor_msgs::msg::CameraInfo>(
            "camera_info",
            10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr cameraInfo) -> void {
                const auto matrix = cameraInfo->k;
                const auto coeffs = cameraInfo->d;
                cameraMatrix_     = {matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]};
                distCoeffs_       = {coeffs[0], coeffs[1], coeffs[2], coeffs[3], coeffs[4]};
            }
        );
    }

    Configuration(const Configuration& other)                = delete;
    Configuration(Configuration&& other) noexcept            = delete;
    Configuration& operator=(const Configuration& other)     = delete;
    Configuration& operator=(Configuration&& other) noexcept = delete;
};


#endif //CONFIGURATION_HPP
