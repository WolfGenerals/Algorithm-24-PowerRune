#include <cstdio>
#include "Alias.hpp"
#include "Converter/ConvertTo3D.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;


class PowerRuneNode final : public Node {
    shared_ptr<Feature>   sourceFeature;
    std::vector<Vec3>     sourceWorldPoints;
    std::vector<Vec2>     sourceImagePoints;
    cv::Matx33f           cameraMatrix;
    cv::Matx<float, 1, 5> distCoeffs;

    ConvertTo3D convertTo3D{sourceWorldPoints, sourceImagePoints, cameraMatrix, distCoeffs};

    Publisher<geometry_msgs::msg::Point>::SharedPtr  rune_in_camera_publisher;
    Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;

    void main(const sensor_msgs::msg::Image::SharedPtr& imageRos) const {
        RCLCPP_INFO(get_logger(), "收到图片");
        const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);
        if (image->image.empty()) {
            RCLCPP_WARN(get_logger(), "image is empty");
            return;
        }

        Image grayImage;
        cvtColor(image->image, grayImage, cv::COLOR_BGR2GRAY);
        const Feature                    feature   = Feature::of(grayImage);
        const Matches                    matches   = Matches::between(*sourceFeature, feature);
        const std::optional<Transform2D> transform = matches.transform();
        RCLCPP_INFO(get_logger(), "转换成功");
        if (!transform)
            return;
        const auto& [rmat, tvec] = convertTo3D.converter(*transform);
        RCLCPP_INFO(get_logger(), "转换3d成功");
        for (const Vec3& point: sourceWorldPoints) {
            geometry_msgs::msg::Point point_msg;
            Vec3                      result;
            result = rmat * point + tvec;
            RCLCPP_INFO(get_logger(), "转换3d成功2");

            point_msg.x = result(0);
            point_msg.y = result(1);
            point_msg.z = result(2);
            RCLCPP_INFO(get_logger(), "发布坐标");
            rune_in_camera_publisher->publish(point_msg);
            break;
        }
    }

public:
    explicit PowerRuneNode(): Node("power_rune") {
        RCLCPP_INFO(get_logger(), "power_rune main start");
        image_subscriber = create_subscription<sensor_msgs::msg::Image>(
            "camera_input",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr imageRos) -> void { main(imageRos); }
        );

        rune_in_camera_publisher = create_publisher<geometry_msgs::msg::Point>("rune_in_camera", 10);
    }

    void init() {
        cameraMatrix = cv::Matx33f{1777, 0, 1920 / 2, 0, 1777, 1071 / 2, 0, 0, 1};
        distCoeffs   = cv::Matx<float, 1, 5>{0, 0, 0, 0, 0};

        Image image = cv::imread(
            "/mnt/c/Projects/C/Algorithm-24/src/power_rune/temp/屏幕截图 2023-12-19 212720.png"
        );
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        sourceFeature = std::make_shared<Feature>(Feature::of(image));
        Vec2 imageSize{static_cast<float>(image.cols), static_cast<float>(image.rows)};

        sourceImagePoints = {
                    {imageSize(0) * 0.54f, imageSize(1) * 0.18f},
                    {imageSize(0) * 0.5f, imageSize(1) * 0.73f},
                    {imageSize(0) * 0.25f, imageSize(1) * 0.85f},
                    {imageSize(0) * 0.75f, imageSize(1) * 0.85f},
                    {imageSize(0) * 0.25f, imageSize(1) * 0.6f},
                    {imageSize(0) * 0.75f, imageSize(1) * 0.6f}
                };
        sourceWorldPoints = {
                    {0.0f, 0.0f, 0.0f},
                    {0.0f, 0.0f, 0.7f},
                    {0.0f, 0.16f, 0.86f},
                    {0.0f, -0.16f, 0.86f},
                    {0.0f, 0.16f, 0.54f},
                    {0.0f, -0.16f, 0.54f}
                };
    }
};


int main(int argc, char** argv) {
    init(argc, argv);
    const auto powerRuneNode = std::make_shared<PowerRuneNode>();
    powerRuneNode->init();
    spin(powerRuneNode);
    printf("hello world power_rune package\n");
    return 0;
}
