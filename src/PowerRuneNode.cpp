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
    cv::Matx33f&          cameraMatrix;
    cv::Matx<float, 1, 5> distCoeffs;

    ConvertTo3D           convertTo3D;

    Publisher<geometry_msgs::msg::Point>::SharedPtr rune_in_camera_publisher;

    void main(const sensor_msgs::msg::Image::SharedPtr& imageRos) {
        cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);

        Image grayImage;
        cvtColor(image->image, grayImage, cv::COLOR_BGR2GRAY);
        const Feature                    feature   = Feature::of(grayImage);
        const Matches                    matches   = Matches::between(*sourceFeature, feature);
        const std::optional<Transform2D> transform = matches.transform();
        if (!transform)
            return;
        const auto& [rmat, tvec] = convertTo3D.converter(*transform);

        for (const Vec3 &point: sourceWorldPoints) {
            geometry_msgs::msg::Point point_msg{};
            Vec3 result;
            result = rmat * point + tvec;
            point_msg.x = result(0);
            point_msg.y = result(1);
            point_msg.z = result(2);
            rune_in_camera_publisher->publish(point_msg);
        }
    }

public:
    explicit PowerRuneNode(): rclcpp::Node("power_rune") {
        create_subscription<sensor_msgs::msg::Image>(
            "camera_input",
            10,
            [this](const sensor_msgs::msg::Image::SharedPtr imageRos) -> void { main(imageRos); }
        );

        auto publisher = create_publisher<geometry_msgs::msg::Point>("rune_in_camera", 10);
    }
};


int main(int argc, char** argv) {
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    spin(std::make_shared<PowerRuneNode>());
    printf("hello world power_rune package\n");
    return 0;
}
