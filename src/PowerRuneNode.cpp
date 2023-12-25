#include <cstdio>
#include "Alias.hpp"
#include "Converter/Transform.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using geometry_msgs::msg::PointStamped;
using ImageMsg = sensor_msgs::msg::Image;


class PowerRuneNode final : public Node {
    vector<double> _world_target = declare_parameter("world_target", vector<double>());
    vector<double> _world_center = declare_parameter("world_center", vector<double>());
    vector<double> _image_target = declare_parameter("image_target", vector<double>());
    vector<double> _image_center = declare_parameter("image_center", vector<double>());
    vector<double> _world_points = declare_parameter("world_points", vector<double>());
    vector<double> _image_points = declare_parameter("image_points", vector<double>());
    string         _source_image = declare_parameter("source_image", "null");

    shared_ptr<Feature> sourceFeature = nullptr;

    std::vector<Vec3> world_points() const {
        vector<Vec3>   points;
        vector<double> rawData;

        get_parameter("world_points", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_points is empty");
        if (rawData.size() % 3 != 0)
            RCLCPP_ERROR(get_logger(), "world_points size error");

        for (int i = 0; i < rawData.size(); i += 3) {
            points.emplace_back(rawData[i], rawData[i + 1], rawData[i + 2]);
        }
        return move(points);
    };

    std::vector<Vec2> image_points() const {
        vector<Vec2>   points;
        vector<double> rawData;

        get_parameter("image_points", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_points is empty");
        if (rawData.size() % 2 != 0)
            RCLCPP_ERROR(get_logger(), "image_points size error");

        for (int i = 0; i < rawData.size(); i += 2) {
            points.emplace_back(rawData[i], rawData[i + 1]);
        }
        return move(points);
    };

    Vec3 world_target() const {
        vector<double> rawData;

        get_parameter("world_target", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_target is empty");
        if (rawData.size() != 3)
            RCLCPP_ERROR(get_logger(), "world_target size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1]),
                    static_cast<float>(rawData[2])
                };
    };

    Vec3 world_center() const {
        vector<double> rawData;

        get_parameter("world_center", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "world_center is empty");
        if (rawData.size() != 3)
            RCLCPP_ERROR(get_logger(), "world_center size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1]),
                    static_cast<float>(rawData[2])
                };
    };

    Vec2 image_target() const {
        vector<double> rawData;

        get_parameter("image_target", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_target is empty");
        if (rawData.size() != 2)
            RCLCPP_ERROR(get_logger(), "image_target size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1])
                };
    };

    Vec2 image_center() const {
        vector<double> rawData;

        get_parameter("image_center", rawData);
        if (rawData.empty())
            RCLCPP_ERROR(get_logger(), "image_center is empty");
        if (rawData.size() != 2)
            RCLCPP_ERROR(get_logger(), "image_center size error");

        return {
                    static_cast<float>(rawData[0]),
                    static_cast<float>(rawData[1])
                };
    };

    int history_size() const {
        int size;
        get_parameter("history_size", size);
        return size;
    };

    Feature source_feature() {
        if (sourceFeature != nullptr) { return *sourceFeature; }
        // load
        string source_image;
        get_parameter("source_image", source_image);
        if (source_image == "null")
            RCLCPP_ERROR(get_logger(), "source_image unset");
        RCLCPP_INFO(get_logger(), "source_image: %s", source_image.c_str());
        Image image = cv::imread(source_image);
        if (image.empty())
            RCLCPP_ERROR(get_logger(), "source_image is empty");
        cvtColor(image, image, cv::COLOR_BGR2GRAY);
        sourceFeature = std::make_shared<Feature>(Feature::of(image));
        return *sourceFeature;
    };

    cv::Matx33f           cameraMatrix;
    cv::Matx<float, 1, 5> distCoeffs;

    deque<double> distances{};

    Publisher<PointStamped>::SharedPtr target_publisher =
            create_publisher<PointStamped>(
                "target",
                10
            );
    Subscription<ImageMsg>::SharedPtr image_subscriber =
            create_subscription<ImageMsg>(
                "/camera",
                10,
                [this](const ImageMsg::SharedPtr imageRos) -> void {
                    const cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(imageRos);
                    if (image->image.empty()) {
                        RCLCPP_WARN(get_logger(), "image is empty");
                        return;
                    }

                    Image grayImage;
                    cvtColor(image->image, grayImage, cv::COLOR_BGR2GRAY);

                    const Feature feature   = Feature::of(grayImage);
                    const Matches matches   = Matches::between(source_feature(), feature);
                    const auto    transform = Transform2D::fit(matches);
                    if (!transform)
                        return;

                    Vec2 target2d = *transform * image_target();
                    Vec2 center2d = *transform * image_center();

                    // double distance = length(target2d - center2d);
                    // distances.push_back(distance);
                    // if (distances.size() > history_size())
                    //     distances.pop_front();
                    // double average = accumulate(distances.begin(), distances.end(), 0.0)
                    //         / static_cast<double>(distances.size());
                    // center2d = target2d +(center2d -target2d)/distance*average;

                    cv::Mat show = image->image;
                    cv::circle(show, {static_cast<int>(target2d(0)), static_cast<int>(target2d(1))}, 5, cv::Scalar(255, 0, 255), 2);
                    cv::circle(show, {static_cast<int>(center2d(0)), static_cast<int>(center2d(1))}, 5, cv::Scalar(255, 255, 0), 2);
                    imshow("show", show);
                    cv::waitKey(1);

                    vector<Vec3> worldPoints = world_points();
                    vector<Vec2> imagePoints = *transform * image_points();

                    worldPoints.push_back(world_target());
                    worldPoints.push_back(world_center());
                    imagePoints.push_back(target2d);
                    imagePoints.push_back(center2d);

                    const Transform3D transform3D = Transform3D::fit(
                        worldPoints,
                        imagePoints,
                        cameraMatrix,
                        distCoeffs
                    );

                    Vec3 target = transform3D * world_target();

                    PointStamped taget_msg;
                    taget_msg.point.x = target(0);
                    taget_msg.point.y = target(1);
                    taget_msg.point.z = target(2);
                    taget_msg.header  = imageRos->header;
                    target_publisher->publish(taget_msg);


                    }
            );

public:
    explicit PowerRuneNode(): Node("power_rune") {
        RCLCPP_INFO(get_logger(), "power_rune main start");
    }

    void init() {
        cameraMatrix = cv::Matx33f{1777, 0, 1920 / 2, 0, 1777, 1071 / 2, 0, 0, 1};
        distCoeffs   = cv::Matx<float, 1, 5>{0, 0, 0, 0, 0};
    }
};


int main(const int argc, char** argv) {
    init(argc, argv);
    const auto powerRuneNode = std::make_shared<PowerRuneNode>();
    powerRuneNode->init();
    spin(powerRuneNode);
    printf("hello world power_rune package\n");
    return 0;
}
